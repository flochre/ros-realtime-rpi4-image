#!/usr/bin/env python3
"""
MaixSense A010 UDP to ROS 2 Bridge Node (Multi-LiDAR Support)
=============================================================
Receives raw 100x100 depth frames over high-speed UDP from one or multiple
STM32H5 Ethernet Stepper Control boards, separates streams by MCU chip_id,
projects 3D coordinates, and publishes native ROS 2 messages per LiDAR device:

Sub-element topics (default):
  - /specific/maixsense/<chip_id>/points      [sensor_msgs/msg/PointCloud2]
  - /specific/maixsense/<chip_id>/image_depth [sensor_msgs/msg/Image]
  - /specific/maixsense/<chip_id>/image_color [sensor_msgs/msg/Image]
  - /specific/maixsense/<chip_id>/camera_info [sensor_msgs/msg/CameraInfo]

Name suffix topics:
  - /specific/maixsense/points_<chip_id>      [sensor_msgs/msg/PointCloud2]
  - /specific/maixsense/image_depth_<chip_id> [sensor_msgs/msg/Image]
  - /specific/maixsense/image_color_<chip_id> [sensor_msgs/msg/Image]
  - /specific/maixsense/camera_info_<chip_id> [sensor_msgs/msg/CameraInfo]

Configuration:
  - rotate_180: Rotate depth frame, color image, and pointcloud 180 degrees.
                Can be set via ROS 2 parameter (`-p rotate_180:=true`) or
                environment variable (`MAIXSENSE_ROTATE_180=1`).
"""

import os
import socket
import struct
import threading
import time
import json
import urllib.request
import urllib.error
import numpy as np

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

try:
    from cv_bridge import CvBridge
    HAVE_CV_BRIDGE = True
except ImportError:
    HAVE_CV_BRIDGE = False

# Precomputed Jet colormap (256x3 BGR uint8) for zero-dependency environments
def _generate_jet_lut():
    x = np.linspace(0.0, 1.0, 256)
    r = np.clip(np.minimum(4.0 * x - 1.5, -4.0 * x + 4.5), 0.0, 1.0)
    g = np.clip(np.minimum(4.0 * x - 0.5, -4.0 * x + 3.5), 0.0, 1.0)
    b = np.clip(np.minimum(4.0 * x + 0.5, -4.0 * x + 2.5), 0.0, 1.0)
    lut = np.stack([b, g, r], axis=-1) * 255.0
    return np.round(lut).astype(np.uint8)

JET_LUT_BGR = _generate_jet_lut()

# UDP packet header formats
# V2 Header (32 bytes packed little-endian with chip_id):
# magic (4B), chip_id (4B), frame_id (4B), sec (4B), nsec (4B),
# rows (2B), cols (2B), chunk_idx (2B), total_chunks (2B),
# payload_offset (2B), payload_len (2B), unit (1B), reserved (3B)
HEADER_STRUCT_V2 = struct.Struct("<IIIIIHHHHHHB3s")
HEADER_SIZE_V2 = HEADER_STRUCT_V2.size
MAGIC_TOF2 = 0x544F4632  # "TOF2"

# V1 Header (28 bytes packed little-endian legacy):
# magic (4B), frame_id (4B), sec (4B), nsec (4B),
# rows (2B), cols (2B), chunk_idx (2B), total_chunks (2B),
# payload_offset (2B), payload_len (2B), unit (1B), reserved (3B)
HEADER_STRUCT_V1 = struct.Struct("<IIIIHHHHHHB3s")
HEADER_SIZE_V1 = HEADER_STRUCT_V1.size
MAGIC_TOF1 = 0x544F4631  # "TOF1"


class LidarDeviceStream:
    """Manages frame reassembly and dedicated ROS 2 publishers for a single LiDAR device."""

    def __init__(
        self,
        node: Node,
        chip_id_str: str,
        base_topic_prefix: str,
        sensor_qos: QoSProfile,
        publish_sub_element: bool,
        publish_suffix: bool,
        sub_element_prefix: str,
    ):
        self.node = node
        self.chip_id = chip_id_str
        self.publish_sub_element = publish_sub_element
        self.publish_suffix = publish_suffix

        # In ROS 2, topic name tokens cannot start with a number.
        # If chip_id starts with a digit (e.g. '046F6260'), ensure valid token by prepending prefix.
        if chip_id_str and (chip_id_str[0].isalpha() or chip_id_str[0] == "_"):
            self.sub_element = chip_id_str
        else:
            prefix = sub_element_prefix if sub_element_prefix else "chip_"
            self.sub_element = f"{prefix}{chip_id_str}"

        clean_base = base_topic_prefix.rstrip("/")

        # 1. Sub-element publishers: /specific/maixsense/<sub_element>/...
        self.pub_sub_points = None
        self.pub_sub_depth = None
        self.pub_sub_color = None
        self.pub_sub_caminfo = None
        if self.publish_sub_element:
            self.topic_sub_points = f"{clean_base}/{self.sub_element}/points"
            self.topic_sub_depth = f"{clean_base}/{self.sub_element}/image_depth"
            self.topic_sub_color = f"{clean_base}/{self.sub_element}/image_color"
            self.topic_sub_caminfo = f"{clean_base}/{self.sub_element}/camera_info"

            self.pub_sub_points = node.create_publisher(PointCloud2, self.topic_sub_points, sensor_qos)
            self.pub_sub_depth = node.create_publisher(Image, self.topic_sub_depth, sensor_qos)
            self.pub_sub_color = node.create_publisher(Image, self.topic_sub_color, sensor_qos)
            self.pub_sub_caminfo = node.create_publisher(CameraInfo, self.topic_sub_caminfo, sensor_qos)

        # 2. Suffix publishers: /specific/maixsense/points_<chip_id>
        self.pub_suf_points = None
        self.pub_suf_depth = None
        self.pub_suf_color = None
        self.pub_suf_caminfo = None
        if self.publish_suffix:
            self.topic_suf_points = f"{clean_base}/points_{self.chip_id}"
            self.topic_suf_depth = f"{clean_base}/image_depth_{self.chip_id}"
            self.topic_suf_color = f"{clean_base}/image_color_{self.chip_id}"
            self.topic_suf_caminfo = f"{clean_base}/camera_info_{self.chip_id}"

            self.pub_suf_points = node.create_publisher(PointCloud2, self.topic_suf_points, sensor_qos)
            self.pub_suf_depth = node.create_publisher(Image, self.topic_suf_depth, sensor_qos)
            self.pub_suf_color = node.create_publisher(Image, self.topic_suf_color, sensor_qos)
            self.pub_suf_caminfo = node.create_publisher(CameraInfo, self.topic_suf_caminfo, sensor_qos)

        # Frame reassembly state
        self.current_frame_id = None
        self.current_frame_data = None
        self.chunks_received = set()
        self.total_chunks = 0
        self.frame_ts = (0, 0)
        self.frame_dims = (100, 100)
        self.frame_unit = 4

        # Statistics
        self.frames_published = 0
        self.last_stat_time = time.time()
        self.last_seen = time.time()

        node.get_logger().info(
            f"Initialized LiDAR stream for [{self.chip_id}] -> Namespace: '{self.sub_element}'"
        )

    def publish_images(self, img_depth, img_color, cam_info):
        """Publish images across active topic modes."""
        if self.pub_sub_depth is not None:
            self.pub_sub_depth.publish(img_depth)
            self.pub_sub_color.publish(img_color)
            self.pub_sub_caminfo.publish(cam_info)

        if self.pub_suf_depth is not None:
            self.pub_suf_depth.publish(img_depth)
            self.pub_suf_color.publish(img_color)
            self.pub_suf_caminfo.publish(cam_info)

    def publish_pointcloud(self, pc2_msg):
        """Publish point cloud across active topic modes."""
        if self.pub_sub_points is not None:
            self.pub_sub_points.publish(pc2_msg)

        if self.pub_suf_points is not None:
            self.pub_suf_points.publish(pc2_msg)

    def record_frame_published(self):
        """Track FPS for this device."""
        self.frames_published += 1
        now = time.time()
        if now - self.last_stat_time >= 5.0:
            elapsed = now - self.last_stat_time
            fps = self.frames_published / elapsed
            self.node.get_logger().info(f"LiDAR [{self.sub_element}] streaming @ {fps:.2f} FPS")
            self.frames_published = 0
            self.last_stat_time = now


class MaixSenseBridge(Node):
    def __init__(self):
        super().__init__("maixsense_bridge")

        # Declare parameters
        self.declare_parameter("udp_port", 9999)
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("base_topic", "/specific/maixsense")
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("frame_id_per_device", False)
        self.declare_parameter("publish_sub_element", True)
        self.declare_parameter("publish_suffix", True)
        self.declare_parameter("sub_element_prefix", "chip_")
        self.declare_parameter("fx", 55.0)
        self.declare_parameter("fy", 55.0)
        self.declare_parameter("u0", 50.0)
        self.declare_parameter("v0", 50.0)
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("publish_points", True)
        self.declare_parameter("publish_images", True)
        self.declare_parameter("filter_invalid", True)
        env_rotate_180 = os.environ.get("MAIXSENSE_ROTATE_180", "0").lower() in ("1", "true", "yes")
        self.declare_parameter("rotate_180", env_rotate_180)

        self.udp_port = int(self.get_parameter("udp_port").value)
        self.bind_host = str(self.get_parameter("bind_host").value)
        self.base_topic = str(self.get_parameter("base_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.frame_id_per_device = bool(self.get_parameter("frame_id_per_device").value)
        self.publish_sub_element = bool(self.get_parameter("publish_sub_element").value)
        self.publish_suffix = bool(self.get_parameter("publish_suffix").value)
        self.sub_element_prefix = str(self.get_parameter("sub_element_prefix").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).lower()
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.u0 = float(self.get_parameter("u0").value)
        self.v0 = float(self.get_parameter("v0").value)
        self.publish_points = bool(self.get_parameter("publish_points").value)
        self.publish_images = bool(self.get_parameter("publish_images").value)
        self.filter_invalid = bool(self.get_parameter("filter_invalid").value)
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)

        self.get_logger().info(
            f"[CONFIG] Setting rotate_180 = {self.rotate_180} "
            f"(env MAIXSENSE_ROTATE_180={os.environ.get('MAIXSENSE_ROTATE_180')})"
        )
        print(
            f"[MaixSense] STARTUP CONFIG: rotate_180 = {self.rotate_180} "
            f"(env MAIXSENSE_ROTATE_180={os.environ.get('MAIXSENSE_ROTATE_180')})",
            flush=True,
        )

        # QoS profile for sensor streams:
        # Use RELIABLE by default so RViz2 displays (which subscribe RELIABLE by default)
        # connect without QoS incompatibility warnings.
        # In DDS, RELIABLE publishers are compatible with both RELIABLE and BEST_EFFORT subscribers.
        if self.qos_reliability in ("best_effort", "besteffort"):
            rel_policy = ReliabilityPolicy.BEST_EFFORT
        else:
            rel_policy = ReliabilityPolicy.RELIABLE

        self.sensor_qos = QoSProfile(
            reliability=rel_policy,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge() if HAVE_CV_BRIDGE else None

        # Multi-device mapping: chip_id_str -> LidarDeviceStream
        self.devices = {}
        self.devices_lock = threading.Lock()
        self.ip_chip_cache = {}

        # Precompute PointCloud2 fields: x (float32), y (float32), z (float32), rgb (uint32)
        self.pc2_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]

        # Precompute coordinate grids for fast projection
        self.cached_rows = 100
        self.cached_cols = 100
        u = np.arange(self.cached_cols, dtype=np.float32)
        v = np.arange(self.cached_rows, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        self.y_factors = -((uu - self.u0) / self.fx)
        self.z_factors = -((vv - self.v0) / self.fy)

        # Start UDP listener thread
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        except Exception as e:
            self.get_logger().warn(f"Failed to set SO_RCVBUF: {e}")
        self.sock.bind((self.bind_host, self.udp_port))
        self.sock.settimeout(0.5)

        self.get_logger().info(
            f"MaixSense UDP Bridge listening on {self.bind_host}:{self.udp_port} "
            f"(Multi-device support enabled, base topic: {self.base_topic}, rotate_180: {self.rotate_180})"
        )

        self.udp_thread = threading.Thread(target=self._udp_worker, daemon=True)
        self.udp_thread.start()

    def _resolve_chip_id_from_ip(self, src_ip: str) -> str:
        """Resolve chip_id for legacy TOF1 packets by querying board HTTP info."""
        if src_ip in self.ip_chip_cache:
            return self.ip_chip_cache[src_ip]

        try:
            url = f"http://{src_ip}/api/hardware/infos"
            req = urllib.request.Request(url, headers={"User-Agent": "MaixSenseBridge"})
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                if data and isinstance(data, dict):
                    chip_id = list(data.keys())[0]
                    if chip_id:
                        chip_str = str(chip_id).upper()
                        self.ip_chip_cache[src_ip] = chip_str
                        self.get_logger().info(f"Resolved legacy LiDAR IP {src_ip} -> chip_id {chip_str}")
                        return chip_str
        except Exception:
            pass

        # Fallback to IP address token if HTTP query not available
        fallback = f"ip_{src_ip.replace('.', '_')}"
        self.ip_chip_cache[src_ip] = fallback
        return fallback

    def _get_device_stream(self, chip_id_str: str) -> LidarDeviceStream:
        """Get or dynamically instantiate a device stream handler for a chip_id."""
        with self.devices_lock:
            if chip_id_str not in self.devices:
                self.devices[chip_id_str] = LidarDeviceStream(
                    node=self,
                    chip_id_str=chip_id_str,
                    base_topic_prefix=self.base_topic,
                    sensor_qos=self.sensor_qos,
                    publish_sub_element=self.publish_sub_element,
                    publish_suffix=self.publish_suffix,
                    sub_element_prefix=self.sub_element_prefix,
                )
            return self.devices[chip_id_str]

    def _udp_worker(self):
        buf_size = 2048
        while self.running:
            try:
                data, addr = self.sock.recvfrom(buf_size)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f"UDP receive error: {e}")
                break

            try:
                if len(data) < 4:
                    continue

                (magic,) = struct.unpack_from("<I", data, 0)

                # Unpack V2 header (TOF2 with embedded chip_id)
                if magic == MAGIC_TOF2:
                    if len(data) < HEADER_SIZE_V2:
                        continue
                    (
                        magic,
                        chip_id_num,
                        frame_id,
                        sec,
                        nsec,
                        rows,
                        cols,
                        chunk_idx,
                        total_chunks,
                        payload_offset,
                        payload_len,
                        unit,
                        _reserved,
                    ) = HEADER_STRUCT_V2.unpack_from(data, 0)
                    chip_id_str = f"{chip_id_num:08X}"
                    header_size = HEADER_SIZE_V2

                # Unpack V1 legacy header (TOF1 without chip_id)
                elif magic == MAGIC_TOF1:
                    if len(data) < HEADER_SIZE_V1:
                        continue
                    (
                        magic,
                        frame_id,
                        sec,
                        nsec,
                        rows,
                        cols,
                        chunk_idx,
                        total_chunks,
                        payload_offset,
                        payload_len,
                        unit,
                        _reserved,
                    ) = HEADER_STRUCT_V1.unpack_from(data, 0)
                    chip_id_str = self._resolve_chip_id_from_ip(addr[0])
                    header_size = HEADER_SIZE_V1

                else:
                    continue

                dev = self._get_device_stream(chip_id_str)
                dev.last_seen = time.time()

                payload = data[header_size : header_size + payload_len]

                # Per-device frame reassembly
                if frame_id != dev.current_frame_id or dev.current_frame_data is None:
                    # Flush previous frame if complete
                    if dev.current_frame_data is not None and len(dev.chunks_received) == dev.total_chunks:
                        self._publish_frame(
                            dev,
                            dev.current_frame_data,
                            dev.frame_dims[0],
                            dev.frame_dims[1],
                            dev.frame_unit,
                            dev.frame_ts[0],
                            dev.frame_ts[1],
                        )

                    dev.current_frame_id = frame_id
                    expected_size = rows * cols
                    dev.current_frame_data = bytearray(expected_size)
                    dev.chunks_received = set()
                    dev.total_chunks = total_chunks
                    dev.frame_dims = (rows, cols)
                    dev.frame_ts = (sec, nsec)
                    dev.frame_unit = unit

                # Insert chunk data into device buffer
                end_offset = payload_offset + len(payload)
                if dev.current_frame_data is not None and end_offset <= len(dev.current_frame_data):
                    dev.current_frame_data[payload_offset:end_offset] = payload
                    dev.chunks_received.add(chunk_idx)

                    # If all chunks arrived, publish immediately
                    if len(dev.chunks_received) == dev.total_chunks:
                        self._publish_frame(
                            dev,
                            dev.current_frame_data,
                            rows,
                            cols,
                            unit,
                            sec,
                            nsec,
                        )
                        dev.current_frame_data = None
                        dev.chunks_received.clear()

            except Exception as e:
                self.get_logger().error(f"Error processing packet: {e}")

    def _publish_frame(self, dev: LidarDeviceStream, raw_bytes, rows, cols, unit, sec, nsec):
        if dev.frames_published == 0:
            self.get_logger().info(f"[{dev.sub_element}] First frame processing (rotate_180={self.rotate_180})")

        depth_arr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((rows, cols))
        if self.rotate_180:
            if HAVE_CV2:
                depth_arr = cv2.rotate(depth_arr, cv2.ROTATE_180)
            else:
                depth_arr = np.ascontiguousarray(np.rot90(depth_arr, 2))

        header = Header()
        header.stamp = Time(sec=sec, nanosec=nsec)
        if self.frame_id_per_device:
            header.frame_id = f"{self.frame_id}_{dev.sub_element}"
        else:
            header.frame_id = self.frame_id

        # Compute colormap once for both image & pointcloud
        color_bgr = None
        if self.publish_images or self.publish_points:
            if HAVE_CV2:
                color_bgr = cv2.applyColorMap(depth_arr, cv2.COLORMAP_JET)
            else:
                color_bgr = JET_LUT_BGR[depth_arr]

        # 1. Publish Depth Image & Color Image
        if self.publish_images and color_bgr is not None:
            if self.bridge is not None:
                img_msg = self.bridge.cv2_to_imgmsg(depth_arr, encoding="mono8")
                img_msg.header = header

                color_msg = self.bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
                color_msg.header = header
            else:
                img_msg = Image(
                    header=header,
                    height=rows,
                    width=cols,
                    encoding="mono8",
                    is_bigendian=False,
                    step=cols,
                    data=depth_arr.tobytes(),
                )
                color_msg = Image(
                    header=header,
                    height=rows,
                    width=cols,
                    encoding="bgr8",
                    is_bigendian=False,
                    step=cols * 3,
                    data=color_bgr.tobytes(),
                )

            # CameraInfo
            cam_info = CameraInfo()
            cam_info.header = header
            cam_info.height = rows
            cam_info.width = cols
            cam_info.distortion_model = "plumb_bob"
            cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            cam_info.k = [
                self.fx, 0.0, self.u0,
                0.0, self.fy, self.v0,
                0.0, 0.0, 1.0,
            ]
            cam_info.p = [
                self.fx, 0.0, self.u0, 0.0,
                0.0, self.fy, self.v0, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]

            # Publish to device-specific topics
            dev.publish_images(img_msg, color_msg, cam_info)

        # 2. Publish Full 3D PointCloud2
        if self.publish_points:
            # Distance in meters
            if unit == 0:
                depth_m = ((depth_arr.astype(np.float32) / 5.1) ** 2) * 0.001
            else:
                depth_m = depth_arr.astype(np.float32) * (unit * 0.001)

            # Fast vectorized body frame coordinates (laser_frame: X forward, Y left, Z up)
            x = depth_m
            if rows == self.cached_rows and cols == self.cached_cols:
                y = self.y_factors * depth_m
                z = self.z_factors * depth_m
            else:
                u = np.arange(cols, dtype=np.float32)
                v = np.arange(rows, dtype=np.float32)
                uu, vv = np.meshgrid(u, v)
                y = -((uu - self.u0) * depth_m) / self.fx
                z = -((vv - self.v0) * depth_m) / self.fy

            # Color mapping
            r = color_bgr[:, :, 2].astype(np.uint32)
            g = color_bgr[:, :, 1].astype(np.uint32)
            b = color_bgr[:, :, 0].astype(np.uint32)
            rgb_packed = (r << 16) | (g << 8) | b

            if self.filter_invalid:
                valid_mask = (depth_arr > 0) & (depth_arr < 255)
            else:
                valid_mask = np.ones((rows, cols), dtype=bool)

            x_valid = x[valid_mask]
            y_valid = y[valid_mask]
            z_valid = z[valid_mask]
            rgb_valid = rgb_packed[valid_mask]

            num_points = len(x_valid)
            if num_points > 0:
                # Pack into structured NumPy array
                point_dtype = np.dtype([
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("rgb", np.uint32),
                ])
                points_data = np.empty(num_points, dtype=point_dtype)
                points_data["x"] = x_valid
                points_data["y"] = y_valid
                points_data["z"] = z_valid
                points_data["rgb"] = rgb_valid

                pc2 = PointCloud2()
                pc2.header = header
                pc2.height = 1
                pc2.width = num_points
                pc2.fields = self.pc2_fields
                pc2.is_bigendian = False
                pc2.point_step = 16
                pc2.row_step = 16 * num_points
                pc2.is_dense = True
                pc2.data = points_data.tobytes()

                # Publish to device-specific topics
                dev.publish_pointcloud(pc2)

        dev.record_frame_published()

    def destroy_node(self):
        self.running = False
        if self.sock:
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MaixSenseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
