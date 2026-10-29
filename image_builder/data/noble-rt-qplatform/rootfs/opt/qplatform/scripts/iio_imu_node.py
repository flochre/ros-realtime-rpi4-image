#!/usr/bin/env python3
# ==============================================================================
# High-Speed IIO IMU Publisher for STMicroelectronics LSM6DSx (PREEMPT-RT)
# ==============================================================================
# Supports:
# 1. Automatic sysfs trigger (iio-trig-sysfs) creation and sensor assignment
# 2. Hardware Output Data Rate (ODR) configuration
# 3. High-throughput triggered buffer streaming via /dev/iio:device*
#    (frame layout parsed from scan_elements/, not hardcoded)
# 4. Accelerometer + gyroscope + magnetometer -> sensor_msgs/Imu (with 9-DOF quaternion)
# 5. Fallback persistent-FD sysfs reading with elevated ODR
# ==============================================================================

import os
import re
import glob
import math
import time
import select
import threading
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

IIO_DEVICES = '/sys/bus/iio/devices'
GAUSS_TO_TESLA = 1e-4  # IIO reports magnetic field in Gauss

# Sensor role -> IIO channel prefix
ROLE_PREFIX = {
    'accel': 'in_accel',
    'gyro': 'in_anglvel',
    'magn': 'in_magn',
}

# scan_elements/*_type format: [be|le]:[s|u]bits/storagebits[Xrepeat]>>shift
SCAN_TYPE_RE = re.compile(r'^(be|le):([su])(\d+)/(\d+)(?:X(\d+))?>>(\d+)$')


def _dev_sort_key(path):
    m = re.search(r'(\d+)$', path)
    return int(m.group(1)) if m else -1


def _read_sysfs(path, default=None):
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except Exception:
        return default


SYSFS_WRITE_TIMEOUT = 2.0  # seconds


def _write_sysfs(path, value, timeout=SYSFS_WRITE_TIMEOUT):
    """Write a sysfs attribute without letting a stuck driver hang the node.

    Some IIO drivers block in write() (e.g. waiting on a bus transaction or a
    lock). The write runs in a daemon thread; on timeout a TimeoutError is
    raised so the caller can log and fall back instead of freezing forever.
    """
    result = {}

    def _do_write():
        try:
            fd = os.open(path, os.O_WRONLY)
            try:
                os.write(fd, f"{value}\n".encode())
            finally:
                os.close(fd)
        except Exception as e:
            result['error'] = e

    t = threading.Thread(target=_do_write, daemon=True)
    t.start()
    t.join(timeout)
    if t.is_alive():
        raise TimeoutError(f"write '{value}' to {path} blocked for more than {timeout:.1f}s")
    if 'error' in result:
        raise result['error']


class ScanLayout:
    """Byte layout of one IIO buffer frame, derived from enabled scan_elements."""

    def __init__(self, dev_path):
        scan_dir = os.path.join(dev_path, 'scan_elements')
        elems = []
        for en_file in glob.glob(os.path.join(scan_dir, '*_en')):
            if _read_sysfs(en_file) != '1':
                continue
            name = os.path.basename(en_file)[:-len('_en')]
            idx = _read_sysfs(os.path.join(scan_dir, f'{name}_index'))
            typ = _read_sysfs(os.path.join(scan_dir, f'{name}_type'), '')
            m = SCAN_TYPE_RE.match(typ)
            if idx is None or not m:
                raise ValueError(f"Cannot parse scan element '{name}' (type='{typ}')")
            endian, sign, bits, storage, repeat, shift = m.groups()
            elems.append((int(idx), name, endian, sign == 's', int(bits),
                          int(storage) // 8, int(repeat or 1), int(shift)))

        # Elements are packed in scan-index order, each naturally aligned
        elems.sort()
        self.channels = {}
        offset = 0
        max_align = 1
        for _, name, endian, signed, bits, nbytes, repeat, shift in elems:
            if offset % nbytes:
                offset += nbytes - (offset % nbytes)
            self.channels[name] = (offset, endian, signed, bits, nbytes, shift)
            offset += nbytes * repeat
            max_align = max(max_align, nbytes)
        # Total frame size is padded to the largest element alignment
        if offset % max_align:
            offset += max_align - (offset % max_align)
        self.frame_size = offset

    def decode(self, frame):
        out = {}
        for name, (off, endian, signed, bits, nbytes, shift) in self.channels.items():
            raw = int.from_bytes(frame[off:off + nbytes], 'little' if endian == 'le' else 'big')
            raw = (raw >> shift) & ((1 << bits) - 1)
            if signed and raw & (1 << (bits - 1)):
                raw -= 1 << bits
            out[name] = raw
        return out


MAX_CONSUMERS_PER_TRIGGER = 2  # Linux kernel default CONFIG_IIO_CONSUMERS_PER_TRIGGER
SHUB_INVALID_SAMPLE = 0x7FFD   # st_lsm6dsx sensor-hub bootstrap marker


class Lsm6dsxIioNode(Node):
    def __init__(self):
        super().__init__('lsm6dsx_iio_node')

        # Parameters
        self.declare_parameter('rate', 100.0)            # Target rate (Hz)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('accel_cov', 0.01)        # (m/s^2)^2
        self.declare_parameter('gyro_cov', 0.001)        # (rad/s)^2
        self.declare_parameter('orientation_cov', 0.005) # rad^2
        self.declare_parameter('accel_gain', 2.0)        # Mahonykp for gravity (roll/pitch)
        self.declare_parameter('mag_gain', 1.5)          # Mahony kp for heading (yaw)
        self.declare_parameter('use_buffer', True)       # Use triggered buffer if True
        self.declare_parameter('use_mag', True)          # Fuse magnetometer in orientation if present

        self.rate = float(self.get_parameter('rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        topic_name = self.get_parameter('topic').value
        self.accel_gain = float(self.get_parameter('accel_gain').value)
        self.mag_gain = float(self.get_parameter('mag_gain').value)
        self.use_buffer = bool(self.get_parameter('use_buffer').value)
        self.use_mag = bool(self.get_parameter('use_mag').value)

        # Locate devices (role -> sysfs path) and group roles per device
        self.devices = self.find_imu_devices()
        if 'accel' not in self.devices or 'gyro' not in self.devices:
            raise RuntimeError(f"Accelerometer/gyroscope IIO devices not found (found: {self.devices})")
        self.dev_roles = {}
        # Order devices so accel & gyro come before magn (st_lsm6dsx shub requires accel master clock)
        for role in ('accel', 'gyro', 'magn'):
            dev = self.devices.get(role)
            if dev:
                self.dev_roles.setdefault(dev, set()).add(role)
        for role, dev in self.devices.items():
            self.get_logger().info(f"{role} device: {dev} ({_read_sysfs(os.path.join(dev, 'name'), '?')})")
        if self.use_mag and 'magn' not in self.devices:
            self.get_logger().warn("No magnetometer IIO device found, using 6-DOF orientation")

        # Initialize sysfs triggers (<= 2 devices per trigger) and configure ODR
        self.triggers = []         # list of (trig_name, trig_path)
        self.dev_triggers = {}     # dev_path -> (trig_name, trig_path)
        self.trigger_now_fds = []  # open FDs for each active trigger's trigger_now
        self.setup_iio_triggers()

        # Read scale factors
        self.accel_scale = self.read_scale('accel', 0.000598)
        self.gyro_scale = self.read_scale('gyro', 0.000153)
        self.mag_scale = 0.0015 * GAUSS_TO_TESLA
        log = f"Loaded scales: accel={self.accel_scale} m/s^2/LSB, gyro={self.gyro_scale} rad/s/LSB"
        if 'magn' in self.devices:
            self.mag_scale = self.read_scale('magn', 0.0015) * GAUSS_TO_TESLA
            log += f", magn={self.mag_scale} T/LSB"
        self.get_logger().info(log)

        # Setup buffer file descriptors or persistent sysfs fds
        self.buffer_mode = False
        self.buf_fds = {}         # sysfs dev path -> /dev/iio:deviceX fd
        self.layouts = {}         # sysfs dev path -> ScanLayout
        self.sysfs_fds = {}       # channel name (e.g. 'in_accel_x') -> file object
        self.mag_sysfs_thread = None
        self.mag_sysfs_stop = threading.Event()
        self.mag_lock = threading.Lock()
        self.pending_mag_raw = None
        self.read_timeout = 0.5 / self.rate

        if self.use_buffer and self.triggers:
            self.buffer_mode = self.init_buffer_streaming()

        if not self.buffer_mode:
            self.init_sysfs_streaming()

        # Orientation filter state (quaternion [w, x, y, z], body -> world ENU/NWU)
        self.q = None
        self.mag_yaw_initialized = False
        self.last_mag = None      # Latest valid scaled mag tuple (mx, my, mz) in Tesla
        self.last_imu_time = None

        # Setup publisher
        self.publisher = self.create_publisher(Imu, topic_name, 10)

        # Setup covariances
        accel_var = float(self.get_parameter('accel_cov').value)
        gyro_var = float(self.get_parameter('gyro_cov').value)
        orient_var = float(self.get_parameter('orientation_cov').value)
        self.accel_cov = [accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var]
        self.gyro_cov = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]
        self.orientation_cov = [orient_var, 0.0, 0.0, 0.0, orient_var, 0.0, 0.0, 0.0, orient_var]

        # Timer loop
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        trig_names = ', '.join(t[0] for t in self.triggers)
        mode = f"Triggered Buffer ({trig_names})" if self.buffer_mode else "Fast Sysfs"
        self.get_logger().info(f"LSM6DSx IMU active @ {self.rate} Hz [Mode: {mode}]")

    # --------------------------------------------------------------------------
    # Device discovery
    # --------------------------------------------------------------------------
    def find_imu_devices(self):
        """Identify devices by the channels they expose (robust to driver naming)."""
        devices = {}
        for dev in sorted(glob.glob(f'{IIO_DEVICES}/iio:device*'), key=_dev_sort_key):
            for role, prefix in ROLE_PREFIX.items():
                if role not in devices and os.path.exists(os.path.join(dev, f'{prefix}_x_raw')):
                    devices[role] = dev
        if not self.use_mag:
            devices.pop('magn', None)
        return devices

    def read_scale(self, role, default_val):
        dev = self.devices.get(role)
        prefix = ROLE_PREFIX[role]
        if not dev:
            return default_val
        for name in (f'{prefix}_scale', f'{prefix}_x_scale', 'scale'):
            val = _read_sysfs(os.path.join(dev, name))
            if val is not None:
                try:
                    return float(val)
                except ValueError:
                    pass
        return default_val

    # --------------------------------------------------------------------------
    # Trigger / ODR setup
    # --------------------------------------------------------------------------
    def find_sysfs_triggers(self):
        """Return all registered iio-trig-sysfs triggers as [(name, trig_dir), ...]."""
        found = []
        for trig_dir in sorted(glob.glob(f'{IIO_DEVICES}/trigger*'), key=_dev_sort_key):
            name = _read_sysfs(os.path.join(trig_dir, 'name'))
            if name and 'sysfstrig' in name:
                found.append((name, trig_dir))
        return found

    def ensure_sysfs_triggers(self, needed):
        """Ensure at least `needed` iio-trig-sysfs triggers exist and return them."""
        triggers = self.find_sysfs_triggers()
        add_trig_path = f"{IIO_DEVICES}/iio_sysfs_trigger/add_trigger"
        if len(triggers) < needed and os.path.exists(add_trig_path):
            for trig_id in range(8):
                if len(triggers) >= needed:
                    break
                if any(name == f'sysfstrig{trig_id}' for name, _ in triggers):
                    continue
                try:
                    _write_sysfs(add_trig_path, trig_id)
                except Exception:
                    continue
                triggers = self.find_sysfs_triggers()
        return triggers

    def disable_device_buffer(self, dev_path):
        """Ensure a device's IIO buffer is disabled before reconfiguring trigger/ODR."""
        buf_enable_path = os.path.join(dev_path, 'buffer', 'enable')
        if _read_sysfs(buf_enable_path) == '1':
            try:
                _write_sysfs(buf_enable_path, 0)
            except Exception as e:
                self.get_logger().warn(f"Could not disable buffer on {dev_path}: {e}")

    def set_device_trigger(self, dev_path, trigger_name):
        """Assign current_trigger for an IIO sensor device."""
        cur_trig_path = os.path.join(dev_path, 'trigger', 'current_trigger')

        if not os.path.exists(cur_trig_path):
            self.get_logger().warn(f"No trigger/current_trigger found for {dev_path}")
            return False

        try:
            self.disable_device_buffer(dev_path)

            if _read_sysfs(cur_trig_path) == trigger_name:
                self.get_logger().info(f"{dev_path} trigger is already {trigger_name}")
                return True

            _write_sysfs(cur_trig_path, trigger_name)
            self.get_logger().info(f"Updated current_trigger for {dev_path} to {trigger_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set current_trigger for {dev_path} to {trigger_name}: {e}")
            return False

    def configure_odr(self, dev_path, target_rate):
        """Set hardware sampling frequency (ODR) to the lowest supported value >= target."""
        roles = self.dev_roles.get(dev_path, ())
        candidates = ['sampling_frequency'] + [f'{ROLE_PREFIX[r]}_sampling_frequency' for r in roles]
        samp_file = None
        avail_file = None
        for cand in candidates:
            p = os.path.join(dev_path, cand)
            if os.path.exists(p):
                samp_file = p
                avail_file = os.path.join(dev_path, f'{cand}_available')
                break
        if not samp_file:
            return

        self.disable_device_buffer(dev_path)

        target_freq = target_rate
        avail = _read_sysfs(avail_file) or _read_sysfs(os.path.join(dev_path, 'sampling_frequency_available'))
        if avail:
            try:
                avail_freqs = [float(x) for x in avail.split()]
                if avail_freqs:
                    suitable = [freq for freq in avail_freqs if freq >= target_rate]
                    target_freq = min(suitable) if suitable else max(avail_freqs)
            except ValueError:
                pass

        current = _read_sysfs(samp_file)
        try:
            if current is not None and abs(float(current) - target_freq) < 1e-3:
                self.get_logger().info(f"Sampling frequency on {dev_path} already {current} Hz")
                return
        except ValueError:
            pass

        self.get_logger().info(f"Setting sampling frequency on {dev_path}: {current} -> {target_freq:g} Hz")
        try:
            _write_sysfs(samp_file, f"{target_freq:g}")
            self.get_logger().info(f"Configured sampling frequency on {dev_path} to {target_freq:g} Hz")
        except Exception as e:
            self.get_logger().warn(f"Could not set sampling frequency on {dev_path}: {e}")

    def setup_iio_triggers(self):
        """Create/discover sysfs triggers and assign them (<= 2 devices per trigger)."""
        # 1. Optional external setup script. Run first so that the sysfs trigger
        #    assignment below is not overridden by it.
        setup_script = "/opt/qplatform/scripts/setup_iio_trigger.sh"
        if os.path.exists(setup_script):
            try:
                subprocess.run(["bash", setup_script, str(int(self.rate))], check=False,
                               stdout=subprocess.DEVNULL, timeout=10)
            except Exception as e:
                self.get_logger().warn(f"Trigger setup script failed: {e}")

        # 2. Attempt to load kernel module (no-op if already loaded / not root)
        try:
            subprocess.run(["modprobe", "iio-trig-sysfs"], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        except Exception:
            pass

        # 3. Disable all device buffers first so no device holds a trigger slot or blocks ODR writes
        devs = list(self.dev_roles.keys())
        for dev in devs:
            self.disable_device_buffer(dev)

        # 4. Linux kernel defaults to CONFIG_IIO_CONSUMERS_PER_TRIGGER=2, so allocate
        #    one sysfs trigger per pair of IIO devices (e.g. sysfstrig0 for accel+gyro,
        #    sysfstrig1 for magn).
        needed_triggers = max(1, math.ceil(len(devs) / MAX_CONSUMERS_PER_TRIGGER))
        self.triggers = self.ensure_sysfs_triggers(needed_triggers)
        if self.triggers:
            for name, path in self.triggers:
                self.get_logger().info(f"Active sysfs trigger: '{name}' at {path}")
        else:
            self.get_logger().warn("Could not find or create any iio-trig-sysfs triggers")

        # 5. Assign triggers and configure ODR on each sensor device
        for idx, dev in enumerate(devs):
            if self.triggers:
                trig_idx = min(idx // MAX_CONSUMERS_PER_TRIGGER, len(self.triggers) - 1)
                trig_name, trig_path = self.triggers[trig_idx]
                if self.set_device_trigger(dev, trig_name):
                    self.dev_triggers[dev] = (trig_name, trig_path)
            self.configure_odr(dev, self.rate)

    # --------------------------------------------------------------------------
    # Triggered buffer mode
    # --------------------------------------------------------------------------
    def setup_device_buffer(self, dev_path, roles):
        """Enable required scan elements + timestamp, enable buffer, return its ScanLayout."""
        buf_enable_path = os.path.join(dev_path, 'buffer', 'enable')
        buf_length_path = os.path.join(dev_path, 'buffer', 'length')
        scan_dir = os.path.join(dev_path, 'scan_elements')

        if not os.path.exists(buf_enable_path) or not os.path.isdir(scan_dir):
            raise RuntimeError(f"{dev_path} has no buffer/scan_elements support")

        self.disable_device_buffer(dev_path)

        # Enable only the channels we need for this device, disable the others
        wanted = tuple(f'{ROLE_PREFIX[r]}_' for r in roles) + ('in_timestamp',)
        for en_file in glob.glob(os.path.join(scan_dir, '*_en')):
            name = os.path.basename(en_file)[:-len('_en')]
            try:
                _write_sysfs(en_file, 1 if name.startswith(wanted) else 0)
            except Exception:
                pass

        layout = ScanLayout(dev_path)
        for role in roles:
            for axis in 'xyz':
                ch = f'{ROLE_PREFIX[role]}_{axis}'
                if ch not in layout.channels:
                    raise RuntimeError(f"Scan element '{ch}' could not be enabled on {dev_path}")

        if os.path.exists(buf_length_path):
            try:
                _write_sysfs(buf_length_path, 100)
            except Exception:
                pass

        # Magnetometer on st_lsm6dsx sensor hub performs multiple I2C transactions on enable
        enable_timeout = 5.0 if 'magn' in roles else SYSFS_WRITE_TIMEOUT
        _write_sysfs(buf_enable_path, 1, timeout=enable_timeout)
        self.get_logger().info(
            f"Triggered buffer enabled for {dev_path} "
            f"(channels={list(layout.channels)}, frame={layout.frame_size} bytes)")
        return layout

    def init_buffer_streaming(self):
        active_trig_paths = []
        try:
            # 1. Enable accel and gyro buffers first (required for LSM6DSx sensor hub clock)
            for dev, roles in self.dev_roles.items():
                if 'magn' in roles and not (roles & {'accel', 'gyro'}):
                    continue
                self.layouts[dev] = self.setup_device_buffer(dev, roles)
                chr_path = f"/dev/{os.path.basename(dev)}"
                self.buf_fds[dev] = os.open(chr_path, os.O_RDONLY | os.O_NONBLOCK)
                self.get_logger().info(f"Opened buffer {chr_path}")
                if dev in self.dev_triggers:
                    _, tpath = self.dev_triggers[dev]
                    if tpath not in active_trig_paths:
                        active_trig_paths.append(tpath)
        except Exception as e:
            self.get_logger().warn(f"Triggered buffer setup failed for IMU, falling back to sysfs: {e}")
            self.close_buffers()
            return False

        # 2. Enable standalone magnetometer buffer independently so a mag error
        #    never degrades accel/gyro buffer streaming
        for dev, roles in self.dev_roles.items():
            if 'magn' not in roles or (roles & {'accel', 'gyro'}):
                continue
            try:
                self.layouts[dev] = self.setup_device_buffer(dev, roles)
                chr_path = f"/dev/{os.path.basename(dev)}"
                self.buf_fds[dev] = os.open(chr_path, os.O_RDONLY | os.O_NONBLOCK)
                self.get_logger().info(f"Opened magnetometer buffer {chr_path}")
                if dev in self.dev_triggers:
                    _, tpath = self.dev_triggers[dev]
                    if tpath not in active_trig_paths:
                        active_trig_paths.append(tpath)
            except Exception as e:
                self.get_logger().warn(
                    f"Magnetometer buffer setup failed on {dev} ({e}); "
                    f"using background sysfs polling for magnetometer")
                self.disable_device_buffer(dev)
                self.start_mag_sysfs_thread(dev)

        try:
            for tpath in active_trig_paths:
                fd = os.open(os.path.join(tpath, 'trigger_now'), os.O_WRONLY)
                self.trigger_now_fds.append(fd)
            return bool(self.trigger_now_fds)
        except Exception as e:
            self.get_logger().warn(f"Could not open trigger_now: {e}")
            self.close_buffers()
            return False

    def start_mag_sysfs_thread(self, mag_dev):
        """Background polling thread for magnetometers that cannot use buffered mode."""
        mag_files = {}
        prefix = ROLE_PREFIX['magn']
        for axis in 'xyz':
            path = os.path.join(mag_dev, f'{prefix}_{axis}_raw')
            try:
                mag_files[axis] = open(path, 'r')
            except OSError as e:
                self.get_logger().warn(f"Cannot open {path}: {e}")
                return

        def _poll_mag():
            period = max(1.0 / min(self.rate, 50.0), 0.02)
            while not self.mag_sysfs_stop.is_set():
                t0 = time.monotonic()
                try:
                    vals = []
                    for axis in 'xyz':
                        f = mag_files[axis]
                        f.seek(0)
                        vals.append(int(f.read().strip()))
                    with self.mag_lock:
                        self.pending_mag_raw = tuple(vals)
                except Exception:
                    pass
                elapsed = time.monotonic() - t0
                if elapsed < period:
                    self.mag_sysfs_stop.wait(period - elapsed)
            for f in mag_files.values():
                try:
                    f.close()
                except Exception:
                    pass

        self.mag_sysfs_stop.clear()
        self.mag_sysfs_thread = threading.Thread(target=_poll_mag, daemon=True)
        self.mag_sysfs_thread.start()

    def close_buffers(self):
        """Close buffer fds and disable buffers (raw sysfs reads return EBUSY otherwise)."""
        self.mag_sysfs_stop.set()
        if self.mag_sysfs_thread is not None:
            self.mag_sysfs_thread.join(timeout=1.0)
            self.mag_sysfs_thread = None
        for fd in self.buf_fds.values():
            try:
                os.close(fd)
            except Exception:
                pass
        self.buf_fds.clear()
        self.layouts.clear()
        for dev in self.dev_roles:
            self.disable_device_buffer(dev)
        for fd in self.trigger_now_fds:
            try:
                os.close(fd)
            except Exception:
                pass
        self.trigger_now_fds.clear()

    def fire_sysfs_trigger(self):
        """Pulse all active sysfs triggers so every attached device captures one frame."""
        for fd in self.trigger_now_fds:
            try:
                os.pwrite(fd, b'1\n', 0)
            except Exception as e:
                self.get_logger().error(f"Failed to fire trigger: {e}", throttle_duration_sec=5.0)

    def read_latest_frame(self, dev, deadline, wait=True):
        """Wait (until deadline if wait=True) for data, drain buffer and decode newest frame."""
        fd = self.buf_fds[dev]
        layout = self.layouts[dev]
        fs = layout.frame_size

        timeout = max(0.0, deadline - time.monotonic()) if wait else 0.0
        r, _, _ = select.select([fd], [], [], timeout)
        if not r:
            return None

        latest = None
        chunk_size = fs * 32
        while True:
            try:
                data = os.read(fd, chunk_size)
            except (BlockingIOError, InterruptedError):
                break
            n = len(data) // fs
            if n:
                latest = data[(n - 1) * fs:n * fs]
            if len(data) < chunk_size:
                break
        return layout.decode(latest) if latest else None

    # --------------------------------------------------------------------------
    # Sysfs fallback mode
    # --------------------------------------------------------------------------
    def init_sysfs_streaming(self):
        self.get_logger().info("Using persistent sysfs file descriptors with elevated ODR")
        for dev, roles in self.dev_roles.items():
            if 'magn' in roles and not (roles & {'accel', 'gyro'}):
                # Poll slow oneshot magnetometer on a background thread so it never stalls IMU
                self.configure_odr(dev, self.rate)
                self.start_mag_sysfs_thread(dev)
                continue
            # Elevated ODR on accel/gyro to eliminate oneshot delay
            target = max(self.rate, 208.0) if roles & {'accel', 'gyro'} else self.rate
            self.configure_odr(dev, target)
            for role in roles:
                prefix = ROLE_PREFIX[role]
                for axis in 'xyz':
                    path = os.path.join(dev, f'{prefix}_{axis}_raw')
                    try:
                        self.sysfs_fds[f'{prefix}_{axis}'] = open(path, 'r')
                    except OSError as e:
                        self.get_logger().warn(f"Cannot open {path}: {e}")

    def read_sysfs_values(self):
        values = {}
        for name, f in self.sysfs_fds.items():
            try:
                f.seek(0)
                values[name] = int(f.read().strip())
            except Exception as e:
                self.get_logger().error(f"Error reading {name}: {e}", throttle_duration_sec=5.0)
        return values

    # --------------------------------------------------------------------------
    # 9-DOF Orientation Quaternion Estimation (Accel + Gyro + Mag)
    # --------------------------------------------------------------------------
    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qw, qx, qy, qz]

    @staticmethod
    def _tilt_compensated_yaw(roll, pitch, mx, my, mz):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        bx = mx * cp + my * sp * sr + mz * sp * cr
        by = my * cr - mz * sr
        return math.atan2(-by, bx)

    def update_orientation(self, ax, ay, az, gx, gy, gz, mag, dt):
        """Update 3D orientation quaternion [qw, qx, qy, qz] using 9-DOF Mahony filter."""
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)

        # Initialize quaternion from gravity (and magnetometer if available)
        if self.q is None:
            if a_norm > 1e-3:
                roll = math.atan2(ay, az)
                pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
                yaw = 0.0
                if mag is not None:
                    m_norm = math.sqrt(mag[0] ** 2 + mag[1] ** 2 + mag[2] ** 2)
                    if m_norm > 1e-9:
                        yaw = self._tilt_compensated_yaw(
                            roll, pitch, mag[0] / m_norm, mag[1] / m_norm, mag[2] / m_norm)
                        self.mag_yaw_initialized = True
                self.q = self._euler_to_quat(roll, pitch, yaw)
            else:
                self.q = [1.0, 0.0, 0.0, 0.0]
            return self.q

        qw, qx, qy, qz = self.q

        # Snap initial yaw as soon as the first valid magnetometer sample arrives
        if not self.mag_yaw_initialized and mag is not None:
            m_norm = math.sqrt(mag[0] ** 2 + mag[1] ** 2 + mag[2] ** 2)
            if m_norm > 1e-9:
                mx, my, mz = mag[0] / m_norm, mag[1] / m_norm, mag[2] / m_norm
                hx = (1.0 - 2.0 * (qy * qy + qz * qz)) * mx + 2.0 * (qx * qy - qw * qz) * my + 2.0 * (qx * qz + qw * qy) * mz
                hy = 2.0 * (qx * qy + qw * qz) * mx + (1.0 - 2.0 * (qx * qx + qz * qz)) * my + 2.0 * (qy * qz - qw * qx) * mz
                yaw_err = math.atan2(-hy, hx)
                half_yaw = 0.5 * yaw_err
                cy = math.cos(half_yaw)
                sy = math.sin(half_yaw)
                # Pre-multiply by world-Z rotation q_yaw = [cy, 0, 0, sy]
                qw, qx, qy, qz = (
                    cy * qw - sy * qz,
                    cy * qx - sy * qy,
                    cy * qy + sy * qx,
                    cy * qz + sy * qw,
                )
                self.mag_yaw_initialized = True

        # Predicted gravity direction in body frame (3rd row of R(q))
        vx = 2.0 * (qx * qz - qw * qy)
        vy = 2.0 * (qy * qz + qw * qx)
        vz = 1.0 - 2.0 * (qx * qx + qy * qy)

        ex = 0.0
        ey = 0.0
        ez = 0.0

        # 1. Accelerometer roll/pitch correction: e_acc = a_unit x v_grav
        if a_norm > 1e-3:
            ax_u = ax / a_norm
            ay_u = ay / a_norm
            az_u = az / a_norm
            ex += self.accel_gain * (ay_u * vz - az_u * vy)
            ey += self.accel_gain * (az_u * vx - ax_u * vz)
            ez += self.accel_gain * (ax_u * vy - ay_u * vx)

        # 2. Magnetometer yaw correction (decoupled around world Z axis so magnetic
        #    inclination / Z-offsets never disturb roll and pitch)
        if mag is not None:
            m_norm = math.sqrt(mag[0] ** 2 + mag[1] ** 2 + mag[2] ** 2)
            if m_norm > 1e-9:
                mx = mag[0] / m_norm
                my = mag[1] / m_norm
                mz = mag[2] / m_norm
                # Rotate measured magnetic field into world horizontal plane
                hx = (1.0 - 2.0 * (qy * qy + qz * qz)) * mx + 2.0 * (qx * qy - qw * qz) * my + 2.0 * (qx * qz + qw * qy) * mz
                hy = 2.0 * (qx * qy + qw * qz) * mx + (1.0 - 2.0 * (qx * qx + qz * qz)) * my + 2.0 * (qy * qz - qw * qx) * mz
                if hx * hx + hy * hy > 1e-6:
                    yaw_err = math.atan2(-hy, hx)
                    # Project world-Z heading error (0, 0, yaw_err) into body frame via R^T
                    ex += self.mag_gain * yaw_err * vx
                    ey += self.mag_gain * yaw_err * vy
                    ez += self.mag_gain * yaw_err * vz

        # Apply feedback to gyroscope rates
        wx = gx + ex
        wy = gy + ey
        wz = gz + ez

        # Integrate quaternion: q_dot = 0.5 * q * [0, wx, wy, wz]
        half_dt = 0.5 * dt
        qw += (-qx * wx - qy * wy - qz * wz) * half_dt
        qx += ( qw * wx + qy * wz - qz * wy) * half_dt
        qy += ( qw * wy - qx * wz + qz * wx) * half_dt
        qz += ( qw * wz + qx * wy - qy * wx) * half_dt

        # Normalize quaternion
        q_norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if q_norm > 1e-9:
            qw /= q_norm
            qx /= q_norm
            qy /= q_norm
            qz /= q_norm
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        self.q = [qw, qx, qy, qz]
        return self.q

    # --------------------------------------------------------------------------
    # Publishing
    # --------------------------------------------------------------------------
    @staticmethod
    def get_vector(values, role):
        prefix = ROLE_PREFIX[role]
        try:
            return tuple(values[f'{prefix}_{axis}'] for axis in 'xyz')
        except KeyError:
            return None

    @staticmethod
    def is_valid_mag_raw(mag_raw):
        if mag_raw is None:
            return False
        if mag_raw == (0, 0, 0):
            return False
        if any(v >= SHUB_INVALID_SAMPLE for v in mag_raw):
            return False
        return True

    def timer_callback(self):
        now_mono = time.monotonic()
        if self.buffer_mode:
            self.fire_sysfs_trigger()
            deadline = now_mono + self.read_timeout
            values = {}
            for dev in self.buf_fds:
                roles = self.dev_roles.get(dev, set())
                # Wait for accel/gyro, poll standalone mag non-blockingly if deadline passed
                is_imu_dev = bool(roles & {'accel', 'gyro'})
                frame = self.read_latest_frame(dev, deadline, wait=is_imu_dev)
                if frame:
                    values.update(frame)
        else:
            values = self.read_sysfs_values()

        # Check if background magnetometer sysfs thread has a new sample
        if self.mag_sysfs_thread is not None and 'in_magn_x' not in values:
            with self.mag_lock:
                if self.pending_mag_raw is not None:
                    values['in_magn_x'], values['in_magn_y'], values['in_magn_z'] = self.pending_mag_raw
                    self.pending_mag_raw = None

        stamp = self.get_clock().now().to_msg()

        # Extract and validate magnetometer reading so it feeds the orientation quaternion
        mag_raw = self.get_vector(values, 'magn')
        if self.is_valid_mag_raw(mag_raw):
            self.last_mag = (
                mag_raw[0] * self.mag_scale,
                mag_raw[1] * self.mag_scale,
                mag_raw[2] * self.mag_scale,
            )

        accel = self.get_vector(values, 'accel')
        gyro = self.get_vector(values, 'gyro')
        if accel is not None and gyro is not None:
            ax = accel[0] * self.accel_scale
            ay = accel[1] * self.accel_scale
            az = accel[2] * self.accel_scale
            gx = gyro[0] * self.gyro_scale
            gy = gyro[1] * self.gyro_scale
            gz = gyro[2] * self.gyro_scale

            if self.last_imu_time is None:
                dt = 1.0 / self.rate
            else:
                dt = max(1e-4, min(0.5, now_mono - self.last_imu_time))
            self.last_imu_time = now_mono

            qw, qx, qy, qz = self.update_orientation(ax, ay, az, gx, gy, gz, self.last_mag, dt)

            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id

            # Scaled values in m/s^2 and rad/s
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.linear_acceleration_covariance = self.accel_cov

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.angular_velocity_covariance = self.gyro_cov

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
            msg.orientation_covariance = self.orientation_cov

            self.publisher.publish(msg)

    def destroy_node(self):
        self.close_buffers()
        for f in self.sysfs_fds.values():
            try:
                f.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Lsm6dsxIioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()