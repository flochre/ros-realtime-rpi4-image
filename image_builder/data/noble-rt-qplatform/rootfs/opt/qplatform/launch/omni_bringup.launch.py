#!/usr/bin/env python3
# ==============================================================================
# QPlatform Omni Robot Bringup Launch File (URDF + ros2_control + IIO IMU + Nav2)
# ==============================================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = "/opt/qplatform"
    hw_install_prefix = "/opt/qplatform/ros2_ws/install/qplatform_hardware"
    if os.path.isdir(hw_install_prefix):
        ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
        if hw_install_prefix not in ament_prefix.split(":"):
            os.environ["AMENT_PREFIX_PATH"] = f"{hw_install_prefix}:{ament_prefix}" if ament_prefix else hw_install_prefix
        hw_lib = os.path.join(hw_install_prefix, "lib")
        ld_lib = os.environ.get("LD_LIBRARY_PATH", "")
        if hw_lib not in ld_lib.split(":"):
            os.environ["LD_LIBRARY_PATH"] = f"{hw_lib}:{ld_lib}" if ld_lib else hw_lib

    urdf_path = os.path.join(pkg_share, "robot_v2_recovered", "urdf", "robot_v2_recovered.urdf")

    # Declare arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_maixsense = LaunchConfiguration("launch_maixsense")
    params_file = LaunchConfiguration("nav2_params_file")
    controller_params = LaunchConfiguration("controller_params")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true"
    )

    declare_launch_nav2 = DeclareLaunchArgument(
        "launch_nav2",
        default_value="true",
        description="Whether to start Nav2 navigation stack"
    )

    declare_launch_imu = DeclareLaunchArgument(
        "launch_imu",
        default_value="true",
        description="Whether to start persistent-FD IIO IMU reader node"
    )

    declare_launch_maixsense = DeclareLaunchArgument(
        "launch_maixsense",
        default_value="true",
        description="Whether to start the MaixSense-A010 UDP-to-ROS 2 PointCloud2/Depth bridge"
    )

    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(pkg_share, "config", "nav2_omni_params.yaml"),
        description="Full path to Nav2 parameter file"
    )

    declare_controller_params = DeclareLaunchArgument(
        "controller_params",
        default_value=os.path.join(pkg_share, "config", "omni_controllers.yaml"),
        description="Full path to ros2_control configuration"
    )

    # Robot State Publisher with recovered URDF
    robot_description = ""
    if os.path.exists(urdf_path):
        with open(urdf_path, "r") as f:
            robot_description = f.read().replace(
                "package://robot_v2_recovered/",
                "file:///opt/qplatform/robot_v2_recovered/",
            )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }],
    )

    # ros2_control Controller Manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"robot_description": robot_description},
            controller_params,
        ],
    )

    # High-speed IIO IMU reader node (standalone script)
    iio_imu_node = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_share, "scripts", "iio_imu_node.py"),
            "--ros-args",
            "-p", "rate:=100.0",
            "-p", "frame_id:=imu_link",
            "-p", "topic:=/imu/data_raw",
        ],
        output="screen",
        condition=IfCondition(launch_imu),
    )

    # MaixSense-A010 ToF LiDAR UDP-to-ROS 2 bridge (demultiplexes all 3 wheel LiDARs on UDP port 9999)
    maixsense_bridge_node = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_share, "scripts", "maixsense_ros_bridge.py"),
            "--port", "9999",
        ],
        output="screen",
        condition=IfCondition(launch_maixsense),
    )

    # EKF Localization node (fuses omni wheel odom + LSM6DS3TR-C IMU)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "ekf_localization.yaml")],
    )

    # Spawn Joint State Broadcaster
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn Omni Wheel Drive Controller
    spawn_omni_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    launch_entities = [
        declare_use_sim_time,
        declare_launch_nav2,
        declare_launch_imu,
        declare_launch_maixsense,
        declare_nav2_params,
        declare_controller_params,
        rsp_node,
        control_node,
        iio_imu_node,
        maixsense_bridge_node,
        ekf_node,
        spawn_jsb,
        spawn_omni_controller,
    ]

    # Nav2 Bringup (only added if ros-jazzy-nav2-bringup is installed)
    try:
        nav2_bringup_dir = FindPackageShare(package="nav2_bringup").find("nav2_bringup")
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
            }.items(),
            condition=IfCondition(launch_nav2),
        )
        launch_entities.append(TimerAction(period=3.0, actions=[nav2_launch]))
    except Exception:
        pass

    return LaunchDescription(launch_entities)


