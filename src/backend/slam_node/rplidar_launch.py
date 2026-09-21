"""
rplidar_launch.py — NIDAR AirMouse: Full SLAM Stack Launch File
===============================================================
Brings up the complete SLAM pipeline in the correct dependency order:

  1. Static TF: base_link → laser
     (describes where the LiDAR is mounted relative to the drone body)

  2. RPLIDAR A2M8 driver
     (reads raw LiDAR data from USB serial, publishes /scan)

  3. slam_toolbox (async online mapping)
     (subscribes /scan, publishes /map and TF map→odom→base_link)

  4. slam_node (this package)
     (relays /map, extracts TF→/drone_pose, monitors scan health)

⚠️  HARDWARE-SPECIFIC VALUES — adjust via .env before deployment:
    RPLIDAR_PORT     (default /dev/ttyUSB0) — USB serial device path
    RPLIDAR_BAUDRATE (default 115200)       — A2M8 baud rate (fixed by hardware)

Usage:
    source install/setup.bash
    ros2 launch slam_node rplidar_launch.py
    # Or with overrides:
    ros2 launch slam_node rplidar_launch.py serial_port:=/dev/ttyUSB1
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """
    Returns a LaunchDescription that starts the full SLAM stack.
    Called automatically by `ros2 launch`.
    """

    # -------------------------------------------------------------------------
    # Resolve paths
    # -------------------------------------------------------------------------
    # slam_params.yaml is installed into the package share directory by setup.py
    slam_node_share = get_package_share_directory("slam_node")
    slam_params_file = os.path.join(slam_node_share, "config", "slam_params.yaml")

    # -------------------------------------------------------------------------
    # Launch arguments (can be overridden on the command line)
    # -------------------------------------------------------------------------
    # ⚠️ HARDWARE: serial_port must match the actual USB device on the Pi4.
    # Run `ls /dev/ttyUSB*` after plugging in the RPLIDAR to confirm.
    # The default reads from .env → RPLIDAR_PORT; falls back to /dev/ttyUSB0.
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value=EnvironmentVariable("RPLIDAR_PORT", default_value="/dev/ttyUSB0"),
        description="USB serial port for RPLIDAR A2M8 (e.g. /dev/ttyUSB0)",
    )

    # ⚠️ HARDWARE: RPLIDAR A2M8 baud rate is fixed at 115200 by the hardware.
    # Do NOT change this unless you are using a different RPLIDAR model.
    serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value=EnvironmentVariable("RPLIDAR_BAUDRATE", default_value="115200"),
        description="Baud rate for RPLIDAR A2M8 (fixed at 115200 for A2M8 hardware)",
    )

    # Frame ID of the LiDAR sensor — must match slam_params.yaml base_frame chain
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="laser",
        description="TF frame ID for the LiDAR scan messages",
    )

    # -------------------------------------------------------------------------
    # Node 1: Static Transform Publisher — base_link → laser
    # -------------------------------------------------------------------------
    # This tells ROS 2 where the LiDAR is physically mounted on the drone body.
    # The RPLIDAR is assumed to be mounted flat at the centre of the drone frame,
    # with no rotation offset.  Adjust x/y/z if your mount is offset.
    #
    # Format: x y z roll pitch yaw  (metres, radians)
    # Current assumption:
    #   x=0.0   — LiDAR centred fore/aft on drone frame
    #   y=0.0   — LiDAR centred left/right
    #   z=0.05  — LiDAR is 5cm above base_link origin (motor plate height)
    #   roll=pitch=yaw=0 — LiDAR is level and aligned with drone forward axis
    #
    # ⚠️ HARDWARE: Measure the actual offset on your drone and update these values.
    static_tf_base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_laser",
        # Arguments: x y z roll pitch yaw parent_frame child_frame
        arguments=["0.0", "0.0", "0.05", "0.0", "0.0", "0.0", "base_link", "laser"],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Node 2: RPLIDAR A2M8 Driver
    # -------------------------------------------------------------------------
    # rplidar_ros driver node — reads from USB serial and publishes /scan.
    # Package: rplidar_ros (install: sudo apt install ros-humble-rplidar-ros)
    #
    # Published topics:
    #   /scan  (sensor_msgs/LaserScan) — 360° scan at ~10Hz, range 0.15–8.0m
    rplidar_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            # ⚠️ HARDWARE: serial_port and serial_baudrate come from launch args above
            "serial_port": LaunchConfiguration("serial_port"),
            "serial_baudrate": LaunchConfiguration("serial_baudrate"),
            # frame_id must match the child frame in the static_transform_publisher above
            "frame_id": LaunchConfiguration("frame_id"),
            # angle_compensate: interpolates angular positions → smoother scans
            "angle_compensate": True,
            # scan_mode: "Standard" for A2M8 (max range 8m, ~8000 samples/rev)
            # Other modes (Boost/Sensitivity) are for A3/S series, not A2M8
            "scan_mode": "Standard",
        }],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Node 3: slam_toolbox — Async Online Mapping
    # -------------------------------------------------------------------------
    # slam_toolbox handles scan matching, pose graph optimisation, loop closure,
    # and publishes:
    #   /map      (nav_msgs/OccupancyGrid)   — occupancy grid of the arena
    #   TF:       map → odom → base_link     — pose estimate
    #
    # Package: slam_toolbox (install: sudo apt install ros-humble-slam-toolbox)
    #
    # We use async_slam_toolbox_node (not sync) to avoid blocking the ROS spinner
    # on Pi4 where CPU cores are limited.
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            slam_params_file,   # all tuning in slam_params.yaml
            {
                # use_sim_time: False — we use wall clock (real hardware, no Gazebo)
                "use_sim_time": False,
            }
        ],
        output="screen",
        # remappings: slam_toolbox default scan topic is /scan — no remap needed
    )

    # -------------------------------------------------------------------------
    # Node 4: slam_node (this package) — started 2s after slam_toolbox
    # -------------------------------------------------------------------------
    # TimerAction delays slam_node startup by 2 seconds to give slam_toolbox
    # time to initialise its internal state machine before we start polling TF.
    # Without this delay, TF lookups fail and spam the log on startup.
    slam_node = TimerAction(
        period=2.0,   # seconds — give slam_toolbox time to boot and populate TF
        actions=[
            Node(
                package="slam_node",
                executable="slam_node",
                name="slam_node",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                }],
            )
        ],
    )

    # -------------------------------------------------------------------------
    # Log messages for operator visibility during launch
    # -------------------------------------------------------------------------
    log_start = LogInfo(msg="[NIDAR] Starting SLAM stack: RPLIDAR → slam_toolbox → slam_node")
    log_hardware = LogInfo(
        msg=["[NIDAR] RPLIDAR port: ", LaunchConfiguration("serial_port"),
             " @ ", LaunchConfiguration("serial_baudrate"), " baud"]
    )

    # -------------------------------------------------------------------------
    # Assemble the LaunchDescription
    # -------------------------------------------------------------------------
    # Order matters: static TF and driver must be up before slam_toolbox starts
    # consuming /scan.  slam_node waits via TimerAction.
    return LaunchDescription([
        # Declare args first
        serial_port_arg,
        serial_baudrate_arg,
        frame_id_arg,
        # Log for operator
        log_start,
        log_hardware,
        # Nodes in dependency order
        static_tf_base_to_laser,   # TF first — slam_toolbox needs it at startup
        rplidar_driver,             # Driver second — provides /scan
        slam_toolbox_node,          # SLAM third — consumes /scan
        slam_node,                  # slam_node last (delayed 2s via TimerAction)
    ])
