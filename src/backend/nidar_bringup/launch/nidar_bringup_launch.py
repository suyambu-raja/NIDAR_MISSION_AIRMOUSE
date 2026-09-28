#!/usr/bin/env python3
"""
nidar_bringup_launch.py
========================
Master launch script for the NIDAR AirMouse Search-and-Rescue UAV autonomy system.
Brings up all core ROS 2 nodes:
  1. slam_node               - 2D LiDAR SLAM & GPS-denied state estimation
  2. mavros_bridge_node      - ArduPilot MAVLink / MAVROS flight control interface
  3. detection_node          - Dual RGB + Thermal survivor detection
  4. tracker_node            - Multi-target tracking & spatial estimation
  5. fusion_node             - Multi-modal RGB/Thermal/Depth consensus fusion
  6. grid_mapper_node        - Global probability grid mapping
  7. corridor_classifier_node- Spatial architecture & corridor analysis
  8. exploration_node        - Autonomous frontier exploration & DWA planning
  9. failsafe_node           - Real-time safety supervisor & emergency RTL
 10. mission_manager_node    - 13-state rescue mission orchestrator
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # -------------------------------------------------------------------------
    # Launch Arguments
    # -------------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )
    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="false",
        description="Automatically start mission execution without GCS command",
    )
    max_survivors_arg = DeclareLaunchArgument(
        "max_survivors",
        default_value="6",
        description="Target number of survivors to locate",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_start = LaunchConfiguration("auto_start")
    max_survivors = LaunchConfiguration("max_survivors")

    # -------------------------------------------------------------------------
    # Node Definitions
    # -------------------------------------------------------------------------
    slam_node = Node(
        package="slam_node",
        executable="slam_node",
        name="slam_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mavros_bridge_node = Node(
        package="mavros_bridge",
        executable="mavros_bridge_node",
        name="mavros_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    detection_node = Node(
        package="detection_node",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "rgb_conf_threshold": 0.50,
            "thermal_conf_threshold": 0.55,
        }],
    )

    tracker_node = Node(
        package="tracker_node",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    fusion_node = Node(
        package="fusion_node",
        executable="fusion_node",
        name="fusion_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    grid_mapper_node = Node(
        package="grid_mapper_node",
        executable="grid_mapper_node",
        name="grid_mapper_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    corridor_classifier_node = Node(
        package="corridor_classifier",
        executable="corridor_classifier_node",
        name="corridor_classifier_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    exploration_node = Node(
        package="exploration_node",
        executable="exploration_node",
        name="exploration_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_linear_speed": 0.6,
            "max_yaw_rate": 0.8,
        }],
    )

    failsafe_node = Node(
        package="failsafe_node",
        executable="failsafe_node",
        name="failsafe_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mission_manager_node = Node(
        package="mission_manager",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "auto_start": auto_start,
            "max_survivors": max_survivors,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        auto_start_arg,
        max_survivors_arg,
        slam_node,
        mavros_bridge_node,
        detection_node,
        tracker_node,
        fusion_node,
        grid_mapper_node,
        corridor_classifier_node,
        exploration_node,
        failsafe_node,
        mission_manager_node,
    ])
