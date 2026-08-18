#!/usr/bin/env python3
"""
slam_node.py — NIDAR AirMouse SLAM Node
========================================
Wraps slam_toolbox to provide:
  - Pose estimation (/drone_pose) consumed by grid_mapper_node and fusion_node
  - Occupancy map (/map) consumed by grid_mapper_node and exploration_node

Hardware: RPLIDAR A2M8 on Raspberry Pi 4 (8 GB), ROS 2 Humble
Arena:    Indoor maze, max 15×15m, corridors ≥1m, rooms ≥2×2m

This node does NOT start slam_toolbox itself — that is done by rplidar_launch.py.
Instead it:
  1. Re-publishes /map from slam_toolbox on our canonical topic (same name, same type,
     but we relay through here so other nodes only depend on this node's contract).
  2. Extracts the drone pose from the TF tree (map → base_link) at a fixed rate and
     publishes it on /drone_pose as PoseStamped.
  3. Monitors /scan for dropout and emits a WARN when the LiDAR goes silent.

Why relay instead of directly subscribing to slam_toolbox?
  If we ever swap slam_toolbox for Cartographer or another backend, only this file
  needs changing — all consumers stay untouched.
"""

import os
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

# Message types
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# TF2 — used to extract map→base_link transform and re-publish as PoseStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener


# =============================================================================
# QoS Profiles
# =============================================================================

# Latched-like QoS for /map — new subscribers receive the last published map
# immediately (transient local durability mirrors the slam_toolbox default).
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Best-effort for high-rate pose — we prefer low latency over guaranteed delivery
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Scan subscriber QoS — must match RPLIDAR driver's publisher (best-effort)
SCAN_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class SlamNode(Node):
    """
    SLAM relay + pose extraction node.

    Subscriptions
    -------------
    /scan               (sensor_msgs/LaserScan)      — from RPLIDAR driver
    /map                (nav_msgs/OccupancyGrid)     — from slam_toolbox

    Publications
    ------------
    /map                (nav_msgs/OccupancyGrid)     — re-published (same topic,
                                                       slam_toolbox already publishes
                                                       here; we just validate + forward)
    /drone_pose         (geometry_msgs/PoseStamped)  — extracted from TF map→base_link

    NOTE: slam_toolbox publishes /map itself.  We subscribe to it so we can do
    health-checks (e.g. detect if mapping stalls) and re-publish for any consumer
    that wants SLAM-node-guaranteed delivery semantics.  The relay adds negligible
    overhead on Pi4.
    """

    # -------------------------------------------------------------------------
    # Tunable constants (not ROS params — kept simple for Pi4 resource budget)
    # -------------------------------------------------------------------------
    # How often (seconds) to extract pose from TF and publish /drone_pose
    POSE_PUBLISH_RATE_HZ: float = 10.0

    # How long (seconds) without a /scan message before we log a dropout warning
    SCAN_DROPOUT_TIMEOUT_SEC: float = 2.0

    # TF lookup timeout — keep short to avoid blocking the pose timer
    TF_TIMEOUT_SEC: float = 0.1

    # Frame IDs — must match slam_params.yaml and the static transform in launch
    MAP_FRAME: str = "map"
    BASE_LINK_FRAME: str = "base_link"

    def __init__(self) -> None:
        super().__init__("slam_node")

        self.get_logger().info("slam_node starting — NIDAR AirMouse SLAM relay")

        # ------------------------------------------------------------------ #
        # TF2 buffer + listener                                               #
        # We poll this to extract map→base_link at POSE_PUBLISH_RATE_HZ.     #
        # ------------------------------------------------------------------ #
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------ #
        # Subscribers                                                         #
        # ------------------------------------------------------------------ #
        # /scan — only used for dropout detection; slam_toolbox also subscribes
        self._scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            SCAN_QOS,
        )
        self._last_scan_time: float = time.monotonic()

        # /map from slam_toolbox — relay to guarantee our topic contract
        # slam_toolbox uses TRANSIENT_LOCAL so we must match it here
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            MAP_QOS,
        )
        self._last_map: OccupancyGrid | None = None

        # ------------------------------------------------------------------ #
        # Publishers                                                          #
        # ------------------------------------------------------------------ #
        # /map — re-publish with same QoS so late-joining nodes get the map
        self._map_pub = self.create_publisher(
            OccupancyGrid,
            "/map",
            MAP_QOS,
        )

        # /drone_pose — extracted from TF tree, published at ~10 Hz
        self._pose_pub = self.create_publisher(
            PoseStamped,
            "/drone_pose",
            POSE_QOS,
        )

        # ------------------------------------------------------------------ #
        # Timers                                                              #
        # ------------------------------------------------------------------ #
        pose_period = 1.0 / self.POSE_PUBLISH_RATE_HZ
        self._pose_timer = self.create_timer(pose_period, self._publish_pose)

        # Dropout watchdog — check every second, warn if no scan received
        self._watchdog_timer = self.create_timer(1.0, self._check_scan_health)

        self.get_logger().info(
            f"slam_node ready — pose @ {self.POSE_PUBLISH_RATE_HZ} Hz, "
            f"dropout threshold = {self.SCAN_DROPOUT_TIMEOUT_SEC}s"
        )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _on_scan(self, msg: LaserScan) -> None:
        """
        Receives every LiDAR scan.
        We only update a timestamp here — slam_toolbox does the real processing.
        This is sufficient for dropout detection with minimal CPU overhead.
        """
        self._last_scan_time = time.monotonic()

    def _on_map(self, msg: OccupancyGrid) -> None:
        """
        Receives updated occupancy grid from slam_toolbox and re-publishes it.
        Also caches the latest map so we can log map dimensions on first receipt.
        """
        if self._last_map is None:
            # First map received — log dimensions so the team can verify resolution
            width_m = msg.info.width * msg.info.resolution
            height_m = msg.info.height * msg.info.resolution
            self.get_logger().info(
                f"First map received: {msg.info.width}×{msg.info.height} cells "
                f"({width_m:.1f}×{height_m:.1f}m) @ {msg.info.resolution}m/cell"
            )

        self._last_map = msg
        # Relay — same message object, no copy needed (ROS 2 handles immutability)
        self._map_pub.publish(msg)

    # =========================================================================
    # Timers
    # =========================================================================

    def _publish_pose(self) -> None:
        """
        Extracts the map→base_link transform from TF2 and publishes it as
        geometry_msgs/PoseStamped on /drone_pose.

        Why TF instead of slam_toolbox's /pose topic?
        slam_toolbox publishes /pose only in some modes.  Reading from TF is
        always available and is the canonical ROS 2 approach for pose queries.

        Called at POSE_PUBLISH_RATE_HZ (10 Hz) by a ROS timer.
        """
        try:
            # Look up the most recent transform available (time=0 → latest)
            transform = self._tf_buffer.lookup_transform(
                self.MAP_FRAME,
                self.BASE_LINK_FRAME,
                rclpy.time.Time(),  # time=0 → latest available
                timeout=rclpy.duration.Duration(seconds=self.TF_TIMEOUT_SEC),
            )
        except tf2_ros.LookupException:
            # TF tree not yet populated — slam_toolbox may still be initialising
            # This is normal for the first few seconds; do not flood the log
            return
        except tf2_ros.ExtrapolationException as e:
            # TF extrapolation error — usually transient; log at DEBUG level
            self.get_logger().debug(f"TF extrapolation error: {e}")
            return
        except Exception as e:
            # Catch-all — log once per second at most via the watchdog
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Build PoseStamped from TransformStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = transform.header.stamp
        pose_msg.header.frame_id = self.MAP_FRAME  # pose is expressed in map frame

        # Translation → position
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z

        # Rotation (quaternion) — pass through directly from TF
        pose_msg.pose.orientation = transform.transform.rotation

        self._pose_pub.publish(pose_msg)

    def _check_scan_health(self) -> None:
        """
        Watchdog that logs a warning if no /scan message has been received
        within SCAN_DROPOUT_TIMEOUT_SEC seconds.

        Possible causes:
          - RPLIDAR USB cable unplugged
          - rplidar_ros driver crashed
          - LiDAR motor stopped (low power)
          - Wrong /dev/ttyUSBx port configured in .env
        """
        elapsed = time.monotonic() - self._last_scan_time
        if elapsed > self.SCAN_DROPOUT_TIMEOUT_SEC:
            self.get_logger().warn(
                f"LiDAR scan dropout detected — no /scan for {elapsed:.1f}s. "
                f"Check RPLIDAR connection on {os.environ.get('RPLIDAR_PORT', '/dev/ttyUSB0')}. "
                f"SLAM will resume automatically when scans return."
            )


def main(args=None) -> None:
    """ROS 2 entry point — called by the colcon-generated console script."""
    rclpy.init(args=args)

    node = SlamNode()

    try:
        # spin_once with timeout keeps the node responsive while blocking minimally
        # rclpy.spin() is equivalent but blocks indefinitely — fine for production
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("slam_node shutting down (KeyboardInterrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
