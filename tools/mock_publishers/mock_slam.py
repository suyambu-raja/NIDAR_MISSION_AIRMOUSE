#!/usr/bin/env python3
"""
mock_slam.py — NIDAR AirMouse: Mock SLAM Publisher
====================================================
⚠️  MOCK ONLY — replace with real slam_node for production ⚠️

Purpose
-------
Unblocks team members who are developing dependent nodes BEFORE the RPLIDAR
hardware or slam_toolbox environment is available on the Raspberry Pi 4.

Who uses this:
  - Member 1 (grid_mapper_node): needs /map to test grid processing logic
  - Member 2 (fusion_node):      needs /drone_pose to test sensor fusion logic

What this publishes (exact same topic names, types, frames as the real slam_node):
  /map          (nav_msgs/OccupancyGrid)      @ ~2 Hz
  /drone_pose   (geometry_msgs/PoseStamped)   @ ~10 Hz

Drone motion pattern: slow figure-8 loop across a 10m × 10m area so dependent
nodes can exercise pose changes without real hardware.

Usage:
  # In your ROS 2 workspace (no hardware needed):
  source /opt/ros/humble/setup.bash
  python3 tools/mock_publishers/mock_slam.py

  # Verify topics are live:
  ros2 topic echo /drone_pose
  ros2 topic echo /map --no-arr   # --no-arr avoids flooding the terminal

  # Visualise in RViz2 (optional):
  rviz2 &  # Add Map display on /map, Axes on /drone_pose

DO NOT deploy this file on the drone.
DO NOT import this in any production node.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

# Message types — identical to real slam_node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime


# =============================================================================
# QoS Profiles — must match real slam_node so subscribers don't reject msgs
# =============================================================================

# /map uses TRANSIENT_LOCAL so late-joining subscribers get the last map
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# /drone_pose uses BEST_EFFORT (low-latency, matches real slam_node)
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


# =============================================================================
# Arena / map constants — sized to match real mission parameters
# =============================================================================

# Arena dimensions from mission spec: max 15×15m
ARENA_SIZE_M: float = 15.0        # metres
MAP_RESOLUTION: float = 0.05      # 5cm/cell — matches slam_params.yaml
# Number of cells per axis = 15m / 0.05m = 300
MAP_CELLS: int = int(ARENA_SIZE_M / MAP_RESOLUTION)   # 300

# Figure-8 motion parameters for mock drone pose
FIGURE8_AMPLITUDE_X: float = 5.0   # [m] half-width of figure-8
FIGURE8_AMPLITUDE_Y: float = 3.0   # [m] half-height of figure-8
FIGURE8_PERIOD_SEC: float = 30.0   # [s] time to complete one full figure-8 loop

# Frame IDs — must exactly match interfaces.md and slam_node.py
MAP_FRAME: str = "map"
BASE_LINK_FRAME: str = "base_link"


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Converts Euler angles (rad) to a geometry_msgs/Quaternion.
    The drone is flying in 2D (map plane), so only yaw changes.
    Roll and pitch stay 0 for level flight.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _build_empty_map(node: Node) -> OccupancyGrid:
    """
    Creates a nav_msgs/OccupancyGrid representing a fully free (empty) arena.

    Cell values:
       0  = FREE  (open space — corridors and rooms)
      -1  = UNKNOWN (unexplored — a real SLAM map starts mostly unknown)
     100  = OCCUPIED (walls)

    This mock uses all-free (0) for simplicity.  Member 1 (grid_mapper_node)
    can replace this with a handcrafted test layout if needed.
    """
    grid = OccupancyGrid()

    # Header — frame_id and timestamp
    grid.header = Header()
    grid.header.frame_id = MAP_FRAME
    grid.header.stamp = node.get_clock().now().to_msg()

    # Map metadata
    grid.info = MapMetaData()
    grid.info.resolution = MAP_RESOLUTION           # 0.05m per cell
    grid.info.width = MAP_CELLS                     # 300 cells wide
    grid.info.height = MAP_CELLS                    # 300 cells tall
    grid.info.map_load_time = node.get_clock().now().to_msg()

    # Map origin: position of cell (0,0) in the map frame.
    # Centred so the drone starts at the middle of the map.
    grid.info.origin.position.x = -ARENA_SIZE_M / 2.0   # -7.5m
    grid.info.origin.position.y = -ARENA_SIZE_M / 2.0   # -7.5m
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation = _euler_to_quaternion(0.0, 0.0, 0.0)

    # Data: 300×300 = 90,000 cells, all free (0)
    # Use a simple border-wall pattern so grid_mapper_node can detect occupancy
    total_cells = MAP_CELLS * MAP_CELLS
    data = [0] * total_cells   # default: all free

    # Draw perimeter walls (1-cell thick border) so the map looks realistic
    for row in range(MAP_CELLS):
        for col in range(MAP_CELLS):
            if row == 0 or row == MAP_CELLS - 1 or col == 0 or col == MAP_CELLS - 1:
                data[row * MAP_CELLS + col] = 100   # OCCUPIED

    grid.data = data
    return grid


class MockSlamPublisher(Node):
    """
    ⚠️  MOCK ONLY — not used in production.

    Publishes:
      /map        at 2 Hz  (OccupancyGrid, empty 15×15m arena)
      /drone_pose at 10 Hz (PoseStamped, figure-8 motion pattern)
    """

    # Publish rates — match real slam_node's rates from interfaces.md
    MAP_RATE_HZ: float = 2.0
    POSE_RATE_HZ: float = 10.0

    def __init__(self) -> None:
        super().__init__("mock_slam_publisher")

        self.get_logger().warn(
            "⚠️  MOCK SLAM NODE RUNNING — not suitable for real hardware. "
            "Replace with real slam_node (rplidar_launch.py) for production."
        )

        # -----------------------------------------------------------------  #
        # Publishers — same topics, types, QoS as real slam_node             #
        # -----------------------------------------------------------------  #
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", MAP_QOS)
        self._pose_pub = self.create_publisher(PoseStamped, "/drone_pose", POSE_QOS)

        # Pre-build the static map once (it never changes in the mock)
        self._map_msg: OccupancyGrid = _build_empty_map(self)

        # Wall-clock start time — used to compute figure-8 trajectory
        self._start_time: float = time.monotonic()

        # -----------------------------------------------------------------  #
        # Timers                                                              #
        # -----------------------------------------------------------------  #
        self._map_timer = self.create_timer(
            1.0 / self.MAP_RATE_HZ, self._publish_map
        )
        self._pose_timer = self.create_timer(
            1.0 / self.POSE_RATE_HZ, self._publish_pose
        )

        self.get_logger().info(
            f"Mock SLAM publisher ready: /map @ {self.MAP_RATE_HZ} Hz, "
            f"/drone_pose @ {self.POSE_RATE_HZ} Hz"
        )

    # =========================================================================
    # Timer callbacks
    # =========================================================================

    def _publish_map(self) -> None:
        """
        Publishes the static empty OccupancyGrid on /map.
        Updates the header stamp each time so downstream nodes see fresh data.
        """
        self._map_msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(self._map_msg)

    def _publish_pose(self) -> None:
        """
        Publishes a PoseStamped on /drone_pose following a figure-8 pattern.

        Figure-8 parametric equations:
          x(t) =  A_x * sin(ω*t)
          y(t) =  A_y * sin(2*ω*t) / 2
          yaw(t) = atan2(dy/dt, dx/dt)   — drone faces direction of travel

        This gives a smooth, continuous trajectory that exercises both
        straight-line motion (corridors) and turning (room entries).
        """
        elapsed = time.monotonic() - self._start_time
        omega = (2.0 * math.pi) / FIGURE8_PERIOD_SEC   # angular frequency

        # Position on figure-8 lissajous curve
        x = FIGURE8_AMPLITUDE_X * math.sin(omega * elapsed)
        y = FIGURE8_AMPLITUDE_Y * math.sin(2.0 * omega * elapsed) / 2.0

        # Yaw = direction of velocity vector (so the drone faces where it moves)
        dx = FIGURE8_AMPLITUDE_X * omega * math.cos(omega * elapsed)
        dy = FIGURE8_AMPLITUDE_Y * omega * math.cos(2.0 * omega * elapsed)
        yaw = math.atan2(dy, dx)

        # Build PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = MAP_FRAME   # pose expressed in map frame

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0         # 2D SLAM — z stays at 0

        # Convert yaw → quaternion (roll=0, pitch=0 for level flight)
        pose.pose.orientation = _euler_to_quaternion(0.0, 0.0, yaw)

        self._pose_pub.publish(pose)


def main(args=None) -> None:
    """Entry point — run with: python3 tools/mock_publishers/mock_slam.py"""
    rclpy.init(args=args)
    node = MockSlamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mock SLAM publisher shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
