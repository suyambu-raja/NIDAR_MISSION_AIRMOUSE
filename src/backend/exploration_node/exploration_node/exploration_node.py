#!/usr/bin/env python3
"""
exploration_node.py — NIDAR AirMouse: Autonomous Exploration ROS 2 Node
========================================================================
Main ROS 2 node for the exploration_node package.

What this node does
-------------------
1. Subscribes to /map, /drone_pose, /survivor_grid_locations, /battery_status
2. Runs the full exploration pipeline at 1 Hz:
      FrontierDetector   → find all frontier cells
      FrontierGrouper    → BFS-cluster cells into meaningful groups
      BatteryMonitor     → check if strategy must switch (EXPLORE/RETURN/EXIT)
      FrontierSelector   → pick best frontier by information-gain utility
      PathPlanner        → A* route from drone pose to frontier center
3. Publishes /goal_pose      (geometry_msgs/PoseStamped)   — where to fly next
             /exploration_status (std_msgs/String, JSON)   — progress metrics
             /planned_path   (nav_msgs/Path)               — A* route for GCS
4. Stops automatically when:
     - All 6 survivors have been confirmed (competition complete)
     - Battery reaches CRITICAL (< 15%) — navigate to exit immediately

Topic names match docs/interfaces.md exactly.

Running with mock hardware
--------------------------
    # Terminal 1:
    python3 tools/mock_publishers/mock_slam.py
    # Terminal 2:
    ros2 run exploration_node exploration_node
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)

# Standard ROS 2 message types
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

# Custom NIDAR message
from nidar_msgs.msg import SurvivorGridArray

# Local utility classes
from exploration_node.frontier_detector import FrontierDetector
from exploration_node.frontier_grouper  import FrontierGrouper
from exploration_node.frontier_selector import FrontierSelector
from exploration_node.path_planner      import PathPlanner
from exploration_node.battery_monitor   import BatteryMonitor, Strategy


# =============================================================================
# QoS Profiles — must match publishers per docs/interfaces.md
# =============================================================================

# /map is TRANSIENT_LOCAL — get last map even if we start late
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
)

# /drone_pose is BEST_EFFORT + VOLATILE — low latency
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10,
)

# /survivor_grid_locations is TRANSIENT_LOCAL — receive full survivor list
SURVIVOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
)

# /battery_status is BEST_EFFORT (hardware sensor)
BATTERY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5,
)

# Output topics: RELIABLE so GCS dashboard never misses an update
OUTPUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
)


# =============================================================================
# Constants
# =============================================================================

PUBLISH_RATE_HZ:  float = 1.0    # Exploration pipeline rate
MAP_FRAME:        str   = "map"  # All poses expressed in this frame
MAX_SURVIVORS:    int   = 6      # Competition rule — stop when all found

# If drone is within this radius of current goal, mark as reached
GOAL_REACHED_RADIUS_M: float = 0.8


class ExplorationNode(Node):
    """
    ROS 2 node: autonomously explores the arena to find all 6 survivors.

    Lifecycle
    ---------
    __init__           → create pubs/subs/timer, instantiate utilities
    _map_cb            → store latest /map
    _pose_cb           → store latest /drone_pose
    _survivors_cb      → update survivor count from /survivor_grid_locations
    _battery_cb        → update battery monitor
    _explore_timer_cb  → run full pipeline at 1 Hz
    """

    def __init__(self) -> None:
        super().__init__("exploration_node")

        # ------------------------------------------------------------------ #
        # Utility class instances                                             #
        # ------------------------------------------------------------------ #
        self._detector = FrontierDetector()
        self._grouper  = FrontierGrouper()
        self._selector = FrontierSelector()
        self._planner  = PathPlanner()
        self._battery  = BatteryMonitor()

        # ------------------------------------------------------------------ #
        # State                                                               #
        # ------------------------------------------------------------------ #
        self._latest_map:   OccupancyGrid | None = None
        self._latest_pose:  PoseStamped   | None = None
        self._survivor_count: int = 0

        # Track current goal to detect arrival
        self._current_goal: tuple | None = None

        # Pipeline execution statistics (included in /exploration_status)
        self._total_ticks:       int = 0
        self._frontiers_found:   int = 0
        self._goals_published:   int = 0

        # True once the mission is complete (stops publishing /goal_pose)
        self._mission_done: bool = False

        # ------------------------------------------------------------------ #
        # Subscribers                                                         #
        # ------------------------------------------------------------------ #
        self.create_subscription(OccupancyGrid, "/map",
                                 self._map_cb, MAP_QOS)

        self.create_subscription(PoseStamped, "/drone_pose",
                                 self._pose_cb, POSE_QOS)

        self.create_subscription(SurvivorGridArray, "/survivor_grid_locations",
                                 self._survivors_cb, SURVIVOR_QOS)

        self.create_subscription(BatteryState, "/battery_status",
                                 self._battery_cb, BATTERY_QOS)

        # ------------------------------------------------------------------ #
        # Publishers                                                          #
        # ------------------------------------------------------------------ #
        self._goal_pub   = self.create_publisher(PoseStamped, "/goal_pose",           OUTPUT_QOS)
        self._status_pub = self.create_publisher(String,      "/exploration_status",  OUTPUT_QOS)
        self._path_pub   = self.create_publisher(Path,        "/planned_path",        OUTPUT_QOS)

        # ------------------------------------------------------------------ #
        # Timer — runs the exploration pipeline at 1 Hz                      #
        # ------------------------------------------------------------------ #
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._explore_timer_cb)

        self.get_logger().info(
            "exploration_node started — waiting for /map and /drone_pose…"
        )

    # =========================================================================
    # Subscriber callbacks
    # =========================================================================

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """Store the latest OccupancyGrid from /map."""
        self._latest_map = msg
        self.get_logger().debug(
            f"Map received: {msg.info.width}x{msg.info.height} cells, "
            f"origin=({msg.info.origin.position.x:.2f}, "
            f"{msg.info.origin.position.y:.2f})"
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        """Store the latest drone pose from /drone_pose."""
        self._latest_pose = msg

    def _survivors_cb(self, msg: SurvivorGridArray) -> None:
        """Update survivor count from /survivor_grid_locations."""
        prev = self._survivor_count
        self._survivor_count = len(msg.locations)

        if self._survivor_count > prev:
            self.get_logger().info(
                f"Survivor count updated: {self._survivor_count}/{MAX_SURVIVORS}"
            )

        # Stop exploration once all survivors are found
        if self._survivor_count >= MAX_SURVIVORS and not self._mission_done:
            self._mission_done = True
            self.get_logger().info(
                "All 6 survivors found! Exploration complete — stopping."
            )

    def _battery_cb(self, msg: BatteryState) -> None:
        """Update battery monitor. BatteryState.percentage is 0.0–1.0."""
        pct = msg.percentage * 100.0  # convert to 0–100 scale
        strategy = self._battery.update(pct)

        if strategy == Strategy.EXIT and not self._mission_done:
            self.get_logger().warn(
                f"CRITICAL battery ({pct:.1f}%) — aborting exploration, "
                "navigating to exit point (0.0, 0.0)!"
            )
            self._mission_done = True  # stop normal exploration

    # =========================================================================
    # Exploration timer callback — the main pipeline
    # =========================================================================

    def _explore_timer_cb(self) -> None:
        """
        Fires at 1 Hz. Runs the full frontier-exploration pipeline and
        publishes /goal_pose, /exploration_status, /planned_path.

        Error handling strategy:
        - If no map yet: log debug and return (publish empty status only)
        - If no drone pose yet: same
        - If no frontiers found: log info, publish status, hold current goal
        - If path planning fails: try next-best frontier (up to 3 attempts)
        - Never crash: all exceptions are caught and logged
        """
        self._total_ticks += 1

        # Always publish status so the GCS dashboard has a heartbeat
        # (even if we can't plan a path yet)

        # ------------------------------------------------------------------
        # Guard: wait for first map and pose
        # ------------------------------------------------------------------
        if self._latest_map is None:
            self._publish_status("waiting_for_map", 0, 0)
            return

        if self._latest_pose is None:
            self._publish_status("waiting_for_pose", 0, 0)
            return

        # ------------------------------------------------------------------
        # Battery strategy overrides everything
        # ------------------------------------------------------------------
        strategy = self._battery.current_strategy

        if strategy == Strategy.EXIT or (
            self._mission_done and self._survivor_count < MAX_SURVIVORS
        ):
            # Emergency: fly directly to exit
            self._publish_goal(self._battery.exit_point[0],
                               self._battery.exit_point[1],
                               label="EXIT")
            self._publish_status("exiting_battery_critical",
                                 self._survivor_count, 0)
            return

        # ------------------------------------------------------------------
        # Mission complete: all survivors found
        # ------------------------------------------------------------------
        if self._mission_done:
            self._publish_status("mission_complete", self._survivor_count, 0)
            return

        # ------------------------------------------------------------------
        # Get drone world position
        # ------------------------------------------------------------------
        drone_x = self._latest_pose.pose.position.x
        drone_y = self._latest_pose.pose.position.y

        # ------------------------------------------------------------------
        # Step 1: Detect frontier cells
        # ------------------------------------------------------------------
        try:
            frontier_cells = self._detector.detect(self._latest_map)
        except Exception as exc:
            self.get_logger().error(f"FrontierDetector failed: {exc}")
            self._publish_status("detector_error", self._survivor_count, 0)
            return

        if not frontier_cells:
            self.get_logger().info(
                "No frontier cells found — arena may be fully explored."
            )
            self._publish_status("fully_explored", self._survivor_count, 0)
            return

        # ------------------------------------------------------------------
        # Step 2: Group frontier cells via BFS
        # ------------------------------------------------------------------
        try:
            groups = self._grouper.group(frontier_cells, self._latest_map)
        except Exception as exc:
            self.get_logger().error(f"FrontierGrouper failed: {exc}")
            self._publish_status("grouper_error", self._survivor_count, 0)
            return

        if not groups:
            self.get_logger().info(
                "All frontier groups below minimum size — nothing to explore."
            )
            self._publish_status("no_valid_groups", self._survivor_count,
                                 len(frontier_cells))
            return

        self._frontiers_found += len(groups)
        self.get_logger().debug(
            f"Found {len(frontier_cells)} frontier cells → {len(groups)} groups"
        )

        # ------------------------------------------------------------------
        # Step 3: Apply battery RETURN strategy — bias toward exit
        # If battery is LOW, temporarily inject exit point as a pseudo-frontier
        # so the selector naturally biases toward home.
        # ------------------------------------------------------------------
        if strategy == Strategy.RETURN:
            self.get_logger().warn(
                f"Battery LOW ({self._battery.last_percentage:.1f}%) — "
                "biasing exploration toward exit point."
            )
            # Choose the group closest to the exit point instead of the
            # highest-utility one — override the selector
            ex, ey = self._battery.exit_point
            selected = min(
                groups,
                key=lambda g: math.hypot(
                    g.center_world[0] - ex,
                    g.center_world[1] - ey
                )
            )
        else:
            # ------------------------------------------------------------------
            # Step 4: Select best frontier by information-gain utility
            # ------------------------------------------------------------------
            selected = self._selector.select(groups, drone_x, drone_y)

        if selected is None:
            # Should not happen — groups is non-empty — but guard anyway
            self._publish_status("selector_returned_none",
                                 self._survivor_count, len(groups))
            return

        goal_x, goal_y = selected.center_world
        score = self._selector.score(selected, drone_x, drone_y)

        self.get_logger().info(
            f"Selected frontier at ({goal_x:.2f}, {goal_y:.2f}) — "
            f"size={selected.size} cells, score={score:.2f}, "
            f"dist={math.hypot(goal_x - drone_x, goal_y - drone_y):.2f}m"
        )

        # ------------------------------------------------------------------
        # Step 5: A* path planning — try up to 3 frontier groups if blocked
        # ------------------------------------------------------------------
        waypoints = None
        tried = 0
        for candidate in groups[:3]:  # try top-3 groups if first is blocked
            if tried == 0:
                cx, cy = goal_x, goal_y
            else:
                cx, cy = candidate.center_world

            try:
                waypoints = self._planner.plan(
                    self._latest_map,
                    (drone_x, drone_y),
                    (cx, cy),
                )
            except Exception as exc:
                self.get_logger().error(f"PathPlanner failed: {exc}")

            if waypoints is not None:
                goal_x, goal_y = cx, cy
                break
            tried += 1

        if waypoints is None:
            self.get_logger().warn(
                "PathPlanner could not find a safe path to any of the "
                "top-3 frontiers — holding current goal."
            )
            self._publish_status("no_path_found", self._survivor_count,
                                 len(groups))
            return

        # ------------------------------------------------------------------
        # Step 6: Publish outputs
        # ------------------------------------------------------------------
        self._publish_goal(goal_x, goal_y, label="EXPLORE")
        self._publish_path(waypoints)
        self._goals_published += 1

        coverage_pct = self._estimate_coverage(self._latest_map)
        self._publish_status(
            "exploring",
            self._survivor_count,
            len(groups),
            coverage_pct=coverage_pct,
            goal_x=goal_x,
            goal_y=goal_y,
            strategy=strategy.name,
            battery_pct=self._battery.last_percentage,
        )

    # =========================================================================
    # Publisher helpers
    # =========================================================================

    def _publish_goal(self, x: float, y: float, label: str = "") -> None:
        """
        Publish a PoseStamped on /goal_pose.

        The drone's yaw is set to face the goal direction from the current pose.
        """
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # Compute yaw so drone faces the goal
        if self._latest_pose is not None:
            dx = x - self._latest_pose.pose.position.x
            dy = y - self._latest_pose.pose.position.y
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        # Convert yaw → quaternion (roll=0, pitch=0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_pub.publish(msg)
        self._current_goal = (x, y)
        self.get_logger().debug(
            f"[{label}] Published /goal_pose: ({x:.2f}, {y:.2f})"
        )

    def _publish_path(self, waypoints) -> None:
        """
        Publish a nav_msgs/Path on /planned_path for GCS visualisation.

        Each waypoint becomes a PoseStamped in the path.
        """
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME

        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.header.stamp    = msg.header.stamp
            pose.header.frame_id = MAP_FRAME
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self._path_pub.publish(msg)

    def _publish_status(
        self,
        state:       str,
        survivors:   int,
        frontiers:   int,
        coverage_pct: float = 0.0,
        goal_x:      float = 0.0,
        goal_y:      float = 0.0,
        strategy:    str   = "EXPLORE",
        battery_pct: float = 100.0,
    ) -> None:
        """
        Publish a JSON status string on /exploration_status.

        JSON format (std_msgs/String payload):
        {
          "state":          "exploring",
          "survivors_found": 3,
          "frontiers":      12,
          "coverage_pct":   47.3,
          "goal":           [5.2, 3.1],
          "strategy":       "EXPLORE",
          "battery_pct":    68.4,
          "ticks":          42
        }
        """
        payload = json.dumps({
            "state":           state,
            "survivors_found": survivors,
            "frontiers":       frontiers,
            "coverage_pct":    round(coverage_pct, 1),
            "goal":            [round(goal_x, 2), round(goal_y, 2)],
            "strategy":        strategy,
            "battery_pct":     round(battery_pct, 1),
            "ticks":           self._total_ticks,
        })
        msg = String()
        msg.data = payload
        self._status_pub.publish(msg)

    # =========================================================================
    # Utility helpers
    # =========================================================================

    @staticmethod
    def _estimate_coverage(map_msg) -> float:
        """
        Estimate what percentage of the arena has been explored (not UNKNOWN).

        Returns a float 0–100 representing the percentage of cells that
        are either FREE or OCCUPIED (i.e., have been mapped by SLAM).
        """
        data = map_msg.data
        if not data:
            return 0.0
        total   = len(data)
        unknown = sum(1 for v in data if v == -1)
        return 100.0 * (total - unknown) / total


# =============================================================================
# Entry point
# =============================================================================

def main(args=None) -> None:
    """
    ROS 2 entry point.

    Run with:
        ros2 run exploration_node exploration_node
    """
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("exploration_node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
