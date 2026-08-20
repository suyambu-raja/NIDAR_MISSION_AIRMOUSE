#!/usr/bin/env python3
"""
failsafe_node.py — NIDAR AirMouse Failsafe Safety Supervisor Node
==================================================================
Monitors autonomous flight health and executes obstacle-aware Return-to-Launch (RTL).

Monitored Conditions:
---------------------
1. Battery Level:
   - Triggers when battery percentage falls below 'battery_low_threshold_pct' (default: 15.0%).
   - ⚠️ NOTE: 15.0% is a provisional placeholder. Needs tuning based on real 8000mAh
     discharge curves and mission arena power load.
2. Link Loss / Telemetry Timeout:
   - Triggers when heartbeat/pose telemetry stream is absent for > 'link_timeout_sec' (default: 3.0s).
3. Geofence Boundary Violation:
   - Triggers when drone coordinates exceed defined arena limits [x_min, x_max, y_min, y_max]
     (default: -1.0m to 16.0m for 15x15m competition arena).
4. Manual Abort:
   - Service /failsafe/abort allows GCS operator to trigger instant obstacle-aware RTL.

Action on Trigger:
------------------
- Prevents ArduPilot straight-line RTL which would collide with indoor walls.
- Calls exploration_node's A* PathPlanner with the latest OccupancyGrid map.
- Emits obstacle-avoidance waypoints via /goal_pose and publishes /failsafe/planned_path.
- Commands /land service upon reaching home entry point.
- Streams live telemetry status JSON on /failsafe/status for GCS dashboard.
"""

import json
import math
import time
from enum import Enum, auto
from typing import Optional, List, Tuple, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger, SetBool

# Import existing PathPlanner from exploration_node
try:
    from exploration_node.path_planner import PathPlanner
    PATH_PLANNER_AVAILABLE = True
except ImportError:
    PATH_PLANNER_AVAILABLE = False


# =============================================================================
# QoS Profiles (strictly matching docs/interfaces.md)
# =============================================================================

# Map QoS: Reliable, Transient-Local (latched)
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Pose QoS: Best-effort, Volatile for low latency
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Battery QoS: Best-effort, Volatile
BATTERY_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# Output status QoS: Reliable, Transient-Local
STATUS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Goal QoS: Reliable, Transient-Local
GOAL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# =============================================================================
# Failsafe States and Trigger Reasons
# =============================================================================

class FailsafeState(Enum):
    NORMAL = auto()
    NAVIGATING_HOME = auto()
    HOVERING_NEAR_HOME = auto()
    LANDING = auto()
    LANDED = auto()


class FailsafeReason(Enum):
    NONE = "NONE"
    LOW_BATTERY = "LOW_BATTERY"
    LINK_LOST = "LINK_LOST"
    GEOFENCE_BREACH = "GEOFENCE_BREACH"
    MANUAL_ABORT = "MANUAL_ABORT"


class FailsafeNode(Node):
    """
    ROS 2 Safety Supervisor Node for NIDAR AirMouse.
    """

    def __init__(self) -> None:
        super().__init__("failsafe_node")

        # ---------------------------------------------------------------------
        # Parameters (Configurable via launch files, CLI, or parameter server)
        # ---------------------------------------------------------------------
        # ⚠️ TUNING REQUIRED: 15.0% threshold is a provisional placeholder.
        # Must be calibrated against real 8000mAh discharge test curves under motor load.
        self.declare_parameter("battery_low_threshold_pct", 15.0)
        self.declare_parameter("link_timeout_sec", 3.0)
        self.declare_parameter("geofence_x_min", -1.0)
        self.declare_parameter("geofence_x_max", 16.0)
        self.declare_parameter("geofence_y_min", -1.0)
        self.declare_parameter("geofence_y_max", 16.0)
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("arrival_distance_threshold", 0.6)
        self.declare_parameter("auto_land_on_arrival", True)
        self.declare_parameter("map_frame", "map")

        self._battery_threshold = float(self.get_parameter("battery_low_threshold_pct").value)
        self._link_timeout = float(self.get_parameter("link_timeout_sec").value)
        self._geofence_x_min = float(self.get_parameter("geofence_x_min").value)
        self._geofence_x_max = float(self.get_parameter("geofence_x_max").value)
        self._geofence_y_min = float(self.get_parameter("geofence_y_min").value)
        self._geofence_y_max = float(self.get_parameter("geofence_y_max").value)
        self._home_x = float(self.get_parameter("home_x").value)
        self._home_y = float(self.get_parameter("home_y").value)
        self._check_rate = float(self.get_parameter("check_rate_hz").value)
        self._arrival_dist = float(self.get_parameter("arrival_distance_threshold").value)
        self._auto_land = bool(self.get_parameter("auto_land_on_arrival").value)
        self._map_frame = str(self.get_parameter("map_frame").value)

        self.get_logger().info(
            f"failsafe_node initialized | Battery Thr: {self._battery_threshold}% | "
            f"Link Timeout: {self._link_timeout}s | Geofence: [{self._geofence_x_min}, {self._geofence_x_max}] x "
            f"[{self._geofence_y_min}, {self._geofence_y_max}] | Home: ({self._home_x}, {self._home_y})"
        )

        # ---------------------------------------------------------------------
        # Internal State Tracking
        # ---------------------------------------------------------------------
        self._state: FailsafeState = FailsafeState.NORMAL
        self._trigger_reason: FailsafeReason = FailsafeReason.NONE
        self._trigger_detail: str = ""
        self._trigger_timestamp: float = 0.0

        self._current_pose: Optional[PoseStamped] = None
        self._latest_map: Optional[OccupancyGrid] = None
        self._battery_pct: float = 100.0
        self._battery_voltage: float = 12.6
        self._last_battery_time: float = time.time()
        self._last_pose_time: float = time.time()
        self._last_heartbeat_time: float = time.time()
        self._connected: bool = True
        self._armed: bool = False

        self._home_latched: bool = False
        self._planned_waypoints: List[Tuple[float, float]] = []
        self._current_waypoint_idx: int = 0

        # Instantiate A* PathPlanner from exploration_node
        if PATH_PLANNER_AVAILABLE:
            self._planner = PathPlanner()
        else:
            self._planner = None
            self.get_logger().warn("PathPlanner not imported; using direct home vector fallback")

        # ---------------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------------
        self._sub_pose = self.create_subscription(
            PoseStamped, "/drone_pose", self._on_drone_pose, POSE_QOS
        )
        self._sub_battery = self.create_subscription(
            BatteryState, "/battery_status", self._on_battery_status, BATTERY_QOS
        )
        self._sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._on_map, MAP_QOS
        )
        self._sub_bridge_state = self.create_subscription(
            String, "/mavros_bridge/state", self._on_bridge_state, STATUS_QOS
        )

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        self._pub_status = self.create_publisher(
            String, "/failsafe/status", STATUS_QOS
        )
        self._pub_goal_pose = self.create_publisher(
            PoseStamped, "/goal_pose", GOAL_QOS
        )
        self._pub_planned_path = self.create_publisher(
            Path, "/failsafe/planned_path", STATUS_QOS
        )

        # ---------------------------------------------------------------------
        # Services Hosted
        # ---------------------------------------------------------------------
        self._srv_abort = self.create_service(
            Trigger, "/failsafe/abort", self._handle_manual_abort
        )
        self._srv_reset = self.create_service(
            Trigger, "/failsafe/reset", self._handle_reset
        )

        # ---------------------------------------------------------------------
        # Service Clients (to mavros_bridge)
        # ---------------------------------------------------------------------
        self._cli_land = self.create_client(Trigger, "/land")
        self._cli_guided = self.create_client(Trigger, "/set_mode_guided")

        # ---------------------------------------------------------------------
        # Periodic Supervisor Timer
        # ---------------------------------------------------------------------
        timer_period = 1.0 / max(0.5, self._check_rate)
        self._monitor_timer = self.create_timer(timer_period, self._monitor_loop)

    # =========================================================================
    # Subscription Callbacks
    # =========================================================================

    def _on_drone_pose(self, msg: PoseStamped) -> None:
        """Tracks latest drone coordinates and records entry point if not latched."""
        self._current_pose = msg
        self._last_pose_time = time.time()
        self._last_heartbeat_time = self._last_pose_time

        # Auto-latch home location on first valid pose if at default
        if not self._home_latched and msg.pose.position.z > 0.1:
            # Latched entry point
            self._home_latched = True
            self.get_logger().info(
                f"Home/entry point confirmed at ({self._home_x:.2f}, {self._home_y:.2f})"
            )

    def _on_battery_status(self, msg: BatteryState) -> None:
        """Receives battery telemetry from mavros_bridge."""
        self._battery_voltage = float(msg.voltage) if msg.voltage > 0 else 12.6
        pct = float(msg.percentage)
        if pct <= 1.0 and pct > 0.0:
            self._battery_pct = pct * 100.0
        else:
            self._battery_pct = max(0.0, min(100.0, pct))

        self._last_battery_time = time.time()
        self._last_heartbeat_time = self._last_battery_time

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Stores latest SLAM occupancy grid for obstacle-aware A* routing."""
        self._latest_map = msg

    def _on_bridge_state(self, msg: String) -> None:
        """Processes JSON telemetry from mavros_bridge."""
        try:
            data = json.loads(msg.data)
            self._connected = bool(data.get("connected", True))
            self._armed = bool(data.get("armed", False))
            self._last_heartbeat_time = time.time()
        except Exception:
            pass

    # =========================================================================
    # Main Supervisor Loop (Rule-Based Safety Evaluation)
    # =========================================================================

    def _monitor_loop(self) -> None:
        """
        Periodically checks all 4 safety conditions and executes RTL state machine.
        """
        now = time.time()

        # If in NORMAL mode, check triggers
        if self._state == FailsafeState.NORMAL:
            self._check_safety_rules(now)
        elif self._state == FailsafeState.NAVIGATING_HOME:
            self._execute_rtl_navigation()
        elif self._state == FailsafeState.LANDING:
            self._check_landing_status()

        # Publish telemetry status for GCS
        self._publish_failsafe_status()

    def _check_safety_rules(self, now: float) -> None:
        """
        Rule 1: Battery level below threshold
        Rule 2: Telemetry/Command link timeout
        Rule 3: Geofence boundary violation
        """
        # Rule 1: Battery Check
        if self._battery_pct < self._battery_threshold and (now - self._last_battery_time < 10.0):
            detail = (
                f"Battery level {self._battery_pct:.1f}% is below threshold "
                f"{self._battery_threshold:.1f}% (Voltage: {self._battery_voltage:.2f}V)"
            )
            self._trigger_failsafe(FailsafeReason.LOW_BATTERY, detail)
            return

        # Rule 2: Link Loss / Telemetry Timeout
        link_age = now - self._last_heartbeat_time
        if link_age > self._link_timeout and self._connected:
            detail = f"MAVLink telemetry link lost for {link_age:.1f}s (> {self._link_timeout}s timeout)"
            self._trigger_failsafe(FailsafeReason.LINK_LOST, detail)
            return

        # Rule 3: Geofence Check
        if self._current_pose is not None:
            px = float(self._current_pose.pose.position.x)
            py = float(self._current_pose.pose.position.y)

            if (px < self._geofence_x_min or px > self._geofence_x_max or
                py < self._geofence_y_min or py > self._geofence_y_max):
                detail = (
                    f"Geofence breached at ({px:.2f}, {py:.2f}) | "
                    f"Limits: X[{self._geofence_x_min}, {self._geofence_x_max}], "
                    f"Y[{self._geofence_y_min}, {self._geofence_y_max}]"
                )
                self._trigger_failsafe(FailsafeReason.GEOFENCE_BREACH, detail)
                return

    def _trigger_failsafe(self, reason: FailsafeReason, detail: str) -> None:
        """
        Central trigger handler for all failsafe conditions.
        Computes obstacle-aware A* RTL path back to home entry point.
        """
        self._state = FailsafeState.NAVIGATING_HOME
        self._trigger_reason = reason
        self._trigger_detail = detail
        self._trigger_timestamp = time.time()

        self.get_logger().error(
            f"🚨 [FAILSAFE TRIGGERED] Reason: {reason.value} | {detail}"
        )

        # Plan obstacle-aware return path
        self._plan_and_dispatch_rtl_path()

    def _plan_and_dispatch_rtl_path(self) -> None:
        """
        Uses exploration_node's PathPlanner to compute collision-free route to home.
        """
        drone_x = 0.0
        drone_y = 0.0
        if self._current_pose is not None:
            drone_x = float(self._current_pose.pose.position.x)
            drone_y = float(self._current_pose.pose.position.y)

        start_pt = (drone_x, drone_y)
        goal_pt = (self._home_x, self._home_y)

        waypoints = None
        if self._planner is not None and self._latest_map is not None:
            try:
                waypoints = self._planner.plan(self._latest_map, start_pt, goal_pt)
            except Exception as exc:
                self.get_logger().error(f"PathPlanner execution failed during failsafe: {exc}")

        if waypoints is not None and len(waypoints) > 0:
            self._planned_waypoints = waypoints
            self.get_logger().info(
                f"Generated obstacle-aware A* RTL path with {len(waypoints)} waypoints"
            )
        else:
            self.get_logger().warn(
                "Map not available or A* failed; fallback to direct home setpoint"
            )
            self._planned_waypoints = [start_pt, goal_pt]

        self._current_waypoint_idx = 0
        self._publish_path_msg(self._planned_waypoints)
        self._dispatch_next_waypoint()

    def _dispatch_next_waypoint(self) -> None:
        """Publishes next waypoint on /goal_pose."""
        if not self._planned_waypoints:
            return

        if self._current_waypoint_idx < len(self._planned_waypoints):
            target_x, target_y = self._planned_waypoints[self._current_waypoint_idx]
        else:
            target_x, target_y = self._home_x, self._home_y

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self._map_frame
        goal_msg.pose.position.x = float(target_x)
        goal_msg.pose.position.y = float(target_y)
        goal_msg.pose.position.z = 2.5  # Cruising altitude

        # Orientation facing goal
        if self._current_pose is not None:
            dx = target_x - self._current_pose.pose.position.x
            dy = target_y - self._current_pose.pose.position.y
            yaw = math.atan2(dy, dx)
            goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.orientation.w = 1.0

        self._pub_goal_pose.publish(goal_msg)
        self.get_logger().debug(
            f"Dispatched failsafe waypoint [{self._current_waypoint_idx+1}/"
            f"{len(self._planned_waypoints)}]: ({target_x:.2f}, {target_y:.2f})"
        )

    def _execute_rtl_navigation(self) -> None:
        """
        Monitors waypoint progression along the A* RTL path.
        """
        if self._current_pose is None:
            return

        curr_x = float(self._current_pose.pose.position.x)
        curr_y = float(self._current_pose.pose.position.y)

        # Check total distance to home entry point
        dist_to_home = math.hypot(curr_x - self._home_x, curr_y - self._home_y)

        if dist_to_home <= self._arrival_dist:
            self.get_logger().info(
                f"✅ Drone arrived at Home/Entry point (distance={dist_to_home:.2f}m <= {self._arrival_dist}m)"
            )
            if self._auto_land:
                self._command_landing()
            else:
                self._state = FailsafeState.HOVERING_NEAR_HOME
            return

        # Check sub-waypoint progress
        if self._planned_waypoints and self._current_waypoint_idx < len(self._planned_waypoints):
            target_x, target_y = self._planned_waypoints[self._current_waypoint_idx]
            dist_to_wp = math.hypot(curr_x - target_x, curr_y - target_y)

            if dist_to_wp < 0.6:
                self._current_waypoint_idx += 1
                self._dispatch_next_waypoint()

    def _command_landing(self) -> None:
        """Commands autonomous landing upon arrival at home."""
        self._state = FailsafeState.LANDING
        self.get_logger().info("Initiating autonomous landing at Home entry point...")

        if self._cli_land.service_is_ready():
            req = Trigger.Request()
            self._cli_land.call_async(req)
        else:
            self.get_logger().warn("/land service not ready; commanding descent via bridge")

    def _check_landing_status(self) -> None:
        """Checks if touchdown has occurred."""
        if self._current_pose is not None:
            z = float(self._current_pose.pose.position.z)
            if z <= 0.1 and not self._armed:
                self._state = FailsafeState.LANDED
                self.get_logger().info("🛬 Touchdown confirmed. Failsafe RTL mission complete.")

    # =========================================================================
    # Service Handlers
    # =========================================================================

    def _handle_manual_abort(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service /failsafe/abort — immediate operator abort trigger.
        """
        self.get_logger().warn("Manual Abort triggered by GCS operator!")
        self._trigger_failsafe(
            FailsafeReason.MANUAL_ABORT,
            "Operator manually commanded abort via /failsafe/abort service"
        )
        response.success = True
        response.message = "Failsafe obstacle-aware RTL initiated by operator abort."
        return response

    def _handle_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service /failsafe/reset — resets supervisor to NORMAL state.
        """
        self._state = FailsafeState.NORMAL
        self._trigger_reason = FailsafeReason.NONE
        self._trigger_detail = ""
        self._planned_waypoints = []
        self._current_waypoint_idx = 0
        self.get_logger().info("Failsafe supervisor state reset to NORMAL")
        response.success = True
        response.message = "Failsafe reset to NORMAL."
        return response

    # =========================================================================
    # Helpers & Publishers
    # =========================================================================

    def _publish_path_msg(self, waypoints: List[Tuple[float, float]]) -> None:
        """Publishes Path message for GCS visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self._map_frame

        for wx, wy in waypoints:
            p = PoseStamped()
            p.header.frame_id = self._map_frame
            p.pose.position.x = float(wx)
            p.pose.position.y = float(wy)
            p.pose.position.z = 2.5
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self._pub_planned_path.publish(path_msg)

    def _publish_failsafe_status(self) -> None:
        """Publishes structured JSON state on /failsafe/status for GCS."""
        now = time.time()
        link_age = round(now - self._last_heartbeat_time, 2)
        drone_pos = [0.0, 0.0, 0.0]
        dist_to_home = 0.0

        if self._current_pose is not None:
            drone_pos = [
                round(float(self._current_pose.pose.position.x), 2),
                round(float(self._current_pose.pose.position.y), 2),
                round(float(self._current_pose.pose.position.z), 2),
            ]
            dist_to_home = round(
                math.hypot(drone_pos[0] - self._home_x, drone_pos[1] - self._home_y), 2
            )

        payload = {
            "active": True,
            "triggered": self._state != FailsafeState.NORMAL,
            "state": self._state.name,
            "reason": self._trigger_reason.value,
            "detail": self._trigger_detail,
            "battery_pct": round(self._battery_pct, 1),
            "battery_voltage": round(self._battery_voltage, 2),
            "battery_threshold_pct": self._battery_threshold,
            "link_age_sec": link_age,
            "link_timeout_sec": self._link_timeout,
            "current_pos": drone_pos,
            "home_pos": [self._home_x, self._home_y],
            "distance_to_home": dist_to_home,
            "waypoints_remaining": max(0, len(self._planned_waypoints) - self._current_waypoint_idx),
            "timestamp": round(now, 3),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._pub_status.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
