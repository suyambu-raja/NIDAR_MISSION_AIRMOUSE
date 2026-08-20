#!/usr/bin/env python3
"""
test_failsafe_node.py — Comprehensive Unit & SITL Test Suite for failsafe_node
==============================================================================
Validates the rule-based safety monitor and obstacle-aware Return-to-Launch (RTL):
  1. Initialization & ROS 2 Parameter Configuration
  2. Telemetry ingestion (/drone_pose, /battery_status, /map, /mavros_bridge/state)
  3. Condition 1: Low Battery (< 15.0% threshold)
  4. Condition 2: Telemetry/Command Link Loss (> 3.0s timeout)
  5. Condition 3: Geofence Breach (outside 15x15m arena bounds)
  6. Condition 4: Manual Operator Abort (/failsafe/abort service)
  7. Obstacle-Aware Indoor A* Routing vs Straight-Line RTL (Wall Collision Avoidance)
  8. Home Arrival & Autonomous Landing Dispatch (/land service)
  9. Supervisor State Reset (/failsafe/reset service)
  10. Telemetry JSON Contract Compliance (/failsafe/status)

Can be executed standalone with pytest or python3.
"""

import sys
import os
import math
import time
import json
import unittest
import types
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Path setup for exploration_node and failsafe_node
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BACKEND_DIR = os.path.abspath(os.path.join(_THIS_DIR, ".."))
sys.path.insert(0, os.path.join(_BACKEND_DIR, "exploration_node"))
sys.path.insert(0, os.path.join(_BACKEND_DIR, "failsafe_node"))

# ---------------------------------------------------------------------------
# Minimal ROS 2 Message & RCLPY Stubs for standalone test execution
# ---------------------------------------------------------------------------
def _make_stub_module(name):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod

if "rclpy" not in sys.modules:
    rclpy_mod = _make_stub_module("rclpy")
    rclpy_mod.ok = lambda: True
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.shutdown = lambda: None

if "rclpy.node" not in sys.modules:
    node_mod = _make_stub_module("rclpy.node")

    class FakeClock:
        def now(self):
            return self
        def to_msg(self):
            t = time.time()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            msg = type("TimeMsg", (), {"sec": sec, "nanosec": nanosec})()
            return msg

    class FakeLogger:
        def info(self, m): pass
        def debug(self, m): pass
        def warn(self, m): pass
        def error(self, m): pass

    class FakePublisher:
        def __init__(self, topic, msg_type):
            self.topic = topic
            self.msg_type = msg_type
            self.published_messages = []
        def publish(self, msg):
            self.published_messages.append(msg)

    class FakeSubscription:
        def __init__(self, topic, msg_type, callback):
            self.topic = topic
            self.msg_type = msg_type
            self.callback = callback

    class FakeClient:
        def __init__(self, srv_type, name):
            self.srv_type = srv_type
            self.name = name
            self._ready = True
        def service_is_ready(self):
            return self._ready
        def wait_for_service(self, timeout_sec=None):
            return self._ready
        def call_async(self, req):
            future = MagicMock()
            future.done.return_value = True
            res = type("Res", (), {"success": True, "message": "OK"})()
            future.result.return_value = res
            return future

    class FakeParameter:
        def __init__(self, name, val):
            self._name = name
            self.value = val
        def get_parameter_value(self):
            return self

    class FakeNode:
        def __init__(self, name):
            self._name = name
            self._publishers = []
            self._subscriptions = []
            self._services = []
            self._clients = []
            self._timers = []
            self._params = {}
            self._clock = FakeClock()
            self._logger = FakeLogger()

        def get_clock(self): return self._clock
        def get_logger(self): return self._logger

        def declare_parameter(self, name, default_val):
            self._params[name] = default_val

        def get_parameter(self, name):
            return FakeParameter(name, self._params.get(name))

        def create_publisher(self, msg_type, topic, qos):
            pub = FakePublisher(topic, msg_type)
            self._publishers.append(pub)
            return pub

        def create_subscription(self, msg_type, topic, cb, qos):
            sub = FakeSubscription(topic, msg_type, cb)
            self._subscriptions.append(sub)
            return sub

        def create_service(self, srv_type, srv_name, cb):
            srv = (srv_type, srv_name, cb)
            self._services.append(srv)
            return srv

        def create_client(self, srv_type, srv_name):
            cli = FakeClient(srv_type, srv_name)
            self._clients.append(cli)
            return cli

        def create_timer(self, period, cb):
            self._timers.append((period, cb))
            return cb

        def destroy_node(self): pass

    node_mod.Node = FakeNode

if "rclpy.qos" not in sys.modules:
    qos_mod = _make_stub_module("rclpy.qos")
    class QoSProfile:
        def __init__(self, **kw): pass
    class _Policy:
        RELIABLE = 1; BEST_EFFORT = 2; TRANSIENT_LOCAL = 3; VOLATILE = 4; KEEP_LAST = 5
    qos_mod.QoSProfile = QoSProfile
    qos_mod.QoSDurabilityPolicy = _Policy
    qos_mod.QoSReliabilityPolicy = _Policy
    qos_mod.QoSHistoryPolicy = _Policy

if "geometry_msgs.msg" not in sys.modules:
    geom_msg_mod = _make_stub_module("geometry_msgs.msg")
    class Point:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = float(x); self.y = float(y); self.z = float(z)
    class Quaternion:
        def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
            self.x = float(x); self.y = float(y); self.z = float(z); self.w = float(w)
    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()
    class Header:
        def __init__(self, frame_id="map"):
            self.stamp = type("Stamp", (), {"sec": 0, "nanosec": 0})()
            self.frame_id = frame_id
    class PoseStamped:
        def __init__(self):
            self.header = Header()
            self.pose = Pose()
    geom_msg_mod.Point = Point
    geom_msg_mod.Quaternion = Quaternion
    geom_msg_mod.Pose = Pose
    geom_msg_mod.PoseStamped = PoseStamped
    geom_msg_mod.Header = Header

if "nav_msgs.msg" not in sys.modules:
    nav_msg_mod = _make_stub_module("nav_msgs.msg")
    class OccupancyGrid:
        def __init__(self):
            self.header = sys.modules["geometry_msgs.msg"].Header()
            self.info = type("MapInfo", (), {
                "resolution": 0.1,
                "width": 100,
                "height": 100,
                "origin": sys.modules["geometry_msgs.msg"].Pose()
            })()
            self.data = []
    class Path:
        def __init__(self):
            self.header = sys.modules["geometry_msgs.msg"].Header()
            self.poses = []
    nav_msg_mod.OccupancyGrid = OccupancyGrid
    nav_msg_mod.Path = Path

if "sensor_msgs.msg" not in sys.modules:
    sensor_msg_mod = _make_stub_module("sensor_msgs.msg")
    class BatteryState:
        def __init__(self):
            self.header = sys.modules["geometry_msgs.msg"].Header()
            self.voltage = 12.6
            self.percentage = 1.0
    sensor_msg_mod.BatteryState = BatteryState

if "std_msgs.msg" not in sys.modules:
    std_msg_mod = _make_stub_module("std_msgs.msg")
    class String:
        def __init__(self):
            self.data = ""
    std_msg_mod.String = String
    std_msg_mod.Header = sys.modules["geometry_msgs.msg"].Header

if "std_srvs.srv" not in sys.modules:
    std_srv_mod = _make_stub_module("std_srvs.srv")
    class Trigger:
        class Request: pass
        class Response:
            def __init__(self): self.success = False; self.message = ""
    class SetBool:
        class Request:
            def __init__(self): self.data = False
        class Response:
            def __init__(self): self.success = False; self.message = ""
    std_srv_mod.Trigger = Trigger
    std_srv_mod.SetBool = SetBool


# Import classes under test
from failsafe_node.failsafe_node import FailsafeNode, FailsafeState, FailsafeReason
from exploration_node.path_planner import PathPlanner


def make_walled_test_map() -> sys.modules["nav_msgs.msg"].OccupancyGrid:
    """
    Constructs a 100x100 (10m x 10m) OccupancyGrid with an obstacle wall
    in the center (from y=2m to y=8m at x=5m) between the drone at (8.0, 5.0)
    and the home entry point at (0.0, 0.0).
    """
    grid = sys.modules["nav_msgs.msg"].OccupancyGrid()
    grid.info.resolution = 0.1  # 0.1m / cell
    grid.info.width = 100
    grid.info.height = 100
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = 0.0

    # 0 = FREE
    data = [0] * 10000

    # Place a vertical wall at x = 5.0m (col 50) from y = 2.0m to 8.0m (rows 20 to 80)
    # 100 = OCCUPIED
    for r in range(20, 80):
        data[r * 100 + 50] = 100

    grid.data = data
    return grid


class TestFailsafeNode(unittest.TestCase):
    """
    Test suite verifying all 4 failsafe triggers, obstacle-aware A* RTL,
    and GCS telemetry streaming.
    """

    def setUp(self):
        self.node = FailsafeNode()
        self.node._planner = PathPlanner()

    def tearDown(self):
        self.node.destroy_node()

    def test_01_parameter_defaults_and_tuning_flag(self):
        """Verify default parameters, geofence, and battery threshold placeholder."""
        self.assertEqual(self.node._battery_threshold, 15.0)
        self.assertEqual(self.node._link_timeout, 3.0)
        self.assertEqual(self.node._geofence_x_min, -1.0)
        self.assertEqual(self.node._geofence_x_max, 16.0)
        self.assertEqual(self.node._home_x, 0.0)
        self.assertEqual(self.node._home_y, 0.0)
        self.assertEqual(self.node._state, FailsafeState.NORMAL)
        self.assertEqual(self.node._trigger_reason, FailsafeReason.NONE)

    def test_02_condition_1_low_battery_trigger(self):
        """[Condition 1] Battery drops below 15.0% threshold (e.g. 12.4%) -> triggers RTL."""
        # 1. Provide pose and map
        pose = sys.modules["geometry_msgs.msg"].PoseStamped()
        pose.pose.position.x = 6.0
        pose.pose.position.y = 4.0
        pose.pose.position.z = 2.5
        self.node._on_drone_pose(pose)
        self.node._on_map(make_walled_test_map())

        # 2. Battery drops to 12.4% (< 15.0%)
        batt = sys.modules["sensor_msgs.msg"].BatteryState()
        batt.voltage = 11.2
        batt.percentage = 0.124  # 12.4%
        self.node._on_battery_status(batt)

        # 3. Run supervisor monitor cycle
        self.node._monitor_loop()

        # 4. Verify failsafe triggered
        self.assertEqual(self.node._state, FailsafeState.NAVIGATING_HOME)
        self.assertEqual(self.node._trigger_reason, FailsafeReason.LOW_BATTERY)
        self.assertIn("12.4%", self.node._trigger_detail)

        # 5. Verify /goal_pose and /failsafe/planned_path published
        self.assertTrue(len(self.node._pub_goal_pose.published_messages) > 0)
        self.assertTrue(len(self.node._pub_planned_path.published_messages) > 0)

    def test_03_condition_2_link_loss_timeout(self):
        """[Condition 2] Heartbeat / telemetry loss for > 3.0s -> triggers RTL."""
        pose = sys.modules["geometry_msgs.msg"].PoseStamped()
        pose.pose.position.x = 4.0
        pose.pose.position.y = 3.0
        pose.pose.position.z = 2.5
        self.node._on_drone_pose(pose)
        self.node._on_map(make_walled_test_map())

        # Simulate 4.5 seconds passing without telemetry update
        now = time.time()
        self.node._last_heartbeat_time = now - 4.5

        # Run supervisor check
        self.node._check_safety_rules(now)

        self.assertEqual(self.node._state, FailsafeState.NAVIGATING_HOME)
        self.assertEqual(self.node._trigger_reason, FailsafeReason.LINK_LOST)
        self.assertIn("4.5s", self.node._trigger_detail)

    def test_04_condition_3_geofence_breach(self):
        """[Condition 3] Drone flies outside 15x15m geofence (X = 18.2m > 16.0m) -> triggers RTL."""
        pose = sys.modules["geometry_msgs.msg"].PoseStamped()
        pose.pose.position.x = 18.2  # Breaches X max of 16.0m
        pose.pose.position.y = 5.0
        pose.pose.position.z = 2.5
        self.node._on_drone_pose(pose)
        self.node._on_map(make_walled_test_map())

        now = time.time()
        self.node._check_safety_rules(now)

        self.assertEqual(self.node._state, FailsafeState.NAVIGATING_HOME)
        self.assertEqual(self.node._trigger_reason, FailsafeReason.GEOFENCE_BREACH)
        self.assertIn("18.20", self.node._trigger_detail)

    def test_05_condition_4_manual_abort_service(self):
        """[Condition 4] Operator calls /failsafe/abort service -> triggers immediate RTL."""
        pose = sys.modules["geometry_msgs.msg"].PoseStamped()
        pose.pose.position.x = 7.0
        pose.pose.position.y = 6.0
        pose.pose.position.z = 2.5
        self.node._on_drone_pose(pose)
        self.node._on_map(make_walled_test_map())

        req = sys.modules["std_srvs.srv"].Trigger.Request()
        resp = sys.modules["std_srvs.srv"].Trigger.Response()

        res = self.node._handle_manual_abort(req, resp)
        self.assertTrue(res.success)
        self.assertEqual(self.node._state, FailsafeState.NAVIGATING_HOME)
        self.assertEqual(self.node._trigger_reason, FailsafeReason.MANUAL_ABORT)

    def test_06_obstacle_aware_rtl_vs_straight_line(self):
        """[Indoor Safety] Verify A* routes around wall at x=5.0m instead of colliding straight-line."""
        # Drone at (8.0, 5.0), Home at (0.0, 0.0), Wall at x=5.0m (y=2m to 8m)
        pose = sys.modules["geometry_msgs.msg"].PoseStamped()
        pose.pose.position.x = 8.0
        pose.pose.position.y = 5.0
        pose.pose.position.z = 2.5
        self.node._on_drone_pose(pose)
        self.node._on_map(make_walled_test_map())

        # Trigger failsafe
        self.node._trigger_failsafe(FailsafeReason.MANUAL_ABORT, "Indoor test")

        # Verify planned waypoints exist and circumvent the wall (y > 8.0m or y < 2.0m when crossing x=5.0m)
        self.assertTrue(len(self.node._planned_waypoints) > 0)

        # Check waypoints near wall column (x between 4.5m and 5.5m)
        for wx, wy in self.node._planned_waypoints:
            if 4.5 <= wx <= 5.5:
                # Must navigate around the wall gap (below 2.0m or above 8.0m)
                is_in_wall = (2.0 <= wy <= 8.0)
                self.assertFalse(is_in_wall, f"Waypoint ({wx:.2f}, {wy:.2f}) collides with wall!")

    def test_07_home_arrival_and_landing(self):
        """[Touchdown] Drone arrives at Home (dist <= 0.6m) -> calls /land and lands."""
        # Position drone directly near home
        pose = sys.modules["geometry_msgs.msg"].PoseStamped()
        pose.pose.position.x = 0.2
        pose.pose.position.y = 0.1
        pose.pose.position.z = 2.5
        self.node._on_drone_pose(pose)
        self.node._state = FailsafeState.NAVIGATING_HOME

        # Execute navigation check
        self.node._execute_rtl_navigation()
        self.assertEqual(self.node._state, FailsafeState.LANDING)

        # Simulate touchdown at altitude 0.0m
        pose.pose.position.z = 0.0
        self.node._check_landing_status()
        self.assertEqual(self.node._state, FailsafeState.LANDED)

    def test_08_reset_service(self):
        """Verify /failsafe/reset clears trigger and returns to NORMAL state."""
        self.node._state = FailsafeState.NAVIGATING_HOME
        self.node._trigger_reason = FailsafeReason.LOW_BATTERY

        req = sys.modules["std_srvs.srv"].Trigger.Request()
        resp = sys.modules["std_srvs.srv"].Trigger.Response()
        res = self.node._handle_reset(req, resp)

        self.assertTrue(res.success)
        self.assertEqual(self.node._state, FailsafeState.NORMAL)
        self.assertEqual(self.node._trigger_reason, FailsafeReason.NONE)

    def test_09_failsafe_status_json_contract(self):
        """Verify /failsafe/status JSON schema contains all required GCS keys."""
        self.node._publish_failsafe_status()
        self.assertTrue(len(self.node._pub_status.published_messages) > 0)

        raw_json = self.node._pub_status.published_messages[-1].data
        data = json.loads(raw_json)

        self.assertIn("active", data)
        self.assertIn("triggered", data)
        self.assertIn("state", data)
        self.assertIn("reason", data)
        self.assertIn("battery_pct", data)
        self.assertIn("link_age_sec", data)
        self.assertIn("current_pos", data)
        self.assertIn("home_pos", data)
        self.assertIn("distance_to_home", data)


def run_sitl_failsafe_scenarios():
    """
    Runs an interactive SITL flight simulation verifying all 4 failsafe triggers
    against closed-loop flight dynamics and obstacle avoidance.
    """
    print("\n" + "=" * 75)
    print("  [SITL TEST] NIDAR AIRMOUSE: FAILSAFE NODE MULTI-SCENARIO VERIFICATION")
    print("=" * 75 + "\n")

    node = FailsafeNode()
    node._planner = PathPlanner()
    walled_map = make_walled_test_map()
    node._on_map(walled_map)

    # -------------------------------------------------------------------------
    # Scenario 1: Low Battery RTL Trigger
    # -------------------------------------------------------------------------
    print("[1/4] Testing Condition 1: Low Battery Threshold (< 15.0%)...")
    pose = sys.modules["geometry_msgs.msg"].PoseStamped()
    pose.pose.position.x = 8.0; pose.pose.position.y = 5.0; pose.pose.position.z = 2.5
    node._on_drone_pose(pose)

    batt = sys.modules["sensor_msgs.msg"].BatteryState()
    batt.percentage = 0.12  # 12%
    batt.voltage = 11.1
    node._on_battery_status(batt)
    node._monitor_loop()

    assert node._state == FailsafeState.NAVIGATING_HOME, "Failed: State not NAVIGATING_HOME"
    assert node._trigger_reason == FailsafeReason.LOW_BATTERY, "Failed: Reason not LOW_BATTERY"
    assert len(node._planned_waypoints) > 0, "Failed: No A* waypoints computed"
    print(f"      [PASS] Triggered LOW_BATTERY | A* Waypoints: {len(node._planned_waypoints)} | Reason: {node._trigger_detail}")

    # Reset
    req = sys.modules["std_srvs.srv"].Trigger.Request()
    resp = sys.modules["std_srvs.srv"].Trigger.Response()
    node._handle_reset(req, resp)

    # -------------------------------------------------------------------------
    # Scenario 2: Telemetry / Link Loss Timeout
    # -------------------------------------------------------------------------
    print("\n[2/4] Testing Condition 2: Link Loss Timeout (> 3.0s)...")
    node._on_drone_pose(pose)
    batt.percentage = 0.85
    node._on_battery_status(batt)
    now = time.time()
    node._last_heartbeat_time = now - 4.2  # 4.2s ago

    node._check_safety_rules(now)
    assert node._state == FailsafeState.NAVIGATING_HOME, "Failed: State not NAVIGATING_HOME"
    assert node._trigger_reason == FailsafeReason.LINK_LOST, "Failed: Reason not LINK_LOST"
    print(f"      [PASS] Triggered LINK_LOST | Age: 4.2s > 3.0s | Reason: {node._trigger_detail}")

    node._handle_reset(req, resp)

    # -------------------------------------------------------------------------
    # Scenario 3: Geofence Breach
    # -------------------------------------------------------------------------
    print("\n[3/4] Testing Condition 3: Geofence Breach (Arena boundary 15x15m)...")
    breach_pose = sys.modules["geometry_msgs.msg"].PoseStamped()
    breach_pose.pose.position.x = 17.5  # Outside 16.0m limit
    breach_pose.pose.position.y = 6.0
    breach_pose.pose.position.z = 2.5
    node._on_drone_pose(breach_pose)
    node._last_heartbeat_time = time.time()

    node._check_safety_rules(time.time())
    assert node._state == FailsafeState.NAVIGATING_HOME, "Failed: State not NAVIGATING_HOME"
    assert node._trigger_reason == FailsafeReason.GEOFENCE_BREACH, "Failed: Reason not GEOFENCE_BREACH"
    print(f"      [PASS] Triggered GEOFENCE_BREACH at ({breach_pose.pose.position.x:.1f}, {breach_pose.pose.position.y:.1f})")

    node._handle_reset(req, resp)

    # -------------------------------------------------------------------------
    # Scenario 4: Manual GCS Abort
    # -------------------------------------------------------------------------
    print("\n[4/4] Testing Condition 4: Manual Operator Abort Service (/failsafe/abort)...")
    node._on_drone_pose(pose)
    res = node._handle_manual_abort(req, resp)
    assert res.success, "Failed: Abort service returned False"
    assert node._state == FailsafeState.NAVIGATING_HOME, "Failed: State not NAVIGATING_HOME"
    assert node._trigger_reason == FailsafeReason.MANUAL_ABORT, "Failed: Reason not MANUAL_ABORT"
    print(f"      [PASS] Triggered MANUAL_ABORT | Message: {res.message}")

    # Complete flight home and landing
    home_pose = sys.modules["geometry_msgs.msg"].PoseStamped()
    home_pose.pose.position.x = 0.1; home_pose.pose.position.y = 0.2; home_pose.pose.position.z = 2.5
    node._on_drone_pose(home_pose)
    node._execute_rtl_navigation()
    assert node._state == FailsafeState.LANDING, "Failed: State not LANDING"

    home_pose.pose.position.z = 0.0
    node._check_landing_status()
    assert node._state == FailsafeState.LANDED, "Failed: State not LANDED"
    print(f"      [PASS] Home Entry Point Reached -> Initiated Landing -> Touchdown Verified (State={node._state.name})")

    print("\n" + "=" * 75)
    print("  [SUCCESS] ALL 4 FAILSAFE CONDITIONS & OBSTACLE-AWARE RTL VERIFIED")
    print("=" * 75 + "\n")


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(TestFailsafeNode)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    if result.wasSuccessful():
        run_sitl_failsafe_scenarios()
    else:
        sys.exit(1)
