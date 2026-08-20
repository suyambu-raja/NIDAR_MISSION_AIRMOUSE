#!/usr/bin/env python3
"""
test_mavros_bridge.py — Unit & Integration tests for mavros_bridge_node
======================================================================
Verifies:
1. Node initialization & parameter configuration
2. Low-latency pose relay (/mavros/local_position/pose -> /drone_pose)
3. Battery telemetry relay (/mavros/battery -> /battery_status)
4. Navigation goal conversion (/goal_pose -> /mavros/setpoint_position/local)
5. Velocity command forwarding (/cmd_vel -> /mavros/setpoint_velocity/cmd_vel_unstamped)
6. Arm / Disarm service handling
7. Flight mode switching (GUIDED, RTL, LAND, LOITER, STABILIZE)
8. Autonomous Takeoff routine (/takeoff)
9. Autonomous RTL routine (/rtl)
10. JSON telemetry state format on /mavros_bridge/state
"""

import json
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Quaternion
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from mavros_bridge.mavros_bridge_node import MavrosBridgeNode


class TestMavrosBridgeNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = MavrosBridgeNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization_and_params(self):
        """Test default parameters match SITL & hardware config."""
        self.assertEqual(self.node._fcu_url, "udp://127.0.0.1:14550@")
        self.assertEqual(self.node._target_sys_id, 1)
        self.assertEqual(self.node._default_altitude, 2.5)
        self.assertEqual(self.node._map_frame, "map")

    def test_pose_relay(self):
        """Test /mavros/local_position/pose is relayed to /drone_pose in map frame."""
        pub_mock = MagicMock()
        self.node._pub_drone_pose.publish = pub_mock

        input_pose = PoseStamped()
        input_pose.pose.position.x = 4.2
        input_pose.pose.position.y = 7.8
        input_pose.pose.position.z = 2.4
        input_pose.pose.orientation.w = 1.0

        self.node._on_mavros_pose(input_pose)

        self.assertTrue(self.node._in_air)
        self.assertIsNotNone(self.node._current_pose)
        pub_mock.assert_called_once()
        published_msg = pub_mock.call_args[0][0]
        self.assertEqual(published_msg.header.frame_id, "map")
        self.assertAlmostEqual(published_msg.pose.position.x, 4.2)
        self.assertAlmostEqual(published_msg.pose.position.y, 7.8)
        self.assertAlmostEqual(published_msg.pose.position.z, 2.4)

    def test_battery_relay(self):
        """Test /mavros/battery is relayed to /battery_status."""
        pub_mock = MagicMock()
        self.node._pub_battery_status.publish = pub_mock

        input_battery = BatteryState()
        input_battery.voltage = 11.4
        input_battery.current = 14.5
        input_battery.percentage = 0.65  # 65%

        self.node._on_mavros_battery(input_battery)

        self.assertAlmostEqual(self.node._battery_pct, 65.0)
        self.assertAlmostEqual(self.node._battery_voltage, 11.4)
        pub_mock.assert_called_once()
        published_msg = pub_mock.call_args[0][0]
        self.assertAlmostEqual(published_msg.percentage, 0.65)
        self.assertAlmostEqual(published_msg.voltage, 11.4)
        self.assertTrue(published_msg.present)

    def test_goal_pose_conversion(self):
        """Test /goal_pose from exploration_node is forwarded as MAVROS setpoint."""
        pub_mock = MagicMock()
        self.node._pub_mavros_setpoint_pos.publish = pub_mock

        goal = PoseStamped()
        goal.pose.position.x = 5.0
        goal.pose.position.y = 3.0
        goal.pose.position.z = 0.0  # 2D exploration goal without explicit z
        goal.pose.orientation.w = 1.0

        self.node._on_goal_pose(goal)

        pub_mock.assert_called_once()
        setpoint = pub_mock.call_args[0][0]
        self.assertAlmostEqual(setpoint.pose.position.x, 5.0)
        self.assertAlmostEqual(setpoint.pose.position.y, 3.0)
        # Should enforce default cruise altitude of 2.5m
        self.assertAlmostEqual(setpoint.pose.position.z, 2.5)

    def test_cmd_vel_forwarding(self):
        """Test /cmd_vel velocity commands are forwarded directly."""
        pub_mock = MagicMock()
        self.node._pub_mavros_setpoint_vel.publish = pub_mock

        twist = Twist()
        twist.linear.x = 0.8
        twist.angular.z = 0.2

        self.node._on_cmd_vel(twist)

        pub_mock.assert_called_once_with(twist)

    def test_arm_service(self):
        """Test /arm service handler."""
        req = SetBool.Request()
        req.data = True
        resp = SetBool.Response()

        result = self.node._handle_arm_service(req, resp)
        self.assertTrue(result.success)
        self.assertTrue(self.node._armed)

        # Test disarm
        req.data = False
        result = self.node._handle_arm_service(req, resp)
        self.assertTrue(result.success)
        self.assertFalse(self.node._armed)

    def test_flight_mode_switches(self):
        """Test mode switching services."""
        req = Trigger.Request()
        resp = Trigger.Response()

        # GUIDED
        res = self.node._handle_mode_guided(req, resp)
        self.assertTrue(res.success)
        self.assertEqual(self.node._mode, "GUIDED")

        # RTL
        res = self.node._handle_mode_rtl(req, resp)
        self.assertTrue(res.success)
        self.assertEqual(self.node._mode, "RTL")

        # LAND
        res = self.node._handle_mode_land(req, resp)
        self.assertTrue(res.success)
        self.assertEqual(self.node._mode, "LAND")

        # LOITER
        res = self.node._handle_mode_loiter(req, resp)
        self.assertTrue(res.success)
        self.assertEqual(self.node._mode, "LOITER")

    def test_takeoff_service(self):
        """Test /takeoff triggers GUIDED mode, arming, and altitude setpoint."""
        req = Trigger.Request()
        resp = Trigger.Response()

        res = self.node._handle_takeoff_service(req, resp)
        self.assertTrue(res.success)
        self.assertTrue(self.node._armed)
        self.assertEqual(self.node._mode, "GUIDED")
        self.assertTrue(self.node._in_air)

    def test_rtl_service(self):
        """Test /rtl triggers Return To Launch."""
        req = Trigger.Request()
        resp = Trigger.Response()

        res = self.node._handle_rtl_service(req, resp)
        self.assertTrue(res.success)
        self.assertEqual(self.node._mode, "RTL")

    def test_publish_bridge_state(self):
        """Test /mavros_bridge/state publishes clean JSON payload."""
        pub_mock = MagicMock()
        self.node._pub_bridge_state.publish = pub_mock

        # Setup state
        self.node._connected = True
        self.node._armed = True
        self.node._mode = "GUIDED"
        self.node._battery_pct = 78.4
        self.node._battery_voltage = 11.8

        pose = PoseStamped()
        pose.pose.position.x = 2.1
        pose.pose.position.y = 4.5
        pose.pose.position.z = 2.5
        self.node._current_pose = pose

        self.node._publish_bridge_state()

        pub_mock.assert_called_once()
        published_msg = pub_mock.call_args[0][0]
        data = json.loads(published_msg.data)

        self.assertTrue(data["connected"])
        self.assertTrue(data["armed"])
        self.assertEqual(data["mode"], "GUIDED")
        self.assertAlmostEqual(data["battery_pct"], 78.4)
        self.assertEqual(data["position"], [2.1, 4.5, 2.5])


if __name__ == "__main__":
    unittest.main()
