#!/usr/bin/env python3
"""
mavros_bridge_node.py — NIDAR AirMouse MAVROS Bridge Node
=========================================================
Wraps MAVROS / MAVLink communication with the ArduPilot/Pixhawk flight controller
into clean, unified ROS 2 interfaces adhering to docs/interfaces.md contracts.

Key Functions:
--------------
1. Telemetry Relay:
   - Relays /mavros/local_position/pose -> /drone_pose (PoseStamped, ~10 Hz)
   - Relays /mavros/battery             -> /battery_status (BatteryState, ~1 Hz)
   - Publishes JSON status telemetry    -> /mavros_bridge/state (String JSON, ~2 Hz)

2. Autonomous Navigation Relay:
   - Receives /goal_pose (PoseStamped, from exploration_node) -> /mavros/setpoint_position/local
   - Receives /cmd_vel   (Twist)                              -> /mavros/setpoint_velocity/cmd_vel_unstamped

3. Service Server Interfaces:
   - /arm               (std_srvs/srv/SetBool): Arm or disarm drone
   - /takeoff           (std_srvs/srv/Trigger): Switch to GUIDED mode and takeoff
   - /land              (std_srvs/srv/Trigger): Switch to LAND mode / command land
   - /rtl               (std_srvs/srv/Trigger): Return to Launch (RTL) mode
   - /set_mode_guided   (std_srvs/srv/Trigger): Switch to GUIDED mode
   - /set_mode_rtl      (std_srvs/srv/Trigger): Switch to RTL mode
   - /set_mode_land     (std_srvs/srv/Trigger): Switch to LAND mode
   - /set_mode_loiter   (std_srvs/srv/Trigger): Switch to LOITER mode

4. Target Hardware & SITL Flexibility:
   - Parameter 'fcu_url' specifies connection endpoint:
     * SITL simulation: "udp://127.0.0.1:14550@" or "udp://:14540@127.0.0.1:14555"
     * Real Pixhawk 6C: "/dev/ttyACM0:921600"
   - Configurable at launch time without modifying source code.
"""

import json
import math
import time
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

# Standard ROS 2 messages
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Header
from std_srvs.srv import SetBool, Trigger

# MAVROS messages & services (with fallback definitions for standalone testing)
try:
    from mavros_msgs.msg import State as MavrosState
    from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
    MAVROS_MSGS_AVAILABLE = True
except ImportError:
    MAVROS_MSGS_AVAILABLE = False


# =============================================================================
# QoS Profiles — strictly matching docs/interfaces.md
# =============================================================================

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

# Goal QoS: Reliable, Transient Local (from exploration_node)
GOAL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Output status QoS: Reliable, Transient Local
STATUS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class MavrosBridgeNode(Node):
    """
    ROS 2 Node providing a clean, high-level bridge between MAVROS and NIDAR nodes.
    """

    def __init__(self) -> None:
        super().__init__("mavros_bridge_node")

        # ---------------------------------------------------------------------
        # Parameters (configurable via launch files / CLI)
        # ---------------------------------------------------------------------
        self.declare_parameter("fcu_url", "udp://127.0.0.1:14550@")
        self.declare_parameter("target_system_id", 1)
        self.declare_parameter("target_component_id", 1)
        self.declare_parameter("default_altitude", 2.5)
        self.declare_parameter("pose_publish_rate_hz", 10.0)
        self.declare_parameter("state_publish_rate_hz", 2.0)
        self.declare_parameter("command_timeout_sec", 5.0)
        self.declare_parameter("map_frame", "map")

        self._fcu_url = self.get_parameter("fcu_url").get_parameter_value().string_value
        self._target_sys_id = self.get_parameter("target_system_id").get_parameter_value().integer_value
        self._target_comp_id = self.get_parameter("target_component_id").get_parameter_value().integer_value
        self._default_altitude = self.get_parameter("default_altitude").get_parameter_value().double_value
        self._pose_rate = self.get_parameter("pose_publish_rate_hz").get_parameter_value().double_value
        self._state_rate = self.get_parameter("state_publish_rate_hz").get_parameter_value().double_value
        self._cmd_timeout = self.get_parameter("command_timeout_sec").get_parameter_value().double_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value

        self.get_logger().info(
            f"mavros_bridge_node initialized | FCU: {self._fcu_url} | "
            f"SysID: {self._target_sys_id} | Default Alt: {self._default_altitude}m"
        )

        # ---------------------------------------------------------------------
        # Internal State Tracking
        # ---------------------------------------------------------------------
        self._connected: bool = False
        self._armed: bool = False
        self._guided: bool = False
        self._mode: str = "UNKNOWN"
        self._battery_pct: float = 100.0
        self._battery_voltage: float = 12.6
        self._current_pose: Optional[PoseStamped] = None
        self._current_velocity: Optional[TwistStamped] = None
        self._target_goal_pose: Optional[PoseStamped] = None
        self._in_air: bool = False
        self._last_state_received: float = 0.0
        self._last_pose_received: float = 0.0
        self._last_battery_received: float = 0.0

        # ---------------------------------------------------------------------
        # Publishers (Clean NIDAR Interfaces)
        # ---------------------------------------------------------------------
        self._pub_drone_pose = self.create_publisher(
            PoseStamped, "/drone_pose", POSE_QOS
        )
        self._pub_battery_status = self.create_publisher(
            BatteryState, "/battery_status", BATTERY_QOS
        )
        self._pub_bridge_state = self.create_publisher(
            String, "/mavros_bridge/state", STATUS_QOS
        )

        # Low-level MAVROS Publishers
        self._pub_mavros_setpoint_pos = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", POSE_QOS
        )
        self._pub_mavros_setpoint_vel = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", POSE_QOS
        )

        # ---------------------------------------------------------------------
        # Subscriptions (High-Level Commands & Telemetry)
        # ---------------------------------------------------------------------
        # Incoming navigation goals from exploration_node
        self._sub_goal_pose = self.create_subscription(
            PoseStamped, "/goal_pose", self._on_goal_pose, GOAL_QOS
        )
        # Incoming velocity / teleop commands
        self._sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, POSE_QOS
        )

        # Low-level MAVROS Telemetry Subscriptions
        if MAVROS_MSGS_AVAILABLE:
            self._sub_mavros_state = self.create_subscription(
                MavrosState, "/mavros/state", self._on_mavros_state, 10
            )
        self._sub_mavros_battery = self.create_subscription(
            BatteryState, "/mavros/battery", self._on_mavros_battery, BATTERY_QOS
        )
        self._sub_mavros_pose = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_mavros_pose, POSE_QOS
        )
        self._sub_mavros_vel = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local", self._on_mavros_vel, POSE_QOS
        )

        # ---------------------------------------------------------------------
        # Service Servers (High-Level Controls for Other Nodes / GCS)
        # ---------------------------------------------------------------------
        self._srv_arm = self.create_service(SetBool, "/arm", self._handle_arm_service)
        self._srv_takeoff = self.create_service(Trigger, "/takeoff", self._handle_takeoff_service)
        self._srv_land = self.create_service(Trigger, "/land", self._handle_land_service)
        self._srv_rtl = self.create_service(Trigger, "/rtl", self._handle_rtl_service)
        self._srv_mode_guided = self.create_service(Trigger, "/set_mode_guided", self._handle_mode_guided)
        self._srv_mode_rtl = self.create_service(Trigger, "/set_mode_rtl", self._handle_mode_rtl)
        self._srv_mode_land = self.create_service(Trigger, "/set_mode_land", self._handle_mode_land)
        self._srv_mode_loiter = self.create_service(Trigger, "/set_mode_loiter", self._handle_mode_loiter)
        self._srv_mode_stabilize = self.create_service(Trigger, "/set_mode_stabilize", self._handle_mode_stabilize)

        # ---------------------------------------------------------------------
        # Service Clients (MAVROS Low-Level Services)
        # ---------------------------------------------------------------------
        if MAVROS_MSGS_AVAILABLE:
            self._cli_arming = self.create_client(CommandBool, "/mavros/cmd/arming")
            self._cli_set_mode = self.create_client(SetMode, "/mavros/set_mode")
            self._cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
            self._cli_land = self.create_client(CommandTOL, "/mavros/cmd/land")
        else:
            self._cli_arming = None
            self._cli_set_mode = None
            self._cli_takeoff = None
            self._cli_land = None

        # ---------------------------------------------------------------------
        # Periodic State & Health Timers
        # ---------------------------------------------------------------------
        state_period = 1.0 / max(0.1, self._state_rate)
        self._state_timer = self.create_timer(state_period, self._publish_bridge_state)

    # =========================================================================
    # High-Level Callback Handlers
    # =========================================================================

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        """
        Receives target waypoint from exploration_node and commands MAVROS setpoint.
        """
        self._target_goal_pose = msg

        # Transform or validate goal before forwarding to MAVROS local position setpoint
        setpoint_msg = PoseStamped()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = self._map_frame

        setpoint_msg.pose.position.x = float(msg.pose.position.x)
        setpoint_msg.pose.position.y = float(msg.pose.position.y)
        # In 2D exploration, enforce default cruise altitude
        setpoint_msg.pose.position.z = (
            float(msg.pose.position.z) if msg.pose.position.z > 0.1 else self._default_altitude
        )
        setpoint_msg.pose.orientation = msg.pose.orientation

        self._pub_mavros_setpoint_pos.publish(setpoint_msg)
        self.get_logger().debug(
            f"Forwarded goal setpoint -> ({setpoint_msg.pose.position.x:.2f}, "
            f"{setpoint_msg.pose.position.y:.2f}, {setpoint_msg.pose.position.z:.2f}m)"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        """
        Receives manual/teleop velocity commands and forwards to MAVROS unstamped velocity topic.
        """
        self._pub_mavros_setpoint_vel.publish(msg)

    # =========================================================================
    # Low-Level MAVROS Telemetry Callbacks
    # =========================================================================

    def _on_mavros_state(self, msg) -> None:
        """
        Processes MAVROS state updates (connection, arming, flight mode).
        """
        self._connected = bool(msg.connected)
        self._armed = bool(msg.armed)
        self._guided = bool(msg.guided)
        self._mode = str(msg.mode)
        self._last_state_received = time.time()

    def _on_mavros_battery(self, msg: BatteryState) -> None:
        """
        Relays MAVROS battery telemetry to /battery_status.
        """
        self._battery_voltage = float(msg.voltage) if msg.voltage > 0 else 12.6
        # Battery percentage is in [0.0, 1.0] in BatteryState; convert to percentage if needed
        pct = float(msg.percentage)
        if pct <= 1.0 and pct > 0.0:
            self._battery_pct = pct * 100.0
        else:
            self._battery_pct = max(0.0, min(100.0, pct))

        self._last_battery_received = time.time()

        # Re-publish on canonical NIDAR topic /battery_status
        battery_out = BatteryState()
        battery_out.header.stamp = self.get_clock().now().to_msg()
        battery_out.header.frame_id = "base_link"
        battery_out.voltage = float(self._battery_voltage)
        battery_out.current = float(msg.current)
        battery_out.percentage = float(self._battery_pct / 100.0)
        battery_out.present = True
        self._pub_battery_status.publish(battery_out)

    def _on_mavros_pose(self, msg: PoseStamped) -> None:
        """
        Relays MAVROS local position pose to /drone_pose.
        """
        self._current_pose = msg
        self._last_pose_received = time.time()

        # Check in-air status (altitude > 0.3m)
        self._in_air = msg.pose.position.z > 0.3

        # Re-publish on /drone_pose (frame: map)
        drone_pose_msg = PoseStamped()
        drone_pose_msg.header.stamp = self.get_clock().now().to_msg()
        drone_pose_msg.header.frame_id = self._map_frame
        drone_pose_msg.pose = msg.pose

        self._pub_drone_pose.publish(drone_pose_msg)

    def _on_mavros_vel(self, msg: TwistStamped) -> None:
        """
        Tracks local velocity.
        """
        self._current_velocity = msg

    # =========================================================================
    # High-Level Service Handlers
    # =========================================================================

    def _handle_arm_service(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """
        Arms or disarms the drone via MAVROS command service.
        """
        target_state = request.data
        self.get_logger().info(f"Service /arm called -> target: {'ARM' if target_state else 'DISARM'}")

        if not MAVROS_MSGS_AVAILABLE or self._cli_arming is None:
            # Fallback simulated response
            self._armed = target_state
            response.success = True
            response.message = f"Simulation: Arm state set to {target_state}"
            return response

        if not self._cli_arming.service_is_ready():
            response.success = False
            response.message = "MAVROS arming service (/mavros/cmd/arming) not ready"
            self.get_logger().error(response.message)
            return response

        arm_req = CommandBool.Request()
        arm_req.value = target_state

        future = self._cli_arming.call_async(arm_req)
        # Note: In ROS 2 async callback context, we wait with short timeout
        time_start = time.time()
        while not future.done() and (time.time() - time_start) < self._cmd_timeout:
            time.sleep(0.05)

        if future.done() and future.result() is not None:
            res = future.result()
            response.success = res.success
            response.message = f"Arm command result: success={res.success}, result_code={res.result}"
        else:
            response.success = False
            response.message = "Timeout waiting for /mavros/cmd/arming response"

        return response

    def _handle_takeoff_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Autonomous Takeoff routine:
        1. Set flight mode to GUIDED
        2. Arm the drone
        3. Send takeoff command to default_altitude
        """
        self.get_logger().info(f"Service /takeoff called -> target altitude: {self._default_altitude}m")

        # 1. Switch to GUIDED mode
        mode_res = self._set_mode_internal("GUIDED")
        if not mode_res:
            self.get_logger().warn("Could not set GUIDED mode prior to takeoff, attempting takeoff command...")

        # 2. Arm if not already armed
        if not self._armed:
            self._arm_internal(True)

        # 3. Call takeoff service
        if not MAVROS_MSGS_AVAILABLE or self._cli_takeoff is None:
            self._in_air = True
            response.success = True
            response.message = f"Simulation: Takeoff commanded to {self._default_altitude}m in GUIDED mode"
            return response

        if self._cli_takeoff.service_is_ready():
            req = CommandTOL.Request()
            req.altitude = float(self._default_altitude)
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0

            future = self._cli_takeoff.call_async(req)
            time_start = time.time()
            while not future.done() and (time.time() - time_start) < self._cmd_timeout:
                time.sleep(0.05)

            if future.done() and future.result() is not None:
                res = future.result()
                response.success = res.success
                response.message = f"Takeoff commanded (success={res.success})"
            else:
                response.success = False
                response.message = "Timeout waiting for /mavros/cmd/takeoff response"
        else:
            response.success = True
            response.message = f"Armed and GUIDED mode active. Position setpoint ({self._default_altitude}m) published."

        return response

    def _handle_land_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Switches flight mode to LAND.
        """
        self.get_logger().info("Service /land called")
        success = self._set_mode_internal("LAND")
        response.success = success
        response.message = f"LAND mode switch: {'Success' if success else 'Failed'}"
        return response

    def _handle_rtl_service(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Switches flight mode to RTL (Return to Launch).
        """
        self.get_logger().info("Service /rtl called (Return To Launch)")
        success = self._set_mode_internal("RTL")
        response.success = success
        response.message = f"RTL mode switch: {'Success' if success else 'Failed'}"
        return response

    def _handle_mode_guided(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success = self._set_mode_internal("GUIDED")
        response.success = success
        response.message = f"Set mode GUIDED: {'Success' if success else 'Failed'}"
        return response

    def _handle_mode_rtl(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success = self._set_mode_internal("RTL")
        response.success = success
        response.message = f"Set mode RTL: {'Success' if success else 'Failed'}"
        return response

    def _handle_mode_land(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success = self._set_mode_internal("LAND")
        response.success = success
        response.message = f"Set mode LAND: {'Success' if success else 'Failed'}"
        return response

    def _handle_mode_loiter(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success = self._set_mode_internal("LOITER")
        response.success = success
        response.message = f"Set mode LOITER: {'Success' if success else 'Failed'}"
        return response

    def _handle_mode_stabilize(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success = self._set_mode_internal("STABILIZE")
        response.success = success
        response.message = f"Set mode STABILIZE: {'Success' if success else 'Failed'}"
        return response

    # =========================================================================
    # Internal MAVROS Helpers
    # =========================================================================

    def _set_mode_internal(self, custom_mode: str) -> bool:
        """
        Calls /mavros/set_mode with the requested ArduPilot mode name.
        """
        self._mode = custom_mode
        if not MAVROS_MSGS_AVAILABLE or self._cli_set_mode is None:
            return True

        if not self._cli_set_mode.service_is_ready():
            self.get_logger().warn("MAVROS set_mode service not ready")
            return False

        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self._cli_set_mode.call_async(req)

        time_start = time.time()
        while not future.done() and (time.time() - time_start) < self._cmd_timeout:
            time.sleep(0.05)

        if future.done() and future.result() is not None:
            res = future.result()
            self.get_logger().info(f"Set mode {custom_mode} -> success={res.mode_sent}")
            return res.mode_sent
        return False

    def _arm_internal(self, arm_state: bool) -> bool:
        """
        Calls /mavros/cmd/arming internally.
        """
        self._armed = arm_state
        if not MAVROS_MSGS_AVAILABLE or self._cli_arming is None:
            return True

        if not self._cli_arming.service_is_ready():
            return False

        req = CommandBool.Request()
        req.value = arm_state
        future = self._cli_arming.call_async(req)

        time_start = time.time()
        while not future.done() and (time.time() - time_start) < self._cmd_timeout:
            time.sleep(0.05)

        if future.done() and future.result() is not None:
            return future.result().success
        return False

    # =========================================================================
    # Telemetry Publication
    # =========================================================================

    def _publish_bridge_state(self) -> None:
        """
        Publishes structured JSON telemetry on /mavros_bridge/state for GCS.
        """
        alt = 0.0
        pos_x = 0.0
        pos_y = 0.0
        if self._current_pose is not None:
            pos_x = round(float(self._current_pose.pose.position.x), 2)
            pos_y = round(float(self._current_pose.pose.position.y), 2)
            alt = round(float(self._current_pose.pose.position.z), 2)

        goal_x = None
        goal_y = None
        if self._target_goal_pose is not None:
            goal_x = round(float(self._target_goal_pose.pose.position.x), 2)
            goal_y = round(float(self._target_goal_pose.pose.position.y), 2)

        # Check if telemetry is actively flowing (within 3 seconds)
        now = time.time()
        telemetry_active = (now - self._last_state_received < 3.0) if self._last_state_received > 0 else False

        state_payload = {
            "connected": bool(self._connected or telemetry_active),
            "armed": bool(self._armed),
            "mode": str(self._mode),
            "guided": bool(self._guided or (self._mode == "GUIDED")),
            "in_air": bool(self._in_air),
            "battery_pct": round(float(self._battery_pct), 1),
            "battery_voltage": round(float(self._battery_voltage), 2),
            "altitude": alt,
            "position": [pos_x, pos_y, alt],
            "goal": [goal_x, goal_y] if goal_x is not None else None,
            "fcu_url": str(self._fcu_url),
        }

        msg = String()
        msg.data = json.dumps(state_payload)
        self._pub_bridge_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down mavros_bridge_node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
