"""
Command Bridge for NIDAR AirMouse GCS.
Translates GCS operator dashboard commands into ROS 2 topic publishes
and MAVROS flight control service/topic calls.
"""
from typing import Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    ROS2_AVAILABLE = True
except ImportError:
    Node = object
    ROS2_AVAILABLE = False


class CommandBridgeNode(Node if ROS2_AVAILABLE else object):
    """
    Translates UI WebSocket commands (START_MISSION, ABORT_MISSION, CHANGE_SLAM_MODE)
    into ROS 2 topic and service calls.
    """

    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__("gcs_command_bridge")
        self.last_executed_command: Optional[Dict[str, Any]] = None

        if ROS2_AVAILABLE:
            self._setup_publishers()

    def _setup_publishers(self):
        """Initializes ROS 2 command publishers."""
        # Arming & flight mode commands
        self.arm_pub = self.create_publisher(Bool, "/mavros/cmd/arming", 10)
        self.mode_pub = self.create_publisher(String, "/mavros/set_mode", 10)

        # Failsafe emergency abort
        self.abort_pub = self.create_publisher(Bool, "/abort", 10)
        self.abort_str_pub = self.create_publisher(String, "/failsafe/abort", 10)

        # SLAM Mode change
        self.slam_mode_pub = self.create_publisher(String, "/slam_mode_change", 10)

    def translate_command(self, action: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Translates a raw UI action and parameters into normalized flight operations.
        Returns a dictionary detailing the translation.
        """
        params = params or {}
        norm_action = action.strip().upper()

        if norm_action in ("START_MISSION", "START"):
            return {
                "action": "START_MISSION",
                "arming": True,
                "flight_mode": "GUIDED",
                "topics": {
                    "/mavros/cmd/arming": True,
                    "/mavros/set_mode": "GUIDED"
                }
            }
        elif norm_action in ("ABORT_MISSION", "ABORT", "EMERGENCY_ABORT"):
            reason = params.get("reason", "Operator Emergency Abort")
            return {
                "action": "ABORT_MISSION",
                "abort": True,
                "reason": reason,
                "flight_mode": "LOITER",
                "topics": {
                    "/abort": True,
                    "/failsafe/abort": reason,
                    "/mavros/set_mode": "LOITER"
                }
            }
        elif norm_action in ("CHANGE_SLAM_MODE", "SET_SLAM_MODE"):
            mode = str(params.get("mode", "REALTIME")).upper()
            return {
                "action": "CHANGE_SLAM_MODE",
                "slam_mode": mode,
                "topics": {
                    "/slam_mode_change": mode
                }
            }
        else:
            return {
                "action": norm_action,
                "status": "UNHANDLED",
                "params": params
            }

    def handle_command(self, cmd_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Receives command payload from WebSocket client and dispatches to ROS 2.
        Fails gracefully if ROS 2 is not initialized or connected.
        """
        action = cmd_data.get("action", "")
        params = cmd_data.get("params", {})

        trans = self.translate_command(action, params)
        self.last_executed_command = trans

        if not ROS2_AVAILABLE:
            print(f"[CommandBridge] ROS 2 not available. Logged command: {trans['action']}")
            return trans

        try:
            if trans["action"] == "START_MISSION":
                # 1. Publish arming request
                arm_msg = Bool()
                arm_msg.data = True
                self.arm_pub.publish(arm_msg)

                # 2. Publish GUIDED flight mode
                mode_msg = String()
                mode_msg.data = "GUIDED"
                self.mode_pub.publish(mode_msg)
                print("[CommandBridge] Dispatched START_MISSION: Armed=True, Mode=GUIDED")

            elif trans["action"] == "ABORT_MISSION":
                # 1. Publish abort to failsafe node
                abort_msg = Bool()
                abort_msg.data = True
                self.abort_pub.publish(abort_msg)

                reason_msg = String()
                reason_msg.data = trans.get("reason", "Operator Emergency Abort")
                self.abort_str_pub.publish(reason_msg)

                # 2. Switch Pixhawk to LOITER / HOLD
                mode_msg = String()
                mode_msg.data = "LOITER"
                self.mode_pub.publish(mode_msg)
                print(f"[CommandBridge] Dispatched ABORT_MISSION: {reason_msg.data}, Mode=LOITER")

            elif trans["action"] == "CHANGE_SLAM_MODE":
                mode_msg = String()
                mode_msg.data = trans.get("slam_mode", "REALTIME")
                self.slam_mode_pub.publish(mode_msg)
                print(f"[CommandBridge] Dispatched CHANGE_SLAM_MODE: {mode_msg.data}")

        except Exception as e:
            print(f"[CommandBridge] Warning: Error publishing command {trans['action']}: {e}")

        return trans
