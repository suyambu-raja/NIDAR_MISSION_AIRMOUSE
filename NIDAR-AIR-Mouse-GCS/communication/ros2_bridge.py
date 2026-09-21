"""
ROS 2 Subscriber Bridge for NIDAR AirMouse GCS.
Subscribes to ROS 2 backend topics and translates messages into
standard GCS WebSocket JSON payloads for the HTML5/JS dashboard.
"""
import asyncio
import json
import math
import time
from typing import Optional, Dict, Any, List

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    Node = object
    ROS2_AVAILABLE = False


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """Converts a quaternion into Euler angles (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ROS2BridgeNode(Node if ROS2_AVAILABLE else object):
    """
    ROS 2 subscriber node that receives topic messages from ROS 2 nodes,
    translates them into GCS dashboard payloads, and queues them thread-safely.
    """

    def __init__(self, message_queue=None, loop: Optional[asyncio.AbstractEventLoop] = None, grid_mgr=None):
        if ROS2_AVAILABLE:
            super().__init__("gcs_ros2_bridge")
        self.message_queue = message_queue
        self.loop = loop
        self.grid_mgr = grid_mgr

        # Cached state
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_z = 0.0
        self.latest_heading = 0.0
        self.latest_velocity = 0.0
        self.latest_battery_pct = 100.0
        self.latest_battery_v = 16.8
        self.latest_armed = True
        self.latest_flight_mode = "GUIDED"
        self.planned_path: List[List[float]] = []
        self.mission_start_time = time.time()

        if ROS2_AVAILABLE:
            self._setup_subscribers()

    def _setup_subscribers(self):
        """Sets up ROS 2 topic subscriptions with appropriate QoS policies."""
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 1. /drone_pose (geometry_msgs/PoseStamped)
        self.create_subscription(PoseStamped, "/drone_pose", self.on_drone_pose, sensor_qos)

        # 2. /map (nav_msgs/OccupancyGrid)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched_qos)

        # 3. /battery_status (sensor_msgs/BatteryState)
        self.create_subscription(BatteryState, "/battery_status", self.on_battery, sensor_qos)

        # 4. /exploration_status (std_msgs/String JSON)
        self.create_subscription(String, "/exploration_status", self.on_exploration_status, reliable_qos)

        # 5. /planned_path (nav_msgs/Path)
        self.create_subscription(Path, "/planned_path", self.on_planned_path, reliable_qos)

        # 6. /fusion_status (std_msgs/String JSON)
        self.create_subscription(String, "/fusion_status", self.on_fusion_status, reliable_qos)

        # 7. /confirmed_survivors (custom nidar_msgs/SurvivorArray)
        try:
            from nidar_msgs.msg import SurvivorArray
            self.create_subscription(SurvivorArray, "/confirmed_survivors", self.on_confirmed_survivors, reliable_qos)
        except ImportError:
            # If nidar_msgs is not on Python path, dynamic import or fallback
            pass

        # 8. /failsafe/status (std_msgs/String JSON)
        self.create_subscription(String, "/failsafe/status", self.on_failsafe_status, reliable_qos)

        # 9. /map_regions (std_msgs/String JSON from corridor_classifier)
        self.create_subscription(String, "/map_regions", self.on_map_regions, reliable_qos)

        # 10. /mavros_bridge/state (std_msgs/String JSON from mavros_bridge)
        self.create_subscription(String, "/mavros_bridge/state", self.on_mavros_bridge_state, reliable_qos)

        # 11. /failsafe/planned_path (nav_msgs/Path)
        self.create_subscription(Path, "/failsafe/planned_path", self.on_failsafe_path, reliable_qos)

    # -------------------------------------------------------------------------
    # Converters & Callbacks
    # -------------------------------------------------------------------------

    def convert_drone_pose(self, msg) -> Dict[str, Any]:
        """Translates PoseStamped message to telemetry JSON payload."""
        pos = msg.pose.position
        ori = msg.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

        deg_heading = (math.degrees(yaw) + 360.0) % 360.0
        deg_roll = math.degrees(roll)
        deg_pitch = math.degrees(pitch)

        self.latest_x = round(pos.x, 2)
        self.latest_y = round(pos.y, 2)
        self.latest_z = round(pos.z, 2)
        self.latest_heading = round(deg_heading, 1)

        grid_cell = "A1"
        if self.grid_mgr:
            grid_cell = self.grid_mgr.metric_to_grid(self.latest_x, self.latest_y)

        return {
            "type": "telemetry",
            "timestamp": time.time(),
            "x": self.latest_x,
            "y": self.latest_y,
            "z": self.latest_z,
            "altitude": self.latest_z,
            "heading": self.latest_heading,
            "yaw": self.latest_heading,
            "roll": round(deg_roll, 1),
            "pitch": round(deg_pitch, 1),
            "velocity": self.latest_velocity,
            "battery_v": round(self.latest_battery_v, 2),
            "battery_pct": round(self.latest_battery_pct, 1),
            "battery_percentage": round(self.latest_battery_pct, 1),
            "armed": self.latest_armed,
            "flight_mode": self.latest_flight_mode,
            "grid_cell": grid_cell,
        }

    def on_drone_pose(self, msg):
        payload = self.convert_drone_pose(msg)
        self._dispatch_normal(payload)

    def convert_map(self, msg) -> Dict[str, Any]:
        """Translates OccupancyGrid message to map_update JSON payload."""
        info = msg.info
        res = float(info.resolution)
        width = int(info.width)
        height = int(info.height)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)

        occupied_cells = []
        free_cells = []

        data = list(msg.data)
        # Extract wall and free space coordinates
        free_step = max(1, int(0.4 / res))  # Subsample free space for JSON efficiency
        for r in range(height):
            row_idx = r * width
            for c in range(width):
                val = data[row_idx + c]
                if val >= 50:
                    occupied_cells.append([round(origin_x + c * res, 2), round(origin_y + r * res, 2)])
                elif val == 0 and (r % free_step == 0) and (c % free_step == 0):
                    free_cells.append([round(origin_x + c * res, 2), round(origin_y + r * res, 2)])

        return {
            "type": "map_update",
            "timestamp": time.time(),
            "resolution": res,
            "width": width,
            "height": height,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "data": data,
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "path_overlay": self.planned_path,
            "is_simulation": False,
            "slam_mode": "REALTIME"
        }

    def on_map(self, msg):
        payload = self.convert_map(msg)
        self._dispatch_normal(payload)

    def convert_confirmed_survivors(self, msg) -> List[Dict[str, Any]]:
        """Translates SurvivorArray message to list of survivor_detected payloads."""
        payloads = []
        detections = getattr(msg, "detections", [])
        for det in detections:
            surv_id_int = getattr(det, "survivor_id", 1)
            surv_id = f"S{max(1, surv_id_int):02d}"
            pos = getattr(det, "position", None)
            x = round(float(getattr(pos, "x", 0.0)), 2) if pos else 0.0
            y = round(float(getattr(pos, "y", 0.0)), 2) if pos else 0.0
            conf = round(float(getattr(det, "confidence", 0.9)), 2)
            src = str(getattr(det, "detection_source", "FUSED")).upper()

            grid_cell = getattr(det, "grid_box", None)
            if not grid_cell and self.grid_mgr:
                grid_cell = self.grid_mgr.metric_to_grid(x, y)
            grid_cell = grid_cell or "A1"

            payloads.append({
                "type": "survivor_detected",
                "id": surv_id,
                "local_x": x,
                "local_y": y,
                "x": x,
                "y": y,
                "grid_cell": grid_cell,
                "confidence": conf,
                "source": src,
                "detection_source": src,
                "status": "CONFIRMED",
                "timestamp": time.strftime("%H:%M:%S")
            })
        return payloads

    def on_confirmed_survivors(self, msg):
        for payload in self.convert_confirmed_survivors(msg):
            self._dispatch_high(payload)

    def convert_battery(self, msg) -> Dict[str, Any]:
        """Extracts battery voltage and percentage."""
        v = float(getattr(msg, "voltage", 16.8))
        pct = float(getattr(msg, "percentage", 1.0))
        # Handle 0.0-1.0 scale vs 0-100 scale
        if pct <= 1.0 and pct > 0.0:
            pct *= 100.0

        self.latest_battery_v = v
        self.latest_battery_pct = pct
        return {"voltage": v, "percentage": pct}

    def on_battery(self, msg):
        self.convert_battery(msg)

    def convert_exploration_status(self, msg) -> Dict[str, Any]:
        """Translates exploration JSON string into mission_status payload."""
        data_str = getattr(msg, "data", "{}")
        try:
            status = json.loads(data_str)
        except Exception:
            status = {}

        raw_state = str(status.get("state", "EXPLORING")).upper()
        # Normalize exploration state names to match dashboard badge
        state_map = {
            "WAITING_FOR_MAP": "READY",
            "WAITING_FOR_POSE": "READY",
            "EXPLORING": "EXPLORING",
            "EXITING_BATTERY_CRITICAL": "EXITING",
            "MISSION_COMPLETE": "MISSION_COMPLETE",
        }
        gcs_state = state_map.get(raw_state, raw_state)

        elapsed = time.time() - self.mission_start_time
        mins, secs = int(elapsed // 60), int(elapsed % 60)
        time_str = f"{mins:02d}:{secs:02d}"

        cov = float(status.get("coverage_pct", 0.0)) / 100.0
        survivor_count = int(status.get("survivors_found", 0))

        return {
            "type": "mission_status",
            "state": gcs_state,
            "elapsed_time": time_str,
            "elapsed_timer": time_str,
            "progress": round(cov, 2),
            "survivor_count": survivor_count,
            "armed": self.latest_armed,
            "is_simulation": False,
            "slam_mode": "REALTIME"
        }

    def on_exploration_status(self, msg):
        payload = self.convert_exploration_status(msg)
        self._dispatch_normal(payload)

    def convert_planned_path(self, msg) -> List[List[float]]:
        """Extracts ordered waypoint coordinates [[x, y], ...] from nav_msgs/Path."""
        poses = getattr(msg, "poses", [])
        pts = []
        for p in poses:
            pos = getattr(p, "pose", None)
            if pos:
                pt = getattr(pos, "position", None)
                if pt:
                    pts.append([round(float(pt.x), 2), round(float(pt.y), 2)])
        self.planned_path = pts
        return pts

    def on_planned_path(self, msg):
        self.convert_planned_path(msg)

    def convert_fusion_status(self, msg) -> Dict[str, Any]:
        """Translates fusion JSON string into event_log payload."""
        data_str = getattr(msg, "data", "{}")
        try:
            status = json.loads(data_str)
        except Exception:
            status = {}

        vis = status.get("visibility_state", "NORMAL")
        det = status.get("active_detector", "Both")
        brightness = status.get("brightness", 0.0)

        msg_str = f"Visibility: {vis} | Detector: {det} | Scene Brightness: {brightness:.1f}"
        return {
            "type": "event_log",
            "timestamp": time.strftime("%H:%M:%S"),
            "level": "INFO" if vis == "NORMAL" else "WARNING",
            "category": "FUSION",
            "message": msg_str
        }

    def on_fusion_status(self, msg):
        payload = self.convert_fusion_status(msg)
        self._dispatch_high(payload)

    def convert_failsafe_status(self, msg) -> Dict[str, Any]:
        """Translates failsafe JSON string into failsafe_status payload."""
        data_str = getattr(msg, "data", "{}")
        try:
            status = json.loads(data_str)
        except Exception:
            status = {}
        return {
            "type": "failsafe_status",
            "timestamp": time.time(),
            "state": status.get("state", "SAFE"),
            "reason": status.get("reason", "NONE"),
            "action": status.get("action", "NONE"),
            "active": status.get("active", False),
            "details": status
        }

    def on_failsafe_status(self, msg):
        payload = self.convert_failsafe_status(msg)
        self._dispatch_high(payload)

    def convert_map_regions(self, msg) -> Dict[str, Any]:
        """Translates corridor classifier JSON into map_regions payload."""
        data_str = getattr(msg, "data", "{}")
        try:
            regions_data = json.loads(data_str)
        except Exception:
            regions_data = {}
        return {
            "type": "map_regions",
            "timestamp": time.time(),
            "regions": regions_data.get("regions", []),
            "total_regions": regions_data.get("total_regions", 0),
            "corridors_count": regions_data.get("corridors_count", 0),
            "rooms_count": regions_data.get("rooms_count", 0),
            "junctions_count": regions_data.get("junctions_count", 0),
        }

    def on_map_regions(self, msg):
        payload = self.convert_map_regions(msg)
        self._dispatch_normal(payload)

    def convert_mavros_bridge_state(self, msg) -> Dict[str, Any]:
        """Translates mavros_bridge state JSON into fcu_state payload."""
        data_str = getattr(msg, "data", "{}")
        try:
            state = json.loads(data_str)
        except Exception:
            state = {}
        self.latest_armed = bool(state.get("armed", self.latest_armed))
        self.latest_flight_mode = str(state.get("mode", self.latest_flight_mode))
        return {
            "type": "fcu_state",
            "timestamp": time.time(),
            "connected": state.get("connected", False),
            "armed": self.latest_armed,
            "guided": state.get("guided", False),
            "mode": self.latest_flight_mode,
            "target_altitude": state.get("target_altitude", 2.5),
            "health": state.get("health", "OK")
        }

    def on_mavros_bridge_state(self, msg):
        payload = self.convert_mavros_bridge_state(msg)
        self._dispatch_normal(payload)

    def on_failsafe_path(self, msg):
        pts = self.convert_planned_path(msg)
        payload = {
            "type": "failsafe_path_update",
            "timestamp": time.time(),
            "path": pts
        }
        self._dispatch_high(payload)

    # -------------------------------------------------------------------------
    # Thread-Safe Queue Dispatch
    # -------------------------------------------------------------------------

    def _dispatch_high(self, payload: Dict[str, Any]):
        if not self.message_queue:
            return
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.message_queue.put_high(payload), self.loop)
        else:
            try:
                self.message_queue.high_queue.put_nowait(payload)
            except Exception:
                pass

    def _dispatch_normal(self, payload: Dict[str, Any]):
        if not self.message_queue:
            return
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self.message_queue.put_normal(payload), self.loop)
        else:
            try:
                self.message_queue.normal_queue.put_nowait(payload)
            except Exception:
                pass
