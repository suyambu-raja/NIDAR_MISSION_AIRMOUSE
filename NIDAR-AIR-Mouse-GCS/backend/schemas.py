"""
Structured Message Schemas for NIDAR AirMouse GCS.
Defines standard JSON payload schemas across Python Backend and HTML5/JS Frontend.
"""
from dataclasses import dataclass, field, asdict
from enum import Enum
import time
from typing import Optional, List, Dict, Any, Tuple


class MessageType(str, Enum):
    TELEMETRY = "telemetry"
    MAP_UPDATE = "map_update"
    SURVIVOR_DETECTED = "survivor_detected"
    SURVIVOR_UPDATE = "survivor_update"
    CAMERA_FRAME = "camera_frame"
    CAMERA_STATUS = "camera_status"
    MISSION_STATUS = "mission_status"
    RADIO_STATUS = "radio_status"
    SYSTEM_HEALTH = "system_health"
    EVENT_LOG = "event_log"
    COMMAND = "command"
    HEARTBEAT = "heartbeat"


@dataclass
class TelemetryPayload:
    timestamp: float = field(default_factory=time.time)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    altitude: float = 0.0
    heading: float = 0.0
    velocity: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    battery_v: float = 16.8
    battery_pct: float = 100.0
    battery_a: float = 0.0
    flight_mode: str = "DISARMED"
    armed: bool = False
    grid_cell: str = "A1"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.TELEMETRY.value
        return d


@dataclass
class MapUpdatePayload:
    timestamp: float = field(default_factory=time.time)
    width: int = 400
    height: int = 400
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    # Compressed run-length or array of free/occupied coordinates for fast JSON transfer
    occupied_cells: List[Tuple[float, float]] = field(default_factory=list)
    free_cells: List[Tuple[float, float]] = field(default_factory=list)
    explored_bounds: Dict[str, float] = field(default_factory=lambda: {"min_x": 0, "max_x": 20, "min_y": 0, "max_y": 20})
    is_simulation: bool = True
    slam_mode: str = "SIMULATION"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.MAP_UPDATE.value
        return d


@dataclass
class SurvivorPayload:
    id: str                                  # e.g., "S01", "S02"
    local_x: float                           # Local Cartesian X (m)
    local_y: float                           # Local Cartesian Y (m)
    grid_x: int                              # Grid column (1-8)
    grid_y: int                              # Grid row (1-8)
    grid_cell: str                           # e.g., "B3", "F5"
    confidence: float                        # 0.0 - 1.0 (e.g. 0.95)
    source: str = "THERMAL"                  # "THERMAL", "RGB", "FUSED"
    status: str = "CONFIRMED"                # "DETECTED", "CONFIRMED"
    timestamp: str = field(default_factory=lambda: time.strftime("%H:%M:%S"))
    rgb_evidence_time: Optional[str] = None
    thermal_evidence_time: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.SURVIVOR_DETECTED.value
        return d


@dataclass
class CameraFramePayload:
    camera_id: str                           # "rgb" or "thermal"
    frame_base64: str                        # Base64 encoded JPEG
    timestamp: float = field(default_factory=time.time)
    fps: float = 0.0
    status: str = "LIVE"                     # "LIVE", "SIMULATED", "NO_SIGNAL"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.CAMERA_FRAME.value
        return d


@dataclass
class MissionStatusPayload:
    state: str = "IDLE"
    elapsed_time: str = "00:00"
    progress: float = 0.0
    survivor_count: int = 0
    armed: bool = False
    is_simulation: bool = True
    slam_mode: str = "SIMULATION"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.MISSION_STATUS.value
        return d


@dataclass
class RadioStatusPayload:
    connected: bool = False
    signal_quality: str = "NO SIGNAL"        # "GOOD", "FAIR", "POOR", "NO SIGNAL"
    rssi_dbm: float = 0.0                    # e.g. -68 dBm
    packet_loss_pct: float = 0.0             # e.g. 0.5%
    latency_ms: float = 0.0                  # e.g. 24 ms
    packets_received: int = 0
    last_packet_timestamp: float = field(default_factory=time.time)
    radio_type: str = "SIMULATED_RFD900X"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.RADIO_STATUS.value
        return d


@dataclass
class SystemHealthPayload:
    backend_status: str = "CONNECTED"
    websocket_status: str = "CONNECTED"
    simulator_status: str = "RUNNING"
    rgb_camera_status: str = "SIMULATED"
    thermal_camera_status: str = "SIMULATED"
    mapping_status: str = "ACTIVE"
    ekf_status: str = "HEALTHY"
    optical_flow_status: str = "HEALTHY"
    lidar_status: str = "HEALTHY"
    last_telemetry_time: float = 0.0
    last_map_time: float = 0.0
    last_survivor_time: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.SYSTEM_HEALTH.value
        return d


@dataclass
class EventPayload:
    timestamp: str = field(default_factory=lambda: time.strftime("%H:%M:%S"))
    level: str = "INFO"                      # "INFO", "SUCCESS", "WARNING", "ERROR", "CRITICAL"
    category: str = "SYSTEM"                 # "SYSTEM", "MISSION", "SURVIVOR", "RADIO", "SAFETY"
    message: str = ""

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["type"] = MessageType.EVENT_LOG.value
        return d


@dataclass
class CommandPayload:
    action: str                              # "connect", "start_mission", "abort_mission", "reset", "pause", "simulate_survivor"
    params: Dict[str, Any] = field(default_factory=dict)
