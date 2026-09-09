"""
Message Models & Data Structures for NIDAR AirMouse Ground Control Station (GCS).
Contains strongly-typed dataclasses for Telemetry, Survivors, Maps, and Events.
"""
from dataclasses import dataclass, field
import time
from typing import List, Optional, Tuple, Any


@dataclass
class TelemetryData:
    """Represents real-time telemetry from Pixhawk / companion computer."""
    timestamp: float = field(default_factory=time.time)
    armed: bool = False
    flight_mode: str = "DISARMED"
    
    # Motion & Position (Metric Local Frame)
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    altitude_m: float = 0.0
    ground_speed_mps: float = 0.0
    
    # Attitude (Degrees)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    heading_deg: float = 0.0
    
    # Power System
    battery_voltage_v: float = 16.8  # 4S LiPo fully charged ~ 16.8V
    battery_current_a: float = 0.0
    battery_percentage: float = 100.0
    
    # Sensor & Link Health Flags
    ekf_healthy: bool = True
    optical_flow_healthy: bool = True
    rangefinder_healthy: bool = True
    lidar_healthy: bool = True
    oak_d_healthy: bool = True
    thermal_healthy: bool = True
    mavlink_connected: bool = False
    video_connected: bool = False


@dataclass
class SurvivorDetection:
    """Represents an autonomously identified human survivor target."""
    survivor_id: str                   # e.g., "S1", "S2"
    x: float                           # Local Cartesian X (meters)
    y: float                           # Local Cartesian Y (meters)
    grid_cell: str                     # e.g., "C4", "F7"
    confidence: float                  # 0.0 to 1.0 (e.g., 0.94 for 94%)
    timestamp: float = field(default_factory=time.time)
    status: str = "DETECTED"           # "DETECTED", "CONFIRMED", "VERIFIED"
    source: str = "AI_OAKD_RGB"        # "AI_OAKD_RGB", "FLIR_THERMAL", "FUSED_AI", "SIMULATION"
    bbox: Optional[Tuple[int, int, int, int]] = None  # (x1, y1, x2, y2) in camera frame


@dataclass
class MapData:
    """Represents a 2D Occupancy Grid Map (ROS / SLAM format)."""
    width: int = 400                   # Grid cell count along X
    height: int = 400                  # Grid cell count along Y
    resolution: float = 0.05           # Meters per cell (5cm default)
    origin_x: float = 0.0              # Metric X coordinate of bottom-left cell
    origin_y: float = 0.0              # Metric Y coordinate of bottom-left cell
    timestamp: float = field(default_factory=time.time)
    # Cell values: -1 = Unknown (unexplored), 0 = Free space, 100 = Occupied (wall/obstacle)
    grid: Any = None                   # numpy.ndarray or 2D list of int8


@dataclass
class MissionEvent:
    """Represents a system or mission event for live display and logging."""
    timestamp: float = field(default_factory=time.time)
    level: str = "INFO"                # "INFO", "WARNING", "ERROR", "CRITICAL", "SUCCESS"
    category: str = "SYSTEM"           # "SYSTEM", "MISSION", "SURVIVOR", "SAFETY", "COMM"
    message: str = ""


@dataclass
class ConnectionStatus:
    """Represents the overall connectivity state of the GCS."""
    is_connected: bool = False
    mavlink_connected: bool = False
    companion_connected: bool = False
    video_connected: bool = False
    slam_connected: bool = False
    last_heartbeat_time: float = 0.0
    rx_rate_bytes_per_sec: float = 0.0
