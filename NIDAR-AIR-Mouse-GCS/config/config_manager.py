"""
Configuration Manager for NIDAR AirMouse Ground Control Station (GCS).
Loads, validates, and provides structured access to configuration settings.
"""
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
from typing import Any, Dict, Optional


@dataclass
class GeneralConfig:
    app_name: str = "NIDAR AirMouse GCS"
    callsign: str = "AIRMOUSE-ALPHA"
    version: str = "1.0.0"
    mode: str = "SIMULATION"  # "SIMULATION" or "HARDWARE"


@dataclass
class CommConfig:
    mavlink_connection: str = "udp:127.0.0.1:14550"
    mavlink_baud: int = 57600
    telemetry_rate_hz: int = 10
    companion_pi_ip: str = "192.168.1.50"
    companion_pi_port: int = 5555
    auto_reconnect: bool = True


@dataclass
class MapConfig:
    grid_width_meters: float = 20.0
    grid_height_meters: float = 20.0
    cell_size_meters: float = 2.5
    grid_origin_x: float = 0.0
    grid_origin_y: float = 0.0
    resolution_meters_per_pixel: float = 0.05
    grid_cols: int = 8
    grid_rows: int = 8


@dataclass
class SafetyConfig:
    battery_warning_percent: float = 20.0
    battery_critical_percent: float = 10.0
    telemetry_timeout_seconds: float = 2.0
    map_timeout_seconds: float = 3.0
    video_timeout_seconds: float = 2.0
    max_mission_time_seconds: int = 600


@dataclass
class VideoConfig:
    source: str = "simulated"  # "simulated", "0" (USB webcam), or RTSP/UDP URL
    fps: int = 30
    width: int = 640
    height: int = 480


@dataclass
class SimulationConfig:
    drone_speed_mps: float = 0.7
    battery_drain_rate_percent_per_min: float = 3.5
    noise_enabled: bool = True


@dataclass
class GCSConfig:
    general: GeneralConfig = field(default_factory=GeneralConfig)
    communication: CommConfig = field(default_factory=CommConfig)
    map: MapConfig = field(default_factory=MapConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    video: VideoConfig = field(default_factory=VideoConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)


class ConfigManager:
    """Manages loading, validating, and saving the GCS configuration file."""

    DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config.json"

    def __init__(self, config_path: Optional[str] = None):
        self.config_path = Path(config_path) if config_path else self.DEFAULT_CONFIG_PATH
        self.config: GCSConfig = self.load_config()

    def load_config(self) -> GCSConfig:
        """Loads configuration from JSON file or returns default if not found."""
        if not self.config_path.exists():
            default_cfg = GCSConfig()
            self.save_config(default_cfg)
            return default_cfg

        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            return GCSConfig(
                general=GeneralConfig(**data.get("general", {})),
                communication=CommConfig(**data.get("communication", {})),
                map=MapConfig(**data.get("map", {})),
                safety=SafetyConfig(**data.get("safety", {})),
                video=VideoConfig(**data.get("video", {})),
                simulation=SimulationConfig(**data.get("simulation", {})),
            )
        except Exception as e:
            print(f"[ConfigManager] Warning: Error parsing config file ({e}). Using defaults.")
            return GCSConfig()

    def save_config(self, cfg: Optional[GCSConfig] = None) -> bool:
        """Saves current configuration to JSON file."""
        if cfg:
            self.config = cfg

        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "general": self.config.general.__dict__,
                "communication": self.config.communication.__dict__,
                "map": self.config.map.__dict__,
                "safety": self.config.safety.__dict__,
                "video": self.config.video.__dict__,
                "simulation": self.config.simulation.__dict__,
            }
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            print(f"[ConfigManager] Error saving config: {e}")
            return False
