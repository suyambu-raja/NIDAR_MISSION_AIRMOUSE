"""UI package for NIDAR AirMouse GCS."""
from .styles import DARK_THEME_QSS, COLOR_PALETTE
from .header_widget import HeaderWidget
from .camera_widget import CameraWidget
from .map_widget import MapWidget
from .telemetry_widget import TelemetryWidget
from .survivor_widget import SurvivorWidget
from .bottom_widget import BottomWidget
from .log_dialog import LogDialog
from .main_window import MainWindow

__all__ = [
    "DARK_THEME_QSS",
    "COLOR_PALETTE",
    "HeaderWidget",
    "CameraWidget",
    "MapWidget",
    "TelemetryWidget",
    "SurvivorWidget",
    "BottomWidget",
    "LogDialog",
    "MainWindow",
]
