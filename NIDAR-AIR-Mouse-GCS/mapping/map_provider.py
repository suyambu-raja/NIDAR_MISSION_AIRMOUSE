"""
Abstract Base MapProvider for NIDAR AirMouse GCS.
Defines the Qt signal interface that all map providers (simulated, ROS 2, UDP) must implement.
"""
from PySide6.QtCore import QObject, Signal
from communication.message_models import MapData


class BaseMapProvider(QObject):
    """
    Abstract interface for receiving 2D occupancy grid map updates.
    Emits `map_updated` signal whenever a new map slice or complete map is available.
    """
    map_updated = Signal(object)  # Emits MapData object

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_running = False

    def start(self):
        """Starts receiving or generating map data."""
        self._is_running = True

    def stop(self):
        """Stops receiving or generating map data."""
        self._is_running = False

    def is_running(self) -> bool:
        return self._is_running
