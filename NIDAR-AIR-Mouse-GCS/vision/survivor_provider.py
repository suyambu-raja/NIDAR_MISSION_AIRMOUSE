"""
Survivor Detection Providers for NIDAR AirMouse GCS.
Provides interfaces for receiving autonomous person/survivor detection events from OAK-D / thermal cameras.
"""
from PySide6.QtCore import QObject, Signal
from communication.message_models import SurvivorDetection


class BaseSurvivorProvider(QObject):
    """
    Abstract interface for receiving survivor localization messages.
    Emits `survivor_detected` signal whenever an autonomous target is localized.
    """
    survivor_detected = Signal(object)  # Emits SurvivorDetection

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_running = False

    def start(self):
        self._is_running = True

    def stop(self):
        self._is_running = False

    def is_running(self) -> bool:
        return self._is_running
