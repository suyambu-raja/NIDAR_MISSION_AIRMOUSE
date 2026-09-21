"""
Connection Manager for NIDAR AirMouse GCS.
Unified gateway switching between Simulation Mode and Real Hardware Mode.
Routes telemetry, video, maps, and survivor detections to GCS subsystems.
"""
import time
from typing import Optional
from PySide6.QtCore import QObject, Signal, Slot
from config.config_manager import GCSConfig
from communication.message_models import (
    TelemetryData,
    SurvivorDetection,
    MapData,
    MissionEvent,
    ConnectionStatus,
)
from communication.mavlink_provider import MavlinkProvider
from vision.video_provider import OpenCVVideoProvider
from simulation.simulator_engine import SimulatorEngine
from mapping.grid_manager import GridManager
from mission.mission_manager import MissionManager
from mission.mission_state import MissionState


class ConnectionManager(QObject):
    """
    Central connection router for GCS.
    Decouples UI and safety modules from whether data originates from physics simulation or real hardware.
    """
    telemetry_updated = Signal(object)      # Emits TelemetryData
    map_updated = Signal(object)            # Emits MapData
    video_frame_updated = Signal(object)    # Emits OpenCV frame (numpy.ndarray)
    survivor_detected = Signal(object)      # Emits SurvivorDetection
    connection_status_changed = Signal(object)  # Emits ConnectionStatus
    event_logged = Signal(object)          # Emits MissionEvent

    def __init__(
        self,
        config: GCSConfig,
        grid_manager: GridManager,
        mission_manager: MissionManager,
        parent=None,
    ):
        super().__init__(parent)
        self.config = config
        self.grid_manager = grid_manager
        self.mission_manager = mission_manager

        self.mode = config.general.mode.upper()  # "SIMULATION" or "HARDWARE"

        # 1. Simulation Engine
        self.simulator = SimulatorEngine(
            grid_manager=self.grid_manager,
            mission_manager=self.mission_manager,
            parent=self,
        )

        # 2. Hardware Providers
        self.mavlink_provider = MavlinkProvider(
            connection_string=config.communication.mavlink_connection,
            baud=config.communication.mavlink_baud,
            parent=self,
        )
        self.hardware_video = OpenCVVideoProvider(
            source=config.video.source,
            fps=config.video.fps,
            parent=self,
        )

        # Status Tracker
        self.status = ConnectionStatus()

        # Connect internal signals
        self._wire_simulation_signals()
        self._wire_hardware_signals()

    def _wire_simulation_signals(self):
        self.simulator.telemetry_updated.connect(self._on_telemetry_received)
        self.simulator.map_updated.connect(self.map_updated.emit)
        self.simulator.video_frame_updated.connect(self.video_frame_updated.emit)
        self.simulator.survivor_detected.connect(self.survivor_detected.emit)
        self.simulator.connection_changed.connect(self._on_sim_connection_changed)

    def _wire_hardware_signals(self):
        self.mavlink_provider.telemetry_received.connect(self._on_telemetry_received)
        self.mavlink_provider.connection_changed.connect(self._on_mavlink_connection_changed)
        self.mavlink_provider.status_message.connect(self._on_mavlink_log)
        self.hardware_video.frame_received.connect(self.video_frame_updated.emit)

    def set_mode(self, mode: str):
        """Switches between 'SIMULATION' and 'HARDWARE' mode."""
        mode_upper = mode.upper()
        if mode_upper != self.mode:
            if self.is_connected():
                self.disconnect_link()

            self.mode = mode_upper
            self.config.general.mode = mode_upper
            self._log_event("INFO", "COMM", f"GCS Mode changed to {self.mode}")

    def is_simulation(self) -> bool:
        return self.mode == "SIMULATION"

    def is_connected(self) -> bool:
        return self.status.is_connected

    def connect_link(self):
        """Initiates connection based on current operating mode."""
        self._log_event("INFO", "COMM", f"Initiating {self.mode} connection...")
        if self.is_simulation():
            self.simulator.connect_sim()
        else:
            self.mavlink_provider.start()
            self.hardware_video.start()

    def disconnect_link(self):
        """Closes active connections."""
        self._log_event("INFO", "COMM", f"Disconnecting {self.mode} link...")
        if self.is_simulation():
            self.simulator.disconnect_sim()
        else:
            self.mavlink_provider.stop()
            self.hardware_video.stop()

        self.status.is_connected = False
        self.status.mavlink_connected = False
        self.status.video_connected = False
        self.connection_status_changed.emit(self.status)

    def toggle_connection(self):
        if self.is_connected():
            self.disconnect_link()
        else:
            self.connect_link()

    def send_emergency_abort(self, reason: str = "Emergency Abort"):
        """Sends abort command to simulator or Pixhawk."""
        self.mission_manager.abort_mission(reason)
        if self.is_simulation():
            pass  # Simulator reacts directly to mission_manager state
        else:
            self.mavlink_provider.send_emergency_abort()

    def _on_sim_connection_changed(self, connected: bool):
        self.status.is_connected = connected
        self.status.mavlink_connected = connected
        self.status.video_connected = connected
        self.status.slam_connected = connected
        self.connection_status_changed.emit(self.status)
        if connected:
            self._log_event("SUCCESS", "COMM", "Connected to SIMULATION environment")
        else:
            self._log_event("INFO", "COMM", "Disconnected from SIMULATION environment")

    def _on_mavlink_connection_changed(self, connected: bool):
        self.status.mavlink_connected = connected
        self.status.is_connected = connected
        self.connection_status_changed.emit(self.status)
        if connected:
            self._log_event("SUCCESS", "COMM", "Pixhawk 6C MAVLink Telemetry Connected")
        else:
            self._log_event("WARNING", "COMM", "Pixhawk 6C MAVLink Telemetry Disconnected")

    def _on_telemetry_received(self, telem: TelemetryData):
        self.telemetry_updated.emit(telem)

    def _on_mavlink_log(self, msg: str):
        self._log_event("INFO", "MAVLINK", msg)

    def _log_event(self, level: str, category: str, message: str):
        evt = MissionEvent(
            timestamp=time.time(),
            level=level,
            category=category,
            message=message,
        )
        self.event_logged.emit(evt)
