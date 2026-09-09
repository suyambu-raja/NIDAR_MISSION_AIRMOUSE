"""
Unified Simulation Engine for NIDAR AirMouse GCS.
Orchestrates simulated telemetry, dynamic SLAM mapping, synthetic video feed, and autonomous survivor detections.
"""
from typing import Optional
from PySide6.QtCore import QObject, Signal, QTimer
from simulation.simulated_telemetry import SimulatedTelemetryProvider
from simulation.simulated_map import SimulatedMapProvider
from simulation.simulated_survivors import SimulatedSurvivorProvider
from simulation.simulated_video import SimulatedVideoProvider
from mapping.grid_manager import GridManager
from mission.mission_manager import MissionManager
from mission.mission_state import MissionState
from communication.message_models import TelemetryData, SurvivorDetection, MapData


class SimulatorEngine(QObject):
    """
    Central simulation coordinator.
    Implements the complete interactive search and mapping mission scenario.
    """
    telemetry_updated = Signal(object)
    map_updated = Signal(object)
    video_frame_updated = Signal(object)
    survivor_detected = Signal(object)
    connection_changed = Signal(bool)

    def __init__(
        self,
        grid_manager: GridManager,
        mission_manager: MissionManager,
        parent=None,
    ):
        super().__init__(parent)
        self.grid_manager = grid_manager
        self.mission_manager = mission_manager

        # Initialize Sub-Providers
        self.telemetry_provider = SimulatedTelemetryProvider(parent=self)
        self.map_provider = SimulatedMapProvider(
            width_meters=grid_manager.grid_width_meters,
            height_meters=grid_manager.grid_height_meters,
            parent=self,
        )
        self.survivor_provider = SimulatedSurvivorProvider(
            grid_manager=self.grid_manager,
            parent=self,
        )
        self.video_provider = SimulatedVideoProvider(parent=self)

        self._is_connected = False
        self._survivor_alert_timer = QTimer(self)
        self._survivor_alert_timer.setSingleShot(True)
        self._survivor_alert_timer.timeout.connect(self._resume_exploring_after_detection)

        # Wire internal signals
        self.telemetry_provider.telemetry_updated.connect(self._on_telemetry_step)
        self.map_provider.map_updated.connect(self.map_updated.emit)
        self.video_provider.frame_received.connect(self.video_frame_updated.emit)
        self.survivor_provider.survivor_detected.connect(self._on_survivor_detected)

        # Listen to external mission state changes
        self.mission_manager.state_changed.connect(self._on_mission_state_changed)

    def connect_sim(self):
        """Connects simulated drone link."""
        if self._is_connected:
            return
        self._is_connected = True
        self.telemetry_provider.start()
        self.map_provider.start()
        self.survivor_provider.start()
        self.video_provider.start()
        self.connection_changed.emit(True)

        if self.mission_manager.current_state == MissionState.IDLE:
            self.mission_manager.transition_to(MissionState.SYSTEM_CHECK, "Connected to simulated drone")
            # Automatically advance to READY after 1 second of self-checks
            QTimer.singleShot(1000, lambda: self.mission_manager.transition_to(
                MissionState.READY, "Pre-flight checks verified: Sensors & Estimator healthy"
            ))

    def disconnect_sim(self):
        """Disconnects simulated drone link."""
        if not self._is_connected:
            return
        self._is_connected = False
        self.telemetry_provider.stop()
        self.map_provider.stop()
        self.survivor_provider.stop()
        self.video_provider.stop()
        self.connection_changed.emit(False)

    def is_connected(self) -> bool:
        return self._is_connected

    def reset_sim(self):
        """Resets all simulation providers to initial state."""
        self.telemetry_provider.reset()
        self.map_provider.reset()
        self.survivor_provider.reset()
        self.mission_manager.reset_mission()

    def _on_telemetry_step(self, telem: TelemetryData):
        """Synchronizes map, vision, and survivors with drone kinematics."""
        # 1. Update SLAM map discovery around drone pose
        self.map_provider.update_drone_pose(telem.x_m, telem.y_m, telem.yaw_deg)

        # 2. Check for nearby survivors
        new_survivors = self.survivor_provider.update_drone_pose(telem.x_m, telem.y_m)
        target_in_view = self.survivor_provider.get_active_survivor_in_view(telem.x_m, telem.y_m)

        # 3. Update synthetic video viewpoint & bounding boxes
        self.video_provider.set_drone_state(
            telem.x_m, telem.y_m, telem.yaw_deg, telem.altitude_m, target_in_view
        )

        # 4. Check for state transitions (e.g. TAKEOFF -> ENTERING -> EXPLORING)
        curr_state = self.mission_manager.current_state
        if curr_state == MissionState.TAKEOFF and telem.altitude_m >= 1.1:
            self.mission_manager.transition_to(MissionState.ENTERING, "Altitude achieved: Entering search maze")
        elif curr_state == MissionState.ENTERING and telem.x_m >= 2.0:
            self.mission_manager.transition_to(MissionState.EXPLORING, "Inside corridor: Beginning autonomous exploration")
        elif curr_state == MissionState.EXPLORING and self.telemetry_provider.is_at_exit():
            self.mission_manager.transition_to(MissionState.EXITING, "Reached arena exit corridor")
        elif curr_state == MissionState.EXITING and telem.altitude_m <= 0.1:
            self.mission_manager.transition_to(MissionState.MISSION_COMPLETE, "Safely landed at recovery zone")

        # Forward telemetry to GCS UI
        self.telemetry_updated.emit(telem)

    def _on_survivor_detected(self, detection: SurvivorDetection):
        """Handles newly discovered survivor."""
        self.mission_manager.increment_survivor_count()
        if self.mission_manager.current_state == MissionState.EXPLORING:
            self.mission_manager.transition_to(
                MissionState.SURVIVOR_DETECTED,
                f"Autonomous detection: {detection.survivor_id} at Grid {detection.grid_cell}"
            )
            # Hold detection state for 3 seconds, then resume exploring
            self._survivor_alert_timer.start(3000)

        self.survivor_detected.emit(detection)

    def _resume_exploring_after_detection(self):
        """Resumes exploration state after survivor confirmation delay."""
        if self.mission_manager.current_state == MissionState.SURVIVOR_DETECTED:
            self.mission_manager.transition_to(
                MissionState.EXPLORING, "Target registered. Continuing search pattern"
            )

    def _on_mission_state_changed(self, old_state: str, new_state: str):
        """Propagates mission state changes down to telemetry generator."""
        state_enum = MissionState(new_state)
        self.telemetry_provider.set_mission_state(state_enum)
