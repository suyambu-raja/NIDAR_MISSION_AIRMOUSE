"""
Main Window for NIDAR AirMouse Ground Control Station (GCS).
Orchestrates Header, Video Feed, 2D SLAM Map, Telemetry, Survivor Cards, and Bottom Mission Controls.
"""
from typing import Optional
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSplitter,
    QMessageBox,
)
from config.config_manager import ConfigManager, GCSConfig
from mapping.grid_manager import GridManager
from mission.mission_manager import MissionManager
from mission.safety_manager import SafetyManager
from logging_system.mission_logger import MissionLogger
from communication.connection_manager import ConnectionManager
from communication.message_models import MissionEvent, TelemetryData, SurvivorDetection
from ui.styles import DARK_THEME_QSS
from ui.header_widget import HeaderWidget
from ui.camera_widget import CameraWidget
from ui.map_widget import MapWidget
from ui.telemetry_widget import TelemetryWidget
from ui.survivor_widget import SurvivorWidget
from ui.bottom_widget import BottomWidget
from ui.log_dialog import LogDialog


class MainWindow(QMainWindow):
    """
    Root Desktop Application Window for NIDAR AirMouse GCS.
    """

    def __init__(self, config_manager: Optional[ConfigManager] = None):
        super().__init__()
        self.setWindowTitle("NIDAR AirMouse GCS - Autonomous Indoor Search & Mapping")
        self.resize(1500, 920)
        self.setMinimumSize(1200, 750)

        # 1. Initialize Core Architecture
        self.config_mgr = config_manager or ConfigManager()
        self.config: GCSConfig = self.config_mgr.config

        self.grid_mgr = GridManager(
            grid_width_meters=self.config.map.grid_width_meters,
            grid_height_meters=self.config.map.grid_height_meters,
            cell_size_meters=self.config.map.cell_size_meters,
            origin_x=self.config.map.grid_origin_x,
            origin_y=self.config.map.grid_origin_y,
        )

        self.mission_mgr = MissionManager(self)
        self.safety_mgr = SafetyManager(self.config.safety, self)
        self.logger = MissionLogger(parent=self)

        self.conn_mgr = ConnectionManager(
            config=self.config,
            grid_manager=self.grid_mgr,
            mission_manager=self.mission_mgr,
            parent=self,
        )

        # 2. Build UI Hierarchy
        self._init_ui()

        # 3. Connect All Signal Routes
        self._connect_signals()

        # Apply Global Dark Stylesheet
        self.setStyleSheet(DARK_THEME_QSS)

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(6)

        # 1. Top Header Bar
        self.header_widget = HeaderWidget(self)
        root_layout.addWidget(self.header_widget)

        # 2. Center Workspace (Horizontal Splitter)
        workspace_splitter = QSplitter(Qt.Horizontal)
        workspace_splitter.setHandleWidth(4)

        # Left Column: Camera Feed
        self.camera_widget = CameraWidget(self)
        self.camera_widget.setMinimumWidth(320)
        workspace_splitter.addWidget(self.camera_widget)

        # Center Column: 2D SLAM & Occupancy Grid Map
        self.map_widget = MapWidget(self.grid_mgr, self)
        self.map_widget.setMinimumWidth(480)
        workspace_splitter.addWidget(self.map_widget)

        # Right Column: Telemetry Dashboard & Survivor Cards (Vertical Splitter)
        right_splitter = QSplitter(Qt.Vertical)
        right_splitter.setHandleWidth(4)

        self.telemetry_widget = TelemetryWidget(self)
        self.survivor_widget = SurvivorWidget(self)

        right_splitter.addWidget(self.telemetry_widget)
        right_splitter.addWidget(self.survivor_widget)
        right_splitter.setStretchFactor(0, 3)
        right_splitter.setStretchFactor(1, 2)

        workspace_splitter.addWidget(right_splitter)

        # Set initial horizontal column proportions (25% Cam, 50% Map, 25% Telemetry/Survivors)
        workspace_splitter.setStretchFactor(0, 3)
        workspace_splitter.setStretchFactor(1, 5)
        workspace_splitter.setStretchFactor(2, 3)

        root_layout.addWidget(workspace_splitter, 1)

        # 3. Bottom Control & Event Terminal Panel
        self.bottom_widget = BottomWidget(self)
        root_layout.addWidget(self.bottom_widget)

    def _connect_signals(self):
        """Wires signals across data providers, safety systems, and GUI widgets."""
        # Header controls
        self.header_widget.connection_toggle_requested.connect(self.conn_mgr.toggle_connection)
        self.header_widget.mode_changed.connect(self.conn_mgr.set_mode)

        # Connection Manager -> UI & Safety
        self.conn_mgr.connection_status_changed.connect(self.header_widget.update_connection_status)
        self.conn_mgr.connection_status_changed.connect(
            lambda s: self.camera_widget.set_stream_connected(s.video_connected)
        )
        self.conn_mgr.telemetry_updated.connect(self._on_telemetry_packet)
        self.conn_mgr.map_updated.connect(self.map_widget.update_map)
        self.conn_mgr.map_updated.connect(lambda m: self.safety_mgr.update_map_received())
        self.conn_mgr.video_frame_updated.connect(self.camera_widget.update_frame)
        self.conn_mgr.video_frame_updated.connect(lambda f: self.safety_mgr.update_video_received())
        self.conn_mgr.survivor_detected.connect(self._on_survivor_detection)
        self.conn_mgr.event_logged.connect(self._on_event_logged)

        # Mission Manager -> UI & Safety
        self.mission_mgr.state_changed.connect(self._on_mission_state_changed)
        self.mission_mgr.timer_updated.connect(lambda t_str, sec: self.header_widget.update_timer(t_str))
        self.mission_mgr.progress_updated.connect(self.bottom_widget.update_progress)
        self.mission_mgr.event_logged.connect(self._on_event_logged)

        # Safety Manager -> Mission & UI
        self.safety_mgr.safety_alert.connect(self._on_safety_alert)
        self.safety_mgr.emergency_abort_requested.connect(self._on_emergency_abort_triggered)

        # Bottom Widget Commands
        self.bottom_widget.start_mission_requested.connect(self.mission_mgr.start_mission)
        self.bottom_widget.abort_mission_requested.connect(
            lambda: self.conn_mgr.send_emergency_abort("Manual Operator Abort")
        )
        self.bottom_widget.reset_mission_requested.connect(self._on_reset_mission)
        self.bottom_widget.view_logs_requested.connect(self._on_open_logs)

        # Camera Widget Thermal Toggle
        self.camera_widget.thermal_mode_toggled.connect(
            self.conn_mgr.simulator.video_provider.set_thermal_mode
        )

    def _on_telemetry_packet(self, telem: TelemetryData):
        self.header_widget.update_telemetry(telem)
        self.telemetry_widget.update_telemetry(telem)
        self.map_widget.update_telemetry(telem)
        self.safety_mgr.update_telemetry(telem)
        self.logger.log_telemetry(telem)

    def _on_survivor_detection(self, detection: SurvivorDetection):
        self.survivor_widget.add_survivor(detection)
        self.map_widget.add_survivor(detection)
        self.logger.log_survivor(detection)

    def _on_mission_state_changed(self, old_state: str, new_state: str):
        self.header_widget.update_mission_state(new_state)
        self.bottom_widget.update_mission_state(new_state)
        self.safety_mgr.set_mission_state(self.mission_mgr.current_state)

    def _on_safety_alert(self, level: str, category: str, message: str):
        evt = MissionEvent(level=level, category=category, message=message)
        self._on_event_logged(evt)

    def _on_emergency_abort_triggered(self, reason: str):
        self.conn_mgr.send_emergency_abort(reason)

    def _on_event_logged(self, event: MissionEvent):
        self.bottom_widget.append_event(event)
        self.logger.log_event(event)

    def _on_reset_mission(self):
        reply = QMessageBox.question(
            self,
            "Reset Mission",
            "Reset mission state and map back to initial staging?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.conn_mgr.simulator.reset_sim()
            self.map_widget.reset()
            self.survivor_widget.reset()
            self.mission_mgr.reset_mission()

    def _on_open_logs(self):
        dlg = LogDialog(self.logger, self)
        dlg.exec()

    def closeEvent(self, event):
        """Safely terminates provider background threads before exiting."""
        self.conn_mgr.disconnect_link()
        event.accept()
