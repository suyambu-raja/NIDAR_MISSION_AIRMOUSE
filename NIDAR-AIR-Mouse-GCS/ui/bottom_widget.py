"""
Bottom Panel Widget for NIDAR AirMouse GCS.
Provides mission state workflow timeline, progress bar, real-time scrolling event log,
and primary mission action controls (START MISSION, EMERGENCY ABORT, RESET, VIEW LOGS).
"""
from datetime import datetime
from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QProgressBar,
    QTextEdit,
    QLabel,
    QGroupBox,
    QMessageBox,
    QFrame,
)
from communication.message_models import MissionEvent
from mission.mission_state import MissionState


class BottomWidget(QGroupBox):
    """
    Bottom control and event monitoring panel.
    """
    start_mission_requested = Signal()
    abort_mission_requested = Signal()
    reset_mission_requested = Signal()
    view_logs_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("MISSION SUPERVISION & SYSTEM EVENT LOG", parent)
        self.setFixedHeight(190)
        self._init_ui()

    def _init_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(8, 14, 8, 8)
        main_layout.setSpacing(10)

        # 1. Left Sub-Panel: State Timeline & Progress & Action Controls
        left_ctrl_box = QVBoxLayout()
        left_ctrl_box.setSpacing(6)

        # Progress Bar
        prog_header = QHBoxLayout()
        lbl_prog_title = QLabel("MISSION PROGRESS:")
        lbl_prog_title.setStyleSheet("font-size: 10px; font-weight: bold; color: #94a3b8;")
        self.lbl_prog_pct = QLabel("0%")
        self.lbl_prog_pct.setStyleSheet("font-size: 10px; font-weight: bold; color: #00e5ff;")
        prog_header.addWidget(lbl_prog_title)
        prog_header.addStretch()
        prog_header.addWidget(self.lbl_prog_pct)
        left_ctrl_box.addLayout(prog_header)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedHeight(12)
        self.progress_bar.setTextVisible(False)
        left_ctrl_box.addWidget(self.progress_bar)

        # Mission Action Buttons
        btn_layout = QGridLayout()
        btn_layout.setSpacing(6)

        self.btn_start = QPushButton("▶ START MISSION")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.setFixedHeight(38)
        self.btn_start.clicked.connect(self.start_mission_requested.emit)

        self.btn_abort = QPushButton("🛑 EMERGENCY ABORT")
        self.btn_abort.setObjectName("btn_abort")
        self.btn_abort.setFixedHeight(38)
        self.btn_abort.clicked.connect(self._on_abort_clicked)

        self.btn_reset = QPushButton("🔄 RESET")
        self.btn_reset.setFixedHeight(28)
        self.btn_reset.clicked.connect(self.reset_mission_requested.emit)

        self.btn_logs = QPushButton("📊 LOG VIEWER")
        self.btn_logs.setFixedHeight(28)
        self.btn_logs.clicked.connect(self.view_logs_requested.emit)

        btn_layout.addWidget(self.btn_start, 0, 0)
        btn_layout.addWidget(self.btn_abort, 0, 1)
        btn_layout.addWidget(self.btn_reset, 1, 0)
        btn_layout.addWidget(self.btn_logs, 1, 1)

        left_ctrl_box.addLayout(btn_layout)
        left_ctrl_box.addStretch()

        left_container = QWidget()
        left_container.setLayout(left_ctrl_box)
        left_container.setFixedWidth(340)
        main_layout.addWidget(left_container)

        # Divider
        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setStyleSheet("color: #282f3a;")
        main_layout.addWidget(div)

        # 2. Right Sub-Panel: Real-time Event Log Terminal
        right_box = QVBoxLayout()
        right_box.setSpacing(4)

        log_hdr = QHBoxLayout()
        lbl_log_title = QLabel("SYSTEM EVENT STREAM")
        lbl_log_title.setStyleSheet("font-size: 10px; font-weight: bold; color: #94a3b8;")
        
        self.btn_clear_log = QPushButton("Clear")
        self.btn_clear_log.setFixedSize(48, 18)
        self.btn_clear_log.setStyleSheet("font-size: 9px; padding: 1px;")
        
        log_hdr.addWidget(lbl_log_title)
        log_hdr.addStretch()
        log_hdr.addWidget(self.btn_clear_log)
        right_box.addLayout(log_hdr)

        self.txt_events = QTextEdit()
        self.txt_events.setReadOnly(True)
        self.txt_events.setStyleSheet("""
            QTextEdit {
                background-color: #0b0d11;
                border: 1px solid #1e242e;
                border-radius: 4px;
                font-family: 'Consolas', monospace;
                font-size: 11px;
                color: #e2e8f0;
            }
        """)
        self.btn_clear_log.clicked.connect(self.txt_events.clear)
        right_box.addWidget(self.txt_events, 1)

        main_layout.addLayout(right_box, 1)

    def _on_abort_clicked(self):
        """Protected emergency abort trigger requiring operator confirmation."""
        reply = QMessageBox.question(
            self,
            "EMERGENCY ABORT CONFIRMATION",
            "Are you sure you want to trigger EMERGENCY ABORT?\n\n"
            "This will immediately halt autonomous navigation, disarm or land the drone, "
            "and terminate the mission.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.abort_mission_requested.emit()

    @Slot(object)
    def append_event(self, event: MissionEvent):
        """Appends a color-coded event log to the scrolling terminal."""
        dt_str = datetime.fromtimestamp(event.timestamp).strftime("%H:%M:%S")

        color_map = {
            "INFO": "#38bdf8",
            "SUCCESS": "#00e676",
            "WARNING": "#ffca28",
            "ERROR": "#f87171",
            "CRITICAL": "#ff1744",
        }
        color = color_map.get(event.level, "#94a3b8")

        html_line = (
            f"<span style='color:#64748b;'>[{dt_str}]</span> "
            f"<b style='color:{color};'>[{event.level}]</b> "
            f"<span style='color:#94a3b8;'>[{event.category}]</span> "
            f"<span style='color:#f1f5f9;'>{event.message}</span>"
        )
        self.txt_events.append(html_line)
        self.txt_events.verticalScrollBar().setValue(
            self.txt_events.verticalScrollBar().maximum()
        )

    @Slot(float)
    def update_progress(self, progress: float):
        pct = int(progress * 100)
        self.progress_bar.setValue(pct)
        self.lbl_prog_pct.setText(f"{pct}%")

    def update_mission_state(self, state_str: str):
        if state_str in {"READY", "IDLE"}:
            self.btn_start.setEnabled(True)
        elif state_str in {"TAKEOFF", "ENTERING", "EXPLORING", "SURVIVOR_DETECTED", "EXITING"}:
            self.btn_start.setEnabled(False)
        elif state_str in {"MISSION_COMPLETE", "ABORTED", "FAILSAFE"}:
            self.btn_start.setEnabled(False)
