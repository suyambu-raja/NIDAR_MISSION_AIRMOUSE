"""
Header Bar Widget for NIDAR AirMouse GCS.
Displays branding, connection control, mission phase badge, flight mode, battery, and mission clock.
"""
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QProgressBar,
    QFrame,
)
from communication.message_models import TelemetryData, ConnectionStatus


class HeaderWidget(QWidget):
    """
    Top-level status and command bar for the GCS application.
    """
    connection_toggle_requested = Signal()
    mode_changed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(56)
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(16, 6, 16, 6)
        layout.setSpacing(12)

        # 1. Branding & Title
        title_box = QHBoxLayout()
        title_box.setSpacing(8)

        self.lbl_logo = QLabel("🛸")
        self.lbl_logo.setStyleSheet("font-size: 20px;")
        
        self.lbl_title = QLabel("NIDAR AIRMOUSE GCS")
        self.lbl_title.setStyleSheet("""
            font-size: 15px;
            font-weight: 800;
            letter-spacing: 1.5px;
            color: #00e5ff;
        """)

        self.lbl_sub = QLabel("GPS-DENIED AUTONOMY")
        self.lbl_sub.setStyleSheet("""
            font-size: 9px;
            font-weight: bold;
            color: #64748b;
            border: 1px solid #334155;
            border-radius: 3px;
            padding: 1px 5px;
        """)

        title_box.addWidget(self.lbl_logo)
        title_box.addWidget(self.lbl_title)
        title_box.addWidget(self.lbl_sub)
        layout.addLayout(title_box)

        # Divider
        layout.addWidget(self._create_divider())

        # 2. Operating Mode Selector
        lbl_mode = QLabel("MODE:")
        lbl_mode.setStyleSheet("font-weight: bold; color: #94a3b8; font-size: 11px;")
        self.combo_mode = QComboBox()
        self.combo_mode.addItems(["SIMULATION", "HARDWARE"])
        self.combo_mode.currentTextChanged.connect(self.mode_changed.emit)

        layout.addWidget(lbl_mode)
        layout.addWidget(self.combo_mode)

        # 3. Connection Control & Status
        self.btn_connect = QPushButton("CONNECT")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.setFixedWidth(110)
        self.btn_connect.clicked.connect(self.connection_toggle_requested.emit)

        self.lbl_conn_status = QLabel("DISCONNECTED")
        self.lbl_conn_status.setObjectName("badge_err")

        layout.addWidget(self.btn_connect)
        layout.addWidget(self.lbl_conn_status)

        # Divider
        layout.addWidget(self._create_divider())

        # 4. Mission State Badge
        lbl_ms = QLabel("MISSION:")
        lbl_ms.setStyleSheet("font-weight: bold; color: #94a3b8; font-size: 11px;")
        self.lbl_mission_state = QLabel("IDLE")
        self.lbl_mission_state.setStyleSheet("""
            background-color: #1e293b;
            color: #94a3b8;
            border: 1px solid #475569;
            border-radius: 4px;
            padding: 3px 10px;
            font-weight: bold;
            font-size: 11px;
        """)

        layout.addWidget(lbl_ms)
        layout.addWidget(self.lbl_mission_state)

        # 5. Flight Mode Badge
        lbl_fm = QLabel("FLIGHT:")
        lbl_fm.setStyleSheet("font-weight: bold; color: #94a3b8; font-size: 11px;")
        self.lbl_flight_mode = QLabel("DISARMED")
        self.lbl_flight_mode.setStyleSheet("""
            background-color: #1e293b;
            color: #f1f5f9;
            border: 1px solid #334155;
            border-radius: 4px;
            padding: 3px 8px;
            font-weight: bold;
            font-size: 11px;
        """)

        layout.addWidget(lbl_fm)
        layout.addWidget(self.lbl_flight_mode)

        # Divider
        layout.addWidget(self._create_divider())

        # 6. Battery Gauge
        batt_box = QHBoxLayout()
        batt_box.setSpacing(6)
        lbl_batt = QLabel("🔋")
        lbl_batt.setStyleSheet("font-size: 14px;")
        
        self.bar_battery = QProgressBar()
        self.bar_battery.setRange(0, 100)
        self.bar_battery.setValue(100)
        self.bar_battery.setFixedSize(90, 18)
        self.bar_battery.setTextVisible(True)
        self.bar_battery.setFormat("%v%")

        self.lbl_battery_volts = QLabel("16.8V")
        self.lbl_battery_volts.setStyleSheet("font-size: 11px; color: #94a3b8; font-weight: bold;")

        batt_box.addWidget(lbl_batt)
        batt_box.addWidget(self.bar_battery)
        batt_box.addWidget(self.lbl_battery_volts)
        layout.addLayout(batt_box)

        # Divider
        layout.addWidget(self._create_divider())

        # 7. Mission Clock
        clock_box = QHBoxLayout()
        clock_box.setSpacing(6)
        lbl_clock_icon = QLabel("⏱️")
        
        self.lbl_timer = QLabel("00:00")
        self.lbl_timer.setStyleSheet("""
            font-family: 'Consolas', monospace;
            font-size: 16px;
            font-weight: bold;
            color: #00e5ff;
            background-color: #0f172a;
            border: 1px solid #1e293b;
            border-radius: 4px;
            padding: 2px 8px;
        """)

        clock_box.addWidget(lbl_clock_icon)
        clock_box.addWidget(self.lbl_timer)
        layout.addLayout(clock_box)

    def _create_divider(self) -> QFrame:
        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setFrameShadow(QFrame.Sunken)
        div.setStyleSheet("color: #2a3342;")
        return div

    def update_connection_status(self, status: ConnectionStatus):
        if status.is_connected:
            self.btn_connect.setText("DISCONNECT")
            self.lbl_conn_status.setText("CONNECTED")
            self.lbl_conn_status.setObjectName("badge_ok")
            self.lbl_conn_status.setStyle(self.lbl_conn_status.style())
        else:
            self.btn_connect.setText("CONNECT")
            self.lbl_conn_status.setText("DISCONNECTED")
            self.lbl_conn_status.setObjectName("badge_err")
            self.lbl_conn_status.setStyle(self.lbl_conn_status.style())

    def update_telemetry(self, telem: TelemetryData):
        self.lbl_flight_mode.setText(telem.flight_mode)
        pct = int(telem.battery_percentage)
        self.bar_battery.setValue(pct)
        self.lbl_battery_volts.setText(f"{telem.battery_voltage_v:.1f}V")

        # Battery Bar Color logic
        if pct < 20:
            chunk_color = "#ff5252"
        elif pct < 40:
            chunk_color = "#ffca28"
        else:
            chunk_color = "#00e676"

        self.bar_battery.setStyleSheet(f"""
            QProgressBar {{
                background-color: #0f172a;
                border: 1px solid #334155;
                border-radius: 3px;
                text-align: center;
                color: #ffffff;
                font-size: 10px;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background-color: {chunk_color};
                border-radius: 2px;
            }}
        """)

    def update_mission_state(self, state_str: str):
        self.lbl_mission_state.setText(state_str)
        color_map = {
            "IDLE": ("#1e293b", "#94a3b8", "#475569"),
            "SYSTEM_CHECK": ("#1e293b", "#38bdf8", "#0284c7"),
            "READY": ("#064e3b", "#34d399", "#059669"),
            "TAKEOFF": ("#1e3a8a", "#60a5fa", "#2563eb"),
            "ENTERING": ("#1e3a8a", "#60a5fa", "#2563eb"),
            "EXPLORING": ("#0f766e", "#2dd4bf", "#14b8a6"),
            "SURVIVOR_DETECTED": ("#831843", "#f472b6", "#db2777"),
            "EXITING": ("#1e3a8a", "#60a5fa", "#2563eb"),
            "MISSION_COMPLETE": ("#064e3b", "#4ade80", "#16a34a"),
            "ABORTED": ("#7f1d1d", "#f87171", "#dc2626"),
            "FAILSAFE": ("#7f1d1d", "#f87171", "#dc2626"),
            "CONNECTION_LOST": ("#78350f", "#fbbf24", "#d97706"),
        }
        bg, fg, border = color_map.get(state_str, ("#1e293b", "#94a3b8", "#475569"))
        self.lbl_mission_state.setStyleSheet(f"""
            background-color: {bg};
            color: {fg};
            border: 1px solid {border};
            border-radius: 4px;
            padding: 3px 10px;
            font-weight: bold;
            font-size: 11px;
        """)

    def update_timer(self, time_str: str):
        self.lbl_timer.setText(time_str)
