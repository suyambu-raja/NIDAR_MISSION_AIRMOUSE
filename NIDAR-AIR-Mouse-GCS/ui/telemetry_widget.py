"""
Telemetry and System Health Widget for NIDAR AirMouse GCS.
Displays flight dynamics, local pose coordinates, power metrics, and subsystem health indicators.
"""
from typing import Dict
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QGroupBox,
    QFrame,
    QScrollArea,
)
from communication.message_models import TelemetryData


class TelemetryWidget(QGroupBox):
    """
    Right-panel widget displaying real-time telemetry gauges and sensor health matrix.
    """

    def __init__(self, parent=None):
        super().__init__("FLIGHT TELEMETRY & SYSTEM HEALTH", parent)
        self._init_ui()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 14, 8, 8)
        main_layout.setSpacing(10)

        # 1. Artificial Horizon / Attitude Widget
        self.attitude_gauge = _AttitudeGauge()
        main_layout.addWidget(self.attitude_gauge)

        # 2. Flight Kinematics Group
        kin_box = QGroupBox("Kinematics & Pose")
        kin_box.setStyleSheet("margin-top: 14px; font-size: 11px; padding: 6px;")
        grid_kin = QGridLayout(kin_box)
        grid_kin.setContentsMargins(6, 10, 6, 6)
        grid_kin.setHorizontalSpacing(8)
        grid_kin.setVerticalSpacing(4)

        self.lbl_alt = self._add_stat_row(grid_kin, 0, "Altitude:", "0.00", "m")
        self.lbl_speed = self._add_stat_row(grid_kin, 1, "Ground Speed:", "0.00", "m/s")
        self.lbl_hdg = self._add_stat_row(grid_kin, 2, "Heading (Yaw):", "000", "°")
        self.lbl_roll = self._add_stat_row(grid_kin, 3, "Roll / Pitch:", "0.0 / 0.0", "°")
        self.lbl_pos = self._add_stat_row(grid_kin, 4, "Local (X, Y):", "(0.00, 0.00)", "m")

        main_layout.addWidget(kin_box)

        # 3. Power System Group
        power_box = QGroupBox("Power Management")
        power_box.setStyleSheet("margin-top: 14px; font-size: 11px; padding: 6px;")
        grid_power = QGridLayout(power_box)
        grid_power.setContentsMargins(6, 10, 6, 6)
        grid_power.setHorizontalSpacing(8)
        grid_power.setVerticalSpacing(4)

        self.lbl_volts = self._add_stat_row(grid_power, 0, "Voltage:", "16.80", "V")
        self.lbl_amps = self._add_stat_row(grid_power, 1, "Current:", "0.00", "A")
        self.lbl_soc = self._add_stat_row(grid_power, 2, "Capacity:", "100", "%")

        main_layout.addWidget(power_box)

        # 4. Sensor & Subsystem Health Matrix
        health_box = QGroupBox("Sensor & Link Health Matrix")
        health_box.setStyleSheet("margin-top: 14px; font-size: 11px; padding: 6px;")
        grid_health = QGridLayout(health_box)
        grid_health.setContentsMargins(6, 10, 6, 6)
        grid_health.setHorizontalSpacing(6)
        grid_health.setVerticalSpacing(6)

        self.health_badges: Dict[str, QLabel] = {}
        sensors = [
            ("EKF Estimator", 0, 0),
            ("Optical Flow", 0, 1),
            ("Rangefinder", 1, 0),
            ("RPLIDAR 2D", 1, 1),
            ("OAK-D AI Cam", 2, 0),
            ("FLIR Thermal", 2, 1),
            ("MAVLink Comm", 3, 0),
            ("Video Stream", 3, 1),
        ]

        for name, r, c in sensors:
            lbl = QLabel(name)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedHeight(22)
            lbl.setStyleSheet("""
                background-color: #1a2332;
                color: #64748b;
                border: 1px solid #334155;
                border-radius: 3px;
                font-size: 10px;
                font-weight: bold;
            """)
            grid_health.addWidget(lbl, r, c)
            self.health_badges[name] = lbl

        main_layout.addWidget(health_box)
        main_layout.addStretch()

    def _add_stat_row(self, layout: QGridLayout, row: int, label_text: str, default_val: str, unit_text: str) -> QLabel:
        lbl_title = QLabel(label_text)
        lbl_title.setStyleSheet("color: #94a3b8; font-weight: 600; font-size: 11px;")
        
        lbl_val = QLabel(default_val)
        lbl_val.setObjectName("telemetry_value")
        lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        lbl_unit = QLabel(unit_text)
        lbl_unit.setObjectName("telemetry_unit")
        lbl_unit.setFixedWidth(24)

        layout.addWidget(lbl_title, row, 0)
        layout.addWidget(lbl_val, row, 1)
        layout.addWidget(lbl_unit, row, 2)
        return lbl_val

    @Slot(object)
    def update_telemetry(self, telem: TelemetryData):
        """Updates all numeric readouts and health badges."""
        self.attitude_gauge.set_attitude(telem.roll_deg, telem.pitch_deg)

        self.lbl_alt.setText(f"{telem.altitude_m:.2f}")
        self.lbl_speed.setText(f"{telem.ground_speed_mps:.2f}")
        self.lbl_hdg.setText(f"{int(telem.heading_deg):03d}")
        self.lbl_roll.setText(f"{telem.roll_deg:+.1f} / {telem.pitch_deg:+.1f}")
        self.lbl_pos.setText(f"({telem.x_m:.2f}, {telem.y_m:.2f})")

        self.lbl_volts.setText(f"{telem.battery_voltage_v:.2f}")
        self.lbl_amps.setText(f"{telem.battery_current_a:.2f}")
        self.lbl_soc.setText(f"{int(telem.battery_percentage)}")

        # Health matrix mapping
        health_map = {
            "EKF Estimator": telem.ekf_healthy,
            "Optical Flow": telem.optical_flow_healthy,
            "Rangefinder": telem.rangefinder_healthy,
            "RPLIDAR 2D": telem.lidar_healthy,
            "OAK-D AI Cam": telem.oak_d_healthy,
            "FLIR Thermal": telem.thermal_healthy,
            "MAVLink Comm": telem.mavlink_connected,
            "Video Stream": telem.video_connected,
        }

        for name, healthy in health_map.items():
            if name in self.health_badges:
                lbl = self.health_badges[name]
                if healthy:
                    lbl.setStyleSheet("""
                        background-color: #064e3b;
                        color: #34d399;
                        border: 1px solid #059669;
                        border-radius: 3px;
                        font-size: 10px;
                        font-weight: bold;
                    """)
                else:
                    lbl.setStyleSheet("""
                        background-color: #450a0a;
                        color: #f87171;
                        border: 1px solid #dc2626;
                        border-radius: 3px;
                        font-size: 10px;
                        font-weight: bold;
                    """)


class _AttitudeGauge(QWidget):
    """Custom QPainter widget rendering an artificial horizon attitude indicator."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(120)
        self.roll = 0.0
        self.pitch = 0.0

    def set_attitude(self, roll_deg: float, pitch_deg: float):
        self.roll = roll_deg
        self.pitch = pitch_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        cx, cy = self.width() / 2.0, self.height() / 2.0
        radius = min(cx, cy) - 6

        # Outer bezel
        painter.setPen(QPen(QColor("#334155"), 2))
        painter.setBrush(QBrush(QColor("#0f172a")))
        painter.drawEllipse(int(cx - radius), int(cy - radius), int(2 * radius), int(2 * radius))

        # Clip to circle
        painter.save()
        painter.setClipRegion(self.rect())

        # Rotate canvas for roll angle
        painter.translate(cx, cy)
        painter.rotate(-self.roll)

        # Pitch offset (1 pitch deg = ~1.5 px)
        pitch_px = max(-radius, min(radius, self.pitch * 1.5))

        # Sky (Top) & Ground (Bottom)
        painter.fillRect(int(-radius), int(-radius + pitch_px), int(2 * radius), int(radius - pitch_px), QColor("#0284c7"))
        painter.fillRect(int(-radius), int(pitch_px), int(2 * radius), int(radius - pitch_px), QColor("#78350f"))

        # Horizon Line
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawLine(int(-radius + 5), int(pitch_px), int(radius - 5), int(pitch_px))

        # Pitch ladder ticks
        for p in [-10, 10]:
            tick_y = int(pitch_px - p * 1.5)
            painter.setPen(QPen(QColor(255, 255, 255, 180), 1))
            painter.drawLine(-15, tick_y, 15, tick_y)

        painter.restore()

        # Fixed Aircraft Reference Symbol (Crosshair Wings)
        painter.setPen(QPen(QColor("#ffea00"), 3))
        painter.drawLine(int(cx - 25), int(cy), int(cx - 8), int(cy))
        painter.drawLine(int(cx + 8), int(cy), int(cx + 25), int(cy))
        painter.drawPoint(int(cx), int(cy))
