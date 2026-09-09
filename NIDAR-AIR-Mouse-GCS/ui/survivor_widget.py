"""
Survivor Localisation Widget for NIDAR AirMouse GCS.
Displays autonomous survivor detections (S1-S6) with Grid Box, Coordinates, Confidence, and Status.
"""
from datetime import datetime
from typing import Dict
from PySide6.QtCore import Qt, Slot
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
from communication.message_models import SurvivorDetection


class SurvivorWidget(QGroupBox):
    """
    Displays cards for up to 6 automatically detected survivors.
    Strictly visualizes incoming autonomous AI detections; no manual tagging required.
    """

    def __init__(self, parent=None):
        super().__init__("AUTONOMOUS SURVIVOR LOCALISATION (S1–S6)", parent)
        self._survivor_cards: Dict[str, _SurvivorCard] = {}
        self._init_ui()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 14, 8, 8)
        main_layout.setSpacing(6)

        # Header summary
        self.lbl_summary = QLabel("DETECTED: 0 / 6 SURVIVORS")
        self.lbl_summary.setStyleSheet("""
            font-size: 11px;
            font-weight: bold;
            color: #00e5ff;
            background-color: #0f172a;
            border: 1px solid #1e293b;
            border-radius: 3px;
            padding: 4px 8px;
        """)
        self.lbl_summary.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.lbl_summary)

        # Scroll area containing survivor cards
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background-color: transparent; border: none;")

        container = QWidget()
        self.card_layout = QVBoxLayout(container)
        self.card_layout.setContentsMargins(0, 0, 0, 0)
        self.card_layout.setSpacing(6)

        # Pre-populate slots S1 through S6
        for i in range(1, 7):
            s_id = f"S{i}"
            card = _SurvivorCard(s_id)
            self.card_layout.addWidget(card)
            self._survivor_cards[s_id] = card

        self.card_layout.addStretch()
        scroll.setWidget(container)
        main_layout.addWidget(scroll)

    @Slot(object)
    def add_survivor(self, detection: SurvivorDetection):
        """Updates the corresponding survivor card with detection data."""
        s_id = detection.survivor_id
        if s_id in self._survivor_cards:
            self._survivor_cards[s_id].set_detection(detection)

            # Update count
            detected_count = sum(1 for card in self._survivor_cards.values() if card.is_detected)
            self.lbl_summary.setText(f"DETECTED: {detected_count} / 6 SURVIVORS")
            self.lbl_summary.setStyleSheet("""
                font-size: 11px;
                font-weight: bold;
                color: #00e676;
                background-color: #064e3b;
                border: 1px solid #059669;
                border-radius: 3px;
                padding: 4px 8px;
            """)

    def reset(self):
        for card in self._survivor_cards.values():
            card.reset()
        self.lbl_summary.setText("DETECTED: 0 / 6 SURVIVORS")
        self.lbl_summary.setStyleSheet("""
            font-size: 11px;
            font-weight: bold;
            color: #00e5ff;
            background-color: #0f172a;
            border: 1px solid #1e293b;
            border-radius: 3px;
            padding: 4px 8px;
        """)


class _SurvivorCard(QFrame):
    """Card representing a single survivor detection slot (e.g. S1)."""

    def __init__(self, survivor_id: str, parent=None):
        super().__init__(parent)
        self.survivor_id = survivor_id
        self.is_detected = False

        self.setFrameShape(QFrame.StyledPanel)
        self._set_idle_style()
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(8)

        # 1. ID Badge
        self.lbl_id = QLabel(self.survivor_id)
        self.lbl_id.setFixedWidth(28)
        self.lbl_id.setAlignment(Qt.AlignCenter)
        self.lbl_id.setStyleSheet("""
            background-color: #1e293b;
            color: #64748b;
            border-radius: 3px;
            font-weight: bold;
            font-size: 12px;
            padding: 3px;
        """)

        # 2. Main Details
        details_layout = QVBoxLayout()
        details_layout.setSpacing(2)

        self.lbl_grid_pos = QLabel("Grid: -- | (0.00m, 0.00m)")
        self.lbl_grid_pos.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")

        self.lbl_meta = QLabel("Status: AWAITING DETECTION")
        self.lbl_meta.setStyleSheet("font-size: 10px; color: #475569;")

        details_layout.addWidget(self.lbl_grid_pos)
        details_layout.addWidget(self.lbl_meta)

        # 3. Confidence Badge
        self.lbl_conf = QLabel("--%")
        self.lbl_conf.setFixedWidth(46)
        self.lbl_conf.setAlignment(Qt.AlignCenter)
        self.lbl_conf.setStyleSheet("""
            background-color: #1e293b;
            color: #64748b;
            border-radius: 3px;
            font-weight: bold;
            font-size: 11px;
            padding: 2px;
        """)

        layout.addWidget(self.lbl_id)
        layout.addLayout(details_layout, 1)
        layout.addWidget(self.lbl_conf)

    def set_detection(self, detection: SurvivorDetection):
        self.is_detected = True
        dt_str = datetime.fromtimestamp(detection.timestamp).strftime("%H:%M:%S")

        self.lbl_grid_pos.setText(f"Grid: {detection.grid_cell} | ({detection.x:.2f}m, {detection.y:.2f}m)")
        self.lbl_grid_pos.setStyleSheet("font-size: 11px; font-weight: bold; color: #ffffff;")

        self.lbl_meta.setText(f"{detection.status} | {detection.source} | {dt_str}")
        self.lbl_meta.setStyleSheet("font-size: 10px; color: #94a3b8;")

        conf_pct = int(detection.confidence * 100)
        self.lbl_conf.setText(f"{conf_pct}%")
        self.lbl_conf.setStyleSheet("""
            background-color: #064e3b;
            color: #34d399;
            border: 1px solid #059669;
            border-radius: 3px;
            font-weight: bold;
            font-size: 11px;
            padding: 2px;
        """)

        self.lbl_id.setStyleSheet("""
            background-color: #00875a;
            color: #ffffff;
            border-radius: 3px;
            font-weight: bold;
            font-size: 12px;
            padding: 3px;
        """)

        self.setStyleSheet("""
            _SurvivorCard {
                background-color: #1a2721;
                border: 1px solid #00b875;
                border-radius: 4px;
            }
        """)

    def _set_idle_style(self):
        self.setStyleSheet("""
            _SurvivorCard {
                background-color: #14171e;
                border: 1px solid #232936;
                border-radius: 4px;
            }
        """)

    def reset(self):
        self.is_detected = False
        self._set_idle_style()
        self.lbl_grid_pos.setText("Grid: -- | (0.00m, 0.00m)")
        self.lbl_grid_pos.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")
        self.lbl_meta.setText("Status: AWAITING DETECTION")
        self.lbl_meta.setStyleSheet("font-size: 10px; color: #475569;")
        self.lbl_conf.setText("--%")
        self.lbl_conf.setStyleSheet("""
            background-color: #1e293b;
            color: #64748b;
            border-radius: 3px;
            font-weight: bold;
            font-size: 11px;
            padding: 2px;
        """)
        self.lbl_id.setStyleSheet("""
            background-color: #1e293b;
            color: #64748b;
            border-radius: 3px;
            font-weight: bold;
            font-size: 12px;
            padding: 3px;
        """)
