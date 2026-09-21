"""
Camera / Vision Widget for NIDAR AirMouse GCS.
Renders live video stream from OAK-D RGB / FLIR Thermal camera with autonomous detection HUD.
"""
import time
from typing import Optional
import cv2
import numpy as np
from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor, QFont
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QGroupBox,
    QCheckBox,
)


class CameraWidget(QGroupBox):
    """
    Left-panel widget displaying live onboard camera feed with HUD overlays.
    """
    thermal_mode_toggled = Signal(bool)

    def __init__(self, parent=None):
        super().__init__("LIVE VISION / OAK-D CAMERA", parent)
        self._init_ui()
        self._last_frame_time = 0.0
        self._frame_count = 0
        self._fps = 0.0

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 14, 8, 8)
        layout.setSpacing(6)

        # Video Frame Display Canvas
        self.lbl_video = QLabel()
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("""
            background-color: #0b0d11;
            border: 1px solid #242b35;
            border-radius: 4px;
        """)
        self.lbl_video.setMinimumSize(320, 240)
        layout.addWidget(self.lbl_video, 1)

        # Bottom Controls & Status Bar
        ctrl_box = QHBoxLayout()
        ctrl_box.setSpacing(8)

        self.chk_thermal = QCheckBox("Thermal View")
        self.chk_thermal.setStyleSheet("color: #00e5ff; font-weight: bold;")
        self.chk_thermal.toggled.connect(self.thermal_mode_toggled.emit)

        self.lbl_fps = QLabel("FPS: 0")
        self.lbl_fps.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: bold;")

        self.lbl_stream_status = QLabel("NO STREAM")
        self.lbl_stream_status.setObjectName("badge_err")

        ctrl_box.addWidget(self.chk_thermal)
        ctrl_box.addStretch()
        ctrl_box.addWidget(self.lbl_fps)
        ctrl_box.addWidget(self.lbl_stream_status)

        layout.addLayout(ctrl_box)

    @Slot(object)
    def update_frame(self, frame: np.ndarray):
        """Receives OpenCV BGR frame and renders it on Qt canvas."""
        if frame is None or not isinstance(frame, np.ndarray):
            return

        now = time.time()
        self._frame_count += 1
        if now - self._last_frame_time >= 1.0:
            self._fps = self._frame_count / (now - self._last_frame_time)
            self._frame_count = 0
            self._last_frame_time = now
            self.lbl_fps.setText(f"FPS: {self._fps:.1f}")

        # Convert OpenCV BGR image to RGB QImage
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        q_img = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Scale pixmap to fit widget size smoothly
        target_size = self.lbl_video.size()
        pixmap = QPixmap.fromImage(q_img).scaled(
            target_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.lbl_video.setPixmap(pixmap)

        # Update status badge
        if self.lbl_stream_status.text() != "STREAM OK":
            self.lbl_stream_status.setText("STREAM OK")
            self.lbl_stream_status.setObjectName("badge_ok")
            self.lbl_stream_status.setStyle(self.lbl_stream_status.style())

    def set_stream_connected(self, connected: bool):
        if not connected:
            self.lbl_stream_status.setText("NO STREAM")
            self.lbl_stream_status.setObjectName("badge_err")
            self.lbl_stream_status.setStyle(self.lbl_stream_status.style())
            self.lbl_fps.setText("FPS: 0")
            self.lbl_video.clear()
            self.lbl_video.setText("Awaiting Video Link...")
            self.lbl_video.setStyleSheet("color: #64748b; font-size: 13px; font-weight: bold; background-color: #0b0d11;")
