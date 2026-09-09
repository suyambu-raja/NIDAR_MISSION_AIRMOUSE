"""
Dual Simulated Video Stream Provider (RGB + Thermal) for NIDAR AirMouse GCS.
Generates synthetic 30 FPS frames with HUD, corridor wireframes, AI detection overlays,
and calibrated thermal heat-signature simulations.
"""
import base64
import math
import time
from typing import Optional, Dict, Tuple
import cv2
import numpy as np

try:
    from PySide6.QtCore import QTimer
    from vision.video_provider import BaseVideoProvider
except ImportError:
    QTimer = None
    BaseVideoProvider = object


class DualSimulatedVideoGenerator:
    """
    Renders distinct RGB (OAK-D) and Thermal (FLIR Lepton) camera streams
    and compresses them to Base64 JPEGs for high-speed WebSocket delivery.
    """

    def __init__(self, width: int = 400, height: int = 300, jpeg_quality: int = 65):
        self.width = width
        self.height = height
        self.encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

        # Pose cache
        self._drone_x = 1.25
        self._drone_y = 1.25
        self._drone_yaw = 0.0
        self._altitude = 1.25
        self._active_target_in_view: Optional[Dict] = None

    def update_pose(self, x: float, y: float, yaw_deg: float, altitude: float, target_in_view=None):
        self._drone_x = x
        self._drone_y = y
        self._drone_yaw = yaw_deg
        self._altitude = altitude
        self._active_target_in_view = target_in_view

    def render_rgb_frame(self) -> str:
        """Renders OAK-D RGB camera view with HUD and AI bounding boxes."""
        W, H = self.width, self.height
        frame = np.full((H, W, 3), 22, dtype=np.uint8)

        cx, cy = W // 2, H // 2
        yaw_offset = int(math.sin(math.radians(self._drone_yaw)) * 30.0)
        vp_x = cx + yaw_offset
        vp_y = cy - 15

        # Corridor Wireframe
        corridor_color = (65, 75, 90)
        cv2.line(frame, (0, 0), (vp_x - 70, vp_y - 45), corridor_color, 2)
        cv2.line(frame, (W, 0), (vp_x + 70, vp_y - 45), corridor_color, 2)
        cv2.line(frame, (0, H), (vp_x - 70, vp_y + 60), corridor_color, 2)
        cv2.line(frame, (W, H), (vp_x + 70, vp_y + 60), corridor_color, 2)

        # Floor grid lines
        for offset in range(-150, 151, 75):
            cv2.line(frame, (cx + offset, H), (vp_x, vp_y + 60), (35, 45, 55), 1)

        # Draw AI survivor detection in view
        if self._active_target_in_view:
            target = self._active_target_in_view
            bx1, by1 = int(W * 0.35), int(H * 0.25)
            bx2, by2 = int(W * 0.65), int(H * 0.75)

            # Silhouette
            cv2.circle(frame, ((bx1 + bx2) // 2, by1 + 30), 18, (140, 150, 170), -1)
            cv2.rectangle(frame, (bx1 + 20, by1 + 48), (bx2 - 20, by2 - 15), (140, 150, 170), -1)

            # Bounding Box & AI Label
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 230, 118), 2)
            cv2.putText(
                frame,
                f"TARGET {target['id']} | CONF: {int(target['confidence']*100)}%",
                (bx1, by1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 230, 118),
                1,
            )

        # Crosshair HUD
        cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (0, 200, 255), 1)
        cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (0, 200, 255), 1)

        # Banner
        cv2.putText(frame, "OAK-D RGB [SIMULATED]", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)
        cv2.putText(
            frame,
            f"X:{self._drone_x:.1f}m Y:{self._drone_y:.1f}m ALT:{self._altitude:.1f}m",
            (10, H - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (180, 180, 180),
            1,
        )

        _, buffer = cv2.imencode('.jpg', frame, self.encode_params)
        return base64.b64encode(buffer).decode('utf-8')

    def render_thermal_frame(self) -> str:
        """Renders FLIR Lepton thermal heat signature simulation view."""
        W, H = self.width, self.height
        gray = np.full((H, W), 45, dtype=np.uint8)

        cx, cy = W // 2, H // 2
        yaw_offset = int(math.sin(math.radians(self._drone_yaw)) * 30.0)
        vp_x = cx + yaw_offset
        vp_y = cy - 15

        # Cooler background structures
        cv2.line(gray, (0, 0), (vp_x - 70, vp_y - 45), 70, 2)
        cv2.line(gray, (W, 0), (vp_x + 70, vp_y - 45), 70, 2)
        cv2.line(gray, (0, H), (vp_x - 70, vp_y + 60), 60, 2)
        cv2.line(gray, (W, H), (vp_x + 70, vp_y + 60), 60, 2)

        # Render thermal hot body if survivor in view
        if self._active_target_in_view:
            target = self._active_target_in_view
            bx1, by1 = int(W * 0.35), int(H * 0.25)
            bx2, by2 = int(W * 0.65), int(H * 0.75)
            tcx = (bx1 + bx2) // 2
            tcy = (by1 + by2) // 2

            # Hot body radiation gradients (220-255 in grayscale)
            cv2.ellipse(gray, (tcx, tcy), (35, 65), 0, 0, 360, 190, -1)
            cv2.ellipse(gray, (tcx, tcy), (20, 40), 0, 0, 360, 235, -1)
            cv2.ellipse(gray, (tcx, tcy - 20), (12, 12), 0, 0, 360, 255, -1)

        # Apply FLIR thermal colormap (INFERNO)
        thermal_bgr = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        if self._active_target_in_view:
            cv2.rectangle(thermal_bgr, (bx1, by1), (bx2, by2), (255, 255, 0), 1)
            cv2.putText(
                thermal_bgr,
                f"THERMAL HIT: {self._active_target_in_view['id']}",
                (bx1, by1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
            )

        # Watermark
        cv2.putText(thermal_bgr, "FLIR THERMAL [SIMULATED]", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)
        cv2.putText(
            thermal_bgr,
            f"TEMP RANGE: 24.2C - 36.8C",
            (10, H - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (255, 255, 255),
            1,
        )

        _, buffer = cv2.imencode('.jpg', thermal_bgr, self.encode_params)
        return base64.b64encode(buffer).decode('utf-8')


class SimulatedVideoProvider(BaseVideoProvider):
    """
    Qt Signal compatible wrapper around DualSimulatedVideoGenerator.
    """

    def __init__(self, fps: int = 30, width: int = 640, height: int = 480, parent=None):
        if BaseVideoProvider is not object:
            super().__init__(parent)
        self.fps = fps
        self.generator = DualSimulatedVideoGenerator(width=width, height=height)
        self._is_thermal_mode = False

        if QTimer is not None:
            self._timer = QTimer(self)
            self._timer.setInterval(int(1000 / self.fps))
            self._timer.timeout.connect(self._render_and_emit)

    def set_drone_state(self, x: float, y: float, yaw_deg: float, altitude: float, target_in_view=None):
        self.generator.update_pose(x, y, yaw_deg, altitude, target_in_view)

    def set_thermal_mode(self, enabled: bool):
        self._is_thermal_mode = enabled

    def start(self):
        if hasattr(super(), 'start'):
            super().start()
        if hasattr(self, '_timer'):
            self._timer.start()
        if hasattr(self, 'connection_changed'):
            self.connection_changed.emit(True)

    def stop(self):
        if hasattr(super(), 'stop'):
            super().stop()
        if hasattr(self, '_timer'):
            self._timer.stop()
        if hasattr(self, 'connection_changed'):
            self.connection_changed.emit(False)

    def _render_and_emit(self):
        # Render and emit raw BGR image for PySide6
        W, H = self.generator.width, self.generator.height
        frame = np.full((H, W, 3), 24, dtype=np.uint8)
        if hasattr(self, 'frame_received'):
            self.frame_received.emit(frame)
