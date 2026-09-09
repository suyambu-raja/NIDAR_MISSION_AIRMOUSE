"""
2D Occupancy Grid Map Widget for NIDAR AirMouse GCS.
High-performance QPainter-based interactive canvas rendering dynamic SLAM maps,
drone pose, flight trail, search grid overlay (A1-H8), and autonomous survivor markers.
"""
import math
from typing import List, Tuple, Optional, Dict
import numpy as np
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, Slot
from PySide6.QtGui import (
    QPainter,
    QColor,
    QPen,
    QBrush,
    QFont,
    QImage,
    QPolygonF,
    QMouseEvent,
    QWheelEvent,
)
from PySide6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QCheckBox,
    QLabel,
    QGroupBox,
)
from mapping.grid_manager import GridManager
from mapping.occupancy_grid import OccupancyGrid
from communication.message_models import MapData, TelemetryData, SurvivorDetection


class MapWidget(QGroupBox):
    """
    Interactive 2D Center Map Widget.
    Supports pan, zoom, fit-to-view, grid overlays, drone pose, and survivor markers.
    """

    def __init__(self, grid_manager: GridManager, parent=None):
        super().__init__("AUTONOMOUS 2D OCCUPANCY GRID & LOCALISATION MAP", parent)
        self.grid_manager = grid_manager
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 14, 8, 8)
        layout.setSpacing(6)

        # Map Canvas
        self.canvas = _MapCanvas(self.grid_manager)
        layout.addWidget(self.canvas, 1)

        # Bottom Map Controls
        ctrl_bar = QHBoxLayout()
        ctrl_bar.setSpacing(8)

        self.btn_fit = QPushButton("Fit Map")
        self.btn_fit.setFixedWidth(80)
        self.btn_fit.clicked.connect(self.canvas.fit_to_view)

        self.btn_zoom_in = QPushButton("➕")
        self.btn_zoom_in.setFixedWidth(36)
        self.btn_zoom_in.clicked.connect(lambda: self.canvas.zoom(1.2))

        self.btn_zoom_out = QPushButton("➖")
        self.btn_zoom_out.setFixedWidth(36)
        self.btn_zoom_out.clicked.connect(lambda: self.canvas.zoom(0.8))

        self.chk_grid = QCheckBox("Search Grid (A1-H8)")
        self.chk_grid.setChecked(True)
        self.chk_grid.toggled.connect(self.canvas.set_show_grid)

        self.chk_trail = QCheckBox("Flight Path")
        self.chk_trail.setChecked(True)
        self.chk_trail.toggled.connect(self.canvas.set_show_trail)

        self.lbl_coords = QLabel("X: 0.00m | Y: 0.00m | Grid: --")
        self.lbl_coords.setStyleSheet("color: #00e5ff; font-family: 'Consolas', monospace; font-weight: bold;")
        self.canvas.cursor_moved.connect(self._on_cursor_moved)

        ctrl_bar.addWidget(self.btn_fit)
        ctrl_bar.addWidget(self.btn_zoom_in)
        ctrl_bar.addWidget(self.btn_zoom_out)
        ctrl_bar.addWidget(self.chk_grid)
        ctrl_bar.addWidget(self.chk_trail)
        ctrl_bar.addStretch()
        ctrl_bar.addWidget(self.lbl_coords)

        layout.addLayout(ctrl_bar)

    def update_map(self, map_data: MapData):
        self.canvas.update_map_data(map_data)

    def update_telemetry(self, telem: TelemetryData):
        self.canvas.update_drone_pose(telem.x_m, telem.y_m, telem.heading_deg)

    def add_survivor(self, detection: SurvivorDetection):
        self.canvas.add_survivor(detection)

    def reset(self):
        self.canvas.reset()

    def _on_cursor_moved(self, x_m: float, y_m: float, grid_cell: str):
        self.lbl_coords.setText(f"X: {x_m:.2f}m | Y: {y_m:.2f}m | Grid: {grid_cell}")


class _MapCanvas(QWidget):
    cursor_moved = Signal(float, float, str)

    def __init__(self, grid_manager: GridManager, parent=None):
        super().__init__(parent)
        self.grid_manager = grid_manager
        self.setMouseTracking(True)
        self.setMinimumSize(450, 400)

        # Transform settings
        self.scale = 20.0  # pixels per meter
        self.offset_x = 40.0
        self.offset_y = 40.0

        # State
        self.drone_x = 1.25
        self.drone_y = 1.25
        self.drone_yaw = 0.0
        self.trail: List[Tuple[float, float]] = [(1.25, 1.25)]
        self.survivors: Dict[str, SurvivorDetection] = {}

        self.map_image: Optional[QImage] = None
        self.map_width_cells = 400
        self.map_height_cells = 400
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # UI Toggles
        self.show_grid = True
        self.show_trail = True

        # Mouse dragging state
        self._dragging = False
        self._last_mouse_pos = None

    def fit_to_view(self):
        """Calculates scale and offset so the full arena is centered."""
        w = self.width() - 80
        h = self.height() - 80
        if w <= 0 or h <= 0:
            return

        scale_x = w / self.grid_manager.grid_width_meters
        scale_y = h / self.grid_manager.grid_height_meters
        self.scale = min(scale_x, scale_y)

        # Center in canvas
        arena_px_w = self.grid_manager.grid_width_meters * self.scale
        arena_px_h = self.grid_manager.grid_height_meters * self.scale
        self.offset_x = (self.width() - arena_px_w) / 2.0
        self.offset_y = self.height() - (self.height() - arena_px_h) / 2.0
        self.update()

    def zoom(self, factor: float):
        """Zooms centered on widget center."""
        cx, cy = self.width() / 2.0, self.height() / 2.0
        wx, wy = self.screen_to_world(cx, cy)
        self.scale = max(5.0, min(120.0, self.scale * factor))
        # Re-adjust offset to keep (wx, wy) at center
        self.offset_x = cx - wx * self.scale
        self.offset_y = cy + wy * self.scale
        self.update()

    def set_show_grid(self, show: bool):
        self.show_grid = show
        self.update()

    def set_show_trail(self, show: bool):
        self.show_trail = show
        self.update()

    def reset(self):
        self.trail.clear()
        self.survivors.clear()
        self.map_image = None
        self.update()

    def world_to_screen(self, x_m: float, y_m: float) -> Tuple[float, float]:
        """Converts metric world coordinates to canvas pixel coordinates."""
        px = self.offset_x + x_m * self.scale
        py = self.offset_y - y_m * self.scale  # Invert Y for screen coordinates
        return px, py

    def screen_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """Converts canvas pixel coordinates to metric world coordinates."""
        x_m = (px - self.offset_x) / self.scale
        y_m = (self.offset_y - py) / self.scale
        return x_m, y_m

    def update_drone_pose(self, x_m: float, y_m: float, yaw_deg: float):
        self.drone_x = x_m
        self.drone_y = y_m
        self.drone_yaw = yaw_deg

        if not self.trail or math.hypot(self.trail[-1][0] - x_m, self.trail[-1][1] - y_m) > 0.15:
            self.trail.append((x_m, y_m))
            if len(self.trail) > 1000:
                self.trail.pop(0)

        self.update()

    def add_survivor(self, detection: SurvivorDetection):
        self.survivors[detection.survivor_id] = detection
        self.update()

    def update_map_data(self, map_data: MapData):
        """Builds an RGB QImage from the 2D occupancy grid."""
        if map_data.grid is None:
            return

        grid = map_data.grid
        self.map_width_cells = map_data.width
        self.map_height_cells = map_data.height
        self.map_resolution = map_data.resolution
        self.map_origin_x = map_data.origin_x
        self.map_origin_y = map_data.origin_y

        h, w = grid.shape
        # Create 3-channel RGB image buffer
        img_buffer = np.zeros((h, w, 3), dtype=np.uint8)

        # UNKNOWN (-1) -> Dark Navy #15181f
        unknown_mask = (grid == OccupancyGrid.UNKNOWN)
        img_buffer[unknown_mask] = [21, 24, 31]

        # FREE (0) -> Slate #2b3644
        free_mask = (grid == OccupancyGrid.FREE)
        img_buffer[free_mask] = [43, 54, 68]

        # OCCUPIED (100) -> Bright Cyan/White #00e5ff
        occupied_mask = (grid == OccupancyGrid.OCCUPIED)
        img_buffer[occupied_mask] = [0, 229, 255]

        # Flip vertically so row 0 is at bottom (World frame)
        img_buffer = np.flipud(img_buffer)

        bytes_per_line = 3 * w
        self.map_image = QImage(
            img_buffer.data, w, h, bytes_per_line, QImage.Format_RGB888
        ).copy()

        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Initial fit on first display
        if self.scale == 20.0:
            self.fit_to_view()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() in (Qt.LeftButton, Qt.MiddleButton):
            self._dragging = True
            self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event: QMouseEvent):
        # Update coordinate readout under cursor
        wx, wy = self.screen_to_world(event.position().x(), event.position().y())
        grid_cell = self.grid_manager.metric_to_grid(wx, wy)
        self.cursor_moved.emit(wx, wy, grid_cell)

        if self._dragging and self._last_mouse_pos is not None:
            delta = event.pos() - self._last_mouse_pos
            self.offset_x += delta.x()
            self.offset_y += delta.y()
            self._last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() in (Qt.LeftButton, Qt.MiddleButton):
            self._dragging = False
            self._last_mouse_pos = None

    def wheelEvent(self, event: QWheelEvent):
        angle = event.angleDelta().y()
        factor = 1.15 if angle > 0 else 0.87
        mouse_pos = event.position()
        wx, wy = self.screen_to_world(mouse_pos.x(), mouse_pos.y())
        self.scale = max(5.0, min(150.0, self.scale * factor))
        self.offset_x = mouse_pos.x() - wx * self.scale
        self.offset_y = mouse_pos.y() + wy * self.scale
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)

        # 1. Canvas Background
        painter.fillRect(self.rect(), QColor("#0d0f12"))

        # 2. Render 2D Occupancy Grid Map
        if self.map_image is not None:
            # Calculate world bounds of map
            map_w_m = self.map_width_cells * self.map_resolution
            map_h_m = self.map_height_cells * self.map_resolution
            p1_x, p1_y = self.world_to_screen(self.map_origin_x, self.map_origin_y + map_h_m)
            p2_x, p2_y = self.world_to_screen(self.map_origin_x + map_w_m, self.map_origin_y)
            dest_rect = QRectF(p1_x, p1_y, p2_x - p1_x, p2_y - p1_y)
            painter.drawImage(dest_rect, self.map_image)
        else:
            # Draw unexplored arena base rectangle
            W_m = self.grid_manager.grid_width_meters
            H_m = self.grid_manager.grid_height_meters
            p1_x, p1_y = self.world_to_screen(0.0, H_m)
            p2_x, p2_y = self.world_to_screen(W_m, 0.0)
            painter.fillRect(QRectF(p1_x, p1_y, p2_x - p1_x, p2_y - p1_y), QColor("#15181f"))

        # 3. Render Search Grid Overlay (A1 - H8)
        if self.show_grid:
            self._draw_search_grid(painter)

        # 4. Render Flight Trail
        if self.show_trail and len(self.trail) > 1:
            self._draw_flight_trail(painter)

        # 5. Render Detected Survivors (S1 - S6)
        self._draw_survivors(painter)

        # 6. Render Drone Pose & Heading
        self._draw_drone(painter)

        # 7. Render Compass & Scale Overlay
        self._draw_hud_overlays(painter)

    def _draw_search_grid(self, painter: QPainter):
        """Draws discrete search grid boundaries and cell labels (A1..H8)."""
        cols, rows, cell_sz = self.grid_manager.get_grid_dimensions()
        pen_grid = QPen(QColor(45, 55, 72, 160), 1, Qt.DashLine)
        pen_border = QPen(QColor(0, 229, 255, 180), 2, Qt.SolidLine)
        font_label = QFont("Segoe UI", 9, QFont.Bold)
        font_cell = QFont("Consolas", 8, QFont.Normal)

        # Boundary Rect
        W_m = self.grid_manager.grid_width_meters
        H_m = self.grid_manager.grid_height_meters
        p1_x, p1_y = self.world_to_screen(0, H_m)
        p2_x, p2_y = self.world_to_screen(W_m, 0)
        painter.setPen(pen_border)
        painter.drawRect(QRectF(p1_x, p1_y, p2_x - p1_x, p2_y - p1_y))

        painter.setPen(pen_grid)
        # Vertical column lines
        for c in range(1, cols):
            x_m = c * cell_sz
            sx1, sy1 = self.world_to_screen(x_m, 0)
            sx2, sy2 = self.world_to_screen(x_m, H_m)
            painter.drawLine(QPointF(sx1, sy1), QPointF(sx2, sy2))

        # Horizontal row lines
        for r in range(1, rows):
            y_m = r * cell_sz
            sx1, sy1 = self.world_to_screen(0, y_m)
            sx2, sy2 = self.world_to_screen(W_m, y_m)
            painter.drawLine(QPointF(sx1, sy1), QPointF(sx2, sy2))

        # Draw Cell Labels (e.g. A1, C4)
        letters = self.grid_manager.ROW_LETTERS
        painter.setFont(font_cell)
        for r in range(rows):
            for c in range(cols):
                cell_name = f"{letters[c]}{r+1}"
                cx_m = (c + 0.5) * cell_sz
                cy_m = (r + 0.5) * cell_sz
                scx, scy = self.world_to_screen(cx_m, cy_m)

                painter.setPen(QColor(100, 116, 139, 100))
                painter.drawText(QRectF(scx - 20, scy - 10, 40, 20), Qt.AlignCenter, cell_name)

        # Draw Axis Column Letters (Bottom) and Row Numbers (Left)
        painter.setFont(font_label)
        painter.setPen(QColor("#00e5ff"))
        for c in range(cols):
            cx_m = (c + 0.5) * cell_sz
            scx, scy = self.world_to_screen(cx_m, 0)
            painter.drawText(QRectF(scx - 15, scy + 4, 30, 16), Qt.AlignCenter, letters[c])

        for r in range(rows):
            cy_m = (r + 0.5) * cell_sz
            scx, scy = self.world_to_screen(0, cy_m)
            painter.drawText(QRectF(scx - 22, scy - 8, 18, 16), Qt.AlignRight | Qt.AlignVCenter, str(r+1))

    def _draw_flight_trail(self, painter: QPainter):
        """Draws glowing flight history breadcrumb trail."""
        pen = QPen(QColor(0, 229, 255, 140), 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        painter.setPen(pen)

        for i in range(len(self.trail) - 1):
            p1 = self.world_to_screen(*self.trail[i])
            p2 = self.world_to_screen(*self.trail[i+1])
            painter.drawLine(QPointF(*p1), QPointF(*p2))

    def _draw_survivors(self, painter: QPainter):
        """Draws high-visibility autonomous survivor pins on map."""
        font_s = QFont("Segoe UI", 8, QFont.Bold)
        painter.setFont(font_s)

        for s_id, surv in self.survivors.items():
            sx, sy = self.world_to_screen(surv.x, surv.y)

            # Outer glowing pulse ring
            painter.setPen(QPen(QColor(0, 230, 118, 180), 2))
            painter.setBrush(QBrush(QColor(0, 230, 118, 50)))
            painter.drawEllipse(QPointF(sx, sy), 14, 14)

            # Center target pin
            painter.setPen(QPen(QColor("#ffffff"), 1.5))
            painter.setBrush(QBrush(QColor("#00e676")))
            painter.drawEllipse(QPointF(sx, sy), 6, 6)

            # Label Box (e.g. "S1 (95%) [B3]")
            label = f"{surv.survivor_id} ({int(surv.confidence*100)}%) [{surv.grid_cell}]"
            painter.setPen(QColor("#000000"))
            painter.setBrush(QBrush(QColor("#00e676")))
            tag_rect = QRectF(sx + 10, sy - 18, 90, 18)
            painter.drawRoundedRect(tag_rect, 3, 3)

            painter.setPen(QColor("#000000"))
            painter.drawText(tag_rect, Qt.AlignCenter, label)

    def _draw_drone(self, painter: QPainter):
        """Draws drone position marker and orientation cone."""
        dx, dy = self.world_to_screen(self.drone_x, self.drone_y)

        # 1. Heading FOV / Sensor Cone (90 deg forward cone)
        yaw_rad = math.radians(self.drone_yaw)
        cone_len = 25.0
        cone_spread = math.radians(35.0)

        left_angle = yaw_rad - cone_spread
        right_angle = yaw_rad + cone_spread

        # Invert dy component for screen coordinates
        p_left = QPointF(
            dx + math.cos(left_angle) * cone_len,
            dy - math.sin(left_angle) * cone_len
        )
        p_right = QPointF(
            dx + math.cos(right_angle) * cone_len,
            dy - math.sin(right_angle) * cone_len
        )

        cone_poly = QPolygonF([QPointF(dx, dy), p_left, p_right])
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(255, 234, 0, 45)))
        painter.drawPolygon(cone_poly)

        # 2. Drone Chassis Marker
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setBrush(QBrush(QColor("#ffea00")))
        painter.drawEllipse(QPointF(dx, dy), 7, 7)

        # 3. Forward Heading Vector Arrow
        tip_x = dx + math.cos(yaw_rad) * 16.0
        tip_y = dy - math.sin(yaw_rad) * 16.0
        painter.setPen(QPen(QColor("#ff1744"), 2.5, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(dx, dy), QPointF(tip_x, tip_y))

    def _draw_hud_overlays(self, painter: QPainter):
        """Renders Compass indicator and metric scale bar."""
        # Top-Right Compass / North Indicator
        cx = self.width() - 40
        cy = 40
        painter.setPen(QPen(QColor(255, 255, 255, 180), 1.5))
        painter.setBrush(QBrush(QColor(15, 23, 42, 180)))
        painter.drawEllipse(QPointF(cx, cy), 18, 18)

        # North Needle (Red Up)
        painter.setPen(QPen(QColor("#ff5252"), 2))
        painter.drawLine(QPointF(cx, cy), QPointF(cx, cy - 14))
        painter.setFont(QFont("Segoe UI", 7, QFont.Bold))
        painter.setPen(QColor("#ff5252"))
        painter.drawText(QRectF(cx - 10, cy - 28, 20, 12), Qt.AlignCenter, "N")

        # Bottom-Left Scale Bar (e.g. 5 meters)
        scale_m = 5.0
        bar_px = scale_m * self.scale
        if bar_px > 30:
            bx = 20
            by = self.height() - 25
            painter.setPen(QPen(QColor("#00e5ff"), 2))
            painter.drawLine(QPointF(bx, by), QPointF(bx + bar_px, by))
            painter.drawLine(QPointF(bx, by - 4), QPointF(bx, by + 4))
            painter.drawLine(QPointF(bx + bar_px, by - 4), QPointF(bx + bar_px, by + 4))

            painter.setFont(QFont("Consolas", 8, QFont.Bold))
            painter.setPen(QColor("#00e5ff"))
            painter.drawText(QRectF(bx, by - 16, bar_px, 14), Qt.AlignCenter, f"{int(scale_m)}m")
