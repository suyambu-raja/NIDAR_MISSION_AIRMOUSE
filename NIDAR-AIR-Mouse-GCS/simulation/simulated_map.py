"""
Simulated Map and SLAM Provider for NIDAR AirMouse GCS.
Simulates progressive indoor 2D laser mapping of an indoor maze arena.
"""
import math
import time
from typing import List, Tuple
import numpy as np
from PySide6.QtCore import QTimer
from mapping.occupancy_grid import OccupancyGrid
from mapping.map_provider import BaseMapProvider
from communication.message_models import MapData


class SimulatedMapProvider(BaseMapProvider):
    """
    Generates dynamic SLAM occupancy grid updates as the drone navigates the indoor arena.
    """

    def __init__(
        self,
        width_meters: float = 20.0,
        height_meters: float = 20.0,
        resolution: float = 0.05,
        parent=None,
    ):
        super().__init__(parent)
        self.width_meters = width_meters
        self.height_meters = height_meters
        self.resolution = resolution

        self.width_cells = int(math.ceil(width_meters / resolution))
        self.height_cells = int(math.ceil(height_meters / resolution))

        self.occupancy_grid = OccupancyGrid(
            width_cells=self.width_cells,
            height_cells=self.height_cells,
            resolution=self.resolution,
            origin_x=0.0,
            origin_y=0.0,
        )

        # Build indoor maze wall segments [(x1, y1, x2, y2), ...]
        self._walls: List[Tuple[float, float, float, float]] = []
        self._generate_maze_walls()

        # Update timer (e.g. 5 Hz map publishing)
        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._publish_map)

        self._drone_x = 1.25
        self._drone_y = 1.25
        self._drone_yaw = 0.0

    def _generate_maze_walls(self):
        """Constructs an indoor layout with corridors, rooms, and partitions."""
        W = self.width_meters
        H = self.height_meters

        # Perimeter boundary walls (with entrance at (0, 1.25) and exit at (20, 18.75))
        self._walls.extend([
            (0, 0, W, 0),        # Bottom wall
            (0, H, W, H),        # Top wall
            (0, 0, 0, 0.5),      # Left bottom
            (0, 2.0, 0, H),      # Left top
            (W, 0, W, H - 2.0),  # Right bottom
            (W, H - 0.5, W, H),  # Right top
        ])

        # Internal corridors & room partition walls
        # Vertical walls with doorways
        self._walls.extend([
            # Wall 1 (X=5.0)
            (5.0, 0.0, 5.0, 4.0),
            (5.0, 6.5, 5.0, 14.0),
            (5.0, 16.5, 5.0, 20.0),
            
            # Wall 2 (X=10.0)
            (10.0, 2.5, 10.0, 8.5),
            (10.0, 11.5, 10.0, 17.5),
            
            # Wall 3 (X=15.0)
            (15.0, 0.0, 15.0, 6.0),
            (15.0, 8.5, 15.0, 15.0),
            (15.0, 17.5, 15.0, 20.0),
        ])

        # Horizontal room divider walls
        self._walls.extend([
            # Bottom row rooms
            (0.0, 5.0, 3.0, 5.0),
            (5.0, 5.0, 8.0, 5.0),
            (10.0, 5.0, 13.0, 5.0),
            
            # Middle section rooms
            (2.0, 10.0, 5.0, 10.0),
            (5.0, 12.0, 10.0, 12.0),
            (12.0, 10.0, 15.0, 10.0),
            (15.0, 12.0, 18.0, 12.0),
            
            # Top row rooms
            (0.0, 15.0, 3.5, 15.0),
            (7.5, 16.0, 10.0, 16.0),
            (10.0, 15.0, 13.5, 15.0),
        ])

    def update_drone_pose(self, x_m: float, y_m: float, yaw_deg: float):
        """Updates drone position and simulates 360-degree LiDAR raycasting."""
        self._drone_x = x_m
        self._drone_y = y_m
        self._drone_yaw = yaw_deg

        # Clear immediate vicinity around drone
        self.occupancy_grid.mark_circle_free(x_m, y_m, radius_m=0.75)

        # Cast 90 simulated LiDAR rays in 360 degrees
        num_rays = 90
        max_lidar_range = 6.0  # RPLIDAR indoor usable range ~ 6m

        for i in range(num_rays):
            angle = math.radians(i * (360.0 / num_rays))
            ray_dir_x = math.cos(angle)
            ray_dir_y = math.sin(angle)

            # Find closest wall intersection along ray
            min_dist = max_lidar_range
            hit_wall = False

            for wx1, wy1, wx2, wy2 in self._walls:
                dist = self._ray_segment_intersect(x_m, y_m, ray_dir_x, ray_dir_y, wx1, wy1, wx2, wy2)
                if dist is not None and 0.05 < dist < min_dist:
                    min_dist = dist
                    hit_wall = True

            hit_x = x_m + ray_dir_x * min_dist
            hit_y = y_m + ray_dir_y * min_dist

            self.occupancy_grid.update_ray(x_m, y_m, hit_x, hit_y, hit_obstacle=hit_wall)

    def _ray_segment_intersect(
        self,
        rx: float, ry: float, rdx: float, rdy: float,
        x1: float, y1: float, x2: float, y2: float
    ) -> Optional[float]:
        """Calculates distance along ray (rx, ry) + t*(rdx, rdy) to line segment (x1, y1)-(x2, y2)."""
        sx = x2 - x1
        sy = y2 - y1
        denom = rdx * sy - rdy * sx
        if abs(denom) < 1e-6:
            return None

        t = ((x1 - rx) * sy - (y1 - ry) * sx) / denom
        u = ((x1 - rx) * rdy - (y1 - ry) * rdx) / denom

        if t > 0.0 and 0.0 <= u <= 1.0:
            return t
        return None

    def start(self):
        super().start()
        self._timer.start()

    def stop(self):
        super().stop()
        self._timer.stop()

    def reset(self):
        self.occupancy_grid.reset()
        self._publish_map()

    def _publish_map(self):
        """Emits map_updated signal with latest occupancy grid."""
        if self._is_running:
            self.map_updated.emit(self.occupancy_grid.to_map_data())
