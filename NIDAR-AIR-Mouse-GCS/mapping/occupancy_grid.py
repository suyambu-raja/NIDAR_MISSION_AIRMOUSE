"""
Occupancy Grid Data Structure for NIDAR AirMouse GCS.
Maintains a 2D discrete probability representation of explored indoor space.
Values:
  -1 : Unknown / Unexplored
   0 : Free / Explored corridor/room space
 100 : Occupied / Wall / Obstacle
"""
import math
from typing import Tuple, Optional, List
import numpy as np
from communication.message_models import MapData


class OccupancyGrid:
    """
    High-performance 2D Occupancy Grid Map representing GPS-denied indoor environments.
    Compatible with ROS 2 nav_msgs/OccupancyGrid standard.
    """

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(
        self,
        width_cells: int = 400,
        height_cells: int = 400,
        resolution: float = 0.05,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ):
        self.width = int(width_cells)
        self.height = int(height_cells)
        self.resolution = float(resolution)  # meters per cell
        self.origin_x = float(origin_x)      # metric X of cell (0, 0)
        self.origin_y = float(origin_y)      # metric Y of cell (0, 0)

        # Initialize full grid with UNKNOWN (-1)
        self.grid = np.full((self.height, self.width), self.UNKNOWN, dtype=np.int8)

    def reset(self):
        """Resets all grid cells to UNKNOWN."""
        self.grid.fill(self.UNKNOWN)

    def world_to_grid(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Converts metric world coordinates (meters) to grid cell indices (col, row)."""
        col = int(math.floor((x_m - self.origin_x) / self.resolution))
        row = int(math.floor((y_m - self.origin_y) / self.resolution))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Converts grid cell indices (col, row) to world coordinates (meters) at cell center."""
        x_m = self.origin_x + (col + 0.5) * self.resolution
        y_m = self.origin_y + (row + 0.5) * self.resolution
        return x_m, y_m

    def is_in_bounds(self, col: int, row: int) -> bool:
        """Checks if cell index (col, row) is within map boundaries."""
        return 0 <= col < self.width and 0 <= row < self.height

    def set_cell(self, col: int, row: int, value: int):
        """Sets a specific grid cell value with bounds checking."""
        if self.is_in_bounds(col, row):
            self.grid[row, col] = np.int8(value)

    def get_cell(self, col: int, row: int) -> int:
        """Gets value of a grid cell. Returns UNKNOWN if out of bounds."""
        if self.is_in_bounds(col, row):
            return int(self.grid[row, col])
        return self.UNKNOWN

    def mark_circle_free(self, center_x_m: float, center_y_m: float, radius_m: float):
        """Marks all cells within a circular radius as FREE (0) if not already occupied."""
        center_col, center_row = self.world_to_grid(center_x_m, center_y_m)
        radius_cells = int(math.ceil(radius_m / self.resolution))

        min_col = max(0, center_col - radius_cells)
        max_col = min(self.width - 1, center_col + radius_cells)
        min_row = max(0, center_row - radius_cells)
        max_row = min(self.height - 1, center_row + radius_cells)

        r_sq = radius_cells * radius_cells
        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                if (c - center_col) ** 2 + (r - center_row) ** 2 <= r_sq:
                    if self.grid[r, c] != self.OCCUPIED:
                        self.grid[r, c] = self.FREE

    def update_ray(self, start_x_m: float, start_y_m: float, end_x_m: float, end_y_m: float, hit_obstacle: bool = True):
        """
        Traces a simulated LiDAR/sensor ray using Bresenham's line algorithm.
        Marks intermediate cells as FREE and the endpoint as OCCUPIED (if hit_obstacle=True).
        """
        c0, r0 = self.world_to_grid(start_x_m, start_y_m)
        c1, r1 = self.world_to_grid(end_x_m, end_y_m)

        # Bresenham's line algorithm
        dx = abs(c1 - c0)
        dy = abs(r1 - r0)
        sx = 1 if c0 < c1 else -1
        sy = 1 if r0 < r1 else -1
        err = dx - dy

        curr_c, curr_r = c0, r0
        while True:
            is_endpoint = (curr_c == c1 and curr_r == r1)

            if self.is_in_bounds(curr_c, curr_r):
                if is_endpoint and hit_obstacle:
                    self.grid[curr_r, curr_c] = self.OCCUPIED
                else:
                    if self.grid[curr_r, curr_c] != self.OCCUPIED:
                        self.grid[curr_r, curr_c] = self.FREE

            if is_endpoint:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_c += sx
            if e2 < dx:
                err += dx
                curr_r += sy

    def to_map_data(self) -> MapData:
        """Packages current occupancy grid into a MapData message."""
        return MapData(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            grid=self.grid.copy(),
        )

    def load_from_map_data(self, map_data: MapData):
        """Updates internal grid from an incoming MapData message."""
        self.width = map_data.width
        self.height = map_data.height
        self.resolution = map_data.resolution
        self.origin_x = map_data.origin_x
        self.origin_y = map_data.origin_y

        if isinstance(map_data.grid, np.ndarray):
            self.grid = map_data.grid.copy()
        else:
            self.grid = np.array(map_data.grid, dtype=np.int8)
