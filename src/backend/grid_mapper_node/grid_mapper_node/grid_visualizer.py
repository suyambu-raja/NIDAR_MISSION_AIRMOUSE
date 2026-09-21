#!/usr/bin/env python3
"""
grid_visualizer.py — NIDAR AirMouse: Grid Map Overlay Renderer
================================================================
Standalone utility (NOT a ROS 2 node) used by grid_mapper_node.

Purpose
-------
Takes a nav_msgs/OccupancyGrid (from /map) and produces a modified copy
with 1×1 m grid lines and survivor position markers overlaid.

The modified grid is published as /grid_map_overlay for the GCS dashboard.

Cell value encoding
--------------------
    0   = FREE space (no change from original map)
    100 = OCCUPIED (walls / obstacles — preserved from SLAM)
    -1  = UNKNOWN (unexplored — preserved from SLAM)
    75  = Grid line (drawn at every 1-metre boundary)
    50  = Survivor marker (drawn at each confirmed survivor's position)

Why a copy and not in-place mutation?
--------------------------------------
The /map data is a Python list (or bytes).  We copy it so we don't corrupt
the original map data stored in grid_mapper_node — the original is needed
for the next overlay render cycle.

Performance note
-----------------
For a 300×300 = 90,000-cell map at 5 cm/cell resolution, the grid lines
at 1 m intervals are drawn at every 20th cell.  The total overlay
computation is O(W + H) per frame which is negligible on Raspberry Pi 4.
"""

import copy
from typing import List, Optional

# Only imported for type annotations — grid_visualizer does not init rclpy.
# This makes unit tests runnable without a live ROS 2 environment.
try:
    from nav_msgs.msg import OccupancyGrid
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    OccupancyGrid = None  # type: ignore


# =============================================================================
# Overlay cell values
# =============================================================================

CELL_GRID_LINE: int = 75        # Drawn at every 1-metre grid boundary
CELL_SURVIVOR: int = 50         # Drawn at each confirmed survivor location
CELL_OCCUPIED: int = 100        # Wall/obstacle (from SLAM — not overwritten)


class GridVisualizer:
    """
    Renders 1×1 m grid lines and survivor markers onto an OccupancyGrid.

    Usage (from grid_mapper_node)
    ------------------------------
        visualizer = GridVisualizer(grid_box_size_m=1.0, arena_size_m=15.0)

        overlay = visualizer.render(
            base_map=map_msg,           # nav_msgs/OccupancyGrid from /map
            survivor_world_positions=[(x1,y1), (x2,y2)],
            grid_origin_x=0.0,
            grid_origin_y=0.0,
        )
        # overlay is a new OccupancyGrid ready to publish on /grid_map_overlay
    """

    def __init__(
        self,
        grid_box_size_m: float = 1.0,
        arena_size_m: float = 15.0,
    ) -> None:
        """
        Parameters
        ----------
        grid_box_size_m : float
            Side length of each grid box in metres (default 1.0).
        arena_size_m : float
            Total arena size in metres (default 15.0 for a 15×15 m arena).
        """
        self.grid_box_size_m: float = grid_box_size_m
        self.arena_size_m: float = arena_size_m

    # =========================================================================
    # Public API
    # =========================================================================

    def render(
        self,
        base_map: 'OccupancyGrid',
        survivor_world_positions: Optional[List[tuple]] = None,
        grid_origin_x: float = 0.0,
        grid_origin_y: float = 0.0,
    ) -> 'OccupancyGrid':
        """
        Produce a new OccupancyGrid with grid lines and survivor markers.

        Parameters
        ----------
        base_map : nav_msgs/OccupancyGrid
            The latest map from /map.  Its header, info, and data are all used.
        survivor_world_positions : list of (x, y) tuples, optional
            SLAM map-frame positions of all confirmed survivors (metres).
            If None or empty, no survivor markers are drawn.
        grid_origin_x, grid_origin_y : float
            Arena entry point in the SLAM map frame — grid cell (A,1) starts
            here.  Defaults to (0,0) — overridden by grid_mapper_node with
            the calibrated origin.

        Returns
        -------
        nav_msgs/OccupancyGrid
            A new message (not the same object as base_map) ready to publish.
        """
        # ------------------------------------------------------------------
        # Step 1: Deep-copy the base map so we don't mutate the original.
        # ------------------------------------------------------------------
        overlay = self._copy_map(base_map)

        # Convenience aliases for readability
        width: int = overlay.info.width          # cells in X (columns)
        height: int = overlay.info.height        # cells in Y (rows)
        resolution: float = overlay.info.resolution   # metres per cell

        # Convert mutable bytes to list[int] so we can index-assign
        data: List[int] = list(overlay.data)

        # ------------------------------------------------------------------
        # Step 2: Draw vertical grid lines (every grid_box_size_m in X).
        # ------------------------------------------------------------------
        # How many cells per grid box (e.g. 1.0m / 0.05m = 20 cells)
        cells_per_box_x = int(self.grid_box_size_m / resolution)
        cells_per_box_y = int(self.grid_box_size_m / resolution)

        # Grid origin offset from map's cell(0,0) origin, in cells
        # map_origin is the world position of cell(0,0); grid_origin is in world coords.
        map_ox: float = base_map.info.origin.position.x
        map_oy: float = base_map.info.origin.position.y

        # Pixel offset of arena grid origin relative to map cell (0,0)
        grid_col_offset = int((grid_origin_x - map_ox) / resolution)
        grid_row_offset = int((grid_origin_y - map_oy) / resolution)

        # Draw vertical lines at col = grid_col_offset + n*cells_per_box_x
        col = grid_col_offset
        while 0 <= col < width:
            # Draw a full vertical stripe (all rows at this column)
            for row in range(height):
                idx = row * width + col
                if 0 <= idx < len(data):
                    # Only draw on FREE cells — don't overwrite walls
                    if data[idx] == 0:
                        data[idx] = CELL_GRID_LINE
            col += cells_per_box_x

        # Also draw lines going backward from origin (negative direction)
        col = grid_col_offset - cells_per_box_x
        while col >= 0:
            for row in range(height):
                idx = row * width + col
                if 0 <= idx < len(data):
                    if data[idx] == 0:
                        data[idx] = CELL_GRID_LINE
            col -= cells_per_box_x

        # ------------------------------------------------------------------
        # Step 3: Draw horizontal grid lines (every grid_box_size_m in Y).
        # ------------------------------------------------------------------
        row = grid_row_offset
        while 0 <= row < height:
            # Draw a full horizontal stripe (all columns at this row)
            for col_i in range(width):
                idx = row * width + col_i
                if 0 <= idx < len(data):
                    if data[idx] == 0:
                        data[idx] = CELL_GRID_LINE
            row += cells_per_box_y

        # Also draw lines backward from origin
        row = grid_row_offset - cells_per_box_y
        while row >= 0:
            for col_i in range(width):
                idx = row * width + col_i
                if 0 <= idx < len(data):
                    if data[idx] == 0:
                        data[idx] = CELL_GRID_LINE
            row -= cells_per_box_y

        # ------------------------------------------------------------------
        # Step 4: Mark each confirmed survivor location with CELL_SURVIVOR.
        # Uses a 3×3 pixel "cross" pattern so it's visible at small zoom.
        # ------------------------------------------------------------------
        if survivor_world_positions:
            for (sx, sy) in survivor_world_positions:
                # Convert world position → map cell index
                cell_col = int((sx - map_ox) / resolution)
                cell_row = int((sy - map_oy) / resolution)

                # Draw a small 3×3 cross centred on the survivor cell
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        r = cell_row + dr
                        c = cell_col + dc
                        if 0 <= r < height and 0 <= c < width:
                            idx = r * width + c
                            # Survivor marker overwrites grid lines but NOT walls
                            if data[idx] != CELL_OCCUPIED:
                                data[idx] = CELL_SURVIVOR

        # ------------------------------------------------------------------
        # Step 5: Write the modified data back and return
        # ------------------------------------------------------------------
        overlay.data = data
        return overlay

    # =========================================================================
    # Private helpers
    # =========================================================================

    @staticmethod
    def _copy_map(base_map: 'OccupancyGrid') -> 'OccupancyGrid':
        """
        Create a deep copy of an OccupancyGrid message.

        We cannot use copy.deepcopy() on ROS 2 messages in all environments
        (message classes may not implement __deepcopy__), so we build a new
        message and copy each field manually.

        For `data`, we make a list copy — this is safe because OccupancyGrid
        data is a flat int8[] that Python represents as a list or bytes object.
        """
        overlay = OccupancyGrid()

        # Copy header (stamp + frame_id)
        overlay.header.stamp = base_map.header.stamp
        overlay.header.frame_id = base_map.header.frame_id

        # Copy map metadata
        overlay.info.resolution = base_map.info.resolution
        overlay.info.width = base_map.info.width
        overlay.info.height = base_map.info.height
        overlay.info.map_load_time = base_map.info.map_load_time

        # Copy origin pose
        overlay.info.origin.position.x = base_map.info.origin.position.x
        overlay.info.origin.position.y = base_map.info.origin.position.y
        overlay.info.origin.position.z = base_map.info.origin.position.z
        overlay.info.origin.orientation.x = base_map.info.origin.orientation.x
        overlay.info.origin.orientation.y = base_map.info.origin.orientation.y
        overlay.info.origin.orientation.z = base_map.info.origin.orientation.z
        overlay.info.origin.orientation.w = base_map.info.origin.orientation.w

        # Copy cell data — explicit list() to ensure mutability
        overlay.data = list(base_map.data)

        return overlay
