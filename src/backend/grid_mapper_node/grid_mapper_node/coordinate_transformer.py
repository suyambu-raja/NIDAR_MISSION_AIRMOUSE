#!/usr/bin/env python3
"""
coordinate_transformer.py — NIDAR AirMouse: Grid Coordinate Transformer
=========================================================================
Standalone utility (NOT a ROS 2 node) used by grid_mapper_node.

Purpose
-------
Converts a survivor's world-frame position (X, Y in metres from SLAM) into
a human-readable arena grid box identifier such as "F4".

Algorithm (from mission spec)
------------------------------
    grid_column = int(world_x ÷ grid_box_size)   → 0=A, 1=B, 2=C, …
    grid_row    = int(world_y ÷ grid_box_size) + 1 → 1-indexed row number
    grid_id     = chr(65 + grid_column) + str(grid_row)

Arena layout
------------
    - 15 × 15 m, divided into 1 × 1 m grid boxes
    - 225 boxes total: columns A→O (15 cols), rows 1→15
    - grid_origin (0, 0) = arena entry point, calibrated at mission start
    - The SLAM map origin offset (map.info.origin) is subtracted before
      the grid calculation so that (0, 0) always maps to box "A1".

Edge cases handled
------------------
    - Exactly on a grid line: int() truncation places it in the lower box.
    - Outside arena bounds: returns None (caller must handle/log this).
    - Negative coordinates: also returns None (can't map to a grid letter).
"""

import math
from typing import Optional, Tuple


# =============================================================================
# Arena constants — must match mission spec and mock_slam.py
# =============================================================================

GRID_BOX_SIZE_M: float = 1.0   # Each grid box is 1×1 metre
ARENA_COLS: int = 15            # A–O  (columns, X axis, left→right)
ARENA_ROWS: int = 15            # 1–15 (rows, Y axis, bottom→top)


class CoordinateTransformer:
    """
    Converts SLAM world-frame (X, Y) coordinates into arena grid box IDs.

    The transformer is NOT a ROS 2 node; it is a plain Python class that
    grid_mapper_node instantiates once and calls per detection.

    Attributes
    ----------
    grid_origin_x : float
        X offset of the arena entry point in the SLAM map frame (metres).
        Default 0.0 — updated via `set_grid_origin()` at mission start.
    grid_origin_y : float
        Y offset of the arena entry point in the SLAM map frame (metres).
    map_origin_x : float
        X of cell (0,0) in the SLAM OccupancyGrid (from map.info.origin).
    map_origin_y : float
        Y of cell (0,0) in the SLAM OccupancyGrid (from map.info.origin).
    """

    def __init__(
        self,
        grid_origin_x: float = 0.0,
        grid_origin_y: float = 0.0,
        map_origin_x: float = 0.0,
        map_origin_y: float = 0.0,
    ) -> None:
        """
        Parameters
        ----------
        grid_origin_x / grid_origin_y
            Arena entry point in the SLAM map frame — this is (0,0) on the
            competition grid.  Updated when the drone first enters the arena.
        map_origin_x / map_origin_y
            Position of OccupancyGrid cell (0,0) expressed in the map frame
            (taken directly from OccupancyGrid.info.origin.position).
        """
        # The arena entry point in SLAM map coordinates (metres).
        # This is set once at mission start via set_grid_origin().
        self.grid_origin_x: float = grid_origin_x
        self.grid_origin_y: float = grid_origin_y

        # OccupancyGrid origin — the SLAM map frame position of cell (0,0).
        # The mock_slam.py publisher sets this to (-7.5, -7.5).
        self.map_origin_x: float = map_origin_x
        self.map_origin_y: float = map_origin_y

    # =========================================================================
    # Public API
    # =========================================================================

    def set_grid_origin(self, map_x: float, map_y: float) -> None:
        """
        Calibrate the arena entry point (grid origin) in the SLAM map frame.

        Call this once when the drone first enters the arena.  All subsequent
        coordinate transformations are relative to this point.

        Parameters
        ----------
        map_x, map_y : float
            Drone pose (from /drone_pose) at the moment the mission starts,
            expressed in the SLAM map frame (metres).
        """
        self.grid_origin_x = map_x
        self.grid_origin_y = map_y

    def set_map_origin(self, map_origin_x: float, map_origin_y: float) -> None:
        """
        Update the OccupancyGrid origin from /map.info.origin.

        Call this every time a new /map message arrives (the origin can shift
        as slam_toolbox extends the map during exploration).

        Parameters
        ----------
        map_origin_x, map_origin_y : float
            OccupancyGrid.info.origin.position.x / .y in metres.
        """
        self.map_origin_x = map_origin_x
        self.map_origin_y = map_origin_y

    def world_to_grid(
        self, world_x: float, world_y: float
    ) -> Optional[str]:
        """
        Convert a SLAM map-frame position to an arena grid box ID.

        Parameters
        ----------
        world_x, world_y : float
            Survivor position in the SLAM map frame (metres).
            This comes from SurvivorDetection.position.x / .y.

        Returns
        -------
        str or None
            Grid box ID such as "F4", or None if the position is
            outside the arena bounds.
        """
        # ------------------------------------------------------------------
        # Step 1: Subtract the arena grid origin so (0,0) = arena entry.
        # The grid_origin is the drone pose at mission start.
        # ------------------------------------------------------------------
        arena_x = world_x - self.grid_origin_x
        arena_y = world_y - self.grid_origin_y

        # ------------------------------------------------------------------
        # Step 2: Apply the coordinate transformation algorithm.
        #   grid_column = int(arena_x / 1.0)  →  0=A, 1=B, 2=C, …
        #   grid_row    = int(arena_y / 1.0) + 1  →  1-indexed row
        # ------------------------------------------------------------------
        grid_col, grid_row = self._arena_to_col_row(arena_x, arena_y)

        if grid_col is None or grid_row is None:
            # Position is outside the valid 15×15 grid
            return None

        # ------------------------------------------------------------------
        # Step 3: Format the grid ID — column letter + row number string
        # ------------------------------------------------------------------
        return self._format_grid_id(grid_col, grid_row)

    def world_to_grid_detailed(
        self, world_x: float, world_y: float
    ) -> Tuple[Optional[str], Optional[int], Optional[int], float, float]:
        """
        Like world_to_grid(), but also returns column/row indices and
        arena-frame coordinates.  Useful for the grid visualizer.

        Returns
        -------
        (grid_id, col_index, row_index, arena_x, arena_y)
            grid_id    : "F4" or None if out-of-bounds
            col_index  : 0-indexed column (0=A … 14=O) or None
            row_index  : 0-indexed row (0=row1 … 14=row15) or None
            arena_x    : X position relative to arena origin (metres)
            arena_y    : Y position relative to arena origin (metres)
        """
        arena_x = world_x - self.grid_origin_x
        arena_y = world_y - self.grid_origin_y

        grid_col, grid_row = self._arena_to_col_row(arena_x, arena_y)

        if grid_col is None or grid_row is None:
            return None, None, None, arena_x, arena_y

        grid_id = self._format_grid_id(grid_col, grid_row)
        return grid_id, grid_col, grid_row - 1, arena_x, arena_y

    def grid_to_world(
        self, grid_id: str
    ) -> Optional[Tuple[float, float]]:
        """
        Reverse lookup: convert a grid box ID to its world-frame centre.

        Parameters
        ----------
        grid_id : str
            Grid box string such as "F4".

        Returns
        -------
        (world_x, world_y) tuple in SLAM map frame, or None on invalid input.
        """
        if not grid_id or len(grid_id) < 2:
            return None

        col_char = grid_id[0].upper()
        try:
            row_num = int(grid_id[1:])
        except ValueError:
            return None

        # Convert letter → column index (0-based)
        col_idx = ord(col_char) - ord('A')
        # Convert row number → row index (0-based)
        row_idx = row_num - 1

        if not (0 <= col_idx < ARENA_COLS and 0 <= row_idx < ARENA_ROWS):
            return None

        # Centre of the grid box = (index + 0.5) * box_size + grid_origin
        world_x = (col_idx + 0.5) * GRID_BOX_SIZE_M + self.grid_origin_x
        world_y = (row_idx + 0.5) * GRID_BOX_SIZE_M + self.grid_origin_y

        return world_x, world_y

    # =========================================================================
    # Private helpers
    # =========================================================================

    def _arena_to_col_row(
        self, arena_x: float, arena_y: float
    ) -> Tuple[Optional[int], Optional[int]]:
        """
        Convert arena-relative (X, Y) to (column_index, row_number).

        Returns (None, None) if the position falls outside the 15×15 grid.

        Edge case: a position exactly on a grid boundary (e.g. arena_x=3.0)
        is placed in the *lower* cell (col 2=C) by int() truncation,
        matching the integer division spec from the mission document.

        Negative coordinates are treated as out-of-bounds.
        """
        if arena_x < 0.0 or arena_y < 0.0:
            # Negative → outside arena
            return None, None

        # Integer division truncation (not rounding) — per algorithm spec
        col_idx = int(arena_x / GRID_BOX_SIZE_M)
        row_num = int(arena_y / GRID_BOX_SIZE_M) + 1  # 1-indexed

        # Clamp: if exactly at the far edge (e.g. arena_x == 15.0),
        # the column would be 15 which is out-of-range.  Treat as the last box.
        # This covers the boundary edge case for positions exactly on the wall.
        if col_idx >= ARENA_COLS:
            if math.isclose(arena_x, ARENA_COLS * GRID_BOX_SIZE_M, rel_tol=1e-6):
                col_idx = ARENA_COLS - 1   # snap to last column
            else:
                return None, None           # truly outside

        if row_num > ARENA_ROWS:
            if math.isclose(arena_y, ARENA_ROWS * GRID_BOX_SIZE_M, rel_tol=1e-6):
                row_num = ARENA_ROWS        # snap to last row
            else:
                return None, None           # truly outside

        return col_idx, row_num

    @staticmethod
    def _format_grid_id(col_idx: int, row_num: int) -> str:
        """
        Build the grid box ID string.

        Parameters
        ----------
        col_idx : int  — 0-based column index (0=A … 14=O)
        row_num : int  — 1-based row number  (1 … 15)

        Returns
        -------
        str — e.g. "F4" for col_idx=5, row_num=4
        """
        # chr(65) = 'A', chr(66) = 'B', …
        return chr(65 + col_idx) + str(row_num)
