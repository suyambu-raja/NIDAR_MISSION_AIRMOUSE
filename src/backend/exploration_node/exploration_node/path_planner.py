#!/usr/bin/env python3
"""
path_planner.py — NIDAR AirMouse: A* Path Planner
===================================================
Standalone utility (NOT a ROS 2 node) used by exploration_node.

Purpose
-------
Given the current OccupancyGrid, a start world position, and a goal world
position, computes the shortest collision-free path using A* search.

The path is returned as an ordered list of world-frame (X, Y) waypoints
that exploration_node can relay to mavros_bridge via /planned_path.

Algorithm: A* (Hart, Nilsson & Raphael, 1968)
----------------------------------------------
A* is optimal and complete for grid graphs. It expands cells in order of
f(n) = g(n) + h(n):
    g(n) = actual cost from start to cell n (sum of step distances)
    h(n) = Euclidean distance heuristic from n to goal
    f(n) = total estimated path cost through n

8-connected movement is used so the drone can move diagonally.
Step cost: 1.0 for cardinal moves, sqrt(2) for diagonal moves.

Wall safety margin
------------------
Cells within SAFETY_CELLS of any OCCUPIED cell (walls) are marked as
unsafe and not expanded. This provides a 0.3m clearance buffer
(6 cells * 0.05m/cell) around all walls, preventing the drone from
flying too close to obstacles.

Performance
-----------
On a 300x300 map worst-case A* is O(n log n) ≈ 70,000 expansions.
With the priority queue (heapq) this takes ~100ms on Raspberry Pi 4 —
acceptable for a 1 Hz planning loop.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

# OccupancyGrid cell value constants
CELL_FREE     = 0
CELL_OCCUPIED = 100
CELL_UNKNOWN  = -1

# Safety clearance: number of cells to keep away from walls.
# 6 cells * 0.05m/cell = 0.30m clearance.
SAFETY_CELLS: int = 6


class PathPlanner:
    """
    A* path planner on a nav_msgs/OccupancyGrid.

    Usage (from exploration_node):
        planner = PathPlanner()
        waypoints = planner.plan(map_msg, (start_x, start_y), (goal_x, goal_y))
        if waypoints is not None:
            # list of (world_x, world_y) tuples from start to goal
    """

    def plan(
        self,
        map_msg,
        start_world: Tuple[float, float],
        goal_world:  Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a safe path from start_world to goal_world.

        Parameters
        ----------
        map_msg : nav_msgs/OccupancyGrid
            Current occupancy grid from /map.
        start_world : (float, float)
            Drone's current position in SLAM map frame (metres).
        goal_world : (float, float)
            Target position in SLAM map frame (metres).

        Returns
        -------
        List of (world_x, world_y) waypoints, or None if:
            - start or goal are out of map bounds
            - goal is inside a wall
            - no collision-free path exists
        """
        width      = map_msg.info.width
        height     = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x   = map_msg.info.origin.position.x
        origin_y   = map_msg.info.origin.position.y

        # Convert world positions to grid (row, col) indices
        start_cell = self._world_to_cell(start_world, origin_x, origin_y, resolution, width, height)
        goal_cell  = self._world_to_cell(goal_world,  origin_x, origin_y, resolution, width, height)

        # Guard: positions outside map bounds
        if start_cell is None or goal_cell is None:
            return None

        # Guard: start equals goal — already at destination
        if start_cell == goal_cell:
            return [goal_world]

        # Build the safe-cell lookup mask (pre-computed, O(n))
        safe_mask = self._build_safe_mask(map_msg.data, width, height)

        # Guard: goal cell is unsafe (inside wall or too close to one)
        if not safe_mask[goal_cell[0] * width + goal_cell[1]]:
            # Try to find the nearest safe cell to the goal within 3 cells
            goal_cell = self._nearest_safe_cell(goal_cell, safe_mask, width, height)
            if goal_cell is None:
                return None

        # ---- A* Search ----
        came_from: Dict[Tuple[int,int], Optional[Tuple[int,int]]] = {}
        g_cost:    Dict[Tuple[int,int], float] = {}

        # Priority queue entries: (f_cost, (row, col))
        open_heap: List[Tuple[float, Tuple[int,int]]] = []

        g_cost[start_cell] = 0.0
        h = self._heuristic(start_cell, goal_cell)
        heapq.heappush(open_heap, (h, start_cell))
        came_from[start_cell] = None

        while open_heap:
            _, current = heapq.heappop(open_heap)

            # Goal reached — reconstruct path
            if current == goal_cell:
                return self._reconstruct_path(
                    came_from, goal_cell, origin_x, origin_y, resolution
                )

            # Expand 8 neighbours
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue

                    nr = current[0] + dr
                    nc = current[1] + dc

                    # Bounds check
                    if nr < 0 or nr >= height or nc < 0 or nc >= width:
                        continue

                    neighbour = (nr, nc)
                    idx = nr * width + nc

                    # Skip unsafe cells (walls / too close to walls / unknown)
                    if not safe_mask[idx]:
                        continue

                    # Step cost: 1.0 for cardinal, sqrt(2) for diagonal
                    step = math.sqrt(2) if (dr != 0 and dc != 0) else 1.0
                    tentative_g = g_cost[current] + step

                    if neighbour not in g_cost or tentative_g < g_cost[neighbour]:
                        g_cost[neighbour] = tentative_g
                        came_from[neighbour] = current
                        f = tentative_g + self._heuristic(neighbour, goal_cell)
                        heapq.heappush(open_heap, (f, neighbour))

        # Open heap exhausted — no path found
        return None

    # =========================================================================
    # Private helpers
    # =========================================================================

    @staticmethod
    def _world_to_cell(
        world: Tuple[float, float],
        origin_x: float, origin_y: float,
        resolution: float,
        width: int, height: int,
    ) -> Optional[Tuple[int, int]]:
        """Convert world (x, y) → grid (row, col). Returns None if out of bounds."""
        col = int((world[0] - origin_x) / resolution)
        row = int((world[1] - origin_y) / resolution)
        if 0 <= row < height and 0 <= col < width:
            return (row, col)
        return None

    @staticmethod
    def _cell_to_world(
        cell: Tuple[int, int],
        origin_x: float, origin_y: float,
        resolution: float,
    ) -> Tuple[float, float]:
        """Convert grid (row, col) → world center (x, y)."""
        row, col = cell
        wx = origin_x + (col + 0.5) * resolution
        wy = origin_y + (row + 0.5) * resolution
        return (wx, wy)

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic between two grid cells."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    @staticmethod
    def _build_safe_mask(data, width: int, height: int) -> List[bool]:
        """
        Pre-compute a boolean mask: True = safe to enter, False = unsafe.

        A cell is unsafe if:
          - It is OCCUPIED (value == 100)
          - It is UNKNOWN  (value == -1)  — avoid flying into unmapped areas
          - It is within SAFETY_CELLS of any OCCUPIED cell

        We use a two-pass approach:
          Pass 1: mark all OCCUPIED cells.
          Pass 2: inflate occupied cells by SAFETY_CELLS using a bounding box
                  approximation (Chebyshev / L-inf distance), which is O(n)
                  and fast enough for real-time use on Raspberry Pi 4.
        """
        total = width * height
        # Start: safe only if FREE
        safe = [data[i] == CELL_FREE for i in range(total)]

        # Inflate walls by SAFETY_CELLS
        # For each occupied cell, mark its neighbourhood as unsafe
        s = SAFETY_CELLS
        for row in range(height):
            for col in range(width):
                if data[row * width + col] == CELL_OCCUPIED:
                    # Mark a (2s+1)x(2s+1) box around this wall cell as unsafe
                    for dr in range(-s, s + 1):
                        for dc in range(-s, s + 1):
                            nr, nc = row + dr, col + dc
                            if 0 <= nr < height and 0 <= nc < width:
                                safe[nr * width + nc] = False

        return safe

    @staticmethod
    def _nearest_safe_cell(
        cell: Tuple[int, int],
        safe_mask: List[bool],
        width: int, height: int,
        search_radius: int = 10,
    ) -> Optional[Tuple[int, int]]:
        """Find the nearest safe cell within search_radius of cell."""
        row, col = cell
        best = None
        best_d = float('inf')
        for dr in range(-search_radius, search_radius + 1):
            for dc in range(-search_radius, search_radius + 1):
                nr, nc = row + dr, col + dc
                if 0 <= nr < height and 0 <= nc < width:
                    if safe_mask[nr * width + nc]:
                        d = math.sqrt(dr*dr + dc*dc)
                        if d < best_d:
                            best_d = d
                            best = (nr, nc)
        return best

    def _reconstruct_path(
        self,
        came_from: Dict,
        goal: Tuple[int, int],
        origin_x: float, origin_y: float,
        resolution: float,
    ) -> List[Tuple[float, float]]:
        """
        Walk backwards from goal through came_from to reconstruct the path,
        then reverse it and convert each cell to world coordinates.
        """
        path = []
        current: Optional[Tuple[int,int]] = goal
        while current is not None:
            path.append(self._cell_to_world(current, origin_x, origin_y, resolution))
            current = came_from.get(current)
        path.reverse()
        return path
