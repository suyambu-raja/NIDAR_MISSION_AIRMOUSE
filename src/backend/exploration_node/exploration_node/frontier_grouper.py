#!/usr/bin/env python3
"""
frontier_grouper.py — NIDAR AirMouse: Frontier BFS Grouper
===========================================================
Standalone utility (NOT a ROS 2 node) used by exploration_node.

Purpose
-------
Takes the raw list of frontier cells from FrontierDetector and groups
connected cells into meaningful frontier regions using BFS (Breadth-First
Search). Each group represents a distinct unexplored area the drone could
fly toward.

Why group?
----------
Raw frontier cells can number in the hundreds. Groups reduce this to a
handful of meaningful targets. A group's size tells us how much new
information we would gain by exploring it (information gain).

Algorithm: BFS connected-component labelling
--------------------------------------------
1. Build a fast lookup set of all frontier cell positions.
2. For each unvisited frontier cell:
   a. Start a BFS queue from that cell.
   b. Expand to all 8-connected frontier neighbours.
   c. Mark each visited cell to avoid re-processing.
   d. All cells reachable in this BFS = one frontier group.
3. Compute the group center as the mean (row, col) of all cells.
4. Convert center to world coordinates using map origin + resolution.
5. Discard groups smaller than MIN_GROUP_SIZE (avoids tiny noise pockets).

Connectivity: 8-connected (includes diagonal neighbours).
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# Minimum number of cells a group must contain to be considered useful.
# Smaller groups are typically sensor noise near corners or thin passages.
MIN_GROUP_SIZE: int = 3


@dataclass
class FrontierGroup:
    """
    One connected region of frontier cells.

    Attributes
    ----------
    center_world  : (world_x, world_y) of the group center in metres
    size          : number of frontier cells in the group
    cells         : list of (row, col) cell indices belonging to this group
    """
    center_world: Tuple[float, float]
    size: int
    cells: List[Tuple[int, int]] = field(default_factory=list)


class FrontierGrouper:
    """
    Groups frontier cells into connected regions using BFS.

    Usage (from exploration_node):
        grouper = FrontierGrouper()
        groups = grouper.group(frontier_cells, map_msg)
        # groups: List[FrontierGroup], sorted by size descending
    """

    def group(
        self,
        frontier_cells: List[Tuple[int, int]],
        map_msg,
    ) -> List[FrontierGroup]:
        """
        Cluster frontier cells into connected groups.

        Parameters
        ----------
        frontier_cells : List[Tuple[int,int]]
            Raw frontier cells from FrontierDetector.detect()
            as (row, col) pairs.
        map_msg : nav_msgs/OccupancyGrid
            Needed for width (grid topology) and info (for world conversion).

        Returns
        -------
        List[FrontierGroup]
            Groups sorted by size descending (largest = most info gain first).
            Empty list if no frontier_cells or all groups are too small.
        """
        # Guard: nothing to group
        if not frontier_cells:
            return []

        width      = map_msg.info.width
        height     = map_msg.info.height
        resolution = map_msg.info.resolution       # metres per cell
        origin_x   = map_msg.info.origin.position.x  # world X of cell (0,0)
        origin_y   = map_msg.info.origin.position.y  # world Y of cell (0,0)

        # Build a fast O(1) lookup set for BFS neighbour checks
        frontier_set = set(frontier_cells)

        visited: set = set()
        groups: List[FrontierGroup] = []

        for start_cell in frontier_cells:
            if start_cell in visited:
                continue  # already assigned to a group

            # --- BFS from this seed cell ---
            group_cells: List[Tuple[int, int]] = []
            queue = deque([start_cell])
            visited.add(start_cell)

            while queue:
                row, col = queue.popleft()
                group_cells.append((row, col))

                # Expand to all 8-connected frontier neighbours
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = row + dr, col + dc
                        # Bounds check
                        if nr < 0 or nr >= height or nc < 0 or nc >= width:
                            continue
                        neighbour = (nr, nc)
                        if neighbour in frontier_set and neighbour not in visited:
                            visited.add(neighbour)
                            queue.append(neighbour)

            # --- Filter out tiny groups (sensor noise) ---
            if len(group_cells) < MIN_GROUP_SIZE:
                continue

            # --- Compute group center in world coordinates ---
            mean_row = sum(r for r, _ in group_cells) / len(group_cells)
            mean_col = sum(c for _, c in group_cells) / len(group_cells)

            # Convert grid (row, col) → world (x, y)
            # world_x = origin_x + col * resolution + resolution/2 (cell center)
            # world_y = origin_y + row * resolution + resolution/2
            center_x = origin_x + (mean_col + 0.5) * resolution
            center_y = origin_y + (mean_row + 0.5) * resolution

            groups.append(FrontierGroup(
                center_world=(center_x, center_y),
                size=len(group_cells),
                cells=group_cells,
            ))

        # Sort largest group first — useful for logging and tie-breaking
        groups.sort(key=lambda g: g.size, reverse=True)
        return groups
