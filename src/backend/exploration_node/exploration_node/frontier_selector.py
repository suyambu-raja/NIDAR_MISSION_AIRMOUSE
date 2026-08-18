#!/usr/bin/env python3
"""
frontier_selector.py — NIDAR AirMouse: Frontier Utility Selector
=================================================================
Standalone utility (NOT a ROS 2 node) used by exploration_node.

Purpose
-------
Given a list of frontier groups and the drone's current world position,
selects the BEST frontier to fly toward using an information-gain utility
function that balances exploration value against travel cost.

Utility Formula
---------------
    utility_score = information_gain - (travel_cost * WEIGHT)

Where:
    information_gain = frontier group SIZE (number of frontier cells)
                       More cells → more unknown area revealed → higher value
    travel_cost      = Euclidean distance (metres) from drone to frontier center
                       Closer frontiers are preferred for efficiency
    WEIGHT           = 0.5 (tunable — higher = prefer closer, lower = prefer larger)

This is a greedy one-step lookahead: the drone picks the frontier that gives
the most expected information relative to how far it has to fly to get there.

Tie-breaking
------------
If two frontiers have equal utility scores (rare with floats, but possible
after rounding), the closer one is chosen — minimising unnecessary travel.
"""

import math
from typing import List, Optional

# Import the FrontierGroup data class defined in frontier_grouper
from exploration_node.frontier_grouper import FrontierGroup

# Weight controlling the trade-off between info gain and travel distance.
# 0.5 means: each extra metre of travel costs 0.5 units of utility score.
# Increase to explore closer areas first; decrease to prefer larger frontiers.
TRAVEL_COST_WEIGHT: float = 0.5


class FrontierSelector:
    """
    Selects the highest-utility frontier group for the drone to explore next.

    Usage (from exploration_node):
        selector = FrontierSelector()
        best = selector.select(groups, drone_x, drone_y)
        if best is not None:
            goal_x, goal_y = best.center_world
    """

    def select(
        self,
        groups: List[FrontierGroup],
        drone_world_x: float,
        drone_world_y: float,
    ) -> Optional[FrontierGroup]:
        """
        Pick the frontier group with the highest utility score.

        Parameters
        ----------
        groups : List[FrontierGroup]
            All valid frontier groups from FrontierGrouper.group().
            May be empty.
        drone_world_x : float
            Drone's current X position in the SLAM map frame (metres).
        drone_world_y : float
            Drone's current Y position in the SLAM map frame (metres).

        Returns
        -------
        FrontierGroup or None
            The group with the highest utility, or None if groups is empty.
        """
        # Edge case: no frontiers available (fully explored or no map yet)
        if not groups:
            return None

        best_group:  Optional[FrontierGroup] = None
        best_score:  float = float('-inf')
        best_dist:   float = float('inf')

        for group in groups:
            gx, gy = group.center_world

            # Euclidean distance from drone to this frontier's center
            dist = math.sqrt(
                (gx - drone_world_x) ** 2 +
                (gy - drone_world_y) ** 2
            )

            # Information gain = how many unexplored cells this frontier borders
            information_gain = float(group.size)

            # Utility: value of exploring this frontier minus cost to reach it
            score = information_gain - (dist * TRAVEL_COST_WEIGHT)

            # Tie-break: if scores are equal, prefer the closer frontier
            if score > best_score or (
                math.isclose(score, best_score, rel_tol=1e-9)
                and dist < best_dist
            ):
                best_score = score
                best_dist  = dist
                best_group = group

        return best_group

    def score(
        self,
        group: FrontierGroup,
        drone_world_x: float,
        drone_world_y: float,
    ) -> float:
        """
        Compute the utility score for a single group without selecting.

        Useful for logging and debugging from exploration_node.
        """
        gx, gy = group.center_world
        dist = math.sqrt(
            (gx - drone_world_x) ** 2 + (gy - drone_world_y) ** 2
        )
        return float(group.size) - (dist * TRAVEL_COST_WEIGHT)
