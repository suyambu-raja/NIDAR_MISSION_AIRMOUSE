#!/usr/bin/env python3
"""
battery_monitor.py — NIDAR AirMouse: Battery-Aware Replanning
==============================================================
Standalone utility (NOT a ROS 2 node) used by exploration_node.

Purpose
-------
Monitors the drone's battery percentage and determines which exploration
strategy the drone should follow. When battery gets low, the drone must
stop exploring new areas and head back toward the arena exit point.

Three strategies
----------------
EXPLORE  (battery > 30%) — Normal frontier exploration. The drone freely
         picks the highest-utility frontier anywhere in the arena.

RETURN   (15% < battery ≤ 30%) — Bias exploration toward the exit point.
         The frontier selector should still find frontiers, but
         exploration_node will inject the exit point as the goal when no
         high-score frontier is close enough to home.

EXIT     (battery ≤ 15%) — Emergency. Ignore all frontiers immediately.
         Navigate directly to the arena exit point (0.0, 0.0) and land.
         A CRITICAL alert is also sent to /exploration_status.

Exit point
----------
The arena entry/exit point is always (0.0, 0.0) in the SLAM map frame —
this is the point where the drone entered the arena and where it must
return by mission end. The competition organiser recharges the drone here.

Thresholds (from competition logistics planning):
    > 30%  → sufficient for several more minutes of exploration
    15-30% → start navigating home before it's too late
    < 15%  → no time left — abort exploration and exit immediately
"""

from enum import Enum, auto


class Strategy(Enum):
    """Exploration strategy determined by battery level."""
    EXPLORE  = auto()   # > 30%  — full frontier exploration
    RETURN   = auto()   # 15–30% — bias toward exit, wrap up exploration
    EXIT     = auto()   # < 15%  — abort, fly to exit immediately


# Battery thresholds (percentage, 0–100)
NORMAL_THRESHOLD: float = 30.0   # Above this: EXPLORE
LOW_THRESHOLD:    float = 15.0   # Above this (and ≤ NORMAL): RETURN
                                  # At or below: EXIT

# Arena exit point in SLAM map frame — always the drone's starting position
EXIT_POINT_X: float = 0.0
EXIT_POINT_Y: float = 0.0


class BatteryMonitor:
    """
    Tracks battery state and recommends the appropriate exploration strategy.

    Usage (from exploration_node):
        monitor = BatteryMonitor()

        # On each /battery_status callback:
        strategy = monitor.update(battery_msg.percentage * 100)
        if strategy == Strategy.EXIT:
            # Navigate directly to exit
    """

    def __init__(self) -> None:
        # Track current strategy to detect transitions (avoid log spam)
        self._current_strategy: Strategy = Strategy.EXPLORE
        self._last_logged_pct: float = 100.0
        # Whether we have already emitted a CRITICAL alert this flight
        self._critical_alerted: bool = False

    @property
    def exit_point(self):
        """Arena exit / entry point as (x, y) world coordinates."""
        return (EXIT_POINT_X, EXIT_POINT_Y)

    def update(self, battery_percentage: float) -> Strategy:
        """
        Evaluate the current battery level and return the correct strategy.

        Parameters
        ----------
        battery_percentage : float
            Battery charge from 0.0 (empty) to 100.0 (full).
            Derived from sensor_msgs/BatteryState.percentage * 100.

        Returns
        -------
        Strategy
            The exploration strategy the drone should follow right now.
        """
        # Clamp to [0, 100] in case of sensor noise
        pct = max(0.0, min(100.0, battery_percentage))

        # Determine strategy from thresholds
        if pct > NORMAL_THRESHOLD:
            new_strategy = Strategy.EXPLORE
        elif pct > LOW_THRESHOLD:
            new_strategy = Strategy.RETURN
        else:
            new_strategy = Strategy.EXIT

        self._current_strategy = new_strategy
        self._last_logged_pct = pct
        return new_strategy

    @property
    def is_critical(self) -> bool:
        """True when battery is in EXIT state."""
        return self._current_strategy == Strategy.EXIT

    @property
    def current_strategy(self) -> Strategy:
        """The last strategy returned by update()."""
        return self._current_strategy

    @property
    def last_percentage(self) -> float:
        """The battery percentage passed to the most recent update() call."""
        return self._last_logged_pct

    def strategy_label(self) -> str:
        """Human-readable strategy name for status messages."""
        return self._current_strategy.name
