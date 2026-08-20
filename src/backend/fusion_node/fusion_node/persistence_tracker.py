#!/usr/bin/env python3
"""
NIDAR AirMouse — Persistence Tracker
Tracks how many consecutive frames a detection appears at the same
world position. Only detections that persist for enough frames pass
through to the confirmation stage.

Algorithm:
    NORMAL visibility:   require 5 consecutive frames
    DEGRADED visibility: require 8 consecutive frames
    Position tolerance:  0.3m (detections within this range are "same position")

    if detection at same position appears → increment counter
    if detection disappears → reset counter
    if counter >= threshold → detection passes persistence gate

Design rationale:
    - Prevents single-frame false positives from triggering confirmation
    - DEGRADED needs more frames because thermal has more noise/artifacts
    - Position tolerance of 0.3m accounts for drone movement jitter
    - Stale entries aged out after max_stale_frames to prevent memory leak
"""

import math
import time
from dataclasses import dataclass
from typing import List, Tuple

from .visibility_estimator import VisibilityState


@dataclass
class PersistentDetection:
    """
    Tracks the persistence state of a single detection at a world position.
    """
    x: float                    # World position X (metres)
    y: float                    # World position Y (metres)
    confidence: float           # Latest fused confidence score
    source: str                 # "rgb", "thermal", or "fused"
    consecutive_frames: int     # Number of consecutive frames seen
    last_update_tick: int       # Last frame tick this detection was updated
    track_id: int = -1          # Tracker-assigned ID if available
    bbox: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


class PersistenceTracker:
    """
    Tracks consecutive frame counts per detection position and gates
    detections that haven't persisted long enough.
    """

    def __init__(
        self,
        persistence_normal: int = 5,
        persistence_degraded: int = 8,
        position_tolerance: float = 0.3,
        max_stale_frames: int = 15
    ):
        """
        Args:
            persistence_normal: Frames required for confirmation in NORMAL visibility
            persistence_degraded: Frames required for confirmation in DEGRADED visibility
            position_tolerance: Maximum distance (metres) between frames for
                                a detection to be considered "same position"
            max_stale_frames: Remove entries not updated for this many ticks
                              to prevent unbounded memory growth
        """
        self.persistence_normal = persistence_normal
        self.persistence_degraded = persistence_degraded
        self.position_tolerance = position_tolerance
        self.max_stale_frames = max_stale_frames

        # Active tracking entries — list of PersistentDetection
        self._tracked: List[PersistentDetection] = []

        # Global frame counter — incremented on every update() call
        self._tick = 0

    def _find_nearest(self, x: float, y: float) -> int:
        """
        Find the index of the nearest tracked detection within tolerance.

        Args:
            x: World X position
            y: World Y position

        Returns:
            Index into self._tracked, or -1 if no match within tolerance
        """
        best_idx = -1
        best_dist = float('inf')

        for i, entry in enumerate(self._tracked):
            dist = math.sqrt((entry.x - x) ** 2 + (entry.y - y) ** 2)
            if dist < self.position_tolerance and dist < best_dist:
                best_dist = dist
                best_idx = i

        return best_idx

    def _age_stale_entries(self):
        """
        Remove tracked entries that haven't been updated recently.
        This prevents the list from growing unbounded and also
        resets persistence for detections that disappeared.
        """
        self._tracked = [
            entry for entry in self._tracked
            if (self._tick - entry.last_update_tick) <= self.max_stale_frames
        ]

    def update(
        self,
        detections: List[Tuple[float, float, float, str, int,
                               Tuple[float, float, float, float]]],
        visibility: VisibilityState
    ) -> List[PersistentDetection]:
        """
        Process a batch of detections for this frame and return only
        those that have met the persistence requirement.

        Args:
            detections: List of (x, y, confidence, source, track_id, bbox) tuples.
                        Each represents a detection that passed confidence fusion.
            visibility: Current visibility state (determines frame threshold)

        Returns:
            List of PersistentDetection that have accumulated enough
            consecutive frames to be considered persistent.
        """
        # Increment global frame tick
        self._tick += 1

        # Determine required persistence for current visibility
        required = (self.persistence_normal if visibility == VisibilityState.NORMAL
                    else self.persistence_degraded)

        # Track which existing entries were updated this tick
        updated_indices = set()

        # Process each incoming detection
        for x, y, confidence, source, track_id, bbox in detections:
            match_idx = self._find_nearest(x, y)

            if match_idx >= 0:
                # Found existing entry at this position — update it
                entry = self._tracked[match_idx]
                entry.x = x  # Update position (may have shifted slightly)
                entry.y = y
                entry.confidence = confidence
                entry.source = source
                entry.track_id = track_id
                entry.bbox = bbox
                entry.consecutive_frames += 1
                entry.last_update_tick = self._tick
                updated_indices.add(match_idx)
            else:
                # New detection — start tracking persistence
                new_entry = PersistentDetection(
                    x=x,
                    y=y,
                    confidence=confidence,
                    source=source,
                    consecutive_frames=1,
                    last_update_tick=self._tick,
                    track_id=track_id,
                    bbox=bbox
                )
                self._tracked.append(new_entry)

        # Age out stale entries (not seen for too long → reset persistence)
        self._age_stale_entries()

        # Return detections that meet the persistence threshold
        persistent = [
            entry for entry in self._tracked
            if entry.consecutive_frames >= required
        ]

        return persistent

    @property
    def tracked_count(self) -> int:
        """Number of positions currently being tracked (regardless of persistence)."""
        return len(self._tracked)

    def reset(self):
        """Clear all tracking state. Useful for testing or mission restart."""
        self._tracked.clear()
        self._tick = 0
