#!/usr/bin/env python3
"""
NIDAR AirMouse — Deduplicator
Prevents duplicate survivor confirmations by comparing new detections
against all previously confirmed survivors using Euclidean distance.

Algorithm:
    for each new confirmed detection:
        distance = sqrt((x2-x1)² + (y2-y1)²)
        for each existing confirmed survivor:
            if distance < dedup_distance → same survivor → update confidence, skip
            if distance >= dedup_distance → new survivor → add to list
    max survivors = 6 (competition hard limit)

Design rationale:
    - 0.5m dedup distance matches spatial aligner threshold
    - Hard 6-survivor limit enforced at this level (last gate before publish)
    - Confidence updated via running average when duplicate detected
    - Returns both the decision (new/duplicate) and assigned survivor ID
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class ConfirmedSurvivor:
    """
    A confirmed, deduplicated survivor in the mission registry.
    """
    survivor_id: int        # Unique sequential ID (1-based)
    x: float                # World X position (metres, map frame)
    y: float                # World Y position (metres, map frame)
    confidence: float       # Running average confidence score
    detection_source: str   # "rgb", "thermal", or "fused"
    detection_count: int    # Number of times this survivor was detected
    bbox: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


class Deduplicator:
    """
    Maintains a registry of confirmed survivors and prevents duplicates.
    Enforces the 6-survivor competition limit.
    """

    def __init__(
        self,
        dedup_distance: float = 0.5,
        max_survivors: int = 6
    ):
        """
        Args:
            dedup_distance: Maximum Euclidean distance (metres) for two
                            positions to be considered the same survivor.
            max_survivors: Maximum number of unique survivors allowed.
                           Competition rule: 6 survivors max.
        """
        self.dedup_distance = dedup_distance
        self.max_survivors = max_survivors

        # Registry of all confirmed unique survivors
        self._confirmed: List[ConfirmedSurvivor] = []

        # Next survivor ID to assign (1-based for human readability)
        self._next_id = 1

    def check(
        self,
        x: float,
        y: float,
        confidence: float,
        source: str,
        bbox: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    ) -> Tuple[bool, int]:
        """
        Check if a detection is a new survivor or a duplicate.

        Args:
            x: World X position (metres)
            y: World Y position (metres)
            confidence: Fused confidence score
            source: Detection source ("rgb", "thermal", "fused")
            bbox: Bounding box [x_min, y_min, w, h]

        Returns:
            (is_new, survivor_id) tuple:
              - is_new=True, survivor_id=N → new survivor added with ID N
              - is_new=False, survivor_id=N → duplicate of existing survivor N
              - is_new=False, survivor_id=-1 → rejected (max limit reached)
        """
        # Check against all existing confirmed survivors
        for survivor in self._confirmed:
            dist = math.sqrt(
                (survivor.x - x) ** 2 + (survivor.y - y) ** 2
            )

            if dist < self.dedup_distance:
                # Duplicate detection — update running average confidence
                # Running average: new_avg = (old_avg * N + new) / (N + 1)
                total = survivor.confidence * survivor.detection_count + confidence
                survivor.detection_count += 1
                survivor.confidence = total / survivor.detection_count

                # Update position to weighted average for better accuracy
                weight = 1.0 / survivor.detection_count
                survivor.x = survivor.x * (1 - weight) + x * weight
                survivor.y = survivor.y * (1 - weight) + y * weight

                # Update bbox and source to latest
                survivor.bbox = bbox
                survivor.detection_source = source

                return False, survivor.survivor_id

        # No matching survivor found — check if we can add a new one
        if len(self._confirmed) >= self.max_survivors:
            # Hard limit reached — reject new detection silently
            return False, -1

        # New unique survivor — add to registry
        new_survivor = ConfirmedSurvivor(
            survivor_id=self._next_id,
            x=x,
            y=y,
            confidence=confidence,
            detection_source=source,
            detection_count=1,
            bbox=bbox
        )
        self._confirmed.append(new_survivor)
        self._next_id += 1

        return True, new_survivor.survivor_id

    @property
    def confirmed_count(self) -> int:
        """Number of unique confirmed survivors."""
        return len(self._confirmed)

    @property
    def is_full(self) -> bool:
        """True if maximum survivor count has been reached."""
        return len(self._confirmed) >= self.max_survivors

    def get_confirmed_list(self) -> List[ConfirmedSurvivor]:
        """Return a copy of all confirmed survivors."""
        return list(self._confirmed)

    def reset(self):
        """Clear all confirmed survivors. Useful for testing or mission restart."""
        self._confirmed.clear()
        self._next_id = 1
