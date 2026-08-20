#!/usr/bin/env python3
"""
survivor_registry.py — NIDAR AirMouse: Survivor Registry
=========================================================
Standalone utility (NOT a ROS 2 node) used by grid_mapper_node.

Purpose
-------
Maintains the authoritative running list of all confirmed, unique survivors
detected during a mission.  Deduplicates incoming detections using Euclidean
Distance Clustering so the same person is never counted twice.

Algorithm: Euclidean Distance Clustering
-----------------------------------------
For each new detection arriving from tracker_node:

    For every already-confirmed survivor in the registry:
        distance = sqrt((x2-x1)² + (y2-y1)²)

    If distance < DEDUP_THRESHOLD_M (0.5 m) from any existing survivor:
        → Same survivor.  Update confidence (weighted average) + timestamps.
    If distance ≥ DEDUP_THRESHOLD_M from ALL existing survivors:
        → New survivor.  Add to registry (unless at MAX_SURVIVORS cap).

Competition constraints
-----------------------
    - MAX_SURVIVORS = 6  (hard-coded competition rule — never exceeded)
    - Once 6 survivors are confirmed, new detections are still checked for
      dedup (to update confidence) but new entries are not added.
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional


# =============================================================================
# Competition constants
# =============================================================================

MAX_SURVIVORS: int = 6          # Competition limit — never add more than 6
DEDUP_THRESHOLD_M: float = 0.5  # Detections within 0.5 m = same survivor


@dataclass
class SurvivorRecord:
    """
    Full record for one confirmed unique survivor.

    Attributes
    ----------
    survivor_id   : int    — unique ID (monotonically assigned, never reused)
    grid_box      : str    — arena grid box ID, e.g. "F4"
    world_x       : float  — SLAM map-frame X position (metres)
    world_y       : float  — SLAM map-frame Y position (metres)
    confidence    : float  — current confidence score (0.0–1.0)
    first_seen    : float  — monotonic timestamp when first detected
    last_seen     : float  — monotonic timestamp of most recent detection
    detection_count : int  — total number of detections merged into this record
    """
    survivor_id: int
    grid_box: str
    world_x: float
    world_y: float
    confidence: float
    first_seen: float = field(default_factory=time.monotonic)
    last_seen: float = field(default_factory=time.monotonic)
    detection_count: int = 1


class SurvivorRegistry:
    """
    Thread-safe (single-threaded ROS 2 spin model) survivor deduplication
    registry.  All methods are O(N) on the number of confirmed survivors —
    with N ≤ 6, this is negligible on Raspberry Pi 4.

    Usage (from grid_mapper_node)
    ------------------------------
        registry = SurvivorRegistry()

        # On each /tracked_survivors callback:
        for det in tracked_survivors.detections:
            if det.is_confirmed:
                record = registry.add_or_update(
                    world_x=det.position.x,
                    world_y=det.position.y,
                    confidence=det.confidence,
                    grid_box=transformer.world_to_grid(det.position.x, det.position.y),
                    tracker_id=det.survivor_id,
                )

        survivors = registry.get_all()   # List[SurvivorRecord]
    """

    def __init__(self) -> None:
        # The authoritative list of confirmed unique survivors.
        # Maximum length is MAX_SURVIVORS (6).
        self._survivors: List[SurvivorRecord] = []

        # Next ID to assign to a newly confirmed survivor.
        # We don't reuse IDs even if the registry were ever cleared.
        self._next_id: int = 0

    # =========================================================================
    # Public API
    # =========================================================================

    def add_or_update(
        self,
        world_x: float,
        world_y: float,
        confidence: float,
        grid_box: Optional[str],
        tracker_id: int = -1,
    ) -> Optional[SurvivorRecord]:
        """
        Process one confirmed survivor detection.

        Steps:
        1. Check against all existing records using Euclidean distance.
        2. If a match is found (distance < 0.5 m) → update that record.
        3. If no match and slots remain → create a new record.
        4. If no match and MAX_SURVIVORS reached → log and discard.

        Parameters
        ----------
        world_x, world_y : float
            Survivor's SLAM map-frame position (metres).
        confidence : float
            Detection confidence from tracker_node (0.0–1.0).
        grid_box : str or None
            Grid box ID from CoordinateTransformer.  May be None if position
            is outside arena — in that case the record is not added but an
            existing record at the same position is still updated.
        tracker_id : int
            Original ID from tracker_node (-1 if unknown).

        Returns
        -------
        SurvivorRecord
            The record that was updated or newly created.
        None
            If grid_box is None (out-of-bounds) AND no existing record matches.
        """
        # ------------------------------------------------------------------
        # Step 1: Search for the nearest existing confirmed survivor.
        # ------------------------------------------------------------------
        nearest_record, nearest_dist = self._find_nearest(world_x, world_y)

        # ------------------------------------------------------------------
        # Step 2: Match found — update existing record.
        # ------------------------------------------------------------------
        if nearest_record is not None and nearest_dist < DEDUP_THRESHOLD_M:
            self._update_record(nearest_record, world_x, world_y, confidence, grid_box)
            return nearest_record

        # ------------------------------------------------------------------
        # Step 3: No match — check for out-of-bounds grid_box.
        # ------------------------------------------------------------------
        if grid_box is None:
            # Can't place survivor on grid — don't create a record.
            return None

        # ------------------------------------------------------------------
        # Step 4: No match, valid grid box — check competition cap.
        # ------------------------------------------------------------------
        if len(self._survivors) >= MAX_SURVIVORS:
            # Competition rule: we cannot exceed 6 confirmed survivors.
            # Detection is silently discarded (caller can log this).
            return None

        # ------------------------------------------------------------------
        # Step 5: New confirmed unique survivor — add to registry.
        # ------------------------------------------------------------------
        record = SurvivorRecord(
            survivor_id=self._next_id,
            grid_box=grid_box,
            world_x=world_x,
            world_y=world_y,
            confidence=confidence,
            first_seen=time.monotonic(),
            last_seen=time.monotonic(),
            detection_count=1,
        )
        self._survivors.append(record)
        self._next_id += 1
        return record

    def get_all(self) -> List[SurvivorRecord]:
        """
        Return a shallow copy of all confirmed survivor records.

        Returns a copy so callers cannot mutate internal state by accident.
        """
        return list(self._survivors)

    def count(self) -> int:
        """Return the number of confirmed unique survivors found so far."""
        return len(self._survivors)

    def is_full(self) -> bool:
        """Return True if the competition survivor limit (6) has been reached."""
        return len(self._survivors) >= MAX_SURVIVORS

    def clear(self) -> None:
        """
        Reset the registry for a new mission.

        Clears all survivor records and resets the ID counter.
        Use only at mission start/reset — never during a live mission.
        """
        self._survivors.clear()
        self._next_id = 0

    # =========================================================================
    # Private helpers
    # =========================================================================

    def _find_nearest(
        self, world_x: float, world_y: float
    ) -> tuple:
        """
        Find the existing survivor record closest to (world_x, world_y).

        Returns
        -------
        (nearest_record, nearest_distance)
            nearest_record   : SurvivorRecord or None if registry is empty
            nearest_distance : float distance in metres (inf if empty)
        """
        nearest_record: Optional[SurvivorRecord] = None
        nearest_dist: float = float('inf')

        for record in self._survivors:
            # Euclidean distance formula: sqrt((x2-x1)² + (y2-y1)²)
            dx = world_x - record.world_x
            dy = world_y - record.world_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < nearest_dist:
                nearest_dist = dist
                nearest_record = record

        return nearest_record, nearest_dist

    def _update_record(
        self,
        record: SurvivorRecord,
        world_x: float,
        world_y: float,
        confidence: float,
        grid_box: Optional[str],
    ) -> None:
        """
        Update an existing survivor record with new detection data.

        Position update: running weighted average biased toward higher-count
        records so early noisy detections don't dominate.

        Confidence update: running average over all merged detections.

        Parameters are identical to add_or_update().
        """
        n = record.detection_count

        # Running weighted-average position (more detections → more stable)
        record.world_x = (record.world_x * n + world_x) / (n + 1)
        record.world_y = (record.world_y * n + world_y) / (n + 1)

        # Running average confidence
        record.confidence = (record.confidence * n + confidence) / (n + 1)

        # Update grid_box if it's newly available (was None before)
        if grid_box is not None:
            record.grid_box = grid_box

        # Bookkeeping
        record.detection_count += 1
        record.last_seen = time.monotonic()
