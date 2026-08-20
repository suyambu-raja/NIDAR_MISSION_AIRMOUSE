#!/usr/bin/env python3
"""
NIDAR AirMouse — Spatial Aligner
Matches RGB detections with thermal detections by comparing their
world-frame positions. Uses Euclidean distance with a configurable
match threshold.

Algorithm:
    world_position = project(camera_detection, drone_pose)
    distance = sqrt((x2-x1)² + (y2-y1)²)
    if distance < match_distance → MATCHED (same person)
    if distance >= match_distance → UNMATCHED (different detections)

Uses greedy nearest-neighbor matching (not Hungarian) because:
- At most 6 survivors in the arena — O(n²) is negligible
- scipy is not available on RPi 4 stock install
- Greedy is deterministic and easier to debug in competition
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class Detection:
    """
    Lightweight detection representation for spatial alignment.
    Decoupled from ROS messages so this module is testable standalone.
    """
    x: float             # World position X in map frame (metres)
    y: float             # World position Y in map frame (metres)
    confidence: float    # Detection confidence (0.0–1.0)
    source: str          # "rgb" or "thermal"
    bbox: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # [x_min, y_min, w, h]
    track_id: int = -1   # Tracker-assigned ID (-1 if untracked)
    depth: float = 0.0   # Depth distance in metres (from OAK-D stereo)


@dataclass
class MatchResult:
    """
    Result of spatial alignment between RGB and thermal detections.
    """
    # Pairs of (rgb_detection, thermal_detection) that are spatially close
    matched_pairs: List[Tuple[Detection, Detection]] = field(default_factory=list)

    # RGB detections that had no nearby thermal match
    unmatched_rgb: List[Detection] = field(default_factory=list)

    # Thermal detections that had no nearby RGB match
    unmatched_thermal: List[Detection] = field(default_factory=list)


class SpatialAligner:
    """
    Aligns detections from RGB and thermal sensors by comparing their
    world-frame positions using Euclidean distance.

    Design rationale:
    - Greedy nearest-neighbor instead of Hungarian algorithm
    - Simple and fast for small N (max 6 survivors)
    - No external dependencies (no scipy needed on RPi 4)
    """

    def __init__(self, match_distance: float = 0.5):
        """
        Args:
            match_distance: Maximum Euclidean distance (metres) for two
                            detections to be considered the same person.
                            Default 0.5m — roughly one body width.
        """
        self.match_distance = match_distance

    @staticmethod
    def euclidean_distance(det1: Detection, det2: Detection) -> float:
        """
        Calculate Euclidean distance between two detections in world frame.

        Args:
            det1: First detection with world position (x, y)
            det2: Second detection with world position (x, y)

        Returns:
            Distance in metres
        """
        return math.sqrt((det2.x - det1.x) ** 2 + (det2.y - det1.y) ** 2)

    @staticmethod
    def project_to_world(
        bbox_center_x: float,
        bbox_center_y: float,
        depth: float,
        drone_x: float,
        drone_y: float,
        drone_yaw: float,
        image_width: float = 640.0,
        hfov: float = 1.2217  # OAK-D horizontal FOV ~70 degrees in radians
    ) -> Tuple[float, float]:
        """
        Project a camera-frame bounding box center to world-frame (X, Y)
        using depth and drone pose. Simplified pinhole projection.

        This is used for thermal detections that don't already have
        world positions. RGB detections from tracker_node already
        carry position in map frame.

        Args:
            bbox_center_x: Bounding box center X in image pixels
            bbox_center_y: Bounding box center Y in image pixels (unused for 2D)
            depth: Distance from camera to detection in metres
            drone_x: Drone X position in map frame
            drone_y: Drone Y position in map frame
            drone_yaw: Drone heading in radians (from quaternion)
            image_width: Camera image width in pixels
            hfov: Horizontal field of view in radians

        Returns:
            (world_x, world_y) position in map frame
        """
        # Calculate horizontal angle offset from image center
        # Positive = right of center, negative = left
        pixel_offset = bbox_center_x - (image_width / 2.0)
        angle_per_pixel = hfov / image_width
        bearing_offset = pixel_offset * angle_per_pixel

        # Total bearing from drone heading
        bearing = drone_yaw + bearing_offset

        # Project depth along bearing to get world offset
        dx = depth * math.cos(bearing)
        dy = depth * math.sin(bearing)

        # Add to drone position for world-frame coordinates
        world_x = drone_x + dx
        world_y = drone_y + dy

        return world_x, world_y

    def align(
        self,
        rgb_detections: List[Detection],
        thermal_detections: List[Detection]
    ) -> MatchResult:
        """
        Match RGB and thermal detections using greedy nearest-neighbor.

        Algorithm:
        1. Build distance matrix between all RGB and thermal detections
        2. Greedily match closest pairs until no more matches below threshold
        3. Remaining unmatched detections go to their respective lists

        Args:
            rgb_detections: List of RGB detections with world positions
            thermal_detections: List of thermal detections with world positions

        Returns:
            MatchResult with matched pairs, unmatched RGB, unmatched thermal
        """
        result = MatchResult()

        # Handle edge cases — no detections from one or both sensors
        if not rgb_detections and not thermal_detections:
            return result

        if not rgb_detections:
            # All thermal detections are unmatched
            result.unmatched_thermal = list(thermal_detections)
            return result

        if not thermal_detections:
            # All RGB detections are unmatched
            result.unmatched_rgb = list(rgb_detections)
            return result

        # Track which detections have been matched
        rgb_matched = set()
        thermal_matched = set()

        # Build list of all (distance, rgb_idx, thermal_idx) pairs
        # then sort by distance for greedy matching
        candidates = []
        for i, rgb_det in enumerate(rgb_detections):
            for j, thermal_det in enumerate(thermal_detections):
                dist = self.euclidean_distance(rgb_det, thermal_det)
                # Only consider pairs within match distance
                if dist < self.match_distance:
                    candidates.append((dist, i, j))

        # Sort by distance — closest pairs matched first (greedy)
        candidates.sort(key=lambda c: c[0])

        # Greedily assign matches — each detection can only match once
        for dist, rgb_idx, thermal_idx in candidates:
            if rgb_idx not in rgb_matched and thermal_idx not in thermal_matched:
                result.matched_pairs.append(
                    (rgb_detections[rgb_idx], thermal_detections[thermal_idx])
                )
                rgb_matched.add(rgb_idx)
                thermal_matched.add(thermal_idx)

        # Collect unmatched detections
        for i, det in enumerate(rgb_detections):
            if i not in rgb_matched:
                result.unmatched_rgb.append(det)

        for j, det in enumerate(thermal_detections):
            if j not in thermal_matched:
                result.unmatched_thermal.append(det)

        return result
