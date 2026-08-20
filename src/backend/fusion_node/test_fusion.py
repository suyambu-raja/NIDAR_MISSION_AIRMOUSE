#!/usr/bin/env python3
"""
NIDAR AirMouse — Fusion Node Unit Tests
Comprehensive test coverage for all fusion pipeline modules.
All tests use mock data — no ROS 2 runtime or real hardware required.

Run with:
    cd src/backend/fusion_node
    python -m pytest test_fusion.py -v

Test coverage:
    1.  Visibility: bright frame → NORMAL
    2.  Visibility: dark frame → DEGRADED
    3.  Visibility: frame skip optimization
    4.  Spatial alignment: nearby detections → matched
    5.  Spatial alignment: far detections → unmatched
    6.  Spatial alignment: mixed matched + unmatched
    7.  Confidence fusion: correct weights in NORMAL visibility
    8.  Confidence fusion: correct weights in DEGRADED visibility
    9.  Confidence fusion: unmatched primary vs secondary penalties
    10. EMA smoothing across frames
    11. Persistence: not confirmed before N frames
    12. Persistence: confirmed after N frames
    13. Persistence: DEGRADED requires more frames
    14. Confidence threshold: above 0.65 → confirmed
    15. Confidence threshold: below 0.65 → rejected
    16. Deduplication: same position = one survivor
    17. Deduplication: different position = two survivors
    18. Maximum 6 survivor limit enforced
    19. RGB-only in NORMAL → confirmed (high weight)
    20. Thermal-only in NORMAL → not confirmed (low weight × penalty)
    21. Thermal-only in DEGRADED → confirmed after persistence
"""

import sys
import os
import math
import importlib
import importlib.util
import pytest
import numpy as np

# ===========================================================================
# Import standalone utility modules WITHOUT triggering the main fusion_node.py
# which requires rclpy (not available outside ROS 2 environment).
#
# Strategy: import each module directly by file path, injecting the
# visibility_estimator dependency into confidence_fuser's namespace
# before it tries to resolve the relative import.
# ===========================================================================

_pkg_dir = os.path.join(os.path.dirname(__file__), 'fusion_node')

def _import_module_from_file(module_name, file_name):
    """Import a Python module directly from file path, bypassing package imports."""
    spec = importlib.util.spec_from_file_location(
        module_name,
        os.path.join(_pkg_dir, file_name)
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod

# Import order matters — confidence_fuser depends on visibility_estimator
_vis_mod = _import_module_from_file('fusion_node.visibility_estimator', 'visibility_estimator.py')
_align_mod = _import_module_from_file('fusion_node.spatial_aligner', 'spatial_aligner.py')
_conf_mod = _import_module_from_file('fusion_node.confidence_fuser', 'confidence_fuser.py')
_pers_mod = _import_module_from_file('fusion_node.persistence_tracker', 'persistence_tracker.py')
_dedup_mod = _import_module_from_file('fusion_node.deduplicator', 'deduplicator.py')

# Pull classes into local namespace
VisibilityEstimator = _vis_mod.VisibilityEstimator
VisibilityState = _vis_mod.VisibilityState
SpatialAligner = _align_mod.SpatialAligner
Detection = _align_mod.Detection
MatchResult = _align_mod.MatchResult
ConfidenceFuser = _conf_mod.ConfidenceFuser
PersistenceTracker = _pers_mod.PersistenceTracker
Deduplicator = _dedup_mod.Deduplicator


# ==========================================================================
# VISIBILITY ESTIMATION TESTS
# ==========================================================================

class TestVisibilityEstimator:
    """Tests for brightness-based visibility estimation."""

    def test_bright_frame_returns_normal(self):
        """Test 1: A well-lit frame (brightness > 50) should return NORMAL."""
        estimator = VisibilityEstimator(
            brightness_threshold=50,
            update_interval=1  # Update every frame for testing
        )

        # Create a bright frame (all pixels = 200, well above threshold of 50)
        bright_frame = np.full((480, 640, 3), 200, dtype=np.uint8)

        result = estimator.update(bright_frame)

        assert result == VisibilityState.NORMAL
        assert estimator.last_brightness == pytest.approx(200.0, abs=1.0)

    def test_dark_frame_returns_degraded(self):
        """Test 2: A dark frame (brightness <= 50) should return DEGRADED."""
        estimator = VisibilityEstimator(
            brightness_threshold=50,
            update_interval=1
        )

        # Create a dark frame (all pixels = 20, below threshold of 50)
        dark_frame = np.full((480, 640, 3), 20, dtype=np.uint8)

        result = estimator.update(dark_frame)

        assert result == VisibilityState.DEGRADED
        assert estimator.last_brightness == pytest.approx(20.0, abs=1.0)

    def test_frame_skip_optimization(self):
        """
        Test 3: Estimator should only recompute every N frames.
        Between updates, it returns the cached state.
        """
        estimator = VisibilityEstimator(
            brightness_threshold=50,
            update_interval=10  # Only update every 10th frame
        )

        # First 10 frames with bright data → triggers update on frame 10
        bright_frame = np.full((100, 100, 3), 200, dtype=np.uint8)
        for i in range(10):
            result = estimator.update(bright_frame)

        assert result == VisibilityState.NORMAL

        # Next 9 frames with dark data → should NOT update (still cached NORMAL)
        dark_frame = np.full((100, 100, 3), 10, dtype=np.uint8)
        for i in range(9):
            result = estimator.update(dark_frame)

        # Should still be NORMAL because we haven't hit the 10th frame yet
        assert result == VisibilityState.NORMAL

        # 10th dark frame → triggers recompute → DEGRADED
        result = estimator.update(dark_frame)
        assert result == VisibilityState.DEGRADED

    def test_grayscale_frame(self):
        """Test that grayscale (2D) frames are handled correctly."""
        estimator = VisibilityEstimator(
            brightness_threshold=50,
            update_interval=1
        )

        # Grayscale frame (no color channels)
        gray_frame = np.full((480, 640), 100, dtype=np.uint8)

        result = estimator.update(gray_frame)
        assert result == VisibilityState.NORMAL

    def test_exactly_at_threshold(self):
        """Test edge case: brightness exactly at threshold → DEGRADED."""
        estimator = VisibilityEstimator(
            brightness_threshold=50,
            update_interval=1
        )

        # Brightness == 50 is NOT above threshold, so should be DEGRADED
        frame = np.full((100, 100, 3), 50, dtype=np.uint8)
        result = estimator.update(frame)
        assert result == VisibilityState.DEGRADED

    def test_reset(self):
        """Test that reset returns estimator to initial state."""
        estimator = VisibilityEstimator(update_interval=1)
        dark = np.full((100, 100), 10, dtype=np.uint8)
        estimator.update(dark)
        assert estimator.current_state == VisibilityState.DEGRADED

        estimator.reset()
        assert estimator.current_state == VisibilityState.NORMAL
        assert estimator.last_brightness == 255.0


# ==========================================================================
# SPATIAL ALIGNMENT TESTS
# ==========================================================================

class TestSpatialAligner:
    """Tests for cross-modal detection matching by world position."""

    def test_nearby_detections_matched(self):
        """
        Test 4: Two detections within 0.5m should be matched.
        """
        aligner = SpatialAligner(match_distance=0.5)

        # RGB detection at (5.0, 3.0)
        rgb = [Detection(x=5.0, y=3.0, confidence=0.9, source="rgb")]
        # Thermal detection at (5.2, 3.1) — 0.22m away
        thermal = [Detection(x=5.2, y=3.1, confidence=0.8, source="thermal")]

        result = aligner.align(rgb, thermal)

        assert len(result.matched_pairs) == 1
        assert len(result.unmatched_rgb) == 0
        assert len(result.unmatched_thermal) == 0
        assert result.matched_pairs[0][0].source == "rgb"
        assert result.matched_pairs[0][1].source == "thermal"

    def test_far_detections_unmatched(self):
        """
        Test 5: Two detections more than 0.5m apart should be unmatched.
        """
        aligner = SpatialAligner(match_distance=0.5)

        # RGB at (2.0, 2.0), thermal at (5.0, 5.0) — 4.24m apart
        rgb = [Detection(x=2.0, y=2.0, confidence=0.9, source="rgb")]
        thermal = [Detection(x=5.0, y=5.0, confidence=0.8, source="thermal")]

        result = aligner.align(rgb, thermal)

        assert len(result.matched_pairs) == 0
        assert len(result.unmatched_rgb) == 1
        assert len(result.unmatched_thermal) == 1

    def test_mixed_matched_and_unmatched(self):
        """
        Test 6: Multiple detections — some match, some don't.
        """
        aligner = SpatialAligner(match_distance=0.5)

        rgb = [
            Detection(x=5.0, y=3.0, confidence=0.9, source="rgb"),   # Matches thermal[0]
            Detection(x=10.0, y=10.0, confidence=0.85, source="rgb")  # No match
        ]
        thermal = [
            Detection(x=5.1, y=3.1, confidence=0.8, source="thermal"),  # Matches rgb[0]
            Detection(x=1.0, y=1.0, confidence=0.7, source="thermal")   # No match
        ]

        result = aligner.align(rgb, thermal)

        assert len(result.matched_pairs) == 1
        assert len(result.unmatched_rgb) == 1
        assert len(result.unmatched_thermal) == 1
        # The matched pair should be the close ones
        assert result.matched_pairs[0][0].x == pytest.approx(5.0)
        assert result.matched_pairs[0][1].x == pytest.approx(5.1)

    def test_empty_rgb_all_thermal_unmatched(self):
        """No RGB detections → all thermal are unmatched."""
        aligner = SpatialAligner(match_distance=0.5)

        thermal = [Detection(x=3.0, y=3.0, confidence=0.8, source="thermal")]
        result = aligner.align([], thermal)

        assert len(result.matched_pairs) == 0
        assert len(result.unmatched_rgb) == 0
        assert len(result.unmatched_thermal) == 1

    def test_empty_thermal_all_rgb_unmatched(self):
        """No thermal detections → all RGB are unmatched."""
        aligner = SpatialAligner(match_distance=0.5)

        rgb = [Detection(x=3.0, y=3.0, confidence=0.9, source="rgb")]
        result = aligner.align(rgb, [])

        assert len(result.matched_pairs) == 0
        assert len(result.unmatched_rgb) == 1
        assert len(result.unmatched_thermal) == 0

    def test_both_empty(self):
        """No detections at all → empty result."""
        aligner = SpatialAligner(match_distance=0.5)
        result = aligner.align([], [])

        assert len(result.matched_pairs) == 0
        assert len(result.unmatched_rgb) == 0
        assert len(result.unmatched_thermal) == 0

    def test_euclidean_distance_calculation(self):
        """Verify Euclidean distance math is correct."""
        d1 = Detection(x=0.0, y=0.0, confidence=1.0, source="rgb")
        d2 = Detection(x=3.0, y=4.0, confidence=1.0, source="thermal")

        dist = SpatialAligner.euclidean_distance(d1, d2)
        assert dist == pytest.approx(5.0)  # 3-4-5 triangle

    def test_project_to_world(self):
        """Verify camera-to-world projection produces sane values."""
        # Drone at origin, facing east (yaw=0), detection in center of image
        wx, wy = SpatialAligner.project_to_world(
            bbox_center_x=320.0,  # Center of 640px image
            bbox_center_y=240.0,
            depth=2.0,            # 2m away
            drone_x=0.0,
            drone_y=0.0,
            drone_yaw=0.0,
            image_width=640.0
        )

        # Detection straight ahead at 2m → should be at (2, 0)
        assert wx == pytest.approx(2.0, abs=0.1)
        assert wy == pytest.approx(0.0, abs=0.1)


# ==========================================================================
# CONFIDENCE FUSION TESTS
# ==========================================================================

class TestConfidenceFuser:
    """Tests for weighted evidence fusion and EMA smoothing."""

    def test_matched_pair_normal_visibility(self):
        """
        Test 7: In NORMAL visibility, matched pair uses RGB=0.7, thermal=0.3.
        """
        fuser = ConfidenceFuser()

        # RGB confidence 0.9, thermal confidence 0.8
        result = fuser.fuse_matched(0.9, 0.8, VisibilityState.NORMAL)

        # Expected: (0.9 × 0.7) + (0.8 × 0.3) = 0.63 + 0.24 = 0.87
        assert result == pytest.approx(0.87, abs=0.001)

    def test_matched_pair_degraded_visibility(self):
        """
        Test 8: In DEGRADED visibility, matched pair uses RGB=0.3, thermal=0.7.
        """
        fuser = ConfidenceFuser()

        result = fuser.fuse_matched(0.9, 0.8, VisibilityState.DEGRADED)

        # Expected: (0.9 × 0.3) + (0.8 × 0.7) = 0.27 + 0.56 = 0.83
        assert result == pytest.approx(0.83, abs=0.001)

    def test_unmatched_primary_penalty(self):
        """
        Test 9a: Unmatched primary sensor gets 0.8× penalty.
        RGB in NORMAL is primary.
        """
        fuser = ConfidenceFuser()

        result = fuser.fuse_unmatched(0.9, "rgb", VisibilityState.NORMAL)

        # Expected: 0.9 × 0.8 = 0.72
        assert result == pytest.approx(0.72, abs=0.001)

    def test_unmatched_secondary_penalty(self):
        """
        Test 9b: Unmatched secondary sensor gets 0.4× penalty.
        Thermal in NORMAL is secondary.
        """
        fuser = ConfidenceFuser()

        result = fuser.fuse_unmatched(0.9, "thermal", VisibilityState.NORMAL)

        # Expected: 0.9 × 0.4 = 0.36
        assert result == pytest.approx(0.36, abs=0.001)

    def test_unmatched_thermal_primary_in_degraded(self):
        """Thermal in DEGRADED is primary → 0.8× penalty."""
        fuser = ConfidenceFuser()

        result = fuser.fuse_unmatched(0.85, "thermal", VisibilityState.DEGRADED)

        # Expected: 0.85 × 0.8 = 0.68
        assert result == pytest.approx(0.68, abs=0.001)

    def test_unmatched_rgb_secondary_in_degraded(self):
        """RGB in DEGRADED is secondary → 0.4× penalty."""
        fuser = ConfidenceFuser()

        result = fuser.fuse_unmatched(0.9, "rgb", VisibilityState.DEGRADED)

        # Expected: 0.9 × 0.4 = 0.36
        assert result == pytest.approx(0.36, abs=0.001)

    def test_ema_smoothing(self):
        """
        Test 10: EMA smoothing across multiple frames.
        Formula: smoothed = (0.3 × new) + (0.7 × previous)
        """
        fuser = ConfidenceFuser(ema_alpha=0.3)

        # First observation — no history, returns raw value
        s1 = fuser.apply_ema(0.8, 5.0, 3.0)
        assert s1 == pytest.approx(0.8, abs=0.001)

        # Second observation at same position — EMA kicks in
        # smoothed = (0.3 × 0.9) + (0.7 × 0.8) = 0.27 + 0.56 = 0.83
        s2 = fuser.apply_ema(0.9, 5.0, 3.0)
        assert s2 == pytest.approx(0.83, abs=0.001)

        # Third observation — blends with previous smoothed (0.83)
        # smoothed = (0.3 × 0.7) + (0.7 × 0.83) = 0.21 + 0.581 = 0.791
        s3 = fuser.apply_ema(0.7, 5.0, 3.0)
        assert s3 == pytest.approx(0.791, abs=0.001)

    def test_ema_different_positions_independent(self):
        """EMA history at different positions should be independent."""
        fuser = ConfidenceFuser(ema_alpha=0.3)

        # Position A
        fuser.apply_ema(0.8, 1.0, 1.0)
        # Position B
        fuser.apply_ema(0.5, 10.0, 10.0)

        # Second update at position A — should use A's history (0.8), not B's
        result_a = fuser.apply_ema(0.9, 1.0, 1.0)
        # smoothed = (0.3 × 0.9) + (0.7 × 0.8) = 0.83
        assert result_a == pytest.approx(0.83, abs=0.001)

    def test_full_fuse_pipeline(self):
        """Test the complete fuse() method (weighted + EMA)."""
        fuser = ConfidenceFuser(ema_alpha=0.3)

        # Matched pair in NORMAL: (0.9 × 0.7) + (0.8 × 0.3) = 0.87
        # First call, no EMA history → returns 0.87
        result = fuser.fuse(
            rgb_confidence=0.9,
            thermal_confidence=0.8,
            source="fused",
            visibility=VisibilityState.NORMAL,
            world_x=5.0,
            world_y=3.0
        )
        assert result == pytest.approx(0.87, abs=0.001)


# ==========================================================================
# PERSISTENCE TRACKING TESTS
# ==========================================================================

class TestPersistenceTracker:
    """Tests for multi-frame persistence gate."""

    def test_not_confirmed_before_threshold(self):
        """
        Test 11: Detection should NOT be confirmed before reaching
        the required frame count (5 for NORMAL).
        """
        tracker = PersistenceTracker(
            persistence_normal=5,
            persistence_degraded=8,
            position_tolerance=0.3
        )

        # Detection at (5.0, 3.0) — only 4 frames (below threshold of 5)
        det = [(5.0, 3.0, 0.9, "rgb", 1, (0.0, 0.0, 0.0, 0.0))]

        for i in range(4):
            persistent = tracker.update(det, VisibilityState.NORMAL)

        # After 4 frames — should NOT be persistent yet
        assert len(persistent) == 0

    def test_confirmed_after_threshold(self):
        """
        Test 12: Detection should be confirmed after reaching
        the required frame count (5 for NORMAL).
        """
        tracker = PersistenceTracker(
            persistence_normal=5,
            persistence_degraded=8,
            position_tolerance=0.3
        )

        det = [(5.0, 3.0, 0.9, "rgb", 1, (0.0, 0.0, 0.0, 0.0))]

        for i in range(5):
            persistent = tracker.update(det, VisibilityState.NORMAL)

        # After 5 frames — should now be persistent
        assert len(persistent) == 1
        assert persistent[0].x == pytest.approx(5.0)
        assert persistent[0].consecutive_frames == 5

    def test_degraded_requires_more_frames(self):
        """
        Test 13: DEGRADED visibility requires 8 frames instead of 5.
        """
        tracker = PersistenceTracker(
            persistence_normal=5,
            persistence_degraded=8,
            position_tolerance=0.3
        )

        det = [(5.0, 3.0, 0.85, "thermal", 1, (0.0, 0.0, 0.0, 0.0))]

        # After 7 frames in DEGRADED — still not enough
        for i in range(7):
            persistent = tracker.update(det, VisibilityState.DEGRADED)
        assert len(persistent) == 0

        # 8th frame — should now pass
        persistent = tracker.update(det, VisibilityState.DEGRADED)
        assert len(persistent) == 1
        assert persistent[0].consecutive_frames == 8

    def test_position_tolerance(self):
        """
        Detection that drifts slightly (within 0.3m tolerance) should
        still be counted as the same detection.
        """
        tracker = PersistenceTracker(
            persistence_normal=3,
            position_tolerance=0.3
        )

        # Slightly different positions each frame (within 0.3m)
        positions = [(5.0, 3.0), (5.1, 3.05), (4.95, 2.95)]

        for x, y in positions:
            det = [(x, y, 0.9, "rgb", 1, (0.0, 0.0, 0.0, 0.0))]
            persistent = tracker.update(det, VisibilityState.NORMAL)

        # Should be counted as 3 consecutive frames at same position
        assert len(persistent) == 1
        assert persistent[0].consecutive_frames == 3

    def test_detection_disappears_resets_counter(self):
        """
        When a detection disappears and reappears, the counter should reset
        (after stale aging removes the entry).
        """
        tracker = PersistenceTracker(
            persistence_normal=5,
            position_tolerance=0.3,
            max_stale_frames=2  # Age out quickly for testing
        )

        det = [(5.0, 3.0, 0.9, "rgb", 1, (0.0, 0.0, 0.0, 0.0))]

        # 3 frames with detection
        for i in range(3):
            tracker.update(det, VisibilityState.NORMAL)

        # 3 frames without detection — should age out the entry
        for i in range(3):
            tracker.update([], VisibilityState.NORMAL)

        # Detection reappears — counter should start from 1
        persistent = tracker.update(det, VisibilityState.NORMAL)
        assert len(persistent) == 0  # Not yet at threshold

    def test_multiple_detections_independent(self):
        """Multiple detections at different positions track independently."""
        tracker = PersistenceTracker(persistence_normal=3)

        det_a = (5.0, 3.0, 0.9, "rgb", 1, (0.0, 0.0, 0.0, 0.0))
        det_b = (10.0, 8.0, 0.85, "rgb", 2, (0.0, 0.0, 0.0, 0.0))

        # Both detections for 3 frames
        for i in range(3):
            persistent = tracker.update([det_a, det_b], VisibilityState.NORMAL)

        assert len(persistent) == 2


# ==========================================================================
# CONFIDENCE THRESHOLD TESTS
# ==========================================================================

class TestConfidenceThreshold:
    """Tests for the confidence threshold decision."""

    def test_above_threshold_confirmed(self):
        """
        Test 14: Detection with fused confidence > 0.65 should be confirmed.
        """
        threshold = 0.65

        # Matched pair in NORMAL: (0.9 × 0.7) + (0.8 × 0.3) = 0.87
        fuser = ConfidenceFuser()
        fused = fuser.fuse_matched(0.9, 0.8, VisibilityState.NORMAL)

        assert fused > threshold
        assert fused == pytest.approx(0.87, abs=0.001)

    def test_below_threshold_rejected(self):
        """
        Test 15: Detection with fused confidence < 0.65 should be rejected.
        """
        threshold = 0.65

        # Unmatched thermal in NORMAL: 0.9 × 0.4 = 0.36
        fuser = ConfidenceFuser()
        fused = fuser.fuse_unmatched(0.9, "thermal", VisibilityState.NORMAL)

        assert fused < threshold
        assert fused == pytest.approx(0.36, abs=0.001)


# ==========================================================================
# DEDUPLICATION TESTS
# ==========================================================================

class TestDeduplicator:
    """Tests for survivor deduplication and max limit enforcement."""

    def test_same_position_one_survivor(self):
        """
        Test 16: Two detections at the same position (< 0.5m) should
        be treated as one survivor.
        """
        dedup = Deduplicator(dedup_distance=0.5, max_survivors=6)

        # First detection
        is_new_1, id_1 = dedup.check(5.0, 3.0, 0.85, "fused")
        assert is_new_1 is True
        assert id_1 == 1

        # Second detection very close (0.1m away)
        is_new_2, id_2 = dedup.check(5.05, 3.05, 0.90, "fused")
        assert is_new_2 is False
        assert id_2 == 1  # Same survivor ID

        # Should still be only 1 confirmed survivor
        assert dedup.confirmed_count == 1

    def test_different_position_two_survivors(self):
        """
        Test 17: Two detections at different positions (> 0.5m) should
        be treated as two separate survivors.
        """
        dedup = Deduplicator(dedup_distance=0.5, max_survivors=6)

        is_new_1, id_1 = dedup.check(5.0, 3.0, 0.85, "fused")
        is_new_2, id_2 = dedup.check(8.0, 7.0, 0.90, "fused")

        assert is_new_1 is True
        assert is_new_2 is True
        assert id_1 == 1
        assert id_2 == 2
        assert dedup.confirmed_count == 2

    def test_max_six_survivors_enforced(self):
        """
        Test 18: After 6 confirmed survivors, new unique detections
        should be rejected with survivor_id=-1.
        """
        dedup = Deduplicator(dedup_distance=0.5, max_survivors=6)

        # Add 6 survivors at distinct positions
        for i in range(6):
            is_new, sid = dedup.check(float(i * 2), float(i * 2), 0.85, "fused")
            assert is_new is True
            assert sid == i + 1

        assert dedup.confirmed_count == 6
        assert dedup.is_full is True

        # 7th unique detection should be rejected
        is_new, sid = dedup.check(20.0, 20.0, 0.95, "fused")
        assert is_new is False
        assert sid == -1

        # Count should still be 6
        assert dedup.confirmed_count == 6

    def test_confidence_updated_on_duplicate(self):
        """Duplicate detection should update the running average confidence."""
        dedup = Deduplicator(dedup_distance=0.5)

        # First: confidence 0.8
        dedup.check(5.0, 3.0, 0.8, "fused")
        # Second: confidence 0.9 at same position
        dedup.check(5.0, 3.0, 0.9, "fused")

        survivors = dedup.get_confirmed_list()
        # Running average: (0.8 + 0.9) / 2 = 0.85
        assert survivors[0].confidence == pytest.approx(0.85, abs=0.01)
        assert survivors[0].detection_count == 2

    def test_reset(self):
        """Reset should clear all confirmed survivors."""
        dedup = Deduplicator()
        dedup.check(5.0, 3.0, 0.9, "fused")
        assert dedup.confirmed_count == 1

        dedup.reset()
        assert dedup.confirmed_count == 0
        assert dedup.is_full is False


# ==========================================================================
# END-TO-END SCENARIO TESTS
# ==========================================================================

class TestEndToEndScenarios:
    """
    Integration-level tests combining multiple modules to verify
    real-world detection scenarios.
    """

    def test_rgb_only_normal_confirmed(self):
        """
        Test 19: RGB-only detection in NORMAL visibility should be
        confirmed (high weight: 0.9 × 0.8 = 0.72 > 0.65 threshold).
        """
        fuser = ConfidenceFuser(ema_alpha=1.0)  # No EMA smoothing for clarity
        tracker = PersistenceTracker(persistence_normal=5)
        dedup = Deduplicator()

        threshold = 0.65

        # Simulate 5 frames of RGB-only detection at (5, 3)
        for i in range(5):
            # Fuse as unmatched RGB in NORMAL → 0.9 × 0.8 = 0.72
            fused_conf = fuser.fuse(
                rgb_confidence=0.9, thermal_confidence=None,
                source="rgb", visibility=VisibilityState.NORMAL,
                world_x=5.0, world_y=3.0
            )
            det = [(5.0, 3.0, fused_conf, "rgb", 1, (0, 0, 0, 0))]
            persistent = tracker.update(det, VisibilityState.NORMAL)

        # After 5 frames → should be persistent
        assert len(persistent) == 1
        assert persistent[0].confidence > threshold

        # Dedup check → should be new survivor
        is_new, sid = dedup.check(
            persistent[0].x, persistent[0].y,
            persistent[0].confidence, persistent[0].source
        )
        assert is_new is True

    def test_thermal_only_normal_not_confirmed(self):
        """
        Test 20: Thermal-only detection in NORMAL visibility should NOT
        be confirmed (low weight: 0.9 × 0.4 = 0.36 < 0.65 threshold).
        """
        fuser = ConfidenceFuser(ema_alpha=1.0)
        threshold = 0.65

        # Fuse as unmatched thermal in NORMAL → 0.9 × 0.4 = 0.36
        fused_conf = fuser.fuse(
            rgb_confidence=None, thermal_confidence=0.9,
            source="thermal", visibility=VisibilityState.NORMAL,
            world_x=5.0, world_y=3.0
        )

        # 0.36 is well below 0.65 threshold — should be rejected
        assert fused_conf < threshold
        assert fused_conf == pytest.approx(0.36, abs=0.001)

    def test_thermal_only_degraded_confirmed_after_persistence(self):
        """
        Test 21: Thermal-only detection in DEGRADED visibility should be
        confirmed after 8 frames of persistence.
        Thermal in DEGRADED: 0.85 × 0.8 = 0.68 > 0.65 threshold.
        """
        fuser = ConfidenceFuser(ema_alpha=1.0)
        tracker = PersistenceTracker(persistence_degraded=8)
        dedup = Deduplicator()

        threshold = 0.65

        # Simulate 8 frames of thermal-only in DEGRADED
        for i in range(8):
            fused_conf = fuser.fuse(
                rgb_confidence=None, thermal_confidence=0.85,
                source="thermal", visibility=VisibilityState.DEGRADED,
                world_x=7.0, world_y=4.0
            )
            det = [(7.0, 4.0, fused_conf, "thermal", -1, (0, 0, 0, 0))]
            persistent = tracker.update(det, VisibilityState.DEGRADED)

        # After 8 frames → should be persistent
        assert len(persistent) == 1

        # Confidence should exceed threshold
        assert persistent[0].confidence > threshold
        assert persistent[0].confidence == pytest.approx(0.68, abs=0.001)

        # Dedup check → should be new survivor
        is_new, sid = dedup.check(
            persistent[0].x, persistent[0].y,
            persistent[0].confidence, persistent[0].source
        )
        assert is_new is True

    def test_matched_pair_fast_confirmation(self):
        """
        Matched RGB+thermal pair in NORMAL should confirm quickly
        with high confidence (0.87 after fusion).
        """
        aligner = SpatialAligner(match_distance=0.5)
        fuser = ConfidenceFuser(ema_alpha=1.0)
        tracker = PersistenceTracker(persistence_normal=5)

        rgb = [Detection(x=5.0, y=3.0, confidence=0.9, source="rgb")]
        thermal = [Detection(x=5.1, y=3.0, confidence=0.8, source="thermal")]

        for i in range(5):
            match_result = aligner.align(rgb, thermal)
            assert len(match_result.matched_pairs) == 1

            rgb_det, therm_det = match_result.matched_pairs[0]
            fused = fuser.fuse(
                rgb_confidence=rgb_det.confidence,
                thermal_confidence=therm_det.confidence,
                source="fused",
                visibility=VisibilityState.NORMAL,
                world_x=rgb_det.x, world_y=rgb_det.y
            )
            det = [(rgb_det.x, rgb_det.y, fused, "fused", 1, (0, 0, 0, 0))]
            persistent = tracker.update(det, VisibilityState.NORMAL)

        assert len(persistent) == 1
        assert persistent[0].confidence == pytest.approx(0.87, abs=0.001)

    def test_full_pipeline_six_survivors(self):
        """
        Full pipeline: detect and confirm 6 unique survivors,
        then verify 7th is rejected.
        """
        fuser = ConfidenceFuser(ema_alpha=1.0)
        tracker = PersistenceTracker(persistence_normal=1)  # 1 frame for speed
        dedup = Deduplicator(dedup_distance=0.5, max_survivors=6)

        threshold = 0.65
        confirmed_ids = []

        for i in range(7):
            x, y = float(i * 2), float(i * 2)

            fused_conf = fuser.fuse(
                rgb_confidence=0.9, thermal_confidence=0.85,
                source="fused", visibility=VisibilityState.NORMAL,
                world_x=x, world_y=y
            )

            det = [(x, y, fused_conf, "fused", i, (0, 0, 0, 0))]
            persistent = tracker.update(det, VisibilityState.NORMAL)

            for entry in persistent:
                if entry.confidence >= threshold:
                    is_new, sid = dedup.check(
                        entry.x, entry.y, entry.confidence, entry.source
                    )
                    if is_new:
                        confirmed_ids.append(sid)

        # Should have exactly 6 confirmed survivors
        assert len(confirmed_ids) == 6
        assert dedup.confirmed_count == 6
        assert dedup.is_full is True


# ==========================================================================
# Run tests
# ==========================================================================

if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
