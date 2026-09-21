#!/usr/bin/env python3
"""
Standalone test for the ByteTrack tracker integration.
Tests the core tracking logic without requiring ROS 2.

Duplicates the lightweight TrackerDetections wrapper to avoid importing
tracker_node.py (which pulls in rclpy at module level).

Run: python test_byte_tracker.py
"""

import sys
import argparse
import numpy as np

# ---------------------------------------------------------------------------
# Import ByteTrack
# ---------------------------------------------------------------------------
try:
    from ultralytics.trackers.byte_tracker import BYTETracker
except ImportError as e:
    print(f"FAIL: Cannot import BYTETracker — {e}")
    print("Install with: pip install ultralytics")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Duplicate of TrackerDetections (from tracker_node.py) — pure numpy, no ROS
# ---------------------------------------------------------------------------
class TrackerDetections:
    """Results-like wrapper expected by Ultralytics BYTETracker."""

    def __init__(self, xywh, conf, cls):
        self.xywh = np.asarray(xywh, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32).reshape(-1)
        self.cls  = np.asarray(cls, dtype=np.int32).reshape(-1)

    def __len__(self):
        return len(self.conf)

    def __getitem__(self, mask):
        return TrackerDetections(self.xywh[mask], self.conf[mask], self.cls[mask])


# ═══════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════

def make_tracker(**overrides):
    """Create a BYTETracker with the same defaults as TrackerNode."""
    defaults = dict(
        track_buffer=45,
        match_thresh=0.7,
        track_high_thresh=0.40,
        track_low_thresh=0.10,
        new_track_thresh=0.45,
        fuse_score=True,
    )
    defaults.update(overrides)
    return BYTETracker(argparse.Namespace(**defaults))


def make_detections(boxes, confs=None):
    """
    boxes: list of [x_center, y_center, w, h]
    confs: list of floats  (default 0.85 each)
    """
    boxes = np.array(boxes, dtype=np.float32).reshape(-1, 4)
    n = len(boxes)
    confs = np.full(n, 0.85, dtype=np.float32) if confs is None else np.array(confs, dtype=np.float32)
    cls = np.zeros(n, dtype=np.int32)
    return TrackerDetections(boxes, confs, cls)


def extract_tracks(output):
    """Return dict {track_id: (x1, y1, x2, y2, score)} from tracker output."""
    tracks = {}
    if len(output) == 0:
        return tracks
    for row in output:
        x1, y1, x2, y2, tid, score, _cls, _idx = row
        tracks[int(tid)] = (x1, y1, x2, y2, float(score))
    return tracks


passed = 0
failed = 0

def check(name, condition, detail=""):
    global passed, failed
    if condition:
        print(f"  ✅ {name}")
        passed += 1
    else:
        print(f"  ❌ {name}  — {detail}")
        failed += 1


# ═══════════════════════════════════════════════════════════════════════════
# Test 1 — TrackerDetections wrapper
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 1: TrackerDetections wrapper ━━━")

boxes = [[100, 200, 50, 60], [300, 400, 70, 80]]
dets = make_detections(boxes)

check("Length correct", len(dets) == 2, f"got {len(dets)}")
check("xywh shape", dets.xywh.shape == (2, 4), f"got {dets.xywh.shape}")
check("conf shape", dets.conf.shape == (2,), f"got {dets.conf.shape}")
check("cls shape",  dets.cls.shape  == (2,), f"got {dets.cls.shape}")

mask = np.array([True, False])
subset = dets[mask]
check("Slice length", len(subset) == 1)
check("Slice xywh",   np.allclose(subset.xywh[0], [100, 200, 50, 60]))

empty = make_detections([])
check("Empty detections", len(empty) == 0)


# ═══════════════════════════════════════════════════════════════════════════
# Test 2 — Single detection → track creation
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 2: Single detection — track creation ━━━")

tracker = make_tracker()
det = make_detections([[200, 200, 40, 60]], confs=[0.90])
out = tracker.update(det)
tracks = extract_tracks(out)

check("One track created", len(tracks) == 1, f"got {len(tracks)}")
if tracks:
    tid = list(tracks.keys())[0]
    check("Track ID is positive int", tid > 0, f"got {tid}")


# ═══════════════════════════════════════════════════════════════════════════
# Test 3 — Track persistence across frames (slow-moving target)
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 3: Track persistence (slow motion, 10 frames) ━━━")

tracker = make_tracker()
ids_seen = set()
for i in range(10):
    cx = 200 + i * 3
    cy = 200 + i * 1
    det = make_detections([[cx, cy, 40, 60]], confs=[0.90])
    out = tracker.update(det)
    tracks = extract_tracks(out)
    if tracks:
        ids_seen.update(tracks.keys())

check("Consistent ID across frames", len(ids_seen) == 1,
      f"expected 1, got {len(ids_seen)}: {ids_seen}")


# ═══════════════════════════════════════════════════════════════════════════
# Test 4 — Multiple simultaneous targets
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 4: Multiple simultaneous targets ━━━")

tracker = make_tracker()
for i in range(5):
    det = make_detections([
        [100 + i * 2, 100, 40, 60],
        [400 + i * 2, 300, 50, 70],
    ], confs=[0.90, 0.88])
    out = tracker.update(det)
    tracks = extract_tracks(out)

check("Two tracks maintained", len(tracks) == 2, f"got {len(tracks)}")
if len(tracks) == 2:
    tids = sorted(tracks.keys())
    check("Distinct IDs", tids[0] != tids[1])


# ═══════════════════════════════════════════════════════════════════════════
# Test 5 — Occlusion → re-identification
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 5: Occlusion and re-identification ━━━")

tracker = make_tracker()

# Phase 1: Establish track
original_id = None
for i in range(10):
    det = make_detections([[200, 200, 40, 60]], confs=[0.90])
    out = tracker.update(det)
    tracks = extract_tracks(out)
    if tracks:
        original_id = list(tracks.keys())[0]

check("Track established", original_id is not None)

# Phase 2: Disappear for 5 frames (within track_buffer=45)
for i in range(5):
    tracker.update(make_detections([]))

lost_ids = {t.track_id for t in tracker.lost_stracks}
check("Track in lost pool", original_id in lost_ids,
      f"lost pool: {lost_ids}, expected {original_id}")

# Phase 3: Reappear nearby
det = make_detections([[205, 202, 40, 60]], confs=[0.88])
out = tracker.update(det)
tracks = extract_tracks(out)

recovered_id = list(tracks.keys())[0] if tracks else None
check("Re-identified with same ID", recovered_id == original_id,
      f"original={original_id}, recovered={recovered_id}")


# ═══════════════════════════════════════════════════════════════════════════
# Test 6 — Track removal after exceeding track_buffer
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 6: Track removal (exceed track_buffer) ━━━")

tracker = make_tracker(track_buffer=5)

for i in range(5):
    det = make_detections([[200, 200, 40, 60]], confs=[0.90])
    tracker.update(det)

# Disappear for buffer + margin
for i in range(10):
    tracker.update(make_detections([]))

lost_ids = {t.track_id for t in tracker.lost_stracks}
check("Track removed from lost pool", len(lost_ids) == 0,
      f"still in lost pool: {lost_ids}")


# ═══════════════════════════════════════════════════════════════════════════
# Test 7 — Low-confidence filtering
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 7: Low-confidence filtering ━━━")

tracker = make_tracker()

det = make_detections([[200, 200, 40, 60]], confs=[0.05])
out = tracker.update(det)
tracks = extract_tracks(out)
check("Sub-threshold detection ignored", len(tracks) == 0,
      f"got {len(tracks)} tracks")

tracker2 = make_tracker()
for i in range(3):
    det = make_detections([[200, 200, 40, 60]], confs=[0.50])
    out = tracker2.update(det)
tracks = extract_tracks(out)
check("Above-threshold detection tracked", len(tracks) >= 1,
      f"got {len(tracks)} tracks")


# ═══════════════════════════════════════════════════════════════════════════
# Test 8 — Coordinate format round-trip
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 8: Coordinate format round-trip ━━━")

tracker = make_tracker()

# Input: xywh = [200, 300, 40, 60]
# Expected xyxy ≈ [180, 270, 220, 330]
for i in range(3):
    det = make_detections([[200, 300, 40, 60]], confs=[0.90])
    out = tracker.update(det)

tracks = extract_tracks(out)
if tracks:
    tid = list(tracks.keys())[0]
    x1, y1, x2, y2, _ = tracks[tid]
    check("x_min ≈ 180", abs(x1 - 180) < 10, f"got {x1:.1f}")
    check("y_min ≈ 270", abs(y1 - 270) < 10, f"got {y1:.1f}")
    check("x_max ≈ 220", abs(x2 - 220) < 10, f"got {x2:.1f}")
    check("y_max ≈ 330", abs(y2 - 330) < 10, f"got {y2:.1f}")
else:
    check("Track exists for coord test", False, "no tracks")


# ═══════════════════════════════════════════════════════════════════════════
# Test 9 — Crossing targets (ID stability under overlap)
#
# ByteTrack's Kalman filter tracks velocity, so its predicted positions
# diverge at the crossing point (A predicted rightward, B predicted
# leftward), allowing correct IoU-based association even through overlap.
#
# NOTE: We skip frames where target centers are closer than one box-width
# apart (< 60 px). At those frames the two tracked centroids are nearly
# identical, making the proximity-based "which track belongs to which
# person?" assignment in the *test harness* inherently ambiguous — this is
# a measurement issue, not a tracker issue.
# ═══════════════════════════════════════════════════════════════════════════
print("\n━━━ Test 9: Crossing targets — ID stability ━━━")

tracker = make_tracker()
id_history_a = []
id_history_b = []

for i in range(20):
    ax = 100 + i * 15     # person A moves right
    bx = 400 - i * 15     # person B moves left — they cross around frame 10
    det = make_detections([
        [ax, 200, 40, 60],
        [bx, 200, 40, 60],
    ], confs=[0.90, 0.90])
    out = tracker.update(det)
    tracks = extract_tracks(out)

    # Skip frames where targets overlap — proximity assignment is ambiguous
    if abs(ax - bx) < 60:
        continue

    # Match tracked output to ground-truth person by spatial proximity
    for tid, (x1, y1, x2, y2, _) in tracks.items():
        cx = (x1 + x2) / 2
        if abs(cx - ax) < abs(cx - bx):
            id_history_a.append(tid)
        else:
            id_history_b.append(tid)

unique_a = set(id_history_a)
unique_b = set(id_history_b)

# Each person must maintain a single, distinct ID throughout — including
# across the crossing point, verified via the well-separated frames only.
check("No ID switch during crossing",
      len(unique_a) == 1 and len(unique_b) == 1 and unique_a != unique_b,
      f"A had IDs {unique_a}, B had IDs {unique_b}")


# ═══════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "═" * 50)
total = passed + failed
print(f"Results: {passed}/{total} passed, {failed} failed")
if failed == 0:
    print("🎉 All tests passed!")
else:
    print("⚠️  Some tests failed — review output above.")
    sys.exit(1)
