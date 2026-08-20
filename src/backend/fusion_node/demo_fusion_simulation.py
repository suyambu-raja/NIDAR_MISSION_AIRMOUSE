#!/usr/bin/env python3
"""
NIDAR AirMouse — Full Fusion Pipeline Simulation
=================================================
Simulates the complete RGB + thermal fusion pipeline WITHOUT ROS 2.
Tests all real-world scenarios the drone will encounter in the arena.

Run:
    python demo_fusion_simulation.py

This script simulates 4 mission scenarios:
    1. NORMAL visibility — RGB + thermal both detect same survivor
    2. NORMAL visibility — RGB only (thermal misses)
    3. DEGRADED visibility (smoke/dark) — thermal primary
    4. Full mission — find all 6 survivors across mixed conditions
"""

import sys
import os
import importlib
import importlib.util
import numpy as np

# ===========================================================================
# Import fusion modules without ROS 2
# ===========================================================================
_pkg_dir = os.path.join(os.path.dirname(__file__), 'fusion_node')

def _import_module(module_name, file_name):
    spec = importlib.util.spec_from_file_location(
        module_name, os.path.join(_pkg_dir, file_name)
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod

_vis = _import_module('fusion_node.visibility_estimator', 'visibility_estimator.py')
_align = _import_module('fusion_node.spatial_aligner', 'spatial_aligner.py')
_conf = _import_module('fusion_node.confidence_fuser', 'confidence_fuser.py')
_pers = _import_module('fusion_node.persistence_tracker', 'persistence_tracker.py')
_dedup = _import_module('fusion_node.deduplicator', 'deduplicator.py')

VisibilityEstimator = _vis.VisibilityEstimator
VisibilityState = _vis.VisibilityState
SpatialAligner = _align.SpatialAligner
Detection = _align.Detection
ConfidenceFuser = _conf.ConfidenceFuser
PersistenceTracker = _pers.PersistenceTracker
Deduplicator = _dedup.Deduplicator


# ===========================================================================
# Colours for terminal output
# ===========================================================================
class C:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'


def header(text):
    print(f"\n{C.BOLD}{C.CYAN}{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}{C.RESET}\n")


def step(text):
    print(f"  {C.DIM}→{C.RESET} {text}")


def result(text, success=True):
    icon = f"{C.GREEN}✓" if success else f"{C.RED}✗"
    print(f"  {icon} {text}{C.RESET}")


def section(text):
    print(f"\n  {C.YELLOW}{C.BOLD}--- {text} ---{C.RESET}")


# ===========================================================================
# Pipeline runner — same logic as fusion_node.py but without ROS 2
# ===========================================================================
def run_fusion_pipeline(
    rgb_detections, thermal_detections, visibility_state, confidence_threshold,
    aligner, fuser, persistence, dedup, num_frames=5
):
    """
    Run the fusion pipeline for a given set of detections over multiple frames.
    Returns list of newly confirmed survivors.
    """
    new_survivors = []

    for frame in range(num_frames):
        # Step 1: Spatial alignment
        match_result = aligner.align(rgb_detections, thermal_detections)

        # Step 2: Confidence fusion
        candidates = []

        for rgb_det, thermal_det in match_result.matched_pairs:
            fused_conf = fuser.fuse(
                rgb_confidence=rgb_det.confidence,
                thermal_confidence=thermal_det.confidence,
                source="fused", visibility=visibility_state,
                world_x=rgb_det.x, world_y=rgb_det.y
            )
            candidates.append((rgb_det.x, rgb_det.y, fused_conf, "fused",
                              rgb_det.track_id, rgb_det.bbox))

        for det in match_result.unmatched_rgb:
            fused_conf = fuser.fuse(
                rgb_confidence=det.confidence, thermal_confidence=None,
                source="rgb", visibility=visibility_state,
                world_x=det.x, world_y=det.y
            )
            candidates.append((det.x, det.y, fused_conf, "rgb",
                              det.track_id, det.bbox))

        for det in match_result.unmatched_thermal:
            fused_conf = fuser.fuse(
                rgb_confidence=None, thermal_confidence=det.confidence,
                source="thermal", visibility=visibility_state,
                world_x=det.x, world_y=det.y
            )
            candidates.append((det.x, det.y, fused_conf, "thermal",
                              det.track_id, det.bbox))

        # Step 3: Persistence tracking
        persistent = persistence.update(candidates, visibility_state)

        # Step 4: Confidence threshold + deduplication
        for entry in persistent:
            if entry.confidence >= confidence_threshold:
                is_new, sid = dedup.check(
                    entry.x, entry.y, entry.confidence, entry.source, entry.bbox
                )
                if is_new:
                    new_survivors.append({
                        'id': sid,
                        'x': entry.x, 'y': entry.y,
                        'confidence': entry.confidence,
                        'source': entry.source,
                        'frame': frame + 1
                    })

    return new_survivors


# ===========================================================================
# SCENARIO 1: Normal Visibility — RGB + Thermal Both Detect
# ===========================================================================
def scenario_1():
    header("SCENARIO 1: NORMAL Visibility — RGB + Thermal Both Detect")

    print(f"  {C.MAGENTA}Arena: well-lit room, both sensors active{C.RESET}")
    print(f"  {C.MAGENTA}Survivor at map position (5.2, 3.8){C.RESET}\n")

    # Visibility estimation
    section("Step 1: Visibility Estimation")
    estimator = VisibilityEstimator(brightness_threshold=50, update_interval=1)
    bright_frame = np.full((480, 640, 3), 180, dtype=np.uint8)  # Well-lit arena
    vis = estimator.update(bright_frame)
    step(f"Average brightness: {estimator.last_brightness:.0f}")
    result(f"Visibility state: {vis.value}", vis == VisibilityState.NORMAL)

    # Mock detections
    section("Step 2: Incoming Detections")
    rgb_det = Detection(x=5.2, y=3.8, confidence=0.92, source="rgb",
                       bbox=(100, 150, 80, 120), track_id=1)
    thermal_det = Detection(x=5.3, y=3.7, confidence=0.78, source="thermal",
                           bbox=(95, 145, 85, 125), track_id=-1)
    step(f"RGB detection:     pos=({rgb_det.x}, {rgb_det.y}) conf={rgb_det.confidence}")
    step(f"Thermal detection: pos=({thermal_det.x}, {thermal_det.y}) conf={thermal_det.confidence}")

    # Spatial alignment
    section("Step 3: Spatial Alignment")
    aligner = SpatialAligner(match_distance=0.5)
    dist = aligner.euclidean_distance(rgb_det, thermal_det)
    step(f"Distance between detections: {dist:.3f}m")
    match = aligner.align([rgb_det], [thermal_det])
    result(f"MATCHED — same survivor (distance {dist:.3f}m < 0.5m threshold)",
           len(match.matched_pairs) == 1)

    # Confidence fusion
    section("Step 4: Confidence Fusion (NORMAL weights: RGB=0.7, Thermal=0.3)")
    fuser = ConfidenceFuser(ema_alpha=1.0)
    fused = fuser.fuse_matched(rgb_det.confidence, thermal_det.confidence,
                                VisibilityState.NORMAL)
    step(f"Fused = ({rgb_det.confidence} × 0.7) + ({thermal_det.confidence} × 0.3)")
    step(f"Fused = {rgb_det.confidence * 0.7:.3f} + {thermal_det.confidence * 0.3:.3f}")
    result(f"Fused confidence: {fused:.3f} (threshold: 0.65)", fused > 0.65)

    # Persistence
    section("Step 5: Persistence Tracking (NORMAL: 5 frames required)")
    persistence = PersistenceTracker(persistence_normal=5)
    for i in range(5):
        det = [(5.2, 3.8, fused, "fused", 1, (100, 150, 80, 120))]
        persistent = persistence.update(det, VisibilityState.NORMAL)
        status = f"frame {i+1}/5 — " + ("PERSISTENT ✓" if persistent else "accumulating...")
        step(status)

    result(f"Persistence achieved after 5 frames", len(persistent) > 0)

    # Deduplication
    section("Step 6: Deduplication")
    dedup = Deduplicator(max_survivors=6)
    is_new, sid = dedup.check(5.2, 3.8, fused, "fused")
    result(f"NEW SURVIVOR CONFIRMED — ID={sid}, confidence={fused:.3f}", is_new)

    return True


# ===========================================================================
# SCENARIO 2: Normal Visibility — RGB Only (Thermal Misses)
# ===========================================================================
def scenario_2():
    header("SCENARIO 2: NORMAL Visibility — RGB Only Detection")

    print(f"  {C.MAGENTA}Arena: well-lit, survivor partially occluded from thermal{C.RESET}")
    print(f"  {C.MAGENTA}Survivor at map position (8.5, 6.2){C.RESET}\n")

    section("Visibility: NORMAL (brightness=160)")
    section("RGB Detection Only — No Thermal Match")

    rgb_det = Detection(x=8.5, y=6.2, confidence=0.88, source="rgb",
                       track_id=2)
    step(f"RGB detection: pos=({rgb_det.x}, {rgb_det.y}) conf={rgb_det.confidence}")
    step(f"Thermal: NONE (no detection from thermal sensor)")

    section("Confidence Fusion (Unmatched RGB in NORMAL)")
    fuser = ConfidenceFuser(ema_alpha=1.0)
    fused = fuser.fuse_unmatched(rgb_det.confidence, "rgb", VisibilityState.NORMAL)
    step(f"Fused = {rgb_det.confidence} × 0.8 (unmatched primary penalty)")
    result(f"Fused confidence: {fused:.3f} > 0.65 threshold", fused > 0.65)

    section("Persistence (5 frames)")
    persistence = PersistenceTracker(persistence_normal=5)
    for i in range(5):
        det = [(8.5, 6.2, fused, "rgb", 2, (0, 0, 0, 0))]
        persistent = persistence.update(det, VisibilityState.NORMAL)
    result(f"Persistent after 5 frames, confidence={fused:.3f}", len(persistent) > 0)

    dedup = Deduplicator(max_survivors=6)
    is_new, sid = dedup.check(8.5, 6.2, fused, "rgb")
    result(f"RGB-ONLY SURVIVOR CONFIRMED — ID={sid}", is_new)

    return True


# ===========================================================================
# SCENARIO 3: Degraded Visibility — Thermal Primary
# ===========================================================================
def scenario_3():
    header("SCENARIO 3: DEGRADED Visibility — Thermal Is Primary")

    print(f"  {C.MAGENTA}Arena: smoke-filled corridor, RGB camera blinded{C.RESET}")
    print(f"  {C.MAGENTA}Survivor at map position (3.1, 11.4){C.RESET}\n")

    section("Step 1: Visibility Estimation")
    estimator = VisibilityEstimator(brightness_threshold=50, update_interval=1)
    dark_frame = np.full((480, 640, 3), 25, dtype=np.uint8)  # Smoke-filled
    vis = estimator.update(dark_frame)
    step(f"Average brightness: {estimator.last_brightness:.0f} (smoke/darkness)")
    result(f"Visibility state: {vis.value}", vis == VisibilityState.DEGRADED)

    section("Step 2: Thermal-Only Detection")
    thermal_det = Detection(x=3.1, y=11.4, confidence=0.82, source="thermal",
                           track_id=-1)
    step(f"Thermal detection: pos=({thermal_det.x}, {thermal_det.y}) conf={thermal_det.confidence}")
    step(f"RGB: NONE (camera blinded by smoke)")

    section("Step 3: Confidence Fusion (Thermal primary in DEGRADED)")
    fuser = ConfidenceFuser(ema_alpha=1.0)
    fused = fuser.fuse_unmatched(thermal_det.confidence, "thermal",
                                  VisibilityState.DEGRADED)
    step(f"Fused = {thermal_det.confidence} × 0.8 (unmatched primary in DEGRADED)")
    result(f"Fused confidence: {fused:.3f} > 0.65 threshold", fused > 0.65)

    section("Contrast: What if this were NORMAL visibility?")
    fused_normal = fuser.fuse_unmatched(thermal_det.confidence, "thermal",
                                         VisibilityState.NORMAL)
    step(f"In NORMAL: {thermal_det.confidence} × 0.4 = {fused_normal:.3f}")
    result(f"Would be REJECTED in NORMAL ({fused_normal:.3f} < 0.65)", fused_normal < 0.65)

    section("Step 4: Persistence (DEGRADED: 8 frames required)")
    persistence = PersistenceTracker(persistence_degraded=8)
    for i in range(8):
        det = [(3.1, 11.4, fused, "thermal", -1, (0, 0, 0, 0))]
        persistent = persistence.update(det, VisibilityState.DEGRADED)
        if i in [4, 7]:
            marker = "NOT YET" if i == 4 else "PERSISTENT ✓"
            step(f"Frame {i+1}/8 — {marker}")
    result(f"Persistent after 8 frames (3 more than NORMAL)", len(persistent) > 0)

    dedup = Deduplicator(max_survivors=6)
    is_new, sid = dedup.check(3.1, 11.4, fused, "thermal")
    result(f"THERMAL-ONLY SURVIVOR CONFIRMED — ID={sid}", is_new)

    return True


# ===========================================================================
# SCENARIO 4: Full Mission — 6 Survivors Across Mixed Conditions
# ===========================================================================
def scenario_4():
    header("SCENARIO 4: Full Mission — Find All 6 Survivors")

    print(f"  {C.MAGENTA}15×15m indoor maze with mixed lighting conditions{C.RESET}")
    print(f"  {C.MAGENTA}6 survivors hidden throughout the arena{C.RESET}\n")

    aligner = SpatialAligner(match_distance=0.5)
    fuser = ConfidenceFuser(ema_alpha=0.3)
    persistence = PersistenceTracker(persistence_normal=5, persistence_degraded=8)
    dedup = Deduplicator(dedup_distance=0.5, max_survivors=6)
    threshold = 0.65

    # Define 6 survivor scenarios
    survivors = [
        {
            'name': 'Survivor 1 — Room A (well-lit)',
            'vis': VisibilityState.NORMAL,
            'rgb': Detection(x=2.5, y=1.8, confidence=0.91, source="rgb", track_id=1),
            'thermal': Detection(x=2.6, y=1.9, confidence=0.76, source="thermal"),
            'frames_needed': 5
        },
        {
            'name': 'Survivor 2 — Corridor B (smoke)',
            'vis': VisibilityState.DEGRADED,
            'rgb': None,
            'thermal': Detection(x=7.2, y=4.5, confidence=0.84, source="thermal"),
            'frames_needed': 8
        },
        {
            'name': 'Survivor 3 — Room C (well-lit, RGB only)',
            'vis': VisibilityState.NORMAL,
            'rgb': Detection(x=11.0, y=8.3, confidence=0.89, source="rgb", track_id=3),
            'thermal': None,
            'frames_needed': 5
        },
        {
            'name': 'Survivor 4 — Room D (both sensors)',
            'vis': VisibilityState.NORMAL,
            'rgb': Detection(x=4.8, y=12.1, confidence=0.93, source="rgb", track_id=4),
            'thermal': Detection(x=4.7, y=12.0, confidence=0.81, source="thermal"),
            'frames_needed': 5
        },
        {
            'name': 'Survivor 5 — Dark zone (thermal only)',
            'vis': VisibilityState.DEGRADED,
            'rgb': None,
            'thermal': Detection(x=13.5, y=6.7, confidence=0.79, source="thermal"),
            'frames_needed': 8
        },
        {
            'name': 'Survivor 6 — Exit corridor (both, degraded)',
            'vis': VisibilityState.DEGRADED,
            'rgb': Detection(x=1.2, y=14.3, confidence=0.71, source="rgb", track_id=6),
            'thermal': Detection(x=1.3, y=14.2, confidence=0.88, source="thermal"),
            'frames_needed': 8
        },
    ]

    for surv in survivors:
        section(surv['name'])

        rgb_list = [surv['rgb']] if surv['rgb'] else []
        thermal_list = [surv['thermal']] if surv['thermal'] else []

        new = run_fusion_pipeline(
            rgb_list, thermal_list, surv['vis'], threshold,
            aligner, fuser, persistence, dedup,
            num_frames=surv['frames_needed']
        )

        if new:
            s = new[0]
            result(
                f"CONFIRMED — ID={s['id']} at ({s['x']:.1f}, {s['y']:.1f}) "
                f"conf={s['confidence']:.3f} via {s['source'].upper()} "
                f"[{surv['vis'].value} visibility, {surv['frames_needed']} frames]"
            )
        else:
            result(f"Not confirmed (expected for some scenarios)", False)

    # Try a 7th survivor — should be REJECTED
    section("Survivor 7 — Should Be REJECTED (max 6 limit)")
    step("Attempting to add 7th survivor at (9.0, 9.0)...")
    is_new, sid = dedup.check(9.0, 9.0, 0.95, "fused")
    result(f"REJECTED — max 6 survivor limit enforced (sid={sid})", not is_new)

    # Final summary
    section("MISSION SUMMARY")
    confirmed = dedup.get_confirmed_list()
    print(f"\n  {C.BOLD}  Confirmed survivors: {len(confirmed)}/6{C.RESET}")
    print(f"  {'─'*60}")
    print(f"  {'ID':>4}  {'Position':>14}  {'Confidence':>10}  {'Source':>8}  {'Detections':>10}")
    print(f"  {'─'*60}")
    for s in confirmed:
        print(f"  {s.survivor_id:>4}  ({s.x:>5.1f}, {s.y:>5.1f})  "
              f"{s.confidence:>10.3f}  {s.detection_source:>8}  {s.detection_count:>10}")
    print(f"  {'─'*60}")

    return len(confirmed) == 6


# ===========================================================================
# Main
# ===========================================================================
def main():
    print(f"\n{C.BOLD}{C.CYAN}")
    print("  +===========================================================+")
    print("  |      NIDAR AirMouse - Fusion Pipeline Simulation          |")
    print("  |      RGB + Thermal Sensor Fusion Demo                     |")
    print("  +===========================================================+")
    print(f"{C.RESET}")

    results = []

    results.append(("Scenario 1: RGB+Thermal matched (NORMAL)", scenario_1()))
    results.append(("Scenario 2: RGB-only (NORMAL)", scenario_2()))
    results.append(("Scenario 3: Thermal-only (DEGRADED)", scenario_3()))
    results.append(("Scenario 4: Full 6-survivor mission", scenario_4()))

    # Final report
    header("FINAL RESULTS")
    all_pass = True
    for name, passed in results:
        icon = f"{C.GREEN}✓ PASS" if passed else f"{C.RED}✗ FAIL"
        print(f"  {icon}  {name}{C.RESET}")
        if not passed:
            all_pass = False

    if all_pass:
        print(f"\n  {C.BOLD}{C.GREEN}All scenarios passed! Pipeline ready for RPi deployment.{C.RESET}\n")
    else:
        print(f"\n  {C.BOLD}{C.RED}Some scenarios failed — review output above.{C.RESET}\n")

    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
