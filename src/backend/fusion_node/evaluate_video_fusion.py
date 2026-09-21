#!/usr/bin/env python3
"""
NIDAR AirMouse — Real Video RGB + Thermal Fusion Evaluation & GIF Generator
===========================================================================
Processes real thermal footage (from YouTube / local dataset), pairs with RGB stream,
runs YOLO detectors on both channels, executes the 6-stage Fusion Node pipeline,
and outputs an annotated side-by-side GIF and video.
"""

import sys
import os
import glob
import json
import importlib
import importlib.util
import numpy as np
import cv2
from PIL import Image as PILImage

# ---------------------------------------------------------------------------
# Import Fusion Node modules
# ---------------------------------------------------------------------------
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

try:
    from ultralytics import YOLO
except ImportError:
    print("Ultralytics YOLO required. Installing or running in venv.")
    sys.exit(1)


def run_video_fusion_test(
    frames_dir: str,
    thermal_model_path: str,
    rgb_model_path: str,
    output_gif_path: str,
    output_mp4_path: str,
    output_report_path: str,
    max_frames: int = 50
):
    print(f"Loading frames from: {frames_dir}")
    frame_files = sorted(glob.glob(os.path.join(frames_dir, "*.jpg")))[:max_frames]
    if not frame_files:
        raise ValueError(f"No frames found in {frames_dir}")

    print(f"Found {len(frame_files)} frames. Loading YOLO models...")
    thermal_model = YOLO(thermal_model_path)
    rgb_model = YOLO(rgb_model_path)

    # Initialize Fusion pipeline components
    visibility_estimator = VisibilityEstimator(brightness_threshold=50, update_interval=1)
    spatial_aligner = SpatialAligner(match_distance=0.5)
    confidence_fuser = ConfidenceFuser(
        rgb_weight_normal=0.7,
        thermal_weight_normal=0.3,
        rgb_weight_degraded=0.3,
        thermal_weight_degraded=0.7,
        ema_alpha=0.3
    )
    persistence_tracker = PersistenceTracker(
        persistence_normal=3,    # 3 frames for compact demo
        persistence_degraded=5,
        position_tolerance=0.3
    )
    deduplicator = Deduplicator(dedup_distance=0.5, max_survivors=6)
    confidence_threshold = 0.60

    # Output storage
    annotated_frames = []
    log_records = []

    print("Processing frames through RGB + Thermal Fusion pipeline...")

    # We will simulate drone movement (slow pan across 15x15m arena)
    for idx, fpath in enumerate(frame_files):
        thermal_frame = cv2.imread(fpath)
        th_h, th_w = thermal_frame.shape[:2]

        # Simulate drone pose in arena (heading slightly changing)
        drone_x = 2.0 + (idx * 0.15)
        drone_y = 3.0 + np.sin(idx * 0.1) * 0.5
        drone_yaw = 0.05 * np.sin(idx * 0.2)

        # Create paired RGB frame (in first half normal lighting, in second half simulate smoke/darkness)
        rgb_frame = thermal_frame.copy()
        if idx >= len(frame_files) // 2:
            # Simulate smoke/low-light degradation
            alpha = max(0.1, 1.0 - ((idx - len(frame_files) // 2) / (len(frame_files) // 2)))
            rgb_frame = (rgb_frame * alpha * 0.3).astype(np.uint8)

        # 1. Visibility estimation
        vis_state = visibility_estimator.update(rgb_frame)
        brightness = visibility_estimator.last_brightness

        # 2. Run thermal detector
        th_res = thermal_model(thermal_frame, conf=0.15, verbose=False)[0]
        thermal_detections = []
        for box in th_res.boxes:
            b = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cx = (b[0] + b[2]) / 2.0
            cy = (b[1] + b[3]) / 2.0
            w_box = b[2] - b[0]
            h_box = b[3] - b[1]

            # Project to world using drone pose
            depth_est = max(1.5, 2.5 - (h_box / th_h) * 1.5)
            wx, wy = SpatialAligner.project_to_world(
                cx, cy, depth_est, drone_x, drone_y, drone_yaw, image_width=th_w
            )
            thermal_detections.append(Detection(
                x=wx, y=wy, confidence=conf, source="thermal",
                bbox=(float(b[0]), float(b[1]), float(w_box), float(h_box)),
                depth=depth_est
            ))

        # 3. Run RGB detector
        rgb_res = rgb_model(rgb_frame, conf=0.15, verbose=False)[0]
        rgb_detections = []
        for box in rgb_res.boxes:
            cls_id = int(box.cls[0])
            # Only consider person class (0 in COCO)
            if cls_id != 0:
                continue
            b = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cx = (b[0] + b[2]) / 2.0
            cy = (b[1] + b[3]) / 2.0
            w_box = b[2] - b[0]
            h_box = b[3] - b[1]

            depth_est = max(1.5, 2.5 - (h_box / th_h) * 1.5)
            wx, wy = SpatialAligner.project_to_world(
                cx, cy, depth_est, drone_x, drone_y, drone_yaw, image_width=th_w
            )
            rgb_detections.append(Detection(
                x=wx, y=wy, confidence=conf, source="rgb",
                bbox=(float(b[0]), float(b[1]), float(w_box), float(h_box)),
                depth=depth_est, track_id=idx + 1
            ))

        # 4. Spatial alignment
        match_result = spatial_aligner.align(rgb_detections, thermal_detections)

        # 5. Confidence Fusion
        fused_candidates = []
        for r_det, t_det in match_result.matched_pairs:
            fconf = confidence_fuser.fuse(
                rgb_confidence=r_det.confidence, thermal_confidence=t_det.confidence,
                source="fused", visibility=vis_state, world_x=r_det.x, world_y=r_det.y
            )
            fused_candidates.append((r_det.x, r_det.y, fconf, "fused", r_det.track_id, r_det.bbox))

        for r_det in match_result.unmatched_rgb:
            fconf = confidence_fuser.fuse(
                rgb_confidence=r_det.confidence, thermal_confidence=None,
                source="rgb", visibility=vis_state, world_x=r_det.x, world_y=r_det.y
            )
            fused_candidates.append((r_det.x, r_det.y, fconf, "rgb", r_det.track_id, r_det.bbox))

        for t_det in match_result.unmatched_thermal:
            fconf = confidence_fuser.fuse(
                rgb_confidence=None, thermal_confidence=t_det.confidence,
                source="thermal", visibility=vis_state, world_x=t_det.x, world_y=t_det.y
            )
            fused_candidates.append((t_det.x, t_det.y, fconf, "thermal", -1, t_det.bbox))

        # 6. Persistence Tracking
        persistent = persistence_tracker.update(fused_candidates, vis_state)

        # 7. Deduplication & Confirmation
        new_confirmed = []
        for entry in persistent:
            if entry.confidence >= confidence_threshold:
                is_new, sid = deduplicator.check(
                    entry.x, entry.y, entry.confidence, entry.source, entry.bbox
                )
                if is_new:
                    new_confirmed.append(sid)

        # -------------------------------------------------------------------
        # Build Visualization Canvas:
        # [RGB Frame] | [Thermal Frame] | [Fusion Node Status & Map HUD]
        # -------------------------------------------------------------------
        disp_w, disp_h = 320, 180
        rgb_disp = cv2.resize(rgb_frame, (disp_w, disp_h))
        th_disp = cv2.resize(thermal_frame, (disp_w, disp_h))

        # Annotate RGB
        cv2.putText(rgb_disp, f"RGB CAMERA (vis: {vis_state.value})", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(rgb_disp, f"Brightness: {brightness:.1f}", (8, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        for r in rgb_detections:
            rx, ry, rw, rh = [int(v * disp_w / th_w) for v in r.bbox]
            cv2.rectangle(rgb_disp, (rx, ry), (rx + rw, ry + rh), (255, 200, 0), 2)
            cv2.putText(rgb_disp, f"RGB: {r.confidence:.2f}", (rx, max(15, ry - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)

        # Annotate Thermal
        cv2.putText(th_disp, "THERMAL CAMERA (YOLO11n)", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 1, cv2.LINE_AA)
        for t in thermal_detections:
            tx, ty, tw, th = [int(v * disp_w / th_w) for v in t.bbox]
            cv2.rectangle(th_disp, (tx, ty), (tx + tw, ty + th), (0, 100, 255), 2)
            cv2.putText(th_disp, f"TH: {t.confidence:.2f}", (tx, max(15, ty - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 100, 255), 1)

        # Create HUD panel
        hud_w = 340
        hud = np.zeros((disp_h, hud_w, 3), dtype=np.uint8)
        hud[:] = (25, 25, 30)

        # Draw HUD info
        cv2.putText(hud, "FUSION NODE (NIDAR)", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        vis_color = (0, 255, 0) if vis_state == VisibilityState.NORMAL else (0, 165, 255)
        cv2.putText(hud, f"Visibility: {vis_state.value}", (10, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, vis_color, 1)

        active_det = "RGB+TH" if (rgb_detections and thermal_detections) else ("RGB" if rgb_detections else ("TH" if thermal_detections else "NONE"))
        cv2.putText(hud, f"Sensors: {active_det} | Matches: {len(match_result.matched_pairs)}", (10, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1)

        cv2.putText(hud, f"Drone Pose: ({drone_x:.1f}m, {drone_y:.1f}m)", (10, 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        confirmed_list = deduplicator.get_confirmed_list()
        cv2.putText(hud, f"Confirmed Survivors: {len(confirmed_list)}/6", (10, 108),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1)

        y_offset = 128
        for s in confirmed_list[-2:]:
            cv2.putText(hud, f" #S{s.survivor_id}: pos=({s.x:.1f},{s.y:.1f}) c={s.confidence:.2f} [{s.detection_source}]",
                        (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1)
            y_offset += 18

        # Combine 3 panels horizontally
        combined = np.hstack([rgb_disp, th_disp, hud])
        annotated_frames.append(combined)

        # Log record
        log_records.append({
            "frame": idx + 1,
            "visibility": vis_state.value,
            "brightness": float(brightness),
            "rgb_count": len(rgb_detections),
            "thermal_count": len(thermal_detections),
            "matched_count": len(match_result.matched_pairs),
            "candidates": len(fused_candidates),
            "persistent_count": len(persistent),
            "confirmed_total": len(confirmed_list),
            "new_confirmed": new_confirmed
        })

    # Save outputs
    print("Writing outputs...")
    os.makedirs(os.path.dirname(output_gif_path), exist_ok=True)

    # 1. Save GIF
    pil_images = [PILImage.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)) for f in annotated_frames]
    if pil_images:
        pil_images[0].save(
            output_gif_path,
            save_all=True,
            append_images=pil_images[1:],
            duration=150,  # ~6.6 fps
            loop=0
        )
        print(f"[OK] Saved GIF: {output_gif_path}")

    # 2. Save MP4 Video
    if annotated_frames:
        h, w = annotated_frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out_vid = cv2.VideoWriter(output_mp4_path, fourcc, 6.0, (w, h))
        for f in annotated_frames:
            out_vid.write(f)
        out_vid.release()
        print(f"[OK] Saved Video: {output_mp4_path}")

    # 3. Save JSON Report
    with open(output_report_path, 'w') as jf:
        json.dump({
            "total_frames_evaluated": len(frame_files),
            "total_confirmed_survivors": deduplicator.confirmed_count,
            "confirmed_survivors": [
                {
                    "survivor_id": s.survivor_id,
                    "x": s.x, "y": s.y,
                    "confidence": s.confidence,
                    "detection_source": s.detection_source,
                    "detection_count": s.detection_count
                } for s in deduplicator.get_confirmed_list()
            ],
            "frame_logs": log_records
        }, jf, indent=2)
    print(f"[OK] Saved Evaluation Report: {output_report_path}")

    return deduplicator.confirmed_count


if __name__ == '__main__':
    # Determine project root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))

    frames_dir = os.path.join(script_dir, 'test_data', 'extracted_frames')
    th_model = os.path.join(project_root, 'src', 'backend', 'detection_node', 'thermal_model', 'thermal_training', 'models', 'thermal_survivor_v1_best.pt')
    rgb_model = os.path.join(project_root, 'src', 'backend', 'detection_node', 'thermal_model', 'yolo26n.pt')
    gif_out = os.path.join(script_dir, 'test_data', 'fusion_test_output.gif')
    mp4_out = os.path.join(script_dir, 'test_data', 'fusion_test_output.mp4')
    report_out = os.path.join(script_dir, 'test_data', 'fusion_report.json')

    print(f"Project Root: {project_root}")
    print(f"Frames Directory: {frames_dir}")
    print(f"Thermal Model: {th_model}")
    print(f"RGB Model: {rgb_model}")

    count = run_video_fusion_test(
        frames_dir, th_model, rgb_model, gif_out, mp4_out, report_out, max_frames=66
    )
    print(f"\n[DONE] Fusion evaluation completed successfully! Confirmed survivors: {count}/6")
