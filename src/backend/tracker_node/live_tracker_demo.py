#!/usr/bin/env python3
"""
NIDAR — Live Camera + ByteTrack Tracker Demo
=============================================
Runs YOLO human detection on webcam feed and passes detections through
ByteTrack to assign persistent track IDs.

Usage:
    python live_tracker_demo.py                     # custom model + camera index 1 (fallback 0)
    python live_tracker_demo.py --model yolov8n.pt  # use pretrained YOLOv8 nano
    python live_tracker_demo.py --camera 0           # force laptop webcam
    python live_tracker_demo.py --conf 0.5           # lower detection confidence

Controls:
    q — quit
    p — pause / resume
    r — reset tracker (clear all track IDs)

No ROS 2 dependency.  Press 'q' to exit.
"""

import sys
import os
import argparse
import time
from collections import defaultdict
from unittest.mock import MagicMock

# Bypass broken SAM import in ultralytics (same workaround as test_camera.py)
try:
    sys.modules['ultralytics.models.sam'] = MagicMock()
except ImportError:
    pass

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.trackers.byte_tracker import BYTETracker


# ─── TrackerDetections wrapper (same as tracker_node.py) ───────────────────
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


# ─── Color palette for track IDs ──────────────────────────────────────────
TRACK_COLORS = [
    (0, 255, 0),     # green
    (255, 0, 0),     # blue
    (0, 0, 255),     # red
    (255, 255, 0),   # cyan
    (0, 255, 255),   # yellow
    (255, 0, 255),   # magenta
    (128, 255, 0),   # spring green
    (255, 128, 0),   # orange-blue
    (0, 128, 255),   # orange
    (255, 0, 128),   # pink
]

def get_track_color(track_id):
    return TRACK_COLORS[track_id % len(TRACK_COLORS)]


def create_tracker():
    """Create a BYTETracker with NIDAR defaults."""
    args = argparse.Namespace(
        track_buffer=45,
        match_thresh=0.7,
        track_high_thresh=0.40,
        track_low_thresh=0.10,
        new_track_thresh=0.45,
        fuse_score=True,
    )
    return BYTETracker(args)


def draw_tracked_box(frame, x1, y1, x2, y2, track_id, score, trail=None):
    """Draw a colored bounding box with track ID and confidence."""
    color = get_track_color(track_id)

    # Bounding box
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    # Label background
    label = f"ID:{track_id}  {score*100:.0f}%"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw + 6, y1), color, -1)
    cv2.putText(frame, label, (x1 + 3, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

    # Trail (centroid history)
    if trail and len(trail) > 1:
        pts = np.array(trail, dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=False, color=color, thickness=2)


def draw_hud(frame, fps, num_tracks, paused):
    """Draw heads-up display with stats."""
    h, w = frame.shape[:2]

    # Semi-transparent bar at top
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 40), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    status = "PAUSED" if paused else "LIVE"
    status_color = (0, 0, 255) if paused else (0, 255, 0)

    cv2.putText(frame, f"NIDAR Tracker | {status}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    cv2.putText(frame, f"FPS: {fps:.0f}  |  Tracks: {num_tracks}", (w - 280, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Controls hint at bottom
    cv2.putText(frame, "q:quit  p:pause  r:reset", (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)


def main():
    parser = argparse.ArgumentParser(description="NIDAR Live Tracker Demo")
    parser.add_argument("--model", default=None,
                        help="Path to YOLO model (default: custom trained model)")
    parser.add_argument("--camera", type=int, default=None,
                        help="Camera index (default: try 1, fallback 0)")
    parser.add_argument("--conf", type=float, default=0.60,
                        help="Detection confidence threshold (default: 0.60)")
    parser.add_argument("--skip", type=int, default=0,
                        help="Process every N+1 frames (0 = every frame)")
    args = parser.parse_args()

    # ── Load YOLO model ──
    if args.model:
        model_path = args.model
    else:
        # Default: custom trained model from detection_node
        model_path = os.path.join(
            os.path.dirname(__file__), '..', 'detection_node', 'human_dataset',
            'runs', 'detect', 'train7', 'weights', 'best.pt'
        )
        if not os.path.exists(model_path):
            # Fallback to pretrained YOLOv8n
            model_path = os.path.join(
                os.path.dirname(__file__), '..', 'detection_node', 'human_dataset',
                'yolov8n.pt'
            )

    print(f"Loading YOLO model: {model_path}")
    model = YOLO(model_path)

    # ── Open camera ──
    if args.camera is not None:
        cam_idx = args.camera
        cap = cv2.VideoCapture(cam_idx)
    else:
        print("Trying camera index 1...")
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            print("Camera 1 not found, falling back to 0...")
            cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("ERROR: Could not open any camera.")
        sys.exit(1)

    print("Camera opened successfully.")

    # ── Initialize tracker ──
    tracker = create_tracker()
    trails = defaultdict(list)    # track_id → list of (cx, cy) for trail drawing
    max_trail_len = 30

    # ── Main loop ──
    print("Starting live tracking... Press 'q' to quit.")
    paused = False
    frame_count = 0
    fps = 0.0
    prev_time = time.time()
    last_detections = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera stream ended.")
            break

        # Resize if very large
        if frame.shape[1] > 1280:
            scale = 1280 / frame.shape[1]
            frame = cv2.resize(frame, (1280, int(frame.shape[0] * scale)))

        if not paused:
            # ── Run YOLO detection (with optional frame skipping) ──
            if frame_count % (args.skip + 1) == 0:
                results = model(frame, conf=args.conf, classes=[0], verbose=False)
                last_detections = results[0] if results else None

            # ── Convert YOLO detections to TrackerDetections ──
            xywh_list = []
            conf_list = []

            if last_detections is not None and last_detections.boxes is not None:
                for box in last_detections.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    w = x2 - x1
                    h = y2 - y1
                    cx = x1 + w / 2.0
                    cy = y1 + h / 2.0
                    xywh_list.append([cx, cy, w, h])
                    conf_list.append(float(box.conf[0]))

            xywh_arr = np.array(xywh_list, dtype=np.float32).reshape(-1, 4)
            conf_arr = np.array(conf_list, dtype=np.float32).reshape(-1)
            cls_arr  = np.zeros(len(conf_list), dtype=np.int32)

            det = TrackerDetections(xywh_arr, conf_arr, cls_arr)

            # ── Update ByteTrack ──
            tracked_output = tracker.update(det)

            # ── Draw tracked results ──
            if len(tracked_output) > 0:
                for row in tracked_output:
                    x1, y1, x2, y2, tid, score, _cls, _idx = row
                    tid = int(tid)
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Update trail
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    trails[tid].append((cx, cy))
                    if len(trails[tid]) > max_trail_len:
                        trails[tid].pop(0)

                    draw_tracked_box(frame, x1, y1, x2, y2, tid, score, trails[tid])

            # ── FPS calculation ──
            now = time.time()
            dt = now - prev_time
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            prev_time = now
            frame_count += 1

        # ── HUD ──
        num_tracks = len(tracked_output) if not paused and len(tracked_output) > 0 else 0
        draw_hud(frame, fps, num_tracks, paused)

        cv2.imshow("NIDAR Live Tracker", frame)

        # ── Controls ──
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            paused = not paused
            print("Paused." if paused else "Resumed.")
        elif key == ord('r'):
            tracker = create_tracker()
            trails.clear()
            print("Tracker reset — all IDs cleared.")

    cap.release()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == '__main__':
    main()
