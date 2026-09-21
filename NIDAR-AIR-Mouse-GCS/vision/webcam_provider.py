"""
Webcam & Live Video Stream Provider with YOLOv8 Detection for NIDAR AirMouse GCS.
Supports laptop built-in webcam (index 0), external USB cameras, and RTSP streams (Tapo / IP cameras).
Integrates seamlessly with the GCS priority message queue and dashboard WebSocket stream.
"""
import base64
import os
from pathlib import Path
import sys
import threading
import time
from typing import Optional, Dict, Any, Union
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    YOLO = None
    ULTRALYTICS_AVAILABLE = False


class WebcamProvider:
    """
    Captures live frames from laptop webcam or USB camera and streams to GCS dashboard.
    Runs non-blocking in a dedicated daemon thread.
    """

    def __init__(self, camera_index: Union[int, str] = 0, width: int = 640, height: int = 480, fps: int = 25):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = max(1, fps)
        self.cap: Optional[cv2.VideoCapture] = None
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._latest_b64: Optional[str] = None
        self._last_frame_time = 0.0
        self._is_live = False

    def start(self, message_queue=None, loop=None):
        """Starts capture loop. Can be called directly or run in a background thread."""
        if self.running:
            return

        src = self.camera_index
        if isinstance(src, str) and src.isdigit():
            src = int(src)

        print(f"[WebcamProvider] Connecting to camera source: {src}...")

        # Use DirectShow backend on Windows for fast hardware initialization if integer index
        if isinstance(src, int) and sys.platform.startswith("win"):
            self.cap = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(src)

        if not self.cap.isOpened():
            print(f"❌ [WebcamProvider] Webcam not found at index/source {src}")
            self.running = False
            with self._lock:
                self._is_live = False
            return

        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        print(f"✅ [WebcamProvider] Webcam connected successfully at index/source {src}")
        self.running = True
        frame_interval = 1.0 / self.fps

        while self.running:
            start_t = time.time()
            ret, frame = self.cap.read()
            if not ret or frame is None:
                with self._lock:
                    self._is_live = False
                time.sleep(0.05)
                continue

            # Resize to target resolution if needed
            h, w = frame.shape[:2]
            if w != self.width or h != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

            # Process frame hook (e.g. YOLO detection in subclass)
            processed_frame = self.process_frame(frame)

            # Encode to JPEG Base64
            _, buffer = cv2.imencode('.jpg', processed_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            b64 = base64.b64encode(buffer).decode('utf-8')

            now = time.time()
            with self._lock:
                self._latest_b64 = b64
                self._last_frame_time = now
                self._is_live = True

            # Dispatch into GCS Message Queue if provided
            if message_queue is not None:
                payload = {
                    "type": "camera_frame",
                    "camera_id": "rgb",
                    "stream": "rgb",
                    "frame_base64": b64,
                    "data": b64,
                    "timestamp": now,
                    "fps": float(self.fps),
                    "status": "LIVE"
                }
                self._dispatch_to_queue(message_queue, payload, loop)

            # Throttle to target FPS
            elapsed = time.time() - start_t
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Override in subclass to add inference/annotations."""
        return frame

    def _dispatch_to_queue(self, message_queue, payload: Dict[str, Any], loop=None):
        """Pushes frame into priority message queue without blocking."""
        try:
            if hasattr(message_queue, "video_rgb_queue"):
                target_q = message_queue.video_rgb_queue
                if target_q.full():
                    try:
                        _ = target_q.get_nowait()
                    except Exception:
                        pass
                target_q.put_nowait(payload)
            elif hasattr(message_queue, "put"):
                message_queue.put(payload)
        except Exception:
            pass

    def get_latest_frame_b64(self) -> Optional[str]:
        """Returns the most recent encoded frame for GCS server rendering."""
        with self._lock:
            if self._is_live and (time.time() - self._last_frame_time < 3.0):
                return self._latest_b64
            return None

    @property
    def is_live(self) -> bool:
        with self._lock:
            return self._is_live and (time.time() - self._last_frame_time < 3.0)

    def stop(self):
        """Stops capture and releases webcam device."""
        self.running = False
        with self._lock:
            self._is_live = False
        if self.cap is not None:
            try:
                self.cap.release()
                print("[WebcamProvider] Webcam device released.")
            except Exception:
                pass
            self.cap = None


class WebcamWithDetection(WebcamProvider):
    """
    Webcam streamer with real-time YOLOv8 object & survivor detection inference.
    Automatically draws bounding boxes, class labels, and confidence on video feed.
    """

    def __init__(
        self,
        camera_index: Union[int, str] = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 25,
        model_path: Optional[str] = None,
        conf: float = 0.45
    ):
        super().__init__(camera_index=camera_index, width=width, height=height, fps=fps)
        self.conf = conf
        self.model = None
        self.model_path = model_path
        self._load_model(model_path)

    def _resolve_model_path(self, model_path: Optional[str]) -> Optional[str]:
        """Resolves model weights path across project hierarchy."""
        gcs_root = Path(__file__).resolve().parent.parent
        proj_root = gcs_root.parent

        candidates = []
        if model_path:
            p = Path(model_path)
            if p.is_absolute() and p.exists():
                return str(p)
            candidates.extend([
                Path(model_path),
                gcs_root / model_path,
                proj_root / model_path,
            ])

        # Standard fallback locations in repository
        candidates.extend([
            proj_root / "src" / "backend" / "detection_node" / "human_dataset" / "runs" / "detect" / "train7" / "weights" / "best.pt",
            proj_root / "yolov8n.pt",
            proj_root / "src" / "backend" / "detection_node" / "human_dataset" / "yolov8n.pt",
        ])

        for cand in candidates:
            if cand.exists():
                return str(cand.resolve())
        return None

    def _load_model(self, model_path: Optional[str]):
        if not ULTRALYTICS_AVAILABLE:
            print("⚠️ [WebcamWithDetection] 'ultralytics' package not available. Running without detection.")
            return

        resolved = self._resolve_model_path(model_path)
        if not resolved:
            print(f"⚠️ [WebcamWithDetection] Model not found at '{model_path}'. Running without detection.")
            return

        try:
            self.model = YOLO(resolved)
            self.model_path = resolved
            print(f"✅ [WebcamWithDetection] YOLOv8 model loaded: {resolved}")
        except Exception as e:
            print(f"⚠️ [WebcamWithDetection] Error loading YOLO model ({e}). Running without detection.")
            self.model = None

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Applies YOLOv8 inference and annotates bounding boxes on the frame."""
        if self.model is None:
            return frame

        try:
            results = self.model(frame, conf=self.conf, verbose=False)
            if results and len(results) > 0:
                annotated = results[0].plot()

                # Add watermark banner indicating live AI detection
                cv2.putText(
                    annotated,
                    "YOLOv8 LIVE DETECTION",
                    (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 230, 118),
                    2,
                )
                return annotated
            return frame
        except Exception as e:
            return frame
