"""
Video Providers for NIDAR AirMouse GCS.
Handles incoming video streams (OpenCV USB, RTSP, UDP, or Simulated frames).
"""
import time
import cv2
import numpy as np
from PySide6.QtCore import QObject, Signal, QThread


class BaseVideoProvider(QObject):
    """Abstract base class for camera/video sources."""
    frame_received = Signal(object)  # Emits numpy.ndarray (OpenCV BGR image)
    connection_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_running = False

    def start(self):
        self._is_running = True

    def stop(self):
        self._is_running = False

    def is_running(self) -> bool:
        return self._is_running


class OpenCVVideoProvider(BaseVideoProvider):
    """
    Video capture provider connecting to USB cameras, video files, or RTSP/UDP streams.
    Runs capture in a background QThread to prevent GUI blocking.
    """

    def __init__(self, source=0, fps=30, parent=None):
        super().__init__(parent)
        self.source = source
        self.fps = fps
        self._thread: Optional[QThread] = None
        self._worker: Optional['_VideoWorker'] = None

    def start(self):
        if self._is_running:
            return
        super().start()
        self._worker = _VideoWorker(self.source, self.fps)
        self._worker.frame_ready.connect(self.frame_received.emit)
        self._worker.status_changed.connect(self.connection_changed.emit)
        self._worker.start()

    def stop(self):
        if not self._is_running:
            return
        super().stop()
        if self._worker:
            self._worker.stop()
            self._worker.wait(1000)
            self._worker = None
        self.connection_changed.emit(False)


class _VideoWorker(QThread):
    frame_ready = Signal(object)
    status_changed = Signal(bool)

    def __init__(self, source, fps):
        super().__init__()
        self.source = source
        self.fps = max(1, fps)
        self._active = True

    def run(self):
        # Convert numeric string to int for webcam index or normalize IP camera URL
        src = self.source
        if isinstance(src, str):
            if src.isdigit():
                src = int(src)
            elif src.startswith("http://") and not src.endswith(("/video", "/videofeed", ".mjpg", ".mjpeg")):
                src = src.rstrip("/") + "/video"

        while self._active:
            cap = cv2.VideoCapture(src)
            if not cap.isOpened():
                self.status_changed.emit(False)
                time.sleep(2.0)
                continue

            self.status_changed.emit(True)
            delay = 1.0 / self.fps

            while self._active:
                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.05)
                    break

                self.frame_ready.emit(frame)
                time.sleep(delay)

            cap.release()
            self.status_changed.emit(False)
            time.sleep(1.0)

    def stop(self):
        self._active = False
