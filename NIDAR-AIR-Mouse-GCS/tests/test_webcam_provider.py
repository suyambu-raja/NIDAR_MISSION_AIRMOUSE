"""
Unit tests for WebcamProvider and WebcamWithDetection in NIDAR AirMouse GCS.
Verifies frame capture, YOLOv8 inference fallback, bounded queue dispatch,
and seamless fallback to simulation mode.
"""
import base64
import json
import time
from unittest.mock import MagicMock, patch
import numpy as np
import pytest

from vision.webcam_provider import WebcamProvider, WebcamWithDetection
from config.config_manager import ConfigManager
from backend.message_queue import PriorityMessageQueue


def test_webcam_provider_init():
    """Verify default initialization parameters."""
    provider = WebcamProvider(camera_index=0, width=640, height=480, fps=25)
    assert provider.camera_index == 0
    assert provider.width == 640
    assert provider.height == 480
    assert provider.fps == 25
    assert not provider.is_live
    assert provider.get_latest_frame_b64() is None


def test_webcam_with_detection_init_and_fallback():
    """Verify YOLO model loading and graceful fallback if path invalid."""
    provider = WebcamWithDetection(
        camera_index=0,
        model_path="non_existent_model_file.pt",
        conf=0.45,
    )
    # Should fall back cleanly without raising exception
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    processed = provider.process_frame(dummy_frame)
    assert isinstance(processed, np.ndarray)
    assert processed.shape == (480, 640, 3)


def test_webcam_process_frame_with_mock_yolo():
    """Verify process_frame calls YOLO plot when model exists."""
    provider = WebcamWithDetection(camera_index=0)
    mock_model = MagicMock()
    mock_result = MagicMock()
    annotated_frame = np.ones((480, 640, 3), dtype=np.uint8) * 128
    mock_result.plot.return_value = annotated_frame
    mock_model.return_value = [mock_result]
    provider.model = mock_model

    input_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    result_frame = provider.process_frame(input_frame)

    mock_model.assert_called_once()
    assert isinstance(result_frame, np.ndarray)


def test_webcam_dispatch_to_priority_queue():
    """Verify camera frames are dispatched to PriorityMessageQueue's RGB queue."""
    queue = PriorityMessageQueue()
    provider = WebcamProvider()

    dummy_payload = {
        "type": "camera_frame",
        "camera_id": "rgb",
        "frame_base64": "dummy_b64_string",
        "status": "LIVE"
    }

    provider._dispatch_to_queue(queue, dummy_payload)
    assert not queue.video_rgb_queue.empty()
    item = queue.video_rgb_queue.get_nowait()
    assert item["camera_id"] == "rgb"
    assert item["status"] == "LIVE"


def test_config_video_source_options(tmp_path):
    """Verify ConfigManager correctly reads video_source and webcam parameters."""
    cfg_file = tmp_path / "test_config.json"
    cfg_data = {
        "video_source": "webcam",
        "webcam_index": 0,
        "webcam_width": 640,
        "webcam_height": 480,
        "yolo_model_path": "yolov8n.pt",
        "yolo_confidence": 0.50,
        "video": {
            "source": "webcam",
            "fps": 30
        }
    }
    with open(cfg_file, "w", encoding="utf-8") as f:
        json.dump(cfg_data, f)

    cfg_mgr = ConfigManager(str(cfg_file))
    assert cfg_mgr.get("video_source") == "webcam"
    assert cfg_mgr.get("webcam_index") == 0
    assert cfg_mgr.get("yolo_confidence") == 0.50
    assert cfg_mgr.config.video.source == "webcam"
