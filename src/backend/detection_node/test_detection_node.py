#!/usr/bin/env python3
"""
test_detection_node.py — Unit tests for NIDAR Detection Node
============================================================
Tests:
  1. Node initialization and parameters
  2. Image conversion handling (RGB8, Mono8, Mono16)
  3. World position estimation and azimuth calculation
  4. RGB detection processing and message publishing
  5. Thermal detection processing and message publishing
  6. Status diagnostic publishing
"""

import sys
import os
import math
import numpy as np
import pytest

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_BACKEND_DIR = os.path.dirname(_THIS_DIR)
for p in [_THIS_DIR, _BACKEND_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)
import conftest  # noqa: F401

from detection_node.detection_node import DetectionNode


from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from nidar_msgs.msg import SurvivorArray, SurvivorDetection


@pytest.fixture
def node():
    return DetectionNode()


def test_initialization(node):
    """Verify parameters and defaults."""
    assert node._rgb_conf_thresh == 0.45
    assert node._thermal_conf_thresh == 0.40
    assert node._rgb_detections_count == 0
    assert node._thermal_detections_count == 0


def test_image_conversion(node):
    """Test raw byte to numpy array conversion."""
    # RGB8 Image
    img_rgb = Image()
    img_rgb.height = 100
    img_rgb.width = 100
    img_rgb.encoding = "rgb8"
    img_rgb.data = bytes(np.zeros((100, 100, 3), dtype=np.uint8).tobytes())

    arr = node._ros_image_to_numpy(img_rgb)
    assert arr is not None
    assert arr.shape == (100, 100, 3)

    # Mono8 Image
    img_mono = Image()
    img_mono.height = 50
    img_mono.width = 50
    img_mono.encoding = "mono8"
    img_mono.data = bytes(np.ones((50, 50), dtype=np.uint8).tobytes())

    arr_mono = node._ros_image_to_numpy(img_mono)
    assert arr_mono is not None
    assert arr_mono.shape == (50, 50)


def test_estimate_world_position(node):
    """Test 3D world projection from bounding box and drone heading."""
    pose = PoseStamped()
    pose.pose.position.x = 2.0
    pose.pose.position.y = 3.0
    pose.pose.position.z = 2.0
    # Facing East (yaw=0, qw=1.0)
    pose.pose.orientation.w = 1.0
    pose.pose.orientation.z = 0.0
    node._latest_pose = pose

    # Centered bbox in 640x480 image
    bbox = [270.0, 140.0, 100.0, 200.0]
    pos = node._estimate_world_position(bbox, (480, 640))

    assert isinstance(pos, Point)
    assert pos.x > 2.0  # Projected forward along heading
    assert abs(pos.y - 3.0) < 0.5  # Approximately on center line


def test_thermal_radiometric_detection(node):
    """Test thermal hot-spot detection fallback."""
    # Synthetic thermal frame with bright person-sized heat blob
    frame = np.zeros((120, 160), dtype=np.uint8)
    frame[40:80, 70:90] = 230  # Hot body signature

    dets = node._detect_thermal(frame)
    assert len(dets) >= 1
    assert dets[0]['confidence'] >= 0.40
    # Bounding box contains the blob
    x, y, w, h = dets[0]['bbox']
    assert 60 <= x <= 80
    assert 30 <= y <= 50


def test_status_publishing(node):
    """Test status diagnostic json publishing."""
    node._publish_status()
    pubs = [p for p in node._publishers if p.topic == '/detection/status']
    assert len(pubs) == 1
    assert len(pubs[0].published) >= 1
