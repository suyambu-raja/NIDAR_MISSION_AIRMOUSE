"""
Automated Unit Test Suite for ROS 2 GCS Integration.
Tests all bridge components (ros2_bridge, video_bridge, command_bridge,
and ros2_integration_manager) using lightweight mock ROS 2 messages.
Runs completely standalone without requiring a live ROS 2 daemon or physical hardware.
"""
import base64
import json
import math
import sys
from pathlib import Path
import pytest
import numpy as np
import cv2

# Ensure GCS root is on sys.path
GCS_ROOT = Path(__file__).resolve().parent.parent
if str(GCS_ROOT) not in sys.path:
    sys.path.insert(0, str(GCS_ROOT))

from communication.ros2_bridge import ROS2BridgeNode, quaternion_to_euler
from communication.video_bridge import VideoBridgeNode, ros_image_to_cv2
from communication.command_bridge import CommandBridgeNode
from communication.ros2_integration_manager import ROS2IntegrationManager
from mapping.grid_manager import GridManager


# =============================================================================
# Mock ROS 2 Message Classes
# =============================================================================

class MockPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class MockPose:
    def __init__(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        self.position = MockPoint(x, y, z)
        self.orientation = MockQuaternion(qx, qy, qz, qw)


class MockPoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        self.pose = MockPose(x, y, z, qx, qy, qz, qw)


class MockMapInfo:
    def __init__(self, resolution=0.05, width=100, height=100, origin_x=0.0, origin_y=0.0):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = MockPose(origin_x, origin_y, 0.0)


class MockOccupancyGrid:
    def __init__(self, resolution=0.05, width=20, height=20, origin_x=0.0, origin_y=0.0):
        self.info = MockMapInfo(resolution, width, height, origin_x, origin_y)
        # Create small test grid with some free (0) and some occupied (100) cells
        self.data = [0] * (width * height)
        # Place a wall at row 10, col 5
        self.data[10 * width + 5] = 100


class MockSurvivorDetection:
    def __init__(self, survivor_id=1, x=5.2, y=3.4, confidence=0.96, detection_source="thermal", grid_box="C2"):
        self.survivor_id = survivor_id
        self.position = MockPoint(x, y, 0.0)
        self.confidence = confidence
        self.detection_source = detection_source
        self.grid_box = grid_box
        self.is_confirmed = True


class MockSurvivorArray:
    def __init__(self, detections=None):
        self.detections = detections or []


class MockBatteryState:
    def __init__(self, voltage=16.4, percentage=0.85):
        self.voltage = voltage
        self.percentage = percentage


class MockString:
    def __init__(self, data=""):
        self.data = data


class MockPath:
    def __init__(self, waypoints=None):
        waypoints = waypoints or [[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]]
        self.poses = [MockPoseStamped(x, y, 0.0) for x, y in waypoints]


class MockImage:
    def __init__(self, height=480, width=640, encoding="bgr8", data=None):
        self.height = height
        self.width = width
        self.encoding = encoding
        if data is None:
            self.data = bytes([128] * (height * width * 3))
        else:
            self.data = data


# =============================================================================
# Unit Tests
# =============================================================================

def test_quaternion_to_euler():
    """Verifies that quaternion conversions yield accurate Euler angles."""
    # Identity quaternion -> 0 heading
    r, p, y = quaternion_to_euler(0, 0, 0, 1)
    assert math.isclose(y, 0.0, abs_tol=1e-4)

    # 90 deg yaw around Z axis: qz = sin(pi/4) = 0.7071, qw = cos(pi/4) = 0.7071
    r, p, y = quaternion_to_euler(0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    deg = math.degrees(y)
    assert math.isclose(deg, 90.0, abs_tol=1e-2)


def test_drone_pose_conversion():
    """Verifies that /drone_pose converts to telemetry JSON matching dashboard requirements."""
    grid_mgr = GridManager(grid_width_meters=20, grid_height_meters=20, cell_size_meters=2.5)
    bridge = ROS2BridgeNode(grid_mgr=grid_mgr)

    # Mock drone at x=3.5, y=5.0, yaw=45 degrees
    qz = math.sin(math.radians(22.5))
    qw = math.cos(math.radians(22.5))
    msg = MockPoseStamped(x=3.5, y=5.0, z=1.25, qx=0, qy=0, qz=qz, qw=qw)

    telem = bridge.convert_drone_pose(msg)

    assert telem["type"] == "telemetry"
    assert telem["x"] == 3.5
    assert telem["y"] == 5.0
    assert telem["z"] == 1.25
    assert telem["altitude"] == 1.25
    assert math.isclose(telem["heading"], 45.0, abs_tol=0.5)
    assert telem["grid_cell"] == "B3"
    assert telem["flight_mode"] == "GUIDED"


def test_map_conversion():
    """Verifies that /map translates to map_update JSON with coordinates."""
    bridge = ROS2BridgeNode()
    msg = MockOccupancyGrid(resolution=0.05, width=20, height=20, origin_x=0.0, origin_y=0.0)

    map_payload = bridge.convert_map(msg)

    assert map_payload["type"] == "map_update"
    assert map_payload["resolution"] == 0.05
    assert map_payload["width"] == 20
    assert map_payload["height"] == 20
    assert map_payload["origin_x"] == 0.0
    assert map_payload["origin_y"] == 0.0
    assert len(map_payload["data"]) == 400
    assert map_payload["slam_mode"] == "REALTIME"

    # Verify occupied cell was extracted at col=5, row=10 -> x = 5*0.05=0.25, y = 10*0.05=0.50
    assert [0.25, 0.50] in map_payload["occupied_cells"]


def test_confirmed_survivors_conversion():
    """Verifies that /confirmed_survivors translates to survivor_detected JSON."""
    grid_mgr = GridManager(grid_width_meters=20, grid_height_meters=20, cell_size_meters=2.5)
    bridge = ROS2BridgeNode(grid_mgr=grid_mgr)

    det1 = MockSurvivorDetection(survivor_id=1, x=6.2, y=3.8, confidence=0.95, detection_source="thermal", grid_box="C2")
    det2 = MockSurvivorDetection(survivor_id=2, x=11.5, y=14.0, confidence=0.91, detection_source="fused", grid_box="E6")
    msg = MockSurvivorArray(detections=[det1, det2])

    surv_payloads = bridge.convert_confirmed_survivors(msg)

    assert len(surv_payloads) == 2
    s1 = surv_payloads[0]
    assert s1["type"] == "survivor_detected"
    assert s1["id"] == "S01"
    assert s1["x"] == 6.2
    assert s1["y"] == 3.8
    assert s1["confidence"] == 0.95
    assert s1["source"] == "THERMAL"
    assert s1["grid_cell"] == "C2"

    s2 = surv_payloads[1]
    assert s2["id"] == "S02"
    assert s2["source"] == "FUSED"


def test_battery_status_conversion():
    """Verifies that /battery_status merges into the bridge cache."""
    bridge = ROS2BridgeNode()
    msg = MockBatteryState(voltage=15.9, percentage=0.74)

    res = bridge.convert_battery(msg)
    assert res["voltage"] == 15.9
    assert res["percentage"] == 74.0
    assert bridge.latest_battery_pct == 74.0
    assert bridge.latest_battery_v == 15.9


def test_exploration_status_conversion():
    """Verifies that /exploration_status JSON string parses to mission_status payload."""
    bridge = ROS2BridgeNode()
    exp_json = json.dumps({
        "state": "exploring",
        "survivors_found": 3,
        "frontiers": 8,
        "coverage_pct": 52.4,
        "strategy": "EXPLORE",
        "battery_pct": 74.0
    })
    msg = MockString(data=exp_json)

    st = bridge.convert_exploration_status(msg)
    assert st["type"] == "mission_status"
    assert st["state"] == "EXPLORING"
    assert st["survivor_count"] == 3
    assert st["progress"] == 0.52
    assert st["slam_mode"] == "REALTIME"


def test_planned_path_conversion():
    """Verifies that /planned_path extracts waypoints correctly."""
    bridge = ROS2BridgeNode()
    msg = MockPath([[1.5, 2.5], [3.0, 4.0], [5.5, 6.0]])

    pts = bridge.convert_planned_path(msg)
    assert len(pts) == 3
    assert pts[0] == [1.5, 2.5]
    assert pts[2] == [5.5, 6.0]
    assert bridge.planned_path == pts


def test_fusion_status_conversion():
    """Verifies that /fusion_status translates to event_log JSON."""
    bridge = ROS2BridgeNode()
    data_str = json.dumps({
        "visibility_state": "NORMAL",
        "active_detector": "Both",
        "brightness": 128.5
    })
    msg = MockString(data=data_str)

    evt = bridge.convert_fusion_status(msg)
    assert evt["type"] == "event_log"
    assert evt["category"] == "FUSION"
    assert evt["level"] == "INFO"
    assert "Brightness: 128.5" in evt["message"]


def test_video_bridge_rgb_and_thermal():
    """Verifies that VideoBridgeNode processes and encodes image frames."""
    bridge = VideoBridgeNode()

    # 1. Test RGB frame encoding
    dummy_bgr = np.zeros((480, 640, 3), dtype=np.uint8)
    dummy_bgr[:] = [255, 100, 50]  # Blue-ish orange
    b64_rgb = bridge.process_rgb_image(dummy_bgr)
    assert b64_rgb is not None
    # Verify valid base64
    raw_jpg = base64.b64decode(b64_rgb)
    assert len(raw_jpg) > 100
    assert raw_jpg[:2] == b"\xff\xd8"  # JPEG SOI marker

    # 2. Test Thermal frame encoding
    dummy_thermal = np.full((240, 320), 180, dtype=np.uint8)
    b64_thermal = bridge.process_thermal_image(dummy_thermal)
    assert b64_thermal is not None
    raw_thermal_jpg = base64.b64decode(b64_thermal)
    assert len(raw_thermal_jpg) > 100
    assert raw_thermal_jpg[:2] == b"\xff\xd8"


def test_command_bridge_translations():
    """Verifies that CommandBridgeNode translates operator UI actions."""
    bridge = CommandBridgeNode()

    # 1. START_MISSION
    res_start = bridge.translate_command("START_MISSION")
    assert res_start["action"] == "START_MISSION"
    assert res_start["arming"] is True
    assert res_start["flight_mode"] == "GUIDED"
    assert res_start["topics"]["/mavros/cmd/arming"] is True

    # 2. ABORT_MISSION
    res_abort = bridge.translate_command("ABORT_MISSION", {"reason": "Low Battery"})
    assert res_abort["action"] == "ABORT_MISSION"
    assert res_abort["abort"] is True
    assert res_abort["flight_mode"] == "LOITER"
    assert res_abort["reason"] == "Low Battery"

    # 3. CHANGE_SLAM_MODE
    res_slam = bridge.translate_command("CHANGE_SLAM_MODE", {"mode": "REALTIME"})
    assert res_slam["action"] == "CHANGE_SLAM_MODE"
    assert res_slam["slam_mode"] == "REALTIME"


def test_graceful_fallback_without_ros2():
    """Verifies that ROS2IntegrationManager falls back gracefully without ROS 2."""
    class MockQueue:
        pass

    mgr = ROS2IntegrationManager(MockQueue())
    # Calling start should not crash even if rclpy is missing
    mgr.start()
    # Stopping should also complete safely
    mgr.stop()
    assert True


if __name__ == "__main__":
    test_funcs = [
        test_quaternion_to_euler,
        test_drone_pose_conversion,
        test_map_conversion,
        test_confirmed_survivors_conversion,
        test_battery_status_conversion,
        test_exploration_status_conversion,
        test_planned_path_conversion,
        test_fusion_status_conversion,
        test_video_bridge_rgb_and_thermal,
        test_command_bridge_translations,
        test_graceful_fallback_without_ros2,
    ]
    passed = 0
    print(f"Running {len(test_funcs)} unit tests...")
    for fn in test_funcs:
        try:
            fn()
            print(f"  [PASS] {fn.__name__}")
            passed += 1
        except Exception as err:
            print(f"  [FAIL] {fn.__name__}: {err}")
            raise
    print(f"\nResult: {passed}/{len(test_funcs)} tests PASSED!")
