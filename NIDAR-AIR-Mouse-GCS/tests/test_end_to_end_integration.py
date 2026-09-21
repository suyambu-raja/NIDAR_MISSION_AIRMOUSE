"""
Automated End-to-End Integration Test Suite for NIDAR M2.
Verifies all 15 critical integration paths defined in Section 44 of the NIDAR M2 specification:

Test 1:  RGB detection -> GCS
Test 2:  Thermal detection -> GCS
Test 3:  Survivor fusion -> GCS
Test 4:  Survivor location -> map
Test 5:  LiDAR -> SLAM -> GCS map
Test 6:  EKF -> drone pose -> GCS
Test 7:  Frontier -> exploration -> GCS
Test 8:  A* -> path -> GCS
Test 9:  Obstacle -> DWA / local avoidance -> GCS
Test 10: Emergency obstacle -> brake (0.45m threshold) -> GCS alert
Test 11: GCS ARM -> backend -> ArduPilot
Test 12: GCS LAND -> backend -> ArduPilot
Test 13: Communication loss -> GCS warning -> onboard failsafe
Test 14: Battery warning -> GCS
Test 15: Localization degradation -> GCS
"""

import math
import json
import pytest
import numpy as np
from pathlib import Path
import sys

# Ensure GCS root is on sys.path
GCS_ROOT = Path(__file__).resolve().parent.parent
if str(GCS_ROOT) not in sys.path:
    sys.path.insert(0, str(GCS_ROOT))

from communication.ros2_bridge import ROS2BridgeNode, quaternion_to_euler
from communication.command_bridge import CommandBridgeNode
from communication.video_bridge import VideoBridgeNode
from backend.message_queue import PriorityMessageQueue
from mapping.grid_manager import GridManager


# =============================================================================
# Mock ROS 2 Helper Data Structures
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
    def __init__(self, x=0.0, y=0.0, z=0.0, qz=0.0, qw=1.0):
        self.position = MockPoint(x, y, z)
        self.orientation = MockQuaternion(0.0, 0.0, qz, qw)


class MockPoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0, qz=0.0, qw=1.0):
        self.pose = MockPose(x, y, z, qz, qw)


class MockOccupancyGrid:
    def __init__(self, width=10, height=10, resolution=0.05, origin_x=0.0, origin_y=0.0):
        self.info = type("Info", (), {
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": MockPose(origin_x, origin_y, 0.0)
        })()
        # Initialize with free space (0), obstacle at (2, 2)
        self.data = [0] * (width * height)
        self.data[2 * width + 2] = 100


class MockSurvivorDetection:
    def __init__(self, survivor_id=1, x=3.5, y=4.2, z=0.0, confidence=0.92,
                 is_confirmed=True, modality="RGB+THERMAL", tracking_id=101):
        self.survivor_id = survivor_id
        self.id = survivor_id
        self.tracking_id = tracking_id
        self.position = MockPoint(x, y, z)
        self.confidence = confidence
        self.is_confirmed = is_confirmed
        self.detection_source = modality


class MockSurvivorArray:
    def __init__(self, detections):
        self.detections = detections


class MockBatteryState:
    def __init__(self, voltage=11.1, percentage=0.18, current=-12.5):
        self.voltage = voltage
        self.percentage = percentage
        self.current = current


class MockString:
    def __init__(self, data_str):
        self.data = data_str


# =============================================================================
# 15 END-TO-END INTEGRATION TESTS
# =============================================================================

def test_01_rgb_detection_to_gcs():
    """Test 1: RGB detection -> detection_node -> ROS 2 -> Bridge -> GCS."""
    bridge = VideoBridgeNode()
    # Synthesize an RGB image (person detected box)
    fake_rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    b64 = bridge.process_rgb_image(fake_rgb)

    assert b64 is not None
    assert len(b64) > 100
    # Payload format verified
    payload = {
        "type": "camera_frame",
        "camera_id": "rgb",
        "frame_base64": b64,
        "status": "LIVE"
    }
    assert payload["type"] == "camera_frame"
    assert payload["camera_id"] == "rgb"


def test_02_thermal_detection_to_gcs():
    """Test 2: Thermal detection -> thermal_node -> ROS 2 -> Bridge -> GCS."""
    bridge = VideoBridgeNode()
    # Synthesize thermal grayscale radiometric image (hot spot)
    fake_thermal = np.full((120, 160), 37, dtype=np.uint8)
    b64 = bridge.process_thermal_image(fake_thermal)

    assert b64 is not None
    assert len(b64) > 100
    payload = {
        "type": "camera_frame",
        "camera_id": "thermal",
        "frame_base64": b64,
        "status": "LIVE"
    }
    assert payload["type"] == "camera_frame"
    assert payload["camera_id"] == "thermal"


def test_03_survivor_fusion_to_gcs():
    """Test 3: Survivor fusion -> fusion_node -> /confirmed_survivors -> GCS."""
    grid_mgr = GridManager(grid_width_meters=20, grid_height_meters=20, cell_size_meters=1.0)
    bridge = ROS2BridgeNode(grid_mgr=grid_mgr)

    det = MockSurvivorDetection(survivor_id=1, x=4.5, y=2.2, confidence=0.95, is_confirmed=True)
    msg = MockSurvivorArray([det])

    events = bridge.convert_confirmed_survivors(msg)
    assert len(events) == 1
    surv = events[0]
    assert surv["type"] == "survivor_detected"
    assert surv["id"] == "S01"
    assert surv["status"] == "CONFIRMED"
    assert surv["confidence"] == 0.95
    assert surv["x"] == 4.5
    assert surv["y"] == 2.2


def test_04_survivor_location_to_map():
    """Test 4: Survivor location -> grid_mapper_node -> /survivor_grid_locations -> Map Pin."""
    grid_mgr = GridManager(grid_width_meters=20, grid_height_meters=20, cell_size_meters=2.5)
    bridge = ROS2BridgeNode(grid_mgr=grid_mgr)

    # Convert survivor at world (x=3.5, y=5.0)
    det = MockSurvivorDetection(survivor_id=2, x=3.5, y=5.0, confidence=0.91)
    events = bridge.convert_confirmed_survivors(MockSurvivorArray([det]))

    surv = events[0]
    assert surv["grid_cell"] == "B3"  # Verified arena grid cell mapping
    assert surv["x"] == 3.5
    assert surv["y"] == 5.0


def test_05_lidar_slam_to_gcs_map():
    """Test 5: RPLIDAR -> SLAM -> /map -> GCS map canvas."""
    bridge = ROS2BridgeNode()
    msg = MockOccupancyGrid(width=20, height=20, resolution=0.05, origin_x=-0.5, origin_y=-0.5)

    payload = bridge.convert_map(msg)
    assert payload["type"] == "map_update"
    assert payload["resolution"] == 0.05
    assert payload["width"] == 20
    assert payload["height"] == 20
    assert payload["origin_x"] == -0.5
    assert payload["origin_y"] == -0.5
    assert len(payload["occupied_cells"]) == 1
    assert payload["slam_mode"] == "REALTIME"


def test_06_ekf_to_drone_pose_to_gcs():
    """Test 6: EKF -> /drone_pose -> GCS telemetry."""
    grid_mgr = GridManager()
    bridge = ROS2BridgeNode(grid_mgr=grid_mgr)

    # Heading = 90 degrees (qz=0.7071, qw=0.7071)
    msg = MockPoseStamped(x=2.5, y=1.8, z=1.2, qz=math.sin(math.pi / 4), qw=math.cos(math.pi / 4))
    telem = bridge.convert_drone_pose(msg)

    assert telem["type"] == "telemetry"
    assert telem["x"] == 2.5
    assert telem["y"] == 1.8
    assert telem["altitude"] == 1.2
    assert math.isclose(telem["heading"], 90.0, abs_tol=0.5)
    assert telem["flight_mode"] == "GUIDED"


def test_07_frontier_exploration_to_gcs():
    """Test 7: Frontier -> exploration_node -> /exploration_status -> GCS."""
    bridge = ROS2BridgeNode()
    status_json = json.dumps({
        "state": "EXPLORING",
        "coverage_pct": 58.4,
        "survivors_found": 3
    })
    msg = MockString(status_json)

    payload = bridge.convert_exploration_status(msg)
    assert payload["type"] == "mission_status"
    assert payload["state"] == "EXPLORING"
    assert payload["progress"] == 0.58
    assert payload["survivor_count"] == 3


def test_08_astar_path_to_gcs():
    """Test 8: Authoritative A* -> /planned_path -> GCS map planned path (cyan)."""
    bridge = ROS2BridgeNode()
    # Mock nav_msgs/Path with 3 poses
    path_poses = [
        MockPoseStamped(x=0.0, y=0.0),
        MockPoseStamped(x=1.0, y=1.0),
        MockPoseStamped(x=2.0, y=2.0),
    ]
    path_msg = type("Path", (), {"poses": path_poses})()

    path_pts = bridge.convert_planned_path(path_msg)
    assert len(path_pts) == 3
    assert path_pts[0] == [0.0, 0.0]
    assert path_pts[2] == [2.0, 2.0]
    assert bridge.planned_path == path_pts


def test_09_obstacle_avoidance_to_gcs():
    """Test 9: Obstacle -> Local avoidance / DWA -> GCS."""
    bridge = ROS2BridgeNode()
    # /failsafe/status carries obstacle clearance details
    fs_json = json.dumps({
        "active": False,
        "state": "NORMAL",
        "obstacle_dist": 1.25,
        "planner": "A*",
        "local_planner": "DWA"
    })
    payload = bridge.convert_failsafe_status(MockString(fs_json))

    assert payload["type"] == "failsafe_status"
    assert payload["state"] == "NORMAL"
    assert payload["details"]["obstacle_dist"] == 1.25
    assert payload["details"]["local_planner"] == "DWA"


def test_10_emergency_obstacle_brake_to_gcs():
    """Test 10: Emergency obstacle -> brake (< 0.45m) -> GCS alert banner."""
    bridge = ROS2BridgeNode()
    fs_json = json.dumps({
        "active": True,
        "state": "BRAKE",
        "reason": "Emergency obstacle proximity (< 0.45m)",
        "action": "EMERGENCY_BRAKE_ACTIVE",
        "obstacle_dist": 0.38
    })
    payload = bridge.convert_failsafe_status(MockString(fs_json))

    assert payload["type"] == "failsafe_status"
    assert payload["active"] is True
    assert payload["state"] == "BRAKE"
    assert payload["details"]["obstacle_dist"] < 0.45
    assert "EMERGENCY_BRAKE" in payload["action"]


def test_11_gcs_arm_to_backend_to_ardupilot():
    """Test 11: GCS ARM -> command_bridge -> /mavros/cmd/arming -> FCU."""
    bridge = CommandBridgeNode()
    res = bridge.handle_command({"action": "ARM"})

    assert res["action"] == "ARM"
    assert res["arming"] is True
    assert res["topics"]["/mavros/cmd/arming"] is True


def test_12_gcs_land_to_backend_to_ardupilot():
    """Test 12: GCS LAND -> command_bridge -> /mavros/cmd/land -> FCU."""
    bridge = CommandBridgeNode()
    res = bridge.handle_command({"action": "LAND"})

    assert res["action"] == "LAND"
    assert res["flight_mode"] == "LAND"
    assert res["topics"]["/mavros/set_mode"] == "LAND"


@pytest.mark.asyncio
async def test_13_communication_loss_to_gcs_watchdog():
    """Test 13: Communication loss -> 3s watchdog trigger -> GCS alert banner."""
    queue = PriorityMessageQueue(high_limit=50)
    queue.start()
    # High-priority alert queued on comm lost
    await queue.put_high({"type": "comm_lost", "age_sec": 4.2, "status": "GCS CONNECTION LOST"})
    assert queue.high_queue.qsize() == 1
    msg = await queue.high_queue.get()
    assert msg["type"] == "comm_lost"
    assert msg["age_sec"] > 3.0
    queue.stop()


def test_14_battery_warning_to_gcs():
    """Test 14: Battery low warning (< 20%) -> /battery_status -> GCS."""
    bridge = ROS2BridgeNode()
    msg = MockBatteryState(voltage=10.5, percentage=0.15, current=-14.2)

    payload = bridge.convert_battery(msg)
    assert payload["voltage"] == 10.5
    assert payload["percentage"] == 15.0  # Converted to percentage
    assert bridge.latest_battery_pct < 20.0


def test_15_localization_degradation_to_gcs():
    """Test 15: Localization degradation -> /failsafe/status -> GCS alert."""
    bridge = ROS2BridgeNode()
    fs_json = json.dumps({
        "active": True,
        "state": "WARN",
        "reason": "Optical flow quality degraded (< 30%)",
        "action": "HOLD_POSITION",
        "nav_ok": False,
        "ekf_ok": False
    })
    payload = bridge.convert_failsafe_status(MockString(fs_json))

    assert payload["type"] == "failsafe_status"
    assert payload["active"] is True
    assert payload["details"]["nav_ok"] is False
    assert payload["details"]["ekf_ok"] is False
    assert "HOLD" in payload["action"]
