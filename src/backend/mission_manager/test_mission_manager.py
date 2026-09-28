"""
test_mission_manager.py - Unit tests for NIDAR Mission Manager Node
Validates 13-state rescue mission FSM, pre-flight self-check, emergency failsafes,
service callbacks, and sensor health publication.
"""

import sys
import time
import json
from pathlib import Path
import pytest

_THIS_DIR = Path(__file__).resolve().parent
_BACKEND_DIR = _THIS_DIR.parent
for p in [str(_THIS_DIR), str(_BACKEND_DIR), str(_BACKEND_DIR.parent)]:
    if p not in sys.path:
        sys.path.insert(0, p)
import conftest  # noqa: F401

from mission_manager.mission_manager_node import MissionManagerNode, RescueMissionState
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point
from nidar_msgs.msg import SurvivorArray, SurvivorDetection, SpaceClassification


class DummyReq:
    pass


class DummyResp:
    def __init__(self):
        self.success = False
        self.message = ""


@pytest.fixture
def manager():
    node = MissionManagerNode()
    yield node
    node.destroy_node()


def test_mission_initial_state(manager):
    """Verify initial state is INIT and tick transitions to SELF_CHECK."""
    assert manager._state == RescueMissionState.INIT
    # Simulate elapsed time > 1.5s
    manager._state_start_time = time.time() - 2.0
    manager._state_machine_loop()
    assert manager._state == RescueMissionState.SELF_CHECK


def test_self_check_pass(manager):
    """Verify SELF_CHECK passes when critical subsystems report nominal."""
    manager._state = RescueMissionState.SELF_CHECK
    manager._state_start_time = time.time() - 2.0
    # Simulate recent pose and battery
    pose = PoseStamped()
    manager._on_drone_pose(pose)
    manager._last_battery_time = time.time()
    manager._battery_pct = 95.0

    manager._state_machine_loop()
    assert manager._state == RescueMissionState.READY


def test_mission_service_start(manager):
    """Verify /mission/start service transitions READY -> TAKEOFF."""
    manager._state = RescueMissionState.READY
    req = DummyReq()
    resp = DummyResp()
    result = manager._handle_start(req, resp)
    assert result.success is True
    assert manager._state == RescueMissionState.TAKEOFF


def test_survivor_detection_transition(manager):
    """Verify receiving detected survivors triggers SURVIVOR_DETECTED in EXPLORE mode."""
    manager._state = RescueMissionState.EXPLORE
    msg = SurvivorArray()
    det = SurvivorDetection()
    det.class_name = "person"
    det.confidence = 0.92
    det.source = "rgb"
    det.x = 320.0
    det.y = 240.0
    msg.detections.append(det)

    manager._on_raw_detections(msg)
    assert manager._state == RescueMissionState.SURVIVOR_DETECTED


def test_failsafe_transition_and_abort(manager):
    """Verify failsafe trigger causes immediate transition to FAILSAFE, then abort service."""
    manager._state = RescueMissionState.EXPLORE
    fs_msg = String()
    fs_msg.data = json.dumps({"triggered": True, "reason": "BATTERY_CRITICAL"})
    manager._on_failsafe_status(fs_msg)
    assert manager._state == RescueMissionState.FAILSAFE

    # Abort service
    req = DummyReq()
    resp = DummyResp()
    result = manager._handle_abort(req, resp)
    assert result.success is True
    assert manager._state == RescueMissionState.ABORTED
