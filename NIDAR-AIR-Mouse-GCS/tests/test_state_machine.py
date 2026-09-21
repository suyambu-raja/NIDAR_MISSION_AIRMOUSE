"""Unit tests for MissionState and MissionManager."""
import sys
from PySide6.QtWidgets import QApplication
import pytest
from mission.mission_state import MissionState, VALID_TRANSITIONS
from mission.mission_manager import MissionManager


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def test_valid_state_transitions(qapp):
    mm = MissionManager()
    assert mm.current_state == MissionState.IDLE

    # Valid forward flow
    assert mm.transition_to(MissionState.SYSTEM_CHECK) is True
    assert mm.current_state == MissionState.SYSTEM_CHECK

    assert mm.transition_to(MissionState.READY) is True
    assert mm.current_state == MissionState.READY

    assert mm.transition_to(MissionState.TAKEOFF) is True
    assert mm.current_state == MissionState.TAKEOFF
    assert mm.is_active is True

    assert mm.transition_to(MissionState.ENTERING) is True
    assert mm.transition_to(MissionState.EXPLORING) is True
    assert mm.transition_to(MissionState.SURVIVOR_DETECTED) is True
    assert mm.transition_to(MissionState.EXPLORING) is True
    assert mm.transition_to(MissionState.EXITING) is True
    assert mm.transition_to(MissionState.MISSION_COMPLETE) is True
    assert mm.is_active is False


def test_invalid_state_transition_prevention(qapp):
    mm = MissionManager()
    assert mm.current_state == MissionState.IDLE

    # IDLE -> MISSION_COMPLETE is not valid
    assert mm.transition_to(MissionState.MISSION_COMPLETE) is False
    assert mm.current_state == MissionState.IDLE


def test_emergency_abort_allowed_from_any_state(qapp):
    mm = MissionManager()
    mm.transition_to(MissionState.READY)
    mm.transition_to(MissionState.TAKEOFF)

    # Abort from active flight
    assert mm.abort_mission("Test emergency trigger") is True
    assert mm.current_state == MissionState.ABORTED
