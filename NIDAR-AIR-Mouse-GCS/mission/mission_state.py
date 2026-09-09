"""
Mission States and Transition Rules for NIDAR AirMouse GCS.
Implements the autonomous search mission state machine.
"""
from enum import Enum, auto
from typing import Set, Dict


class MissionState(str, Enum):
    """
    Standard Mission States for the GPS-denied indoor search challenge.
    """
    IDLE = "IDLE"
    SYSTEM_CHECK = "SYSTEM_CHECK"
    READY = "READY"
    TAKEOFF = "TAKEOFF"
    ENTERING = "ENTERING"
    EXPLORING = "EXPLORING"
    SURVIVOR_DETECTED = "SURVIVOR_DETECTED"
    EXITING = "EXITING"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    
    # Failsafe & Emergency States
    ABORTED = "ABORTED"
    FAILSAFE = "FAILSAFE"
    CONNECTION_LOST = "CONNECTION_LOST"


# Allowed state transitions for safe deterministic flow
VALID_TRANSITIONS: Dict[MissionState, Set[MissionState]] = {
    MissionState.IDLE: {
        MissionState.SYSTEM_CHECK,
        MissionState.READY,
        MissionState.ABORTED,
    },
    MissionState.SYSTEM_CHECK: {
        MissionState.READY,
        MissionState.IDLE,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
    },
    MissionState.READY: {
        MissionState.TAKEOFF,
        MissionState.IDLE,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
    },
    MissionState.TAKEOFF: {
        MissionState.ENTERING,
        MissionState.EXPLORING,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
        MissionState.CONNECTION_LOST,
    },
    MissionState.ENTERING: {
        MissionState.EXPLORING,
        MissionState.SURVIVOR_DETECTED,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
        MissionState.CONNECTION_LOST,
    },
    MissionState.EXPLORING: {
        MissionState.SURVIVOR_DETECTED,
        MissionState.EXITING,
        MissionState.MISSION_COMPLETE,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
        MissionState.CONNECTION_LOST,
    },
    MissionState.SURVIVOR_DETECTED: {
        MissionState.EXPLORING,
        MissionState.EXITING,
        MissionState.MISSION_COMPLETE,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
        MissionState.CONNECTION_LOST,
    },
    MissionState.EXITING: {
        MissionState.MISSION_COMPLETE,
        MissionState.FAILSAFE,
        MissionState.ABORTED,
        MissionState.CONNECTION_LOST,
    },
    MissionState.MISSION_COMPLETE: {
        MissionState.IDLE,
        MissionState.READY,
    },
    MissionState.ABORTED: {
        MissionState.IDLE,
        MissionState.READY,
    },
    MissionState.FAILSAFE: {
        MissionState.IDLE,
        MissionState.ABORTED,
    },
    MissionState.CONNECTION_LOST: {
        MissionState.FAILSAFE,
        MissionState.ABORTED,
        MissionState.EXPLORING,  # If connection recovered
        MissionState.IDLE,
    },
}
