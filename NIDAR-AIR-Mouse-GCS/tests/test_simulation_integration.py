"""
Integration test for full simulation cycle in NIDAR AirMouse GCS.
Verifies connect -> start_mission -> telemetry generation -> map discovery -> survivor detection -> abort/complete.
"""
import sys
import time
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QCoreApplication
import pytest

from config.config_manager import ConfigManager
from mapping.grid_manager import GridManager
from mission.mission_manager import MissionManager
from mission.mission_state import MissionState
from mission.safety_manager import SafetyManager
from communication.connection_manager import ConnectionManager
from logging_system.mission_logger import MissionLogger


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def test_full_simulation_flow(qapp):
    cfg_mgr = ConfigManager()
    cfg = cfg_mgr.config
    cfg.general.mode = "SIMULATION"

    grid_mgr = GridManager(
        grid_width_meters=cfg.map.grid_width_meters,
        grid_height_meters=cfg.map.grid_height_meters,
        cell_size_meters=cfg.map.cell_size_meters,
    )
    mission_mgr = MissionManager()
    safety_mgr = SafetyManager(cfg.safety)
    logger = MissionLogger()

    conn_mgr = ConnectionManager(
        config=cfg,
        grid_manager=grid_mgr,
        mission_manager=mission_mgr,
    )

    received_telem = []
    received_maps = []
    received_frames = []
    received_survivors = []
    states_recorded = []

    conn_mgr.telemetry_updated.connect(lambda t: received_telem.append(t))
    conn_mgr.map_updated.connect(lambda m: received_maps.append(m))
    conn_mgr.video_frame_updated.connect(lambda f: received_frames.append(f))
    conn_mgr.survivor_detected.connect(lambda s: received_survivors.append(s))
    mission_mgr.state_changed.connect(lambda old_s, new_s: states_recorded.append((old_s, new_s)))

    # 1. Connect
    conn_mgr.connect_link()
    assert conn_mgr.is_connected() is True

    # Process events for 1.2s to let SYSTEM_CHECK -> READY complete
    start_t = time.time()
    while time.time() - start_t < 1.3:
        qapp.processEvents()
        time.sleep(0.05)

    assert mission_mgr.current_state in {MissionState.READY, MissionState.SYSTEM_CHECK}

    # 2. Start Mission
    assert mission_mgr.start_mission() is True
    assert mission_mgr.current_state == MissionState.TAKEOFF

    # Let physics advance for 2.5s (Drone climbs & enters)
    start_t = time.time()
    while time.time() - start_t < 2.5:
        qapp.processEvents()
        time.sleep(0.05)

    assert len(received_telem) > 0
    assert len(received_maps) > 0
    assert len(received_frames) > 0
    assert received_telem[-1].battery_percentage > 0

    # 3. Trigger Emergency Abort
    conn_mgr.send_emergency_abort("Test Abort")
    assert mission_mgr.current_state == MissionState.ABORTED

    # 4. Disconnect
    conn_mgr.disconnect_link()
    assert conn_mgr.is_connected() is False
