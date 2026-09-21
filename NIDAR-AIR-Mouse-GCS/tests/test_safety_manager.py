"""Unit tests for SafetyManager."""
import sys
from PySide6.QtWidgets import QApplication
import pytest
from mission.safety_manager import SafetyManager
from mission.mission_state import MissionState
from communication.message_models import TelemetryData
from config.config_manager import SafetyConfig


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def test_battery_critical_alert(qapp):
    cfg = SafetyConfig(battery_warning_percent=20.0, battery_critical_percent=10.0)
    safety = SafetyManager(cfg)
    safety.set_mission_state(MissionState.EXPLORING)

    alerts = []
    aborts = []
    safety.safety_alert.connect(lambda lvl, cat, msg: alerts.append((lvl, cat, msg)))
    safety.emergency_abort_requested.connect(lambda reason: aborts.append(reason))

    # Send nominal telemetry
    nominal_telem = TelemetryData(battery_percentage=80.0, battery_voltage_v=16.0)
    safety.update_telemetry(nominal_telem)
    assert len(alerts) == 0
    assert len(aborts) == 0

    # Send low battery telemetry (18%)
    low_telem = TelemetryData(battery_percentage=18.0, battery_voltage_v=14.2)
    safety.update_telemetry(low_telem)
    assert len(alerts) == 1
    assert alerts[0][0] == "WARNING"
    assert len(aborts) == 0

    # Send critical battery telemetry (8%)
    crit_telem = TelemetryData(battery_percentage=8.0, battery_voltage_v=13.2)
    safety.update_telemetry(crit_telem)
    assert len(alerts) == 2
    assert alerts[1][0] == "CRITICAL"
    assert len(aborts) == 1
