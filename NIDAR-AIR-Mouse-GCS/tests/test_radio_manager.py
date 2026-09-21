"""Unit tests for RadioInterface and SimulatedRadioLink."""
import pytest
from backend.radio_manager import SimulatedRadioLink


@pytest.mark.asyncio
async def test_simulated_radio_connect_disconnect():
    radio = SimulatedRadioLink()
    assert radio.is_connected is False

    # Connect
    await radio.connect()
    assert radio.is_connected is True
    assert radio.rssi_dbm > -70.0
    status = radio.get_status_payload()
    assert status.connected is True
    assert status.signal_quality in {"GOOD", "FAIR"}

    # Update distance (Simulate moving far into arena)
    radio.update_drone_distance(18.0)
    assert radio.rssi_dbm < -65.0
    assert radio.latency_ms > 20.0

    # Disconnect
    await radio.disconnect()
    assert radio.is_connected is False
    status_disc = radio.get_status_payload()
    assert status_disc.signal_quality == "NO SIGNAL"
