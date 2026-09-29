"""
Test suite verifying all TEAM HANTRAMANAV GCS backend command handlers.
Tests arm, disarm, takeoff, land, set_mode, rtl, pause, resume, waypoints,
calibrations, and parameter management over WebSocket.
"""
import asyncio
import json
import pytest
from pathlib import Path
import sys

PROJECT_DIR = Path(__file__).resolve().parent.parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import aiohttp
from backend.server import GCSServer


@pytest.mark.asyncio
async def test_backend_flight_and_mission_commands():
    test_port = 8089
    server = GCSServer(host="127.0.0.1", port=test_port)
    await server.start_server()
    await asyncio.sleep(0.3)

    received_events = []
    received_types = {}

    url = f"http://127.0.0.1:{test_port}/ws"
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(url) as ws:
            async def reader():
                async for msg in ws:
                    if msg.type == aiohttp.WSMsgType.TEXT:
                        data = json.loads(msg.data)
                        t = data.get("type", "")
                        received_types[t] = data
                        if t == "event_log":
                            received_events.append(data.get("message", ""))

            reader_task = asyncio.create_task(reader())
            await asyncio.sleep(0.3)

            # 1. ARM
            await ws.send_json({"action": "arm", "params": {"action": "ARM"}})
            await asyncio.sleep(0.2)
            assert server.sim_telemetry._armed is True

            # 2. TAKEOFF
            await ws.send_json({"action": "takeoff", "params": {"altitude": 2.5}})
            await asyncio.sleep(0.2)
            assert server.sim_telemetry._armed is True
            assert server.sim_telemetry.target_altitude == 2.5

            # 3. SET_MODE
            await ws.send_json({"action": "set_mode", "params": {"mode": "AUTO"}})
            await asyncio.sleep(0.2)
            assert server.sim_telemetry._flight_mode == "AUTO"

            # 4. PAUSE & RESUME
            await ws.send_json({"action": "pause_autonomy", "params": {"action": "PAUSE"}})
            await asyncio.sleep(0.2)
            assert server.is_paused is True

            await ws.send_json({"action": "resume_autonomy", "params": {"action": "RESUME"}})
            await asyncio.sleep(0.2)
            assert server.is_paused is False

            # 5. WRITE & READ WAYPOINTS
            test_wps = [
                {"id": 1, "command": "TAKEOFF", "lat": 4.25015, "lon": 5.89985, "alt": 2.5, "x": 1.25, "y": 1.25},
                {"id": 2, "command": "WAYPOINT", "lat": 4.25045, "lon": 5.90015, "alt": 2.5, "x": 5.0, "y": 5.0},
            ]
            await ws.send_json({"action": "write_waypoints", "params": {"waypoints": test_wps}})
            await asyncio.sleep(0.2)
            assert len(server.sim_telemetry.waypoints) == 2

            await ws.send_json({"action": "read_waypoints", "params": {}})
            await asyncio.sleep(0.2)
            assert "waypoints" in received_types

            # 6. CALIBRATIONS
            await ws.send_json({"action": "calibrate_imu", "params": {}})
            await asyncio.sleep(0.5)
            assert any("IMU 6-point calibration complete" in ev for ev in received_events)

            await ws.send_json({"action": "calibrate_compass", "params": {}})
            await asyncio.sleep(0.5)
            assert any("Compass calibration SUCCESS" in ev for ev in received_events)

            await ws.send_json({"action": "calibrate_radio", "params": {}})
            await asyncio.sleep(0.5)
            assert any("Radio Calibration Nominal" in ev for ev in received_events)

            # 7. PARAMETERS
            await ws.send_json({"action": "get_parameters", "params": {}})
            await asyncio.sleep(0.2)
            assert "parameters" in received_types

            await ws.send_json({"action": "set_parameters", "params": {"parameters": {"WPNAV_SPEED": 80}}})
            await asyncio.sleep(0.2)
            assert any("Updated 1 parameters" in ev for ev in received_events)

            # 8. SIMULATE SURVIVOR
            await ws.send_json({"action": "simulate_survivor", "params": {"x": 8.0, "y": 8.0, "confidence": 0.97}})
            await asyncio.sleep(0.2)
            assert "survivor_detected" in received_types

            # 9. RTL, LAND, DISARM, RESET
            await ws.send_json({"action": "rtl", "params": {"action": "RTL"}})
            await asyncio.sleep(0.2)
            assert server.sim_telemetry._flight_mode == "RTL"

            await ws.send_json({"action": "land", "params": {"action": "LAND"}})
            await asyncio.sleep(0.2)
            assert server.sim_telemetry._flight_mode == "LAND"

            await ws.send_json({"action": "disarm", "params": {"action": "DISARM"}})
            await asyncio.sleep(0.2)
            assert server.sim_telemetry._armed is False

            await ws.send_json({"action": "reset", "params": {}})
            await asyncio.sleep(0.2)
            assert server.mission_state.value == "READY"

            reader_task.cancel()

    await server.stop_server()
