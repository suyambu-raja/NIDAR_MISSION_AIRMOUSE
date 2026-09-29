import asyncio
import json
import pytest
import aiohttp

BASE_URL = "http://127.0.0.1:8081"
WS_URL = "ws://127.0.0.1:8081/ws"

@pytest.mark.asyncio
async def test_static_assets_and_html():
    async with aiohttp.ClientSession() as session:
        # 1. Main Dashboard HTML
        async with session.get(f"{BASE_URL}/") as resp:
            assert resp.status == 200
            html = await resp.text()
            assert "TEAM HANTRAMANAV GCS" in html
            assert "view-data" in html
            assert "view-plan" in html
            assert "view-setup" in html
            assert "view-config" in html
            assert "view-simulation" in html
            assert "view-help" in html
            assert "mission_planner_features.js" in html

        # 2. Key CSS & JS Assets
        assets = [
            "/css/styles.css",
            "/js/ws_client.js",
            "/js/map_renderer.js",
            "/js/telemetry_view.js",
            "/js/camera_view.js",
            "/js/survivor_manager.js",
            "/js/radio_view.js",
            "/js/event_log.js",
            "/js/controls.js",
            "/js/mission_planner_features.js",
            "/js/app.js",
        ]
        for asset in assets:
            async with session.get(f"{BASE_URL}{asset}") as resp:
                assert resp.status == 200, f"Asset failed: {asset}"
                content = await resp.text()
                assert len(content) > 100, f"Asset empty: {asset}"

@pytest.mark.asyncio
async def test_full_mission_planner_websocket_workflow():
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(WS_URL) as ws:
            received_messages = []
            stop_reader = False

            async def reader():
                while not stop_reader:
                    try:
                        msg = await ws.receive_json(timeout=0.5)
                        received_messages.append(msg)
                    except Exception:
                        pass

            reader_task = asyncio.create_task(reader())

            try:
                # 1. Verify telemetry stream
                await asyncio.sleep(0.5)
                telem_msgs = [m for m in received_messages if m.get("type") == "telemetry"]
                assert len(telem_msgs) > 0, "Failed to receive telemetry"
                latest_telem = telem_msgs[-1]
                assert "altitude" in latest_telem
                assert "flight_mode" in latest_telem

                # 2. Arm & Takeoff
                await ws.send_json({"action": "arm", "params": {"action": "ARM"}})
                await asyncio.sleep(0.3)
                await ws.send_json({"action": "takeoff", "params": {"altitude": 2.5}})
                await asyncio.sleep(0.4)

                # 3. Mode LOITER
                await ws.send_json({"action": "set_mode", "params": {"mode": "LOITER"}})
                await asyncio.sleep(0.3)

                # 4. Write & Read Waypoints
                test_wps = [
                    {"id": 1, "command": "TAKEOFF", "lat": 4.2501, "lon": 5.8998, "alt": 2.5},
                    {"id": 2, "command": "WAYPOINT", "lat": 4.2505, "lon": 5.9002, "alt": 2.5},
                    {"id": 3, "command": "RETURN_TO_LAUNCH", "lat": 4.2501, "lon": 5.8998, "alt": 2.5}
                ]
                await ws.send_json({"action": "write_waypoints", "params": {"waypoints": test_wps}})
                await asyncio.sleep(0.3)
                await ws.send_json({"action": "read_waypoints", "params": {}})
                await asyncio.sleep(0.6)

                wp_msgs = [m for m in received_messages if m.get("type") == "waypoints"]
                assert len(wp_msgs) > 0, "Failed to read waypoints back from FCU"
                assert len(wp_msgs[-1].get("waypoints", [])) == 3

                # 5. Parameters Get & Set
                await ws.send_json({"action": "get_parameters", "params": {}})
                await asyncio.sleep(0.4)
                param_msgs = [m for m in received_messages if m.get("type") == "parameters"]
                assert len(param_msgs) > 0, "Failed to receive onboard parameters"
                params = param_msgs[-1].get("parameters", {})
                assert "EK3_SRC1_POSXY" in params
                assert "WPNAV_SPEED" in params

                await ws.send_json({"action": "set_parameters", "params": {"parameters": {"WPNAV_SPEED": 85}}})
                await asyncio.sleep(0.2)

                # 6. Sensor Calibrations (IMU, Compass, Radio)
                await ws.send_json({"action": "calibrate_imu", "params": {}})
                await ws.send_json({"action": "calibrate_compass", "params": {}})
                await ws.send_json({"action": "calibrate_radio", "params": {}})
                await asyncio.sleep(0.4)

                # 7. Payload, Speed, Return Alt & Flight Modes
                await ws.send_json({"action": "drop_payload", "params": {"pin": 9, "pwm": 1900}})
                await ws.send_json({"action": "set_speed", "params": {"speed": 1.2}})
                await ws.send_json({"action": "set_return_alt", "params": {"alt": 3.0}})
                await ws.send_json({"action": "save_flight_modes", "params": {"modes": ["STABILIZE", "ALTHOLD", "LOITER", "AUTO", "GUIDED", "RTL"]}})
                await asyncio.sleep(0.3)

                # 8. Pause, Resume, RTL, Land, Disarm
                await ws.send_json({"action": "pause_autonomy", "params": {}})
                await asyncio.sleep(0.2)
                await ws.send_json({"action": "resume_autonomy", "params": {}})
                await asyncio.sleep(0.2)
                await ws.send_json({"action": "rtl", "params": {}})
                await ws.send_json({"action": "land", "params": {}})
                await ws.send_json({"action": "disarm", "params": {}})
                await asyncio.sleep(0.3)

                events = [m.get("message", "") for m in received_messages if m.get("type") == "event_log"]
                assert any("Motors ARMED" in ev for ev in events)
                assert any("LOITER" in ev for ev in events)
                assert any("Waypoints to FCU" in ev for ev in events)
                assert any("IMU" in ev for ev in events)
                assert any("Compass" in ev for ev in events)
                assert any("Radio" in ev for ev in events)

            finally:
                stop_reader = True
                reader_task.cancel()
