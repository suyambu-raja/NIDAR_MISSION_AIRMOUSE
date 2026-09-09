"""Unit tests for GCSServer (HTTP & WebSocket Endpoints)."""
import asyncio
import aiohttp
import pytest
from backend.server import GCSServer


@pytest.mark.asyncio
async def test_server_startup_and_static_route():
    server = GCSServer(host="127.0.0.1", port=8089)
    await server.start_server()

    async with aiohttp.ClientSession() as session:
        # Check HTTP static route
        async with session.get("http://127.0.0.1:8089/") as resp:
            assert resp.status == 200
            html = await resp.text()
            assert "NIDAR AIRMOUSE GCS" in html

        # Check WebSocket connection
        async with session.ws_connect("http://127.0.0.1:8089/ws") as ws:
            # Receive initial state snapshot
            msg = await ws.receive_json(timeout=2.0)
            assert msg is not None
            assert "type" in msg

    await server.stop_server()


@pytest.mark.asyncio
async def test_slam_mode_switching():
    server = GCSServer(host="127.0.0.1", port=8091)
    await server.start_server()
    assert server.slam_mode == "SIMULATION"

    async with aiohttp.ClientSession() as session:
        async with session.ws_connect("http://127.0.0.1:8091/ws") as ws:
            # Send set_slam_mode command to switch to REALTIME
            await ws.send_json({"action": "set_slam_mode", "params": {"mode": "REALTIME"}})
            await asyncio.sleep(0.3)
            assert server.slam_mode == "REALTIME"

            # Switch back to SIMULATION
            await ws.send_json({"action": "set_slam_mode", "params": {"mode": "SIMULATION"}})
            await asyncio.sleep(0.3)
            assert server.slam_mode == "SIMULATION"

    await server.stop_server()

