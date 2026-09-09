"""
Stress test for WebSocket Queue and Buffer Overflow Prevention.
Simulates high telemetry traffic + dual video streaming + rapid reconnects.
"""
import asyncio
import aiohttp
import pytest
from backend.server import GCSServer


@pytest.mark.asyncio
async def test_high_throughput_and_rapid_reconnects():
    server = GCSServer(host="127.0.0.1", port=8091)
    await server.start_server()

    # Rapid connect/disconnect cycles
    for cycle in range(5):
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect("http://127.0.0.1:8091/ws") as ws:
                # Send connect command
                await ws.send_json({"action": "connect"})
                
                # Receive burst of initial packets
                for _ in range(15):
                    msg = await ws.receive_json(timeout=2.0)
                    assert msg is not None

                # Send simulated survivor trigger
                await ws.send_json({"action": "simulate_survivor", "params": {"source": "STRESS_TEST"}})

                # Close abruptly
                await ws.close()

    # Verify server queue is clean and running with 0 active clients
    assert server.queue.client_count == 0
    await server.stop_server()
