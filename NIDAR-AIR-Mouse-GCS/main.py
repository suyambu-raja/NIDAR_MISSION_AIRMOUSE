"""
NIDAR AirMouse Ground Control Station (GCS) - Main Application Entry Point.
Autonomous GPS-Denied Indoor Search, Mapping, and Survivor Localisation Challenge.

Architecture:
  - Python Async Backend (aiohttp + WebSockets)
  - HTML5 + CSS3 + JavaScript Web GCS Dashboard
  - GPS-Denied 2D Occupancy Grid SLAM Map
  - Dual Vision Pipelines (OAK-D RGB & FLIR Thermal)
  - RF Radio Link Abstraction & Bounded Queue Management

Usage:
    python main.py
"""
import asyncio
import sys
import webbrowser
from pathlib import Path

# Add project root to sys.path
ROOT_DIR = Path(__file__).resolve().parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from backend.server import GCSServer
from config.config_manager import ConfigManager


async def run_gcs(host: str = "127.0.0.1", port: int = 8080, auto_open_browser: bool = True):
    """Runs the asynchronous GCS Backend and serves the HTML5 Frontend."""
    print("=" * 72)
    print("   NIDAR AIRMOUSE GROUND CONTROL STATION (GCS)")
    print("   Track 1 - Mission 2 (AIR Mouse) Autonomous Indoor Challenge")
    print("=" * 72)

    cfg_mgr = ConfigManager()
    server = None
    current_port = port
    max_attempts = 5

    for attempt in range(max_attempts):
        try:
            server = GCSServer(host=host, port=current_port, config_mgr=cfg_mgr)
            await server.start_server()
            break
        except OSError as err:
            if getattr(err, "errno", None) in (10048, 98) or "10048" in str(err):
                print(f"[GCS] Notice: Port {current_port} is busy. Trying port {current_port + 1}...")
                current_port += 1
                if attempt == max_attempts - 1:
                    print(f"[GCS] Error: Could not find an open port between {port} and {current_port}.")
                    return
            else:
                raise

    url = f"http://{host}:{current_port}"
    print(f"\n[GCS] Web Dashboard ready: {url}")
    print("[GCS] Press Ctrl+C in this terminal to shut down.\n")

    if auto_open_browser:
        # Give server a moment to start before opening browser
        await asyncio.sleep(0.5)
        try:
            webbrowser.open(url)
        except Exception as e:
            print(f"[GCS] Note: Could not auto-open browser ({e}). Please visit {url}")

    # Keep server running until interrupted
    try:
        while True:
            await asyncio.sleep(3600)
    except (asyncio.CancelledError, KeyboardInterrupt):
        print("\n[GCS] Shutting down server gracefully...")
    finally:
        if server:
            await server.stop_server()
        print("[GCS] Server stopped.")


def main():
    """Main sync wrapper."""
    try:
        asyncio.run(run_gcs())
    except KeyboardInterrupt:
        print("\n[GCS] Shutdown completed.")
        sys.exit(0)


if __name__ == "__main__":
    main()
