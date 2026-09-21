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
from pathlib import Path
import sys
import threading
import webbrowser

# Add project root to sys.path
ROOT_DIR = Path(__file__).resolve().parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from backend.server import GCSServer
from config.config_manager import ConfigManager
from communication.ros2_integration_manager import ROS2IntegrationManager


async def run_gcs(host: str = "0.0.0.0", port: int = 8080, auto_open_browser: bool = True):
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

    # Initialize and start ROS 2 Integration Manager
    ros2_manager = ROS2IntegrationManager(server.queue, server=server)
    ros2_manager.start()

    # Video Source Configuration (Webcam / YOLO / RTSP / Simulation)
    video_source = str(cfg_mgr.get("video_source", "simulation")).lower()
    webcam_provider = None

    if video_source in ("webcam", "usb", "rtsp"):
        try:
            from vision.webcam_provider import WebcamWithDetection
            cam_idx = cfg_mgr.get("webcam_index", 0)
            if video_source == "rtsp":
                cam_idx = cfg_mgr.get("rtsp_url", cam_idx)

            webcam = WebcamWithDetection(
                camera_index=cam_idx,
                width=int(cfg_mgr.get("webcam_width", 640)),
                height=int(cfg_mgr.get("webcam_height", 480)),
                model_path=cfg_mgr.get("yolo_model_path", "src/backend/detection_node/human_dataset/best.pt"),
                conf=float(cfg_mgr.get("yolo_confidence", 0.45)),
            )
            server.webcam_provider = webcam
            webcam_thread = threading.Thread(
                target=webcam.start,
                args=(server.queue,),
                daemon=True
            )
            webcam_thread.start()
            webcam_provider = webcam
            print(f"✅ [GCS] Webcam started ({video_source}) — streaming to GCS with YOLOv8 detection")
        except Exception as e:
            print(f"⚠️ [GCS] Could not initialize webcam stream ({e}). Falling back to simulation.")

    display_host = "127.0.0.1" if host == "0.0.0.0" else host
    url = f"http://{display_host}:{current_port}"
    print(f"\n[GCS] Web Dashboard ready: {url}")
    print(f"[GCS] Also accessible at: http://localhost:{current_port}")
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
        if webcam_provider:
            webcam_provider.stop()
        if ros2_manager:
            ros2_manager.stop()
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
