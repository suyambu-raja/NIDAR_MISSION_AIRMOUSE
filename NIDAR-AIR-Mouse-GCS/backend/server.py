"""
Unified Async Backend & WebSocket Server for NIDAR AirMouse GCS.
Serves the HTML5/JS dashboard statically and orchestrates high-throughput real-time telemetry,
dynamic 2D SLAM mapping, dual video streams, RF radio layer, and autonomous survivor detections.
"""
import asyncio
import base64
import json
import os
from pathlib import Path
import threading
import time
from typing import Optional, Set
import aiohttp
from aiohttp import web
import cv2
import numpy as np

from config.config_manager import ConfigManager, GCSConfig
from mapping.grid_manager import GridManager
from mapping.occupancy_grid import OccupancyGrid
from backend.schemas import (
    MessageType,
    TelemetryPayload,
    MapUpdatePayload,
    SurvivorPayload,
    CameraFramePayload,
    MissionStatusPayload,
    RadioStatusPayload,
    SystemHealthPayload,
    EventPayload,
    CommandPayload,
)
from backend.message_queue import PriorityMessageQueue
from backend.radio_manager import RadioInterface, SimulatedRadioLink
from backend.survivor_engine import SurvivorEngine
from simulation.simulated_telemetry import SimulatedTelemetryProvider
from simulation.simulated_map import SimulatedMapProvider
from simulation.simulated_video import DualSimulatedVideoGenerator
from mission.mission_state import MissionState


class LiveCameraReceiver:
    """
    Non-blocking background frame grabber for IP Webcam / RTSP / USB video streams.
    Eliminates buffer lag and manages auto-reconnects smoothly.
    """

    def __init__(self, source: str, target_width: int = 640, target_height: int = 480, quality: int = 65):
        self.source = source
        self.target_width = target_width
        self.target_height = target_height
        self.encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]

        # Normalize IP webcam URL
        if isinstance(self.source, str) and self.source.startswith("http://") and not self.source.endswith(("/video", "/videofeed", ".mjpg", ".mjpeg")):
            self._normalized_source = self.source.rstrip("/") + "/video"
        else:
            self._normalized_source = self.source

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._latest_b64: Optional[str] = None
        self._last_frame_time = 0.0
        self._is_live = False
        self._lock = threading.Lock()

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_latest_frame_b64(self) -> Optional[str]:
        with self._lock:
            if self._is_live and (time.time() - self._last_frame_time < 3.5):
                return self._latest_b64
            return None

    @property
    def is_live(self) -> bool:
        with self._lock:
            return self._is_live and (time.time() - self._last_frame_time < 3.5)

    def _worker(self):
        src = self._normalized_source
        if isinstance(src, str) and src.isdigit():
            src = int(src)

        print(f"[LiveCameraReceiver] Background capture thread started for: {src}")

        while self._running:
            cap = None
            try:
                cap = cv2.VideoCapture(src)
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass

                if not cap.isOpened():
                    with self._lock:
                        self._is_live = False
                    time.sleep(2.0)
                    continue

                print(f"[LiveCameraReceiver] Connected to camera feed: {src}")

                while self._running:
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        with self._lock:
                            self._is_live = False
                        time.sleep(0.05)
                        break

                    # Resize to dashboard resolution
                    if self.target_width and self.target_height:
                        h, w = frame.shape[:2]
                        scale = min(self.target_width / w, self.target_height / h)
                        nw, nh = int(w * scale), int(h * scale)
                        frame_resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
                    else:
                        frame_resized = frame

                    _, buffer = cv2.imencode('.jpg', frame_resized, self.encode_params)
                    b64 = base64.b64encode(buffer).decode('utf-8')

                    with self._lock:
                        self._latest_b64 = b64
                        self._last_frame_time = time.time()
                        self._is_live = True

                    time.sleep(0.03)  # ~30 FPS rate limit
            except Exception as e:
                with self._lock:
                    self._is_live = False
                time.sleep(2.0)
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass


class GCSServer:
    """
    Core GCS Web & WebSocket Server.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 8080, config_mgr: Optional[ConfigManager] = None):
        self.host = host
        self.port = port
        self.config_mgr = config_mgr or ConfigManager()
        self.config: GCSConfig = self.config_mgr.config

        # 1. Coordinate Grid & Survivor Engine
        self.grid_mgr = GridManager(
            grid_width_meters=self.config.map.grid_width_meters,
            grid_height_meters=self.config.map.grid_height_meters,
            cell_size_meters=self.config.map.cell_size_meters,
        )
        self.survivor_engine = SurvivorEngine(self.grid_mgr)

        # 2. Priority Bounded Queue (Eliminates Buffer Errors)
        self.queue = PriorityMessageQueue()

        # 3. Radio / RF Link Layer
        self.radio = SimulatedRadioLink()

        # 4. Simulation Providers
        self.sim_telemetry = SimulatedTelemetryProvider()
        self.sim_map = SimulatedMapProvider(
            width_meters=self.config.map.grid_width_meters,
            height_meters=self.config.map.grid_height_meters,
        )
        self.sim_video = DualSimulatedVideoGenerator()

        # 5. Live Camera Stream Receiver (OAK-D / IP Webcam)
        self.webcam_provider = None
        vid_src = self.config.video.source
        if vid_src and vid_src.lower() not in ("simulated", "simulation", "webcam"):
            self.live_camera = LiveCameraReceiver(
                source=vid_src,
                target_width=self.config.video.width,
                target_height=self.config.video.height,
            )
        else:
            self.live_camera = None

        # Ground truth simulated survivors
        self._sim_targets = [
            {"id": "S01", "x": 3.75, "y": 6.25, "conf": 0.95, "source": "THERMAL", "radius": 2.5},
            {"id": "S02", "x": 13.75, "y": 11.25, "conf": 0.92, "source": "RGB", "radius": 2.5},
        ]
        self._detected_targets = set()

        # Mission State
        self.mission_state = MissionState.READY
        self.mission_start_time: Optional[float] = None
        self.elapsed_time_str = "00:00"
        self.is_paused = False
        self.is_connected = True
        self.slam_mode = "SIMULATION"
        self.stored_waypoints: list = []

        # aiohttp App & Tasks
        self.app = web.Application()
        self.runner: Optional[web.AppRunner] = None
        self._tasks: list[asyncio.Task] = []
        self.frontend_dir = Path(__file__).resolve().parent.parent / "frontend"

        self._setup_routes()

    def _setup_routes(self):
        # Explicit index route & static assets
        self.app.router.add_get('/', self._index_handler)
        self.app.router.add_get('/index.html', self._index_handler)
        self.app.router.add_get('/ws', self._websocket_handler)
        self.app.router.add_static('/', path=str(self.frontend_dir), name='static')

    async def _index_handler(self, request):
        return web.FileResponse(self.frontend_dir / "index.html")

    async def start_server(self):
        """Initializes and runs the web server and background simulation loops."""
        self.queue.start()
        if self.live_camera:
            self.live_camera.start()

        # Connect system and initialize simulated sensors & map
        self.is_connected = True
        await self.radio.connect()
        self.sim_telemetry.start()
        self.sim_map.start()
        self.sim_map.update_drone_pose(1.25, 1.25, 0.0)

        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, self.host, self.port)
        await site.start()
        print(f"[GCSServer] Web Dashboard online at http://{self.host}:{self.port}")

        # Start core background async processing loops
        self._tasks.append(asyncio.create_task(self._telemetry_loop()))
        self._tasks.append(asyncio.create_task(self._video_loop()))
        self._tasks.append(asyncio.create_task(self._status_heartbeat_loop()))

    async def stop_server(self):
        """Gracefully tears down server and tasks."""
        for t in self._tasks:
            t.cancel()
        if self.live_camera:
            self.live_camera.stop()
        self.queue.stop()
        if self.runner:
            await self.runner.cleanup()

    async def _websocket_handler(self, request):
        """Manages WebSocket connections and incoming client commands."""
        ws = web.WebSocketResponse(heartbeat=15.0)
        await ws.prepare(request)

        self.queue.register_client(ws)
        print(f"[GCSServer] Client connected. Active clients: {self.queue.client_count}")

        # Send initial full state snapshot
        await self._send_initial_state(ws)

        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        await self._handle_client_command(data)
                    except Exception as e:
                        print(f"[GCSServer] Error parsing client message: {e}")
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    print(f"[GCSServer] WebSocket connection closed with error: {ws.exception()}")
        finally:
            self.queue.unregister_client(ws)
            print(f"[GCSServer] Client disconnected. Active clients: {self.queue.client_count}")

        return ws

    async def _send_initial_state(self, ws: web.WebSocketResponse):
        """Sends current state, survivors, and radio status to a newly connected client."""
        # 1. Mission Status
        st = MissionStatusPayload(
            state=self.mission_state.value,
            elapsed_time=self.elapsed_time_str,
            progress=self._calculate_progress(),
            survivor_count=self.survivor_engine.survivor_count,
            armed=self.sim_telemetry._armed,
            is_simulation=(self.slam_mode == "SIMULATION"),
            slam_mode=self.slam_mode,
        )
        await ws.send_json(st.to_dict())

        # 2. Radio Status
        rf = self.radio.get_status_payload()
        await ws.send_json(rf.to_dict())

        # 3. Existing Survivors
        for surv in self.survivor_engine.get_all_survivors():
            await ws.send_json(surv.to_dict())

    async def _handle_client_command(self, cmd_data: dict):
        """Processes operator commands from the frontend."""
        action = cmd_data.get("action", "")
        params = cmd_data.get("params", {})
        norm_action = str(action).lower().strip()

        if norm_action == "connect":
            await self._connect_system()
        elif norm_action == "disconnect":
            await self._disconnect_system()
        elif norm_action in ("arm", "arm_drone"):
            self.sim_telemetry.arm()
            await self._log_event("SUCCESS", "MOTORS", "Motors ARMED - System in GUIDED/ARMED mode")
            await self._broadcast_status()
        elif norm_action in ("disarm", "disarm_drone"):
            self.sim_telemetry.disarm()
            await self._log_event("WARNING", "MOTORS", "Motors DISARMED - Rotor spin halted")
            await self._broadcast_status()
        elif norm_action == "takeoff":
            alt = float(params.get("altitude", 2.5))
            self.is_connected = True
            self.mission_start_time = self.mission_start_time or time.time()
            await self._set_mission_state(MissionState.TAKEOFF, f"Climbing to target altitude {alt}m")
            self.sim_telemetry.takeoff(alt)
            await self._log_event("SUCCESS", "NAV", f"Autonomous Takeoff initiated to {alt}m")
        elif norm_action == "land":
            self.sim_telemetry.land()
            await self._log_event("WARNING", "NAV", "LAND mode commanded - Vehicle descending to touchdown")
            await self._broadcast_status()
        elif norm_action == "set_mode":
            mode = str(params.get("mode", "GUIDED")).upper()
            self.sim_telemetry.set_mode(mode)
            await self._log_event("INFO", "MODE", f"Flight Mode updated to {mode}")
            await self._broadcast_status()
        elif norm_action == "rtl":
            self.sim_telemetry.rtl()
            await self._log_event("WARNING", "NAV", "RTL (Return To Launch) commanded - returning to home coordinate (1.25, 1.25)")
            await self._broadcast_status()
        elif norm_action in ("pause", "pause_autonomy"):
            self.is_paused = True
            self.sim_telemetry._ground_speed = 0.0
            await self._log_event("WARNING", "MISSION", "Mission and trajectory PAUSED (Holding position)")
            await self._broadcast_status()
        elif norm_action in ("resume", "resume_autonomy"):
            self.is_paused = False
            await self._log_event("SUCCESS", "MISSION", "Mission RESUMED - Continuing autonomous search")
            await self._broadcast_status()
        elif norm_action == "start_mission":
            await self._start_mission()
        elif norm_action in ("abort_mission", "abort"):
            reason = params.get("reason", "Operator Emergency Abort")
            await self._abort_mission(reason)
        elif norm_action == "reset":
            await self._reset_mission()
        elif norm_action == "simulate_survivor":
            await self._inject_manual_survivor(params)
        elif norm_action == "write_waypoints":
            wp_list = params.get("waypoints", [])
            self.stored_waypoints = wp_list
            coords = []
            lat_base = 4.2501500
            lon_base = 5.8998500
            for wp in wp_list:
                x = wp.get("x")
                y = wp.get("y")
                if x is None or y is None:
                    lat = float(wp.get("lat", lat_base))
                    lon = float(wp.get("lon", lon_base))
                    x = round(1.25 + (lon - lon_base) / 0.00008, 2)
                    y = round(1.25 + (lat - lat_base) / 0.00008, 2)
                    x = max(0.5, min(19.5, x))
                    y = max(0.5, min(19.5, y))
                coords.append((float(x), float(y)))
            if coords:
                self.sim_telemetry.set_waypoints(coords)
                path_payload = {
                    "type": "path_update",
                    "path": coords,
                    "frontiers": []
                }
                await self.queue.put_high(path_payload)
            await self._log_event("SUCCESS", "MAVLINK", f"Transmitted {len(wp_list)} Waypoints to FCU EEPROM successfully")
        elif norm_action == "read_waypoints":
            wps_out = getattr(self, "stored_waypoints", [])
            if not wps_out:
                wps_out = [
                    {
                        "id": idx + 1,
                        "command": "WAYPOINT" if idx > 0 else "TAKEOFF",
                        "lat": 4.25015 + (pt[1] - 1.25) * 0.00008,
                        "lon": 5.89985 + (pt[0] - 1.25) * 0.00008,
                        "alt": 2.5,
                        "frame": "Relative",
                        "p1": 0, "p2": 2, "p3": 0, "p4": 0
                    }
                    for idx, pt in enumerate(self.sim_telemetry.waypoints)
                ]
            wp_payload = {
                "type": "waypoints",
                "waypoints": wps_out
            }
            await self.queue.put_high(wp_payload)
            await self._log_event("INFO", "MAVLINK", f"Read {len(wps_out)} active Waypoints from FCU")
        elif norm_action == "calibrate_imu":
            await self._log_event("INFO", "CALIB", "Accel / IMU 6-point calibration started. Level vehicle...")
            await asyncio.sleep(0.4)
            await self._log_event("SUCCESS", "CALIB", "IMU 6-point calibration complete. Offsets: X:+0.02 Y:-0.01 Z:+9.81 m/s²")
        elif norm_action == "calibrate_compass":
            await self._log_event("INFO", "CALIB", "Compass live calibration started. Rotate vehicle around all axes...")
            await asyncio.sleep(0.4)
            await self._log_event("SUCCESS", "CALIB", "Compass calibration SUCCESS. Fitness: 99.4%, Offsets: [12.4, -4.2, 8.1]")
        elif norm_action == "calibrate_radio":
            await self._log_event("INFO", "RADIO", "Calibrating RF link channels and power levels...")
            await asyncio.sleep(0.4)
            await self._log_event("SUCCESS", "RADIO", "Radio Calibration Nominal: 915 MHz, 20 dBm, 57600 baud, 100% link budget")
        elif norm_action == "set_parameters":
            p_dict = params.get("parameters", {})
            await self._log_event("SUCCESS", "PARAMS", f"Updated {len(p_dict)} parameters in FCU Non-Volatile Memory (RAM/EEPROM)")
        elif norm_action == "get_parameters":
            params_payload = {
                "type": "parameters",
                "parameters": {
                    "EK3_SRC1_POSXY": 3,
                    "EK3_SRC1_VELXY": 5,
                    "FS_THR_ENABLE": 1,
                    "FS_BATT_VOLT": 14.8,
                    "WPNAV_SPEED": 70,
                    "RTL_ALT": 250,
                    "PILOT_SPEED_UP": 250
                }
            }
            await self.queue.put_high(params_payload)
            await self._log_event("INFO", "PARAMS", "Parameters refreshed from FCU memory")
        elif norm_action == "set_slam_mode":
            new_mode = str(params.get("mode", "SIMULATION")).upper()
            if new_mode in ["SIMULATION", "REALTIME"]:
                self.slam_mode = new_mode
                is_sim = (new_mode == "SIMULATION")
                msg = f"SLAM Mode switched to {new_mode} ({'Dynamic 2D Raycasting Simulator' if is_sim else 'Live LiDAR / Companion Pi Grid'})"
                await self._log_event("SUCCESS" if is_sim else "WARNING", "MAP", msg)
                await self._broadcast_status()

    async def _broadcast_status(self):
        """Broadcasts immediate mission and arming status snapshot."""
        st = MissionStatusPayload(
            state=self.mission_state.value,
            elapsed_time=self.elapsed_time_str,
            progress=self._calculate_progress(),
            survivor_count=self.survivor_engine.survivor_count,
            armed=self.sim_telemetry._armed,
            is_simulation=(self.slam_mode == "SIMULATION"),
            slam_mode=self.slam_mode,
        )
        await self.queue.put_high(st.to_dict())

    async def _connect_system(self):
        self.is_connected = True
        await self.radio.connect()
        self.sim_telemetry.start()
        self.sim_map.start()

        await self._log_event("SUCCESS", "COMM", "Connected to simulated drone over RF link")
        await self._set_mission_state(MissionState.SYSTEM_CHECK, "Verifying sensor and estimator health")
        
        # Advance to READY after 1 sec check
        await asyncio.sleep(1.0)
        await self._set_mission_state(MissionState.READY, "Pre-flight checks nominal: Ready to launch")

    async def _disconnect_system(self):
        self.is_connected = False
        await self.radio.disconnect()
        self.sim_telemetry.stop()
        self.sim_map.stop()
        await self._set_mission_state(MissionState.IDLE, "Disconnected from drone")
        await self._log_event("WARNING", "COMM", "RADIO LINK LOST: System disconnected")

    async def _start_mission(self):
        self.is_connected = True
        self.sim_telemetry.start()
        self.sim_map.start()
        self.mission_start_time = time.time()
        self.is_paused = False
        await self._set_mission_state(MissionState.TAKEOFF, "Autonomous takeoff initiated")
        await self._log_event("SUCCESS", "MISSION", "Autonomous search and mapping mission started")

    async def _abort_mission(self, reason: str):
        await self._set_mission_state(MissionState.ABORTED, f"EMERGENCY ABORT: {reason}")
        await self._log_event("CRITICAL", "SAFETY", f"EMERGENCY ABORT EXECUTED: {reason}")

    async def _reset_mission(self):
        self.sim_telemetry.reset()
        self.sim_map.reset()
        self.survivor_engine.reset()
        self._detected_targets.clear()
        self.mission_start_time = None
        self.elapsed_time_str = "00:00"
        self.is_paused = False
        self.sim_map.update_drone_pose(1.25, 1.25, 0.0)
        await self._set_mission_state(MissionState.READY, "Mission reset to staging position")
        await self._log_event("INFO", "MISSION", "System reset to initial state")

    async def _inject_manual_survivor(self, params: dict):
        x = float(params.get("x", self.sim_telemetry._x))
        y = float(params.get("y", self.sim_telemetry._y))
        conf = float(params.get("confidence", 0.94))
        source = str(params.get("source", "MANUAL_SIM")).upper()
        payload, is_new = self.survivor_engine.process_detection(x, y, conf, source)
        if is_new:
            await self.queue.put_high(payload.to_dict())
            await self._log_event("SUCCESS", "SURVIVOR", f"Survivor {payload.id} detected at Grid {payload.grid_cell} (Confidence: {int(conf*100)}%)")

    async def _set_mission_state(self, new_state: MissionState, reason: str = ""):
        self.mission_state = new_state
        self.sim_telemetry.set_mission_state(new_state)

        st = MissionStatusPayload(
            state=new_state.value,
            elapsed_time=self.elapsed_time_str,
            progress=self._calculate_progress(),
            survivor_count=self.survivor_engine.survivor_count,
            armed=self.sim_telemetry._armed,
            is_simulation=(self.slam_mode == "SIMULATION"),
            slam_mode=self.slam_mode,
        )
        await self.queue.put_high(st.to_dict())
        if reason:
            await self._log_event("INFO", "MISSION", f"State: {new_state.value} ({reason})")

    def _calculate_progress(self) -> float:
        prog_map = {
            MissionState.IDLE: 0.0,
            MissionState.SYSTEM_CHECK: 0.05,
            MissionState.READY: 0.10,
            MissionState.TAKEOFF: 0.15,
            MissionState.ENTERING: 0.25,
            MissionState.EXPLORING: 0.55,
            MissionState.SURVIVOR_DETECTED: 0.70,
            MissionState.EXITING: 0.90,
            MissionState.MISSION_COMPLETE: 1.0,
            MissionState.ABORTED: 0.0,
        }
        return prog_map.get(self.mission_state, 0.0)

    async def _log_event(self, level: str, category: str, message: str):
        evt = EventPayload(
            timestamp=time.strftime("%H:%M:%S"),
            level=level,
            category=category,
            message=message,
        )
        await self.queue.put_high(evt.to_dict())

    async def _telemetry_loop(self):
        """10 Hz Telemetry & SLAM Map Kinematics update loop."""
        while True:
            try:
                if self.is_connected and not self.is_paused:
                    # Advance simulated physics
                    self.sim_telemetry._step_physics()
                    x, y, yaw = self.sim_telemetry.get_current_pose()
                    alt = self.sim_telemetry._altitude

                    # Update RF distance & path loss
                    dist_to_base = ((x - 1.25)**2 + (y - 1.25)**2)**0.5
                    self.radio.update_drone_distance(dist_to_base)
                    self.radio.record_incoming_packet()

                    # Update SLAM Map
                    self.sim_map.update_drone_pose(x, y, yaw)

                    # Check for survivor proximity
                    target_in_view = None
                    for t in self._sim_targets:
                        dist = ((t["x"] - x)**2 + (t["y"] - y)**2)**0.5
                        if dist <= t["radius"]:
                            target_in_view = t
                            if t["id"] not in self._detected_targets:
                                self._detected_targets.add(t["id"])
                                surv_payload, is_new = self.survivor_engine.process_detection(
                                    t["x"], t["y"], t["conf"], t["source"], survivor_id_hint=t["id"]
                                )
                                if is_new:
                                    await self.queue.put_high(surv_payload.to_dict())
                                    await self._log_event(
                                        "SUCCESS", "SURVIVOR",
                                        f"AUTONOMOUS TARGET {surv_payload.id} LOCALISED at Grid {surv_payload.grid_cell} ({surv_payload.source} CONF: {int(t['conf']*100)}%)"
                                    )
                                    if self.mission_state == MissionState.EXPLORING:
                                        await self._set_mission_state(MissionState.SURVIVOR_DETECTED, f"Target {surv_payload.id} in view")
                                        # Schedule return to exploring
                                        asyncio.create_task(self._resume_exploring_after_delay(2.5))

                    # Update video viewpoint
                    self.sim_video.update_pose(x, y, yaw, alt, target_in_view)

                    # Check state progression
                    if self.mission_state == MissionState.TAKEOFF and alt >= 1.1:
                        await self._set_mission_state(MissionState.ENTERING, "Cruising altitude reached")
                    elif self.mission_state == MissionState.ENTERING and x >= 2.2:
                        await self._set_mission_state(MissionState.EXPLORING, "Inside search arena corridors")
                    elif self.mission_state == MissionState.EXPLORING and self.sim_telemetry.is_at_exit():
                        await self._set_mission_state(MissionState.EXITING, "Approaching arena exit")
                    elif self.mission_state == MissionState.EXITING and alt <= 0.15:
                        await self._set_mission_state(MissionState.MISSION_COMPLETE, "Safely landed in recovery zone")

                    # Build & enqueue Telemetry payload
                    telem = TelemetryPayload(
                        timestamp=time.time(),
                        x=round(x, 2),
                        y=round(y, 2),
                        z=round(alt, 2),
                        altitude=round(alt, 2),
                        heading=round(yaw, 1),
                        velocity=round(self.sim_telemetry._ground_speed, 2),
                        roll=round(self.sim_telemetry._roll, 1),
                        pitch=round(self.sim_telemetry._pitch, 1),
                        yaw=round(yaw, 1),
                        battery_v=round(self.sim_telemetry._battery_v, 2),
                        battery_pct=round(self.sim_telemetry._battery_pct, 1),
                        battery_a=round(self.sim_telemetry._battery_a, 2),
                        flight_mode=self.sim_telemetry._flight_mode,
                        armed=self.sim_telemetry._armed,
                        grid_cell=self.grid_mgr.metric_to_grid(x, y),
                    )
                    await self.queue.put_normal(telem.to_dict())

                    # Build & enqueue Map update
                    # Extract occupied wall points and free explored points
                    grid_arr = self.sim_map.occupancy_grid.grid
                    occ_indices = np.argwhere(grid_arr == OccupancyGrid.OCCUPIED)
                    free_indices = np.argwhere(grid_arr == OccupancyGrid.FREE)

                    # Subsample points for efficient WebSocket transmission
                    res = self.sim_map.resolution
                    occ_points = [[round(c * res, 2), round(r * res, 2)] for r, c in occ_indices[::2]]
                    free_points = [[round(c * res, 2), round(r * res, 2)] for r, c in free_indices[::16]]

                    is_sim = (self.slam_mode == "SIMULATION")
                    map_payload = MapUpdatePayload(
                        timestamp=time.time(),
                        width=self.sim_map.width_cells,
                        height=self.sim_map.height_cells,
                        resolution=self.sim_map.resolution,
                        occupied_cells=occ_points,
                        free_cells=free_points,
                        is_simulation=is_sim,
                        slam_mode=self.slam_mode,
                    )
                    await self.queue.put_normal(map_payload.to_dict())

                await asyncio.sleep(0.1)  # 10 Hz

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[GCSServer] Telemetry loop exception: {e}")
                await asyncio.sleep(0.1)

    async def _resume_exploring_after_delay(self, delay: float):
        await asyncio.sleep(delay)
        if self.mission_state == MissionState.SURVIVOR_DETECTED:
            await self._set_mission_state(MissionState.EXPLORING, "Target logged: Continuing corridor sweep")

    async def _video_loop(self):
        """12 FPS Dual Video Frame rendering and bounded queue dispatch."""
        while True:
            try:
                # Check for live camera stream frame (Webcam / OAK-D / RTSP)
                live_rgb_b64 = None
                if getattr(self, "webcam_provider", None) and self.webcam_provider.is_live:
                    live_rgb_b64 = self.webcam_provider.get_latest_frame_b64()
                elif self.live_camera and self.live_camera.is_live:
                    live_rgb_b64 = self.live_camera.get_latest_frame_b64()
                has_live_feed = live_rgb_b64 is not None

                if self.is_connected or has_live_feed:
                    if has_live_feed:
                        rgb_b64 = live_rgb_b64
                        rgb_status = "LIVE"
                    else:
                        rgb_b64 = self.sim_video.render_rgb_frame()
                        rgb_status = "SIMULATED"

                    thermal_b64 = self.sim_video.render_thermal_frame()

                    rgb_payload = CameraFramePayload(
                        camera_id="rgb",
                        frame_base64=rgb_b64,
                        fps=12.0,
                        status=rgb_status,
                    )
                    thermal_payload = CameraFramePayload(
                        camera_id="thermal",
                        frame_base64=thermal_b64,
                        fps=12.0,
                        status="SIMULATED",
                    )

                    await self.queue.put_video_frame("rgb", rgb_payload.to_dict())
                    await self.queue.put_video_frame("thermal", thermal_payload.to_dict())

                await asyncio.sleep(0.08)  # ~12.5 FPS

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[GCSServer] Video loop error: {e}")
                await asyncio.sleep(0.1)

    async def _status_heartbeat_loop(self):
        """1 Hz Mission clock & RF link quality broadcast."""
        while True:
            try:
                if self.mission_start_time and self.mission_state not in {
                    MissionState.IDLE, MissionState.MISSION_COMPLETE, MissionState.ABORTED
                } and not self.is_paused:
                    elapsed = time.time() - self.mission_start_time
                    mins = int(elapsed // 60)
                    secs = int(elapsed % 60)
                    self.elapsed_time_str = f"{mins:02d}:{secs:02d}"

                # Broadcast Radio Status
                rf = self.radio.get_status_payload()
                await self.queue.put_normal(rf.to_dict())

                # Broadcast System Health
                health = SystemHealthPayload(
                    backend_status="CONNECTED",
                    websocket_status="CONNECTED",
                    simulator_status="RUNNING" if self.is_connected else "IDLE",
                    rgb_camera_status="LIVE" if (self.live_camera and self.live_camera.is_live) else ("SIMULATED" if self.is_connected else "NO_SIGNAL"),
                    thermal_camera_status="SIMULATED" if self.is_connected else "NO_SIGNAL",
                    mapping_status="ACTIVE" if self.is_connected else "IDLE",
                    ekf_status="HEALTHY",
                    optical_flow_status="HEALTHY",
                    lidar_status="HEALTHY",
                    last_telemetry_time=time.time(),
                    last_map_time=time.time(),
                    last_survivor_time=time.time(),
                )
                await self.queue.put_normal(health.to_dict())

                # Broadcast Mission Status update
                st = MissionStatusPayload(
                    state=self.mission_state.value,
                    elapsed_time=self.elapsed_time_str,
                    progress=self._calculate_progress(),
                    survivor_count=self.survivor_engine.survivor_count,
                    armed=self.sim_telemetry._armed,
                    is_simulation=(self.slam_mode == "SIMULATION"),
                    slam_mode=self.slam_mode,
                )
                await self.queue.put_normal(st.to_dict())

                await asyncio.sleep(1.0)

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[GCSServer] Heartbeat error: {e}")
                await asyncio.sleep(1.0)
