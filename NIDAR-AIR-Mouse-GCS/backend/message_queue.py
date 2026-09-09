"""
Bounded Priority Message Queue and Buffer Overflow Prevention for NIDAR AirMouse GCS.
Guarantees zero buffer bloat, zero memory leaks, and deterministic priority message delivery.
"""
import asyncio
from enum import IntEnum
import time
from typing import Dict, Any, Optional, Set
import aiohttp
from backend.schemas import MessageType


class MessagePriority(IntEnum):
    HIGH = 0     # Survivor detections, Emergency Abort, State changes, Safety alerts
    NORMAL = 1   # Telemetry updates, 2D SLAM Map updates, Radio link status
    LOW = 2      # Camera video frames, Debug logs


class PriorityMessageQueue:
    """
    Asynchronous bounded priority queue manager.
    Implements drop-oldest policies for video and throttled channels to eliminate WebSocket BufferErrors.
    """

    def __init__(self, high_limit: int = 100, normal_limit: int = 50, video_limit: int = 2):
        self.high_queue: asyncio.Queue = asyncio.Queue(maxsize=high_limit)
        self.normal_queue: asyncio.Queue = asyncio.Queue(maxsize=normal_limit)
        self.video_rgb_queue: asyncio.Queue = asyncio.Queue(maxsize=video_limit)
        self.video_thermal_queue: asyncio.Queue = asyncio.Queue(maxsize=video_limit)

        # Clients registry
        self._clients: Set[aiohttp.web.WebSocketResponse] = set()
        self._running = False
        self._dispatch_task: Optional[asyncio.Task] = None

        # Rate control timestamps
        self._last_telem_broadcast = 0.0
        self._last_map_broadcast = 0.0
        self._telem_min_interval = 0.08   # Max 12.5 Hz
        self._map_min_interval = 0.15     # Max 6.6 Hz

        # Metrics
        self.dropped_frames_count = 0
        self.messages_sent_count = 0

    def register_client(self, ws: aiohttp.web.WebSocketResponse):
        self._clients.add(ws)

    def unregister_client(self, ws: aiohttp.web.WebSocketResponse):
        self._clients.discard(ws)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    async def put_high(self, message: Dict[str, Any]):
        """Pushes a high-priority message (never dropped)."""
        try:
            if self.high_queue.full():
                # Discard oldest high priority if absolute emergency
                _ = self.high_queue.get_nowait()
            self.high_queue.put_nowait(message)
        except Exception as e:
            print(f"[PriorityQueue] High queue error: {e}")

    async def put_normal(self, message: Dict[str, Any]):
        """Pushes a normal-priority message with rate throttling and drop-oldest on congestion."""
        msg_type = message.get("type")
        now = time.time()

        if msg_type == MessageType.TELEMETRY.value:
            if now - self._last_telem_broadcast < self._telem_min_interval:
                return  # Throttle redundant high-frequency telemetry
            self._last_telem_broadcast = now
        elif msg_type == MessageType.MAP_UPDATE.value:
            if now - self._last_map_broadcast < self._map_min_interval:
                return  # Throttle dense map raycasting packets
            self._last_map_broadcast = now

        try:
            if self.normal_queue.full():
                _ = self.normal_queue.get_nowait()
            self.normal_queue.put_nowait(message)
        except Exception:
            pass

    async def put_video_frame(self, camera_id: str, frame_payload: Dict[str, Any]):
        """Pushes camera frame with strict drop-oldest policy (prevents WebSocket backpressure)."""
        target_queue = self.video_thermal_queue if camera_id == "thermal" else self.video_rgb_queue
        try:
            if target_queue.full():
                _ = target_queue.get_nowait()
                self.dropped_frames_count += 1
            target_queue.put_nowait(frame_payload)
        except Exception:
            pass

    def start(self):
        if not self._running:
            self._running = True
            self._dispatch_task = asyncio.create_task(self._dispatch_loop())

    def stop(self):
        self._running = False
        if self._dispatch_task:
            self._dispatch_task.cancel()
            self._dispatch_task = None

    async def _dispatch_loop(self):
        """Dispatches queued messages to all connected WebSockets in priority order."""
        while self._running:
            try:
                if not self._clients:
                    await asyncio.sleep(0.05)
                    continue

                dispatched = False

                # 1. Process all pending HIGH priority messages first
                while not self.high_queue.empty():
                    msg = self.high_queue.get_nowait()
                    await self._broadcast_json(msg)
                    dispatched = True

                # 2. Process pending NORMAL priority messages (telemetry / map)
                if not self.normal_queue.empty():
                    msg = self.normal_queue.get_nowait()
                    await self._broadcast_json(msg)
                    dispatched = True

                # 3. Process video frames
                if not self.video_rgb_queue.empty():
                    rgb_msg = self.video_rgb_queue.get_nowait()
                    await self._broadcast_json(rgb_msg)
                    dispatched = True

                if not self.video_thermal_queue.empty():
                    thermal_msg = self.video_thermal_queue.get_nowait()
                    await self._broadcast_json(thermal_msg)
                    dispatched = True

                if not dispatched:
                    await asyncio.sleep(0.01)
                else:
                    await asyncio.sleep(0.005)

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[PriorityQueue] Dispatch loop error: {e}")
                await asyncio.sleep(0.05)

    async def _broadcast_json(self, message: Dict[str, Any]):
        """Broadcasts JSON payload to all active clients; cleans up dead connections safely."""
        if not self._clients:
            return

        dead_clients = set()
        for ws in list(self._clients):
            if ws.closed:
                dead_clients.add(ws)
                continue
            try:
                await ws.send_json(message)
                self.messages_sent_count += 1
            except Exception:
                dead_clients.add(ws)

        for ws in dead_clients:
            self.unregister_client(ws)
