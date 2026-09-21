"""
Video Bridge for NIDAR AirMouse GCS.
Subscribes to ROS 2 RGB and Thermal Image topics, compresses and encodes frames
to base64 JPEG, and pushes them into the priority message queue for the dashboard.
"""
import asyncio
import base64
import time
from typing import Optional, Dict, Any
import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    ROS2_AVAILABLE = True
except ImportError:
    Node = object
    ROS2_AVAILABLE = False


def ros_image_to_cv2(msg) -> Optional[np.ndarray]:
    """
    Converts a ROS sensor_msgs/Image message to an OpenCV BGR numpy array
    without requiring cv_bridge as a strict dependency.
    """
    try:
        # Check if cv_bridge is available first
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            if "mono" in msg.encoding or "16" in msg.encoding:
                return bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

        # Fallback pure-numpy buffer unpacking
        encoding = getattr(msg, "encoding", "bgr8").lower()
        height = int(msg.height)
        width = int(msg.width)
        data = msg.data

        if isinstance(data, (list, tuple)):
            data = bytes(data)

        if encoding in ("bgr8", "rgb8"):
            arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
            if encoding == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return arr
        elif encoding in ("mono8", "8uc1"):
            return np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        elif encoding in ("mono16", "16uc1"):
            arr = np.frombuffer(data, dtype=np.uint16).reshape((height, width))
            # Normalize 16-bit thermal range to 8-bit
            norm = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            return cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        else:
            # Generic 3-channel attempt
            arr = np.frombuffer(data, dtype=np.uint8)
            if len(arr) == height * width * 3:
                return arr.reshape((height, width, 3))
            elif len(arr) == height * width:
                return arr.reshape((height, width))
            return None
    except Exception as e:
        return None


class VideoBridgeNode(Node if ROS2_AVAILABLE else object):
    """
    ROS 2 video subscriber node that converts camera feeds to base64 JPEG
    with optimized compression for real-time WebSocket distribution.
    """

    def __init__(self, message_queue=None, loop: Optional[asyncio.AbstractEventLoop] = None):
        if ROS2_AVAILABLE:
            super().__init__("gcs_video_bridge")
        self.message_queue = message_queue
        self.loop = loop

        self.rgb_quality = 70
        self.thermal_quality = 60
        self.rgb_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.rgb_quality]
        self.thermal_encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.thermal_quality]

        # Rate control
        self.last_rgb_time = 0.0
        self.last_thermal_time = 0.0
        self.min_frame_interval = 0.06  # Max ~16 FPS

        if ROS2_AVAILABLE:
            self._setup_subscribers()

    def _setup_subscribers(self):
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        # RGB Camera (OAK-D)
        self.create_subscription(Image, "/camera/image_raw", self.on_rgb_frame, sensor_qos)
        self.create_subscription(Image, "/camera_frame", self.on_rgb_frame, sensor_qos)

        # Thermal Camera (FLIR Lepton)
        self.create_subscription(Image, "/thermal/image_raw", self.on_thermal_frame, sensor_qos)
        self.create_subscription(Image, "/thermal_frame", self.on_thermal_frame, sensor_qos)

    def process_rgb_image(self, img_np: np.ndarray) -> Optional[str]:
        """Resizes to 640x480, compresses to JPEG, and base64 encodes."""
        if img_np is None or img_np.size == 0:
            return None
        h, w = img_np.shape[:2]
        if w != 640 or h != 480:
            img_np = cv2.resize(img_np, (640, 480), interpolation=cv2.INTER_LINEAR)
        ret, buf = cv2.imencode(".jpg", img_np, self.rgb_encode_param)
        if not ret:
            return None
        return base64.b64encode(buf).decode("utf-8")

    def process_thermal_image(self, img_np: np.ndarray) -> Optional[str]:
        """Resizes to 320x240, applies thermal colormap if needed, and encodes to base64."""
        if img_np is None or img_np.size == 0:
            return None
        if len(img_np.shape) == 2:
            img_np = cv2.applyColorMap(img_np, cv2.COLORMAP_INFERNO)
        h, w = img_np.shape[:2]
        if w != 320 or h != 240:
            img_np = cv2.resize(img_np, (320, 240), interpolation=cv2.INTER_LINEAR)
        ret, buf = cv2.imencode(".jpg", img_np, self.thermal_encode_param)
        if not ret:
            return None
        return base64.b64encode(buf).decode("utf-8")

    def on_rgb_frame(self, msg):
        now = time.time()
        if now - self.last_rgb_time < self.min_frame_interval:
            return
        self.last_rgb_time = now

        frame = ros_image_to_cv2(msg)
        if frame is None:
            return
        b64 = self.process_rgb_image(frame)
        if not b64:
            return

        payload = {
            "type": "camera_frame",
            "camera_id": "rgb",
            "stream": "rgb",
            "frame_base64": b64,
            "data": b64,
            "timestamp": now,
            "fps": 15.0,
            "status": "LIVE"
        }
        self._dispatch_video("rgb", payload)

    def on_thermal_frame(self, msg):
        now = time.time()
        if now - self.last_thermal_time < self.min_frame_interval:
            return
        self.last_thermal_time = now

        frame = ros_image_to_cv2(msg)
        if frame is None:
            return
        b64 = self.process_thermal_image(frame)
        if not b64:
            return

        payload = {
            "type": "camera_frame",
            "camera_id": "thermal",
            "stream": "thermal",
            "frame_base64": b64,
            "data": b64,
            "timestamp": now,
            "fps": 15.0,
            "status": "LIVE"
        }
        self._dispatch_video("thermal", payload)

    def _dispatch_video(self, camera_id: str, payload: Dict[str, Any]):
        if not self.message_queue:
            return
        if self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.message_queue.put_video_frame(camera_id, payload), self.loop
            )
        else:
            target_q = (
                self.message_queue.video_thermal_queue
                if camera_id == "thermal"
                else self.message_queue.video_rgb_queue
            )
            try:
                if target_q.full():
                    _ = target_q.get_nowait()
                target_q.put_nowait(payload)
            except Exception:
                pass
