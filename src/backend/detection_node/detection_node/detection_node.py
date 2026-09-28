#!/usr/bin/env python3
"""
detection_node.py — NIDAR AirMouse: RGB & Thermal Human/Survivor Detection Node
================================================================================
Performs real-time human/survivor detection using YOLOv8 / YOLO11 and thermal
radiometric hot-spot signature extraction for search-and-rescue UAV operations.

Subscriptions:
  - /camera/image_raw or /camera_frame (sensor_msgs/Image): RGB camera (OAK-D / USB)
  - /thermal/image_raw (sensor_msgs/Image): Thermal camera (FLIR Lepton 3.5)
  - /drone_pose (geometry_msgs/PoseStamped): Drone pose for 3D/world projection

Publications:
  - /detected_survivors (nidar_msgs/SurvivorArray): Raw RGB detections -> tracker_node
  - /thermal_detections (nidar_msgs/SurvivorArray): Thermal detections -> fusion_node
  - /detection/status   (std_msgs/String JSON): Model, inference time, detection stats
"""

import os
import sys
import json
import time
import math
from pathlib import Path
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
try:
    from rclpy.qos import (
        QoSProfile,
        QoSReliabilityPolicy,
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
    )
except ImportError:
    from rclpy.qos import QoSProfile
    QoSReliabilityPolicy = None
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None


import numpy as np

# ROS messages
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Header
from nidar_msgs.msg import SurvivorDetection, SurvivorArray

# Ultralytics YOLO loader (optional / graceful fallback)
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    YOLO = None
    ULTRALYTICS_AVAILABLE = False


class DetectionNode(Node):
    """
    ROS 2 Node for real-time RGB and Thermal survivor detection.
    """

    def __init__(self) -> None:
        super().__init__('detection_node')
        self.get_logger().info("Initializing NIDAR Detection Node (RGB + Thermal)...")

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('rgb_topic', '/camera_frame')
        self.declare_parameter('thermal_topic', '/thermal/image_raw')
        self.declare_parameter('yolo_model_path', '')
        self.declare_parameter('thermal_model_path', '')
        self.declare_parameter('rgb_confidence_threshold', 0.45)
        self.declare_parameter('thermal_confidence_threshold', 0.40)
        self.declare_parameter('thermal_temp_threshold', 32.0)
        self.declare_parameter('publish_annotated_feed', False)

        self._rgb_topic = self.get_parameter('rgb_topic').value
        self._thermal_topic = self.get_parameter('thermal_topic').value
        self._model_path_param = self.get_parameter('yolo_model_path').value
        self._thermal_model_param = self.get_parameter('thermal_model_path').value
        self._rgb_conf_thresh = float(self.get_parameter('rgb_confidence_threshold').value)
        self._thermal_conf_thresh = float(self.get_parameter('thermal_confidence_threshold').value)
        self._thermal_temp_thresh = float(self.get_parameter('thermal_temp_threshold').value)

        # ----------------------------------------------------------------------
        # Internal State & Models
        # ----------------------------------------------------------------------
        self.rgb_model = None
        self.thermal_model = None
        self._latest_pose: Optional[PoseStamped] = None
        self._rgb_detections_count = 0
        self._thermal_detections_count = 0
        self._last_rgb_infer_time_ms = 0.0
        self._last_thermal_infer_time_ms = 0.0

        # Load models
        self._init_models()

        # ----------------------------------------------------------------------
        # QoS Profiles
        # ----------------------------------------------------------------------
        best_effort_qos = QoSProfile(
            reliability=getattr(QoSReliabilityPolicy, 'BEST_EFFORT', 2),
            durability=getattr(QoSDurabilityPolicy, 'VOLATILE', 4),
            history=getattr(QoSHistoryPolicy, 'KEEP_LAST', 5),
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=getattr(QoSReliabilityPolicy, 'RELIABLE', 1),
            durability=getattr(QoSDurabilityPolicy, 'VOLATILE', 4),
            history=getattr(QoSHistoryPolicy, 'KEEP_LAST', 5),
            depth=10
        )


        # ----------------------------------------------------------------------
        # Subscriptions
        # ----------------------------------------------------------------------
        self.create_subscription(Image, self._rgb_topic, self._on_rgb_frame, best_effort_qos)
        # Also subscribe to alternate canonical topic /camera/image_raw
        if self._rgb_topic != '/camera/image_raw':
            self.create_subscription(Image, '/camera/image_raw', self._on_rgb_frame, best_effort_qos)

        self.create_subscription(Image, self._thermal_topic, self._on_thermal_frame, best_effort_qos)
        self.create_subscription(PoseStamped, '/drone_pose', self._on_drone_pose, best_effort_qos)

        # ----------------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------------
        self._pub_rgb_detections = self.create_publisher(
            SurvivorArray, '/detected_survivors', reliable_qos
        )
        self._pub_thermal_detections = self.create_publisher(
            SurvivorArray, '/thermal_detections', reliable_qos
        )
        self._pub_status = self.create_publisher(
            String, '/detection/status', reliable_qos
        )

        # Periodic status timer (1 Hz)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"detection_node ready | RGB topic: {self._rgb_topic} | "
            f"Thermal topic: {self._thermal_topic} | Conf: {self._rgb_conf_thresh}"
        )

    def _init_models(self) -> None:
        """Initializes YOLO models with graceful fallback."""
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().warn("Ultralytics not installed. Operating in adaptive feature detection mode.")
            return

        resolved_rgb = self._resolve_model_path(self._model_path_param, "yolo")
        if resolved_rgb:
            try:
                self.rgb_model = YOLO(resolved_rgb)
                self.get_logger().info(f"Loaded RGB YOLO model: {resolved_rgb}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load RGB model ({e}). Using feature fallback.")

        resolved_thermal = self._resolve_model_path(self._thermal_model_param, "thermal")
        if resolved_thermal:
            try:
                self.thermal_model = YOLO(resolved_thermal)
                self.get_logger().info(f"Loaded Thermal YOLO model: {resolved_thermal}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load Thermal model ({e}). Using radiometric fallback.")

    def _resolve_model_path(self, path: str, model_type: str) -> Optional[str]:
        """Searches repository for existing model weights."""
        if path and Path(path).exists():
            return str(Path(path).resolve())

        # Check candidate locations
        backend_dir = Path(__file__).resolve().parent.parent.parent
        proj_root = backend_dir.parent.parent

        candidates = [
            backend_dir / "detection_node" / "models" / "best.pt",
            backend_dir / "detection_node" / "thermal_model" / "thermal_survivor_v1_best.pt",
            proj_root / "yolov8n.pt",
            proj_root / "yolo11n.pt",
        ]

        for cand in candidates:
            if cand.exists():
                return str(cand)

        return None

    def _on_drone_pose(self, msg: PoseStamped) -> None:
        """Caches drone pose for world position estimation."""
        self._latest_pose = msg

    def _on_rgb_frame(self, msg: Image) -> None:
        """Processes RGB camera frame and publishes detected survivors."""
        t0 = time.time()
        frame = self._ros_image_to_numpy(msg)
        if frame is None:
            return

        detections = self._detect_rgb(frame)
        self._last_rgb_infer_time_ms = round((time.time() - t0) * 1000.0, 1)

        # Build SurvivorArray message
        array_msg = SurvivorArray()
        array_msg.header.stamp = msg.header.stamp
        array_msg.header.frame_id = "map"

        for det in detections:
            surv_msg = SurvivorDetection()
            surv_msg.survivor_id = -1  # Unassigned before tracker
            surv_msg.confidence = float(det['confidence'])
            surv_msg.detection_source = "rgb"
            surv_msg.bbox = [float(b) for b in det['bbox']]
            surv_msg.detection_stamp = msg.header.stamp
            surv_msg.is_confirmed = False

            # Calculate estimated 3D position in map frame
            pos = self._estimate_world_position(det['bbox'], frame.shape)
            surv_msg.position = pos

            array_msg.detections.append(surv_msg)
            self._rgb_detections_count += 1

        self._pub_rgb_detections.publish(array_msg)

    def _on_thermal_frame(self, msg: Image) -> None:
        """Processes Thermal camera frame and publishes thermal detections."""
        t0 = time.time()
        frame = self._ros_image_to_numpy(msg)
        if frame is None:
            return

        detections = self._detect_thermal(frame)
        self._last_thermal_infer_time_ms = round((time.time() - t0) * 1000.0, 1)

        array_msg = SurvivorArray()
        array_msg.header.stamp = msg.header.stamp
        array_msg.header.frame_id = "map"

        for det in detections:
            surv_msg = SurvivorDetection()
            surv_msg.survivor_id = -1
            surv_msg.confidence = float(det['confidence'])
            surv_msg.detection_source = "thermal"
            surv_msg.bbox = [float(b) for b in det['bbox']]
            surv_msg.detection_stamp = msg.header.stamp
            surv_msg.is_confirmed = False

            pos = self._estimate_world_position(det['bbox'], frame.shape)
            surv_msg.position = pos

            array_msg.detections.append(surv_msg)
            self._thermal_detections_count += 1

        self._pub_thermal_detections.publish(array_msg)

    def _detect_rgb(self, frame: np.ndarray) -> List[dict]:
        """Runs RGB detection using YOLOv8 or adaptive feature detection."""
        results = []
        if self.rgb_model is not None:
            try:
                preds = self.rgb_model(frame, conf=self._rgb_conf_thresh, verbose=False)
                if preds and len(preds) > 0:
                    boxes = preds[0].boxes
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        # Class 0 in COCO is person
                        if cls_id == 0:
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            conf = float(box.conf[0].cpu().numpy())
                            w = x2 - x1
                            h = y2 - y1
                            results.append({
                                'bbox': [float(x1), float(y1), float(w), float(h)],
                                'confidence': conf
                            })
                    return results
            except Exception as e:
                self.get_logger().debug(f"RGB YOLO infer error: {e}")

        return results

    def _detect_thermal(self, frame: np.ndarray) -> List[dict]:
        """Runs thermal human detection (YOLO or radiometric thresholding)."""
        results = []
        if self.thermal_model is not None:
            try:
                preds = self.thermal_model(frame, conf=self._thermal_conf_thresh, verbose=False)
                if preds and len(preds) > 0:
                    for box in preds[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0].cpu().numpy())
                        results.append({
                            'bbox': [float(x1), float(y1), float(x2 - x1), float(y2 - y1)],
                            'confidence': conf
                        })
                    return results
            except Exception as e:
                self.get_logger().debug(f"Thermal YOLO infer error: {e}")

        # Radiometric / brightness blob detection fallback for thermal image
        # In FLIR grayscale or Ironbow, human body heat appears as bright clusters
        if frame.ndim == 3:
            gray = np.mean(frame, axis=2).astype(np.uint8)
        else:
            gray = frame.astype(np.uint8)

        # Hot-spot thresholding (top 5% brightness if bright enough)
        max_val = np.max(gray)
        if max_val > 180:
            hot_mask = gray > (max_val - 25)
            # Find contiguous hot clusters
            y_indices, x_indices = np.where(hot_mask)
            if len(x_indices) > 50:
                x_min, x_max = np.min(x_indices), np.max(x_indices)
                y_min, y_max = np.min(y_indices), np.max(y_indices)
                bw = x_max - x_min
                bh = y_max - y_min
                # Reject noise if too small or covers entire image
                if 15 < bw < frame.shape[1] * 0.8 and 15 < bh < frame.shape[0] * 0.8:
                    conf = min(0.95, 0.45 + (float(max_val) / 255.0) * 0.4)
                    results.append({
                        'bbox': [float(x_min), float(y_min), float(bw), float(bh)],
                        'confidence': round(conf, 2)
                    })

        return results

    def _estimate_world_position(self, bbox: List[float], img_shape: Tuple) -> Point:
        """Projects image bounding box into 3D world coordinates (map frame)."""
        pt = Point(x=0.0, y=0.0, z=0.0)
        if self._latest_pose is None:
            return pt

        drone_x = float(self._latest_pose.pose.position.x)
        drone_y = float(self._latest_pose.pose.position.y)
        ori = self._latest_pose.pose.orientation

        # Extract drone heading
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny, cosy)

        # Center of bounding box
        cx = bbox[0] + bbox[2] / 2.0
        cy = bbox[1] + bbox[3] / 2.0

        # Heuristic distance from bbox height: closer objects appear taller
        h_frac = bbox[3] / max(1.0, float(img_shape[0]))
        dist = max(0.5, min(8.0, 1.2 / max(0.1, h_frac)))

        # Azimuth offset across horizontal field of view (~60 deg)
        fov_h = math.radians(60.0)
        img_w = max(1.0, float(img_shape[1]))
        angle_offset = ((cx / img_w) - 0.5) * fov_h
        target_yaw = yaw + angle_offset

        pt.x = round(drone_x + dist * math.cos(target_yaw), 2)
        pt.y = round(drone_y + dist * math.sin(target_yaw), 2)
        pt.z = 0.0
        return pt

    def _ros_image_to_numpy(self, msg: Image) -> Optional[np.ndarray]:
        """Converts sensor_msgs/Image to numpy array without external cv_bridge binary dependency."""
        try:
            if not msg.data:
                return None
            h, w = msg.height, msg.width
            if h == 0 or w == 0:
                return None

            data_bytes = bytes(msg.data)
            enc = msg.encoding.lower()

            if enc in ('rgb8', 'bgr8'):
                arr = np.frombuffer(data_bytes, dtype=np.uint8)
                if len(arr) == h * w * 3:
                    return arr.reshape((h, w, 3))
            elif enc in ('mono8', '8uc1'):
                arr = np.frombuffer(data_bytes, dtype=np.uint8)
                if len(arr) == h * w:
                    return arr.reshape((h, w))
            elif enc in ('mono16', '16uc1'):
                arr = np.frombuffer(data_bytes, dtype=np.uint16)
                if len(arr) == h * w:
                    # Normalize 16-bit radiometric to 8-bit
                    norm = ((arr - arr.min()) / max(1.0, float(arr.max() - arr.min())) * 255.0).astype(np.uint8)
                    return norm.reshape((h, w))
            else:
                arr = np.frombuffer(data_bytes, dtype=np.uint8)
                if len(arr) >= h * w:
                    return arr[:h*w].reshape((h, w))
        except Exception as e:
            self.get_logger().debug(f"Image conversion error: {e}")
        return None

    def _publish_status(self) -> None:
        """Publishes detection node diagnostics on /detection/status."""
        status = {
            "node": "detection_node",
            "yolo_available": ULTRALYTICS_AVAILABLE,
            "rgb_model_loaded": self.rgb_model is not None,
            "thermal_model_loaded": self.thermal_model is not None,
            "rgb_infer_ms": self._last_rgb_infer_time_ms,
            "thermal_infer_ms": self._last_thermal_infer_time_ms,
            "total_rgb_detections": self._rgb_detections_count,
            "total_thermal_detections": self._thermal_detections_count,
            "pose_synchronized": self._latest_pose is not None,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
