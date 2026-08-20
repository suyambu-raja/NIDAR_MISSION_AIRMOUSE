#!/usr/bin/env python3
"""
NIDAR AirMouse — Fusion Node (Main ROS 2 Node)
Receives detections from RGB tracker and thermal detector, cross-checks
them using spatial alignment, calculates combined confidence via weighted
evidence fusion + EMA, enforces multi-frame persistence, deduplicates,
and publishes confirmed survivors.

Pipeline (runs at 5Hz via timer):
    1. Visibility estimation   — is it dark/smoky?
    2. Detection aggregation   — collect recent RGB + thermal within time window
    3. Spatial alignment       — match cross-modal detections by world position
    4. Confidence fusion       — weighted combination + EMA smoothing
    5. Persistence gate        — require N consecutive frames
    6. Confidence threshold    — reject low-confidence detections
    7. Deduplication           — prevent duplicate survivor tags
    8. Publish confirmed       — /confirmed_survivors + /fusion_status

Subscriptions:
    /tracked_survivors    (nidar_msgs/SurvivorArray)    ~30Hz from tracker_node
    /thermal_detections   (nidar_msgs/SurvivorArray)    ~8Hz from thermal_node
    /drone_pose           (geometry_msgs/PoseStamped)   ~10Hz from slam_node
    /camera_frame         (sensor_msgs/Image)           ~30Hz from OAK-D

Publications:
    /confirmed_survivors  (nidar_msgs/SurvivorArray)    ~5Hz
    /fusion_status        (std_msgs/String JSON)        ~2Hz
"""

import json
import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

import numpy as np

# ROS 2 message types
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

# Custom NIDAR messages
from nidar_msgs.msg import SurvivorDetection, SurvivorArray

# Fusion pipeline modules
from .visibility_estimator import VisibilityEstimator, VisibilityState
from .spatial_aligner import SpatialAligner, Detection, MatchResult
from .confidence_fuser import ConfidenceFuser
from .persistence_tracker import PersistenceTracker
from .deduplicator import Deduplicator


def quaternion_to_yaw(orientation) -> float:
    """
    Extract yaw (heading) angle from a quaternion.
    For 2D SLAM, only yaw is relevant (roll and pitch are 0).

    Args:
        orientation: geometry_msgs Quaternion (x, y, z, w)

    Returns:
        Yaw angle in radians
    """
    # Standard quaternion-to-Euler yaw extraction
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FusionNode(Node):
    """
    Main ROS 2 fusion node. Orchestrates the complete sensor fusion pipeline.
    """

    def __init__(self):
        super().__init__('fusion_node')
        self.get_logger().info("Initializing NIDAR Fusion Node...")

        # ==================================================================
        # ROS 2 Parameters — all tunable at runtime via `ros2 param set`
        # ==================================================================
        self.declare_parameter('confidence_threshold', 0.65)
        self.declare_parameter('rgb_weight_normal', 0.7)
        self.declare_parameter('thermal_weight_normal', 0.3)
        self.declare_parameter('rgb_weight_degraded', 0.3)
        self.declare_parameter('thermal_weight_degraded', 0.7)
        self.declare_parameter('brightness_threshold', 50)
        self.declare_parameter('persistence_normal', 5)
        self.declare_parameter('persistence_degraded', 8)
        self.declare_parameter('match_distance', 0.5)
        self.declare_parameter('dedup_distance', 0.5)
        self.declare_parameter('max_survivors', 6)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('detection_window_ms', 200.0)
        self.declare_parameter('visibility_update_interval', 10)

        # Read initial parameter values
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.detection_window_ms = self.get_parameter('detection_window_ms').value

        # Register dynamic parameter callback
        self.add_on_set_parameters_callback(self._parameters_callback)

        # ==================================================================
        # Initialize pipeline modules with parameter values
        # ==================================================================
        self.visibility_estimator = VisibilityEstimator(
            brightness_threshold=self.get_parameter('brightness_threshold').value,
            update_interval=self.get_parameter('visibility_update_interval').value
        )

        self.spatial_aligner = SpatialAligner(
            match_distance=self.get_parameter('match_distance').value
        )

        self.confidence_fuser = ConfidenceFuser(
            rgb_weight_normal=self.get_parameter('rgb_weight_normal').value,
            thermal_weight_normal=self.get_parameter('thermal_weight_normal').value,
            rgb_weight_degraded=self.get_parameter('rgb_weight_degraded').value,
            thermal_weight_degraded=self.get_parameter('thermal_weight_degraded').value,
            ema_alpha=self.get_parameter('ema_alpha').value
        )

        self.persistence_tracker = PersistenceTracker(
            persistence_normal=self.get_parameter('persistence_normal').value,
            persistence_degraded=self.get_parameter('persistence_degraded').value
        )

        self.deduplicator = Deduplicator(
            dedup_distance=self.get_parameter('dedup_distance').value,
            max_survivors=self.get_parameter('max_survivors').value
        )

        # ==================================================================
        # Detection buffers — cache incoming detections with timestamps
        # ==================================================================
        # Each entry: (timestamp_sec, Detection)
        self._rgb_buffer: List[Tuple[float, Detection]] = []
        self._thermal_buffer: List[Tuple[float, Detection]] = []

        # Latest drone pose (cached from /drone_pose subscription)
        self._latest_pose: Optional[PoseStamped] = None
        self._pose_received = False

        # Latest camera frame for visibility estimation
        self._latest_frame: Optional[np.ndarray] = None

        # ==================================================================
        # QoS Profiles
        # ==================================================================
        # Reliable QoS for detection topics (don't drop survivors)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Best-effort QoS for high-rate sensor streams (pose, camera)
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ==================================================================
        # Subscribers
        # ==================================================================
        # RGB tracked survivors from tracker_node (~30Hz)
        self.sub_tracked = self.create_subscription(
            SurvivorArray,
            '/tracked_survivors',
            self._tracked_callback,
            reliable_qos
        )

        # Thermal detections from thermal_node (~8Hz)
        self.sub_thermal = self.create_subscription(
            SurvivorArray,
            '/thermal_detections',
            self._thermal_callback,
            reliable_qos
        )

        # Drone pose from slam_node (~10Hz)
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/drone_pose',
            self._pose_callback,
            best_effort_qos
        )

        # Camera frame from OAK-D (~30Hz) — for visibility estimation only
        self.sub_camera = self.create_subscription(
            Image,
            '/camera_frame',
            self._camera_callback,
            best_effort_qos
        )

        # ==================================================================
        # Publishers
        # ==================================================================
        # Confirmed survivors → grid_mapper_node + GCS dashboard
        self.pub_confirmed = self.create_publisher(
            SurvivorArray,
            '/confirmed_survivors',
            reliable_qos
        )

        # Fusion status → GCS dashboard (JSON payload)
        self.pub_status = self.create_publisher(
            String,
            '/fusion_status',
            reliable_qos
        )

        # ==================================================================
        # Timers — drive the fusion pipeline and status publishing
        # ==================================================================
        # Main fusion pipeline at 5Hz (200ms interval)
        self.fusion_timer = self.create_timer(0.2, self._fusion_pipeline)

        # Status publisher at 2Hz (500ms interval)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info("NIDAR Fusion Node initialized successfully.")
        self.get_logger().info(
            f"  confidence_threshold={self.confidence_threshold}, "
            f"max_survivors={self.deduplicator.max_survivors}"
        )

    # ======================================================================
    # Subscription Callbacks — buffer incoming data
    # ======================================================================

    def _tracked_callback(self, msg: SurvivorArray):
        """
        Buffer incoming RGB tracked detections.
        Converts SurvivorDetection messages to lightweight Detection objects.
        """
        now = self.get_clock().now().nanoseconds / 1e9  # seconds

        for det in msg.detections:
            detection = Detection(
                x=det.position.x,
                y=det.position.y,
                confidence=det.confidence,
                source="rgb",
                bbox=(det.bbox[0], det.bbox[1], det.bbox[2], det.bbox[3]),
                track_id=det.survivor_id
            )
            self._rgb_buffer.append((now, detection))

        # Prune old entries beyond the detection window
        self._prune_buffer(self._rgb_buffer, now)

    def _thermal_callback(self, msg: SurvivorArray):
        """
        Buffer incoming thermal detections.
        If thermal detections lack world positions, project them using drone pose.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        for det in msg.detections:
            # Check if thermal detection has a valid world position
            if det.position.x != 0.0 or det.position.y != 0.0:
                # Thermal node already provided world position
                world_x = det.position.x
                world_y = det.position.y
            elif self._latest_pose is not None:
                # Project from image space to world using drone pose
                drone_x = self._latest_pose.pose.position.x
                drone_y = self._latest_pose.pose.position.y
                drone_yaw = quaternion_to_yaw(self._latest_pose.pose.orientation)

                # Use bbox center for projection
                bbox_cx = det.bbox[0] + det.bbox[2] / 2.0
                bbox_cy = det.bbox[1] + det.bbox[3] / 2.0

                # Estimate depth from bbox height (heuristic: bigger box = closer)
                # Default to 2m if no depth info — conservative estimate for indoor
                estimated_depth = max(det.bbox[3] * 0.01, 0.5) if det.bbox[3] > 0 else 2.0

                world_x, world_y = SpatialAligner.project_to_world(
                    bbox_cx, bbox_cy, estimated_depth,
                    drone_x, drone_y, drone_yaw
                )
            else:
                # No pose available — skip this detection with warning
                self.get_logger().warn(
                    "Thermal detection received but drone pose unavailable. Skipping.",
                    throttle_duration_sec=5.0
                )
                continue

            detection = Detection(
                x=world_x,
                y=world_y,
                confidence=det.confidence,
                source="thermal",
                bbox=(det.bbox[0], det.bbox[1], det.bbox[2], det.bbox[3]),
                track_id=det.survivor_id
            )
            self._thermal_buffer.append((now, detection))

        self._prune_buffer(self._thermal_buffer, now)

    def _pose_callback(self, msg: PoseStamped):
        """Cache the latest drone pose for spatial projection."""
        self._latest_pose = msg
        if not self._pose_received:
            self._pose_received = True
            self.get_logger().info("Drone pose received — spatial alignment enabled.")

    def _camera_callback(self, msg: Image):
        """
        Convert ROS Image to numpy array for visibility estimation.
        Only processes the brightness — does not store full frames.
        """
        try:
            # Convert ROS Image message to numpy array
            # Encoding can be 'rgb8', 'bgr8', 'mono8', etc.
            if msg.encoding in ('rgb8', 'bgr8'):
                # 3-channel color image
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
            elif msg.encoding == 'mono8':
                # Single-channel grayscale
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width
                )
            else:
                # Unsupported encoding — try raw conversion
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, -1
                )

            # Update visibility estimator (internally handles frame skipping)
            self.visibility_estimator.update(frame)

        except Exception as e:
            self.get_logger().warn(
                f"Failed to process camera frame: {e}",
                throttle_duration_sec=5.0
            )

    # ======================================================================
    # Main Fusion Pipeline — runs at 5Hz
    # ======================================================================

    def _fusion_pipeline(self):
        """
        Core fusion pipeline. Called at 5Hz by timer.

        Steps:
        1. Get current visibility state
        2. Collect recent detections from both buffers
        3. Spatial alignment — match RGB ↔ thermal
        4. Confidence fusion — weighted combination + EMA
        5. Persistence tracking — require N consecutive frames
        6. Confidence thresholding — reject below 0.65
        7. Deduplication — prevent duplicate survivors
        8. Publish confirmed survivors
        """
        # Step 0: If max survivors already found, skip pipeline
        if self.deduplicator.is_full:
            return

        # Step 1: Get current visibility state
        visibility = self.visibility_estimator.current_state

        # Step 2: Collect recent detections within the time window
        now = self.get_clock().now().nanoseconds / 1e9
        window_sec = self.detection_window_ms / 1000.0

        rgb_dets = [det for ts, det in self._rgb_buffer
                    if (now - ts) <= window_sec]
        thermal_dets = [det for ts, det in self._thermal_buffer
                        if (now - ts) <= window_sec]

        # If no detections from either sensor, still update persistence
        # (to age out stale entries) but with empty list
        if not rgb_dets and not thermal_dets:
            self.persistence_tracker.update([], visibility)
            return

        # Step 3: Spatial alignment — match RGB and thermal detections
        match_result = self.spatial_aligner.align(rgb_dets, thermal_dets)

        # Step 4 & 5: Confidence fusion + collect for persistence
        candidates = []  # (x, y, confidence, source, track_id, bbox)

        # Process matched pairs — highest quality fused detections
        for rgb_det, thermal_det in match_result.matched_pairs:
            fused_conf = self.confidence_fuser.fuse(
                rgb_confidence=rgb_det.confidence,
                thermal_confidence=thermal_det.confidence,
                source="fused",
                visibility=visibility,
                world_x=rgb_det.x,
                world_y=rgb_det.y
            )
            # Use RGB position (more precise with depth camera)
            candidates.append((
                rgb_det.x, rgb_det.y, fused_conf, "fused",
                rgb_det.track_id, rgb_det.bbox
            ))

        # Process unmatched RGB detections
        for det in match_result.unmatched_rgb:
            fused_conf = self.confidence_fuser.fuse(
                rgb_confidence=det.confidence,
                thermal_confidence=None,
                source="rgb",
                visibility=visibility,
                world_x=det.x,
                world_y=det.y
            )
            candidates.append((
                det.x, det.y, fused_conf, "rgb",
                det.track_id, det.bbox
            ))

        # Process unmatched thermal detections
        for det in match_result.unmatched_thermal:
            fused_conf = self.confidence_fuser.fuse(
                rgb_confidence=None,
                thermal_confidence=det.confidence,
                source="thermal",
                visibility=visibility,
                world_x=det.x,
                world_y=det.y
            )
            candidates.append((
                det.x, det.y, fused_conf, "thermal",
                det.track_id, det.bbox
            ))

        # Step 5: Persistence tracking — require consecutive frames
        persistent = self.persistence_tracker.update(candidates, visibility)

        # Step 6 & 7: Confidence threshold + deduplication → publish
        confirmed_msg = SurvivorArray()
        confirmed_msg.header.stamp = self.get_clock().now().to_msg()
        confirmed_msg.header.frame_id = "map"

        for entry in persistent:
            # Apply confidence threshold
            if entry.confidence < self.confidence_threshold:
                continue

            # Check deduplication
            is_new, survivor_id = self.deduplicator.check(
                x=entry.x,
                y=entry.y,
                confidence=entry.confidence,
                source=entry.source,
                bbox=entry.bbox
            )

            if survivor_id == -1:
                # Max survivors reached — log once and skip
                self.get_logger().info(
                    "Maximum survivor limit reached. Ignoring new detection.",
                    throttle_duration_sec=10.0
                )
                continue

            if is_new:
                self.get_logger().info(
                    f"NEW SURVIVOR CONFIRMED! ID={survivor_id} "
                    f"at ({entry.x:.2f}, {entry.y:.2f}) "
                    f"confidence={entry.confidence:.3f} "
                    f"source={entry.source}"
                )

            # Build SurvivorDetection message for confirmed survivor
            det_msg = SurvivorDetection()
            det_msg.survivor_id = survivor_id
            det_msg.position.x = entry.x
            det_msg.position.y = entry.y
            det_msg.position.z = 0.0  # 2D SLAM — z is always 0
            det_msg.confidence = entry.confidence
            det_msg.detection_source = entry.source
            det_msg.bbox = list(entry.bbox)
            det_msg.detection_stamp = self.get_clock().now().to_msg()
            det_msg.is_confirmed = True

            confirmed_msg.detections.append(det_msg)

        # Publish all confirmed survivors (full running list from deduplicator)
        # Include ALL confirmed survivors, not just new ones this frame
        all_confirmed = self.deduplicator.get_confirmed_list()
        if all_confirmed:
            full_msg = SurvivorArray()
            full_msg.header.stamp = self.get_clock().now().to_msg()
            full_msg.header.frame_id = "map"

            for survivor in all_confirmed:
                det_msg = SurvivorDetection()
                det_msg.survivor_id = survivor.survivor_id
                det_msg.position.x = survivor.x
                det_msg.position.y = survivor.y
                det_msg.position.z = 0.0
                det_msg.confidence = survivor.confidence
                det_msg.detection_source = survivor.detection_source
                det_msg.bbox = list(survivor.bbox)
                det_msg.detection_stamp = self.get_clock().now().to_msg()
                det_msg.is_confirmed = True
                full_msg.detections.append(det_msg)

            self.pub_confirmed.publish(full_msg)

    # ======================================================================
    # Status Publisher — runs at 2Hz
    # ======================================================================

    def _publish_status(self):
        """
        Publish fusion node status as JSON on /fusion_status.
        Matches the pattern used by exploration_node's /exploration_status.
        """
        status = {
            "visibility_state": self.visibility_estimator.current_state.value,
            "active_detector": self._get_active_detector(),
            "brightness": round(self.visibility_estimator.last_brightness, 1),
            "pending_rgb": len(self._rgb_buffer),
            "pending_thermal": len(self._thermal_buffer),
            "tracked_positions": self.persistence_tracker.tracked_count,
            "confirmed_survivors": self.deduplicator.confirmed_count,
            "max_survivors": self.deduplicator.max_survivors,
            "confidence_threshold": self.confidence_threshold,
            "pose_available": self._pose_received
        }

        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    def _get_active_detector(self) -> str:
        """
        Determine which detector(s) are currently active based on
        recent buffer contents and visibility state.
        """
        has_rgb = len(self._rgb_buffer) > 0
        has_thermal = len(self._thermal_buffer) > 0

        if has_rgb and has_thermal:
            return "Both"
        elif has_rgb:
            return "RGB"
        elif has_thermal:
            return "Thermal"
        else:
            return "None"

    # ======================================================================
    # Buffer Management
    # ======================================================================

    def _prune_buffer(
        self,
        buffer: List[Tuple[float, Detection]],
        now: float,
        max_age_sec: float = 1.0
    ):
        """
        Remove detections older than max_age_sec from the buffer.
        Keeps memory bounded on RPi 4.

        Args:
            buffer: Detection buffer to prune (modified in-place)
            now: Current timestamp in seconds
            max_age_sec: Maximum age before removal
        """
        # Remove entries older than max_age_sec
        cutoff = now - max_age_sec
        buffer[:] = [(ts, det) for ts, det in buffer if ts >= cutoff]

    # ======================================================================
    # Dynamic Parameter Updates
    # ======================================================================

    def _parameters_callback(self, params):
        """
        Handle dynamic parameter updates from `ros2 param set`.
        Updates internal module configurations without requiring node restart.
        """
        for param in params:
            name = param.name
            value = param.value

            if name == 'confidence_threshold':
                self.confidence_threshold = value
                self.get_logger().info(f"Updated confidence_threshold to {value}")

            elif name == 'brightness_threshold':
                self.visibility_estimator.brightness_threshold = value
                self.get_logger().info(f"Updated brightness_threshold to {value}")

            elif name == 'match_distance':
                self.spatial_aligner.match_distance = value
                self.get_logger().info(f"Updated match_distance to {value}")

            elif name == 'dedup_distance':
                self.deduplicator.dedup_distance = value
                self.get_logger().info(f"Updated dedup_distance to {value}")

            elif name == 'max_survivors':
                self.deduplicator.max_survivors = int(value)
                self.get_logger().info(f"Updated max_survivors to {value}")

            elif name == 'persistence_normal':
                self.persistence_tracker.persistence_normal = int(value)
                self.get_logger().info(f"Updated persistence_normal to {value}")

            elif name == 'persistence_degraded':
                self.persistence_tracker.persistence_degraded = int(value)
                self.get_logger().info(f"Updated persistence_degraded to {value}")

            elif name == 'rgb_weight_normal':
                self.confidence_fuser.rgb_weight_normal = value
                self.get_logger().info(f"Updated rgb_weight_normal to {value}")

            elif name == 'thermal_weight_normal':
                self.confidence_fuser.thermal_weight_normal = value
                self.get_logger().info(f"Updated thermal_weight_normal to {value}")

            elif name == 'rgb_weight_degraded':
                self.confidence_fuser.rgb_weight_degraded = value
                self.get_logger().info(f"Updated rgb_weight_degraded to {value}")

            elif name == 'thermal_weight_degraded':
                self.confidence_fuser.thermal_weight_degraded = value
                self.get_logger().info(f"Updated thermal_weight_degraded to {value}")

            elif name == 'ema_alpha':
                self.confidence_fuser.ema_alpha = value
                self.get_logger().info(f"Updated ema_alpha to {value}")

            elif name == 'detection_window_ms':
                self.detection_window_ms = value
                self.get_logger().info(f"Updated detection_window_ms to {value}")

            elif name == 'visibility_update_interval':
                self.visibility_estimator.update_interval = int(value)
                self.get_logger().info(f"Updated visibility_update_interval to {value}")

        return SetParametersResult(successful=True)


def main(args=None):
    """Entry point for ros2 run fusion_node fusion."""
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NIDAR Fusion Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
