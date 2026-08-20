#!/usr/bin/env python3
"""
NIDAR AirMouse — Survivor Tracker Node
Uses Ultralytics ByteTrack internally to track detections across frames.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import argparse

# Import custom message types
from nidar_msgs.msg import SurvivorDetection, SurvivorArray

# Import ByteTrack from Ultralytics
try:
    from ultralytics.trackers.byte_tracker import BYTETracker
except ImportError as e:
    import sys
    print(f"Failed to import BYTETracker from Ultralytics: {e}", file=sys.stderr)
    print("Please ensure you are running in the correct environment with Ultralytics installed.", file=sys.stderr)
    sys.exit(1)


class TrackerDetections:
    """
    Wrapper class to format custom detections into the Results-like interface
    expected by the Ultralytics BYTETracker.
    """
    def __init__(self, xywh: np.ndarray, conf: np.ndarray, cls: np.ndarray):
        """
        Args:
            xywh (np.ndarray): Center coordinates + dimensions (N, 4) -> [x_center, y_center, width, height]
            conf (np.ndarray): Confidence scores (N,)
            cls (np.ndarray): Class indices (N,)
        """
        self.xywh = np.asarray(xywh, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32).reshape(-1)
        self.cls = np.asarray(cls, dtype=np.int32).reshape(-1)

    def __len__(self):
        return len(self.conf)

    def __getitem__(self, mask):
        return TrackerDetections(self.xywh[mask], self.conf[mask], self.cls[mask])


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.get_logger().info("Initializing NIDAR Tracker Node (ByteTrack)...")

        # ----------------------------------------------------------------------
        # ROS 2 Parameters Definition & Dynamic Configuration
        # ----------------------------------------------------------------------
        # Tune default parameters specifically for a slow-moving indoor drone:
        # - track_buffer: Default pedestrian tracking is 30 frames. For slow drones,
        #   raising this allows holding tracks across longer occlusions, but increases 
        #   risk of ID switches when multiple targets are close.
        # - match_thresh: Association IoU threshold. Raise to avoid false associations.
        self.declare_parameter('track_buffer', 45)       # Keep lost tracks alive for 45 frames (~1.5 sec at 30 fps)
        self.declare_parameter('match_thresh', 0.7)      # IoU threshold for matching
        self.declare_parameter('track_high_thresh', 0.40) # Confidence threshold for first-stage tracking
        self.declare_parameter('track_low_thresh', 0.10)  # Confidence threshold for second-stage tracking
        self.declare_parameter('new_track_thresh', 0.45)  # Min confidence to initiate a new track
        self.declare_parameter('fuse_score', True)        # Fuse confidence score with motion/IoU cost

        # Read parameters
        self.track_buffer = self.get_parameter('track_buffer').value
        self.match_thresh = self.get_parameter('match_thresh').value
        self.track_high_thresh = self.get_parameter('track_high_thresh').value
        self.track_low_thresh = self.get_parameter('track_low_thresh').value
        self.new_track_thresh = self.get_parameter('new_track_thresh').value
        self.fuse_score = self.get_parameter('fuse_score').value

        # Register dynamic parameter update callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ----------------------------------------------------------------------
        # ByteTracker Initialization
        # ----------------------------------------------------------------------
        tracker_args = argparse.Namespace(
            track_buffer=self.track_buffer,
            match_thresh=self.match_thresh,
            track_high_thresh=self.track_high_thresh,
            track_low_thresh=self.track_low_thresh,
            new_track_thresh=self.new_track_thresh,
            fuse_score=self.fuse_score
        )
        self.tracker = BYTETracker(tracker_args)

        # ----------------------------------------------------------------------
        # State Tracking for Logging Transitions
        # ----------------------------------------------------------------------
        # Keeping sets of track IDs from the previous frame to log transitions
        self.prev_active_track_ids = set()
        self.prev_lost_track_ids = set()

        # ----------------------------------------------------------------------
        # Publishers and Subscribers
        # ----------------------------------------------------------------------
        # Subscribe to detection/fusion node output (raw survivor detections)
        self.sub_detections = self.create_subscription(
            SurvivorArray,
            '/detected_survivors',
            self.detections_callback,
            10
        )

        # Publish tracked survivors
        self.pub_tracked = self.create_publisher(
            SurvivorArray,
            '/tracked_survivors',
            10
        )

        self.get_logger().info("NIDAR Tracker Node initialized successfully.")

    def parameters_callback(self, params):
        """
        Callback triggered dynamically when ROS 2 parameters are updated.
        """
        for param in params:
            if param.name == 'track_buffer':
                self.track_buffer = param.value
                self.tracker.args.track_buffer = self.track_buffer
                self.tracker.max_frames_lost = self.track_buffer
                self.get_logger().info(f"Updated track_buffer to: {self.track_buffer}")
            elif param.name == 'match_thresh':
                self.match_thresh = param.value
                self.tracker.args.match_thresh = self.match_thresh
                self.get_logger().info(f"Updated match_thresh to: {self.match_thresh}")
            elif param.name == 'track_high_thresh':
                self.track_high_thresh = param.value
                self.tracker.args.track_high_thresh = self.track_high_thresh
                self.get_logger().info(f"Updated track_high_thresh to: {self.track_high_thresh}")
            elif param.name == 'track_low_thresh':
                self.track_low_thresh = param.value
                self.tracker.args.track_low_thresh = self.track_low_thresh
                self.get_logger().info(f"Updated track_low_thresh to: {self.track_low_thresh}")
            elif param.name == 'new_track_thresh':
                self.new_track_thresh = param.value
                self.tracker.args.new_track_thresh = self.new_track_thresh
                self.get_logger().info(f"Updated new_track_thresh to: {self.new_track_thresh}")
            elif param.name == 'fuse_score':
                self.fuse_score = param.value
                self.tracker.args.fuse_score = self.fuse_score
                self.get_logger().info(f"Updated fuse_score to: {self.fuse_score}")

        return SetParametersResult(successful=True)

    def detections_callback(self, msg: SurvivorArray):
        """
        Processes incoming survivor detections, runs ByteTrack, logs state transitions,
        and publishes tracked survivor arrays.
        """
        # If there are no detections, we still need to update the tracker
        # so it can predict lost tracklets and increment the frame counter.
        xywh_list = []
        conf_list = []
        cls_list = []

        for det in msg.detections:
            # ------------------------------------------------------------------
            # Coordinate conversion:
            # Bounding box in SurvivorDetection is [x_min, y_min, width, height].
            # ByteTrack expects center format [x_center, y_center, width, height].
            # ------------------------------------------------------------------
            x_min, y_min, w, h = det.bbox
            x_center = x_min + w / 2.0
            y_center = y_min + h / 2.0
            
            xywh_list.append([x_center, y_center, w, h])
            conf_list.append(det.confidence)
            # Default to class index 0 (person/survivor)
            cls_list.append(0)

        # Convert to numpy arrays
        xywh_arr = np.array(xywh_list, dtype=np.float32).reshape(-1, 4)
        conf_arr = np.array(conf_list, dtype=np.float32).reshape(-1)
        cls_arr = np.array(cls_list, dtype=np.int32).reshape(-1)

        # Wrap into TrackerDetections for ByteTrack
        detections = TrackerDetections(xywh_arr, conf_arr, cls_arr)

        # ----------------------------------------------------------------------
        # Update Tracker
        # ----------------------------------------------------------------------
        # ByteTracker.update returns active tracked objects in format:
        # [ [x_min, y_min, x_max, y_max, track_id, score, cls, idx], ... ]
        tracked_outputs = self.tracker.update(detections)

        # ----------------------------------------------------------------------
        # State transitions and logging
        # ----------------------------------------------------------------------
        current_active_track_ids = set()
        if len(tracked_outputs) > 0:
            current_active_track_ids = set(tracked_outputs[:, 4].astype(int))

        # Retrieve current lost tracks from ByteTracker's pool
        current_lost_track_ids = {t.track_id for t in self.tracker.lost_stracks}

        # Transition Logic:
        # 1. New track created: Track ID is active, but was not active or lost in previous frame.
        new_tracks = current_active_track_ids - self.prev_active_track_ids - self.prev_lost_track_ids
        # 2. Track re-matched: Track ID is active, and was in lost pool in previous frame.
        #    NOTE: track_buffer determines the lifespan of a lost track. If an object is occluded
        #    or the drone turns away, it remains in the lost pool for up to `track_buffer` frames.
        #    If it reappears before this limit, it is successfully re-matched with the same ID.
        #    If the limit is exceeded, it gets removed, causing an ID switch when it reappears.
        rematched_tracks = current_active_track_ids & self.prev_lost_track_ids
        # 3. Track lost: Track ID was active in previous frame, but is now in the lost pool.
        lost_tracks = (self.prev_active_track_ids - current_active_track_ids) & current_lost_track_ids
        # 4. Track removed (Optional/Clean debug): Track ID has timed out of lost pool and is deleted.
        removed_tracks = (self.prev_active_track_ids | self.prev_lost_track_ids) - (current_active_track_ids | current_lost_track_ids)

        # Log changes using the rclpy logger
        for track_id in new_tracks:
            self.get_logger().info(f"New track created: ID {track_id}")

        for track_id in rematched_tracks:
            self.get_logger().info(f"Track re-matched: ID {track_id}")

        for track_id in lost_tracks:
            self.get_logger().warn(f"Track lost (occlusion/out-of-view): ID {track_id}")

        for track_id in removed_tracks:
            self.get_logger().info(f"Track deleted (exceeded track_buffer): ID {track_id}")

        # Update previous frame states
        self.prev_active_track_ids = current_active_track_ids
        self.prev_lost_track_ids = current_lost_track_ids

        # ----------------------------------------------------------------------
        # Construct and Publish Tracked Message
        # ----------------------------------------------------------------------
        out_msg = SurvivorArray()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = msg.header.frame_id

        for track_res in tracked_outputs:
            x_min, y_min, x_max, y_max, track_id, score, _, idx = track_res
            idx = int(idx)

            # Construct tracked SurvivorDetection
            tracked_det = SurvivorDetection()
            tracked_det.survivor_id = int(track_id)
            tracked_det.confidence = float(score)
            
            # Map tracking output coordinates back to [x_min, y_min, width, height]
            w_tracked = x_max - x_min
            h_tracked = y_max - y_min
            tracked_det.bbox = [float(x_min), float(y_min), float(w_tracked), float(h_tracked)]
            
            # Retain original properties if the detection index matches
            if 0 <= idx < len(msg.detections):
                orig_det = msg.detections[idx]
                tracked_det.position = orig_det.position
                tracked_det.detection_source = orig_det.detection_source
                tracked_det.detection_stamp = orig_det.detection_stamp
                tracked_det.is_confirmed = True
            else:
                # Fallback defaults if index is out of bounds
                tracked_det.detection_source = "tracked"
                tracked_det.detection_stamp = msg.header.stamp
                tracked_det.is_confirmed = True

            out_msg.detections.append(tracked_det)

        # Publish the array of tracked survivors
        self.pub_tracked.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down NIDAR Tracker Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
