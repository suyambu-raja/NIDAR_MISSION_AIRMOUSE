#!/usr/bin/env python3
"""
grid_mapper_node.py — NIDAR AirMouse: Grid Mapper ROS 2 Node
=============================================================
Main ROS 2 node for the grid_mapper_node package.

What this node does
-------------------
1. Subscribes to /drone_pose, /map, /tracked_survivors
2. Converts each confirmed survivor's SLAM position → arena grid box ID
   using CoordinateTransformer (e.g. (6.3m, 4.7m) → "F4")
3. Deduplicates survivors using Euclidean Distance Clustering (SurvivorRegistry)
4. Publishes /survivor_grid_locations (nidar_msgs/SurvivorGridArray) @ ~2 Hz
5. Publishes /grid_map_overlay (nav_msgs/OccupancyGrid) @ ~2 Hz

Topic names are exactly as defined in docs/interfaces.md.

Running with mock hardware
--------------------------
    # Terminal 1:
    python3 tools/mock_publishers/mock_slam.py
    # Terminal 2:
    ros2 run grid_mapper_node grid_mapper

For full build instructions see docs/interfaces.md and the deliverables
section at the bottom of this file.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

# Standard ROS 2 messages
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

# Custom NIDAR messages
from nidar_msgs.msg import SurvivorArray, SurvivorGridArray, SurvivorGridLocation

# Local utility classes (same package)
from grid_mapper_node.coordinate_transformer import CoordinateTransformer
from grid_mapper_node.survivor_registry import SurvivorRegistry
from grid_mapper_node.grid_visualizer import GridVisualizer


# =============================================================================
# QoS Profiles — must match publishers per docs/interfaces.md
# =============================================================================

# /map is TRANSIENT_LOCAL so we receive the last map even if we start late
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# /drone_pose is BEST_EFFORT + VOLATILE (low latency)
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# /tracked_survivors — RELIABLE so we never miss a confirmed detection
SURVIVOR_SUB_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Output topics: RELIABLE so GCS dashboard doesn't miss updates
OUTPUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# =============================================================================
# Node constants
# =============================================================================

# Publish rate for both output topics (matches /map rate as per interfaces.md)
PUBLISH_RATE_HZ: float = 2.0

# Coordinate frame for all position data
MAP_FRAME: str = "map"

# Flag: calibrate grid origin automatically from the first drone pose received.
# In a real mission this would be triggered by a "mission start" service call,
# but for development we auto-calibrate on first pose.
AUTO_CALIBRATE_ORIGIN: bool = True


class GridMapperNode(Node):
    """
    ROS 2 node: receives SLAM data + survivor tracks → publishes grid map.

    Lifecycle
    ---------
    __init__         → create pubs/subs/timer, instantiate utility classes
    _map_callback    → store latest /map, update transformer map origin
    _pose_callback   → store latest /drone_pose, optionally calibrate origin
    _survivors_cb    → process /tracked_survivors, update registry
    _publish_timer   → publish /survivor_grid_locations + /grid_map_overlay
    """

    def __init__(self) -> None:
        super().__init__("grid_mapper_node")

        # ------------------------------------------------------------------ #
        # Utility class instances                                             #
        # ------------------------------------------------------------------ #

        # CoordinateTransformer converts (X,Y) → "F4" grid box ID.
        # Grid origin starts at (0,0) in the SLAM map frame; we update it
        # once the drone's first pose arrives (if AUTO_CALIBRATE_ORIGIN=True).
        self._transformer = CoordinateTransformer(
            grid_origin_x=0.0,
            grid_origin_y=0.0,
            map_origin_x=0.0,
            map_origin_y=0.0,
        )

        # SurvivorRegistry deduplicates detections and caps at 6 survivors.
        self._registry = SurvivorRegistry()

        # GridVisualizer overlays 1m grid lines + survivor markers on /map.
        self._visualizer = GridVisualizer(
            grid_box_size_m=1.0,
            arena_size_m=15.0,
        )

        # ------------------------------------------------------------------ #
        # State: latest received messages                                     #
        # ------------------------------------------------------------------ #

        # Most recent /map OccupancyGrid (None until first map arrives)
        self._latest_map: OccupancyGrid | None = None

        # Most recent /drone_pose (None until first pose arrives)
        self._latest_pose: PoseStamped | None = None

        # True once grid origin has been calibrated from drone pose
        self._origin_calibrated: bool = False

        # ------------------------------------------------------------------ #
        # Subscribers                                                         #
        # ------------------------------------------------------------------ #

        # /map — TRANSIENT_LOCAL so we get the map even if we start after SLAM
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            MAP_QOS,
        )

        # /drone_pose — BEST_EFFORT, high frequency pose
        self._pose_sub = self.create_subscription(
            PoseStamped,
            "/drone_pose",
            self._pose_callback,
            POSE_QOS,
        )

        # /tracked_survivors — confirmed survivor detections from tracker_node
        self._survivors_sub = self.create_subscription(
            SurvivorArray,
            "/tracked_survivors",
            self._survivors_callback,
            SURVIVOR_SUB_QOS,
        )

        # ------------------------------------------------------------------ #
        # Publishers                                                          #
        # ------------------------------------------------------------------ #

        # /survivor_grid_locations — all confirmed survivors with grid box IDs
        self._grid_locs_pub = self.create_publisher(
            SurvivorGridArray,
            "/survivor_grid_locations",
            OUTPUT_QOS,
        )

        # /grid_map_overlay — /map with grid lines + survivor markers
        self._overlay_pub = self.create_publisher(
            OccupancyGrid,
            "/grid_map_overlay",
            OUTPUT_QOS,
        )

        # ------------------------------------------------------------------ #
        # Publish timer — fires at PUBLISH_RATE_HZ (2 Hz)                   #
        # ------------------------------------------------------------------ #
        self._pub_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self._publish_timer_callback,
        )

        self.get_logger().info(
            "grid_mapper_node started — subscribing to /map, /drone_pose, "
            "/tracked_survivors; publishing /survivor_grid_locations, /grid_map_overlay"
        )

    # =========================================================================
    # Subscriber callbacks
    # =========================================================================

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """
        Receive a new OccupancyGrid from /map (from slam_node or mock_slam.py).

        Actions:
        - Store the map for use in the publish timer.
        - Update the transformer with the new map origin (slam_toolbox can
          shift the origin as the map grows during exploration).
        """
        self._latest_map = msg

        # Keep the transformer's map origin in sync with the OccupancyGrid.
        # This accounts for the mock_slam.py offset of (-7.5, -7.5).
        self._transformer.set_map_origin(
            map_origin_x=msg.info.origin.position.x,
            map_origin_y=msg.info.origin.position.y,
        )

        self.get_logger().debug(
            f"Map received: {msg.info.width}×{msg.info.height} cells, "
            f"origin=({msg.info.origin.position.x:.2f}, "
            f"{msg.info.origin.position.y:.2f})"
        )

    def _pose_callback(self, msg: PoseStamped) -> None:
        """
        Receive a new drone pose from /drone_pose.

        Actions:
        - Store the pose for optional use (future: camera→map transform).
        - If AUTO_CALIBRATE_ORIGIN=True and origin hasn't been calibrated yet,
          use the first pose received as the arena entry point (grid origin).
        """
        self._latest_pose = msg

        # Auto-calibrate grid origin from the first drone pose.
        # In a real mission this would be triggered by a mission-start event.
        if AUTO_CALIBRATE_ORIGIN and not self._origin_calibrated:
            drone_x = msg.pose.position.x
            drone_y = msg.pose.position.y
            self._transformer.set_grid_origin(drone_x, drone_y)
            self._origin_calibrated = True
            self.get_logger().info(
                f"Grid origin calibrated at drone pose: "
                f"({drone_x:.3f}, {drone_y:.3f}) — this is arena box A1."
            )

    def _survivors_callback(self, msg: SurvivorArray) -> None:
        """
        Receive a batch of survivor detections from /tracked_survivors.

        Only processes detections where is_confirmed=True.

        For each confirmed detection:
        1. Extract world position from SurvivorDetection.position (map frame).
        2. Convert to grid box ID via CoordinateTransformer.
        3. Register with SurvivorRegistry (dedup + cap enforcement).
        4. Log new survivors.
        """
        if not msg.detections:
            return

        newly_added: int = 0

        for det in msg.detections:
            # Only process detections that tracker_node has confirmed
            if not det.is_confirmed:
                continue

            # Extract world position from the detection (map frame)
            world_x: float = det.position.x
            world_y: float = det.position.y
            confidence: float = float(det.confidence)

            # Convert world position → grid box ID (e.g. "F4")
            grid_box = self._transformer.world_to_grid(world_x, world_y)

            if grid_box is None:
                # Survivor position is outside the 15×15 m arena bounds
                self.get_logger().warn(
                    f"Survivor id={det.survivor_id} at ({world_x:.2f}, {world_y:.2f}) "
                    f"is outside arena bounds — skipping grid assignment."
                )

            # Register with dedup registry (grid_box may be None — registry handles it)
            count_before = self._registry.count()
            record = self._registry.add_or_update(
                world_x=world_x,
                world_y=world_y,
                confidence=confidence,
                grid_box=grid_box,
                tracker_id=det.survivor_id,
            )

            if record is not None and self._registry.count() > count_before:
                # A new unique survivor was added
                newly_added += 1
                self.get_logger().info(
                    f"NEW survivor registered — id={record.survivor_id}, "
                    f"grid={record.grid_box}, "
                    f"pos=({record.world_x:.2f}, {record.world_y:.2f}), "
                    f"conf={record.confidence:.2f}"
                )
            elif record is not None:
                # Existing survivor confidence/position updated
                self.get_logger().debug(
                    f"Survivor id={record.survivor_id} updated — "
                    f"grid={record.grid_box}, detections={record.detection_count}"
                )

        if self._registry.is_full() and newly_added > 0:
            self.get_logger().warn(
                "⚠️  Maximum survivor limit (6) reached — "
                "further new detections will be ignored."
            )

    # =========================================================================
    # Publish timer callback
    # =========================================================================

    def _publish_timer_callback(self) -> None:
        """
        Fires at 2 Hz — publishes both output topics.

        Skips publication if no map has been received yet (avoids publishing
        empty/invalid data at startup).
        """
        if self._latest_map is None:
            # Wait for the first /map before publishing anything
            self.get_logger().debug("Waiting for /map before publishing...")
            return

        # Publish survivor grid locations
        self._publish_survivor_locations()

        # Publish grid map overlay (requires a valid map)
        self._publish_grid_overlay()

    # =========================================================================
    # Publisher helpers
    # =========================================================================

    def _publish_survivor_locations(self) -> None:
        """
        Build and publish a SurvivorGridArray message on /survivor_grid_locations.

        Contains ALL confirmed survivors found so far (running list, not just
        the latest batch from tracker_node).
        """
        msg = SurvivorGridArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME

        # Convert each SurvivorRecord → SurvivorGridLocation message
        for record in self._registry.get_all():
            loc = SurvivorGridLocation()
            loc.survivor_id = record.survivor_id
            loc.grid_box = record.grid_box if record.grid_box else "??"
            loc.world_x = float(record.world_x)
            loc.world_y = float(record.world_y)
            loc.confidence = float(record.confidence)
            msg.locations.append(loc)

        self._grid_locs_pub.publish(msg)

        self.get_logger().debug(
            f"Published /survivor_grid_locations: {len(msg.locations)} survivors"
        )

    def _publish_grid_overlay(self) -> None:
        """
        Build and publish a grid-line-overlaid OccupancyGrid on /grid_map_overlay.

        Steps:
        1. Collect world positions of all confirmed survivors for marking.
        2. Call GridVisualizer.render() with the latest base map.
        3. Update the overlay header stamp and publish.
        """
        # Collect survivor world positions for the visualizer
        survivor_positions = [
            (r.world_x, r.world_y)
            for r in self._registry.get_all()
            if r.grid_box is not None
        ]

        # Render the overlay (deep-copied + grid lines + survivor markers)
        try:
            overlay = self._visualizer.render(
                base_map=self._latest_map,
                survivor_world_positions=survivor_positions,
                grid_origin_x=self._transformer.grid_origin_x,
                grid_origin_y=self._transformer.grid_origin_y,
            )
        except Exception as exc:
            # Never crash the publish loop — log the error and skip this cycle
            self.get_logger().error(
                f"GridVisualizer.render() failed: {exc} — skipping overlay publish."
            )
            return

        # Freshen the header timestamp before publishing
        overlay.header.stamp = self.get_clock().now().to_msg()
        overlay.header.frame_id = MAP_FRAME

        self._overlay_pub.publish(overlay)

        self.get_logger().debug("Published /grid_map_overlay")


# =============================================================================
# Entry point
# =============================================================================

def main(args=None) -> None:
    """
    ROS 2 entry point.

    Run with:
        ros2 run grid_mapper_node grid_mapper
    """
    rclpy.init(args=args)
    node = GridMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("grid_mapper_node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
