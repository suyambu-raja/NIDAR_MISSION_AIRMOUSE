#!/usr/bin/env python3
"""
corridor_classifier_node.py — NIDAR AirMouse: Corridor & Room Semantic Classifier Node
======================================================================================
Subscribes to SLAM /map (OccupancyGrid) and labels connected regions as:
  - "room" (roughly square, ~2m x 2m)
  - "corridor" (elongated, width ~1m, length >> width)
  - "junction" (multi-way corridor / room intersection)
  - "unclassified" (ambiguous / partial geometry)

Publishes labeled regions on /map_regions (std_msgs/String JSON, ~1 Hz) for GCS MapView.
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from corridor_classifier.region_classifier import RegionClassifier


# QoS Profiles matching docs/interfaces.md
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

STATUS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class CorridorClassifierNode(Node):
    """
    ROS 2 Node that classifies SLAM occupancy grid regions into rooms, corridors, and junctions.
    """

    def __init__(self) -> None:
        super().__init__("corridor_classifier_node")

        # ---------------------------------------------------------------------
        # Parameters (Configurable via launch / CLI / YAML)
        # ---------------------------------------------------------------------
        self.declare_parameter("room_min_area_sqm", 2.5)
        self.declare_parameter("room_max_area_sqm", 6.0)
        self.declare_parameter("room_min_aspect_ratio", 0.70)
        self.declare_parameter("room_max_aspect_ratio", 1.45)
        self.declare_parameter("corridor_min_aspect_ratio", 1.8)
        self.declare_parameter("corridor_min_width_m", 0.75)
        self.declare_parameter("corridor_max_width_m", 2.2)
        self.declare_parameter("corridor_min_area_sqm", 2.0)
        self.declare_parameter("min_region_cells", 20)
        self.declare_parameter("publish_rate_hz", 1.0)

        self._room_min_area = float(self.get_parameter("room_min_area_sqm").value)
        self._room_max_area = float(self.get_parameter("room_max_area_sqm").value)
        self._room_min_aspect = float(self.get_parameter("room_min_aspect_ratio").value)
        self._room_max_aspect = float(self.get_parameter("room_max_aspect_ratio").value)
        self._corridor_min_aspect = float(self.get_parameter("corridor_min_aspect_ratio").value)
        self._corridor_min_width = float(self.get_parameter("corridor_min_width_m").value)
        self._corridor_max_width = float(self.get_parameter("corridor_max_width_m").value)
        self._corridor_min_area = float(self.get_parameter("corridor_min_area_sqm").value)
        self._min_cells = int(self.get_parameter("min_region_cells").value)
        self._publish_rate = float(self.get_parameter("publish_rate_hz").value)

        # Initialize geometric classifier engine
        self._classifier = RegionClassifier(
            room_min_area_sqm=self._room_min_area,
            room_max_area_sqm=self._room_max_area,
            room_min_aspect_ratio=self._room_min_aspect,
            room_max_aspect_ratio=self._room_max_aspect,
            corridor_min_aspect_ratio=self._corridor_min_aspect,
            corridor_min_width_m=self._corridor_min_width,
            corridor_max_width_m=self._corridor_max_width,
            corridor_min_area_sqm=self._corridor_min_area,
            min_region_cells=self._min_cells,
        )

        self._latest_map = None
        self._last_classified_regions = []

        # ---------------------------------------------------------------------
        # Subscriptions & Publishers
        # ---------------------------------------------------------------------
        self._sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._on_map, MAP_QOS
        )
        self._pub_map_regions = self.create_publisher(
            String, "/map_regions", STATUS_QOS
        )

        # Periodic classification & publication timer
        timer_period = 1.0 / max(0.1, self._publish_rate)
        self._timer = self.create_timer(timer_period, self._process_and_publish)

        self.get_logger().info(
            f"corridor_classifier_node initialized | Room: [{self._room_min_area}-{self._room_max_area}]m² | "
            f"Corridor min aspect: {self._corridor_min_aspect} | Rate: {self._publish_rate}Hz"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Stores latest OccupancyGrid."""
        self._latest_map = msg

    def _process_and_publish(self) -> None:
        """Runs geometric segmentation on map and publishes JSON payload on /map_regions."""
        if self._latest_map is None:
            return

        try:
            regions = self._classifier.segment_and_classify(self._latest_map)
            self._last_classified_regions = regions

            payload = {
                "timestamp": self.get_clock().now().to_msg().sec + (self.get_clock().now().to_msg().nanosec * 1e-9),
                "total_regions": len(regions),
                "rooms_count": sum(1 for r in regions if r["type"] == "room"),
                "corridors_count": sum(1 for r in regions if r["type"] == "corridor"),
                "junctions_count": sum(1 for r in regions if r["type"] == "junction"),
                "unclassified_count": sum(1 for r in regions if r["type"] == "unclassified"),
                "regions": regions,
            }

            msg = String()
            msg.data = json.dumps(payload)
            self._pub_map_regions.publish(msg)

        except Exception as exc:
            self.get_logger().error(f"Region classification error: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
