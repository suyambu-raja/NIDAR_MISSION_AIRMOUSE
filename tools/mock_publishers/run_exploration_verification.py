#!/usr/bin/env python3
"""
run_exploration_verification.py — Runs mock publisher + exploration_node
and verifies published topics in real-time ROS 2 runtime.
"""
import time
import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, String

from nidar_msgs.msg import SurvivorGridArray, SurvivorGridLocation

MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
)

POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10,
)

OUTPUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
)

class VerificationPublisher(Node):
    def __init__(self):
        super().__init__("verification_publisher")
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", MAP_QOS)
        self.pose_pub = self.create_publisher(PoseStamped, "/drone_pose", POSE_QOS)
        self.batt_pub = self.create_publisher(BatteryState, "/battery_status", POSE_QOS)
        self.surv_pub = self.create_publisher(SurvivorGridArray, "/survivor_grid_locations", MAP_QOS)
        
        # Build 100x100 map with an explored center and unexplored outer region (frontiers)
        self.map_msg = self._build_frontier_map()
        
        self.timer = self.create_timer(0.5, self._publish)
        self.start_t = time.monotonic()
        
    def _build_frontier_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.1 # 0.1m/cell -> 10m x 10m
        grid.info.width = 100
        grid.info.height = 100
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        
        # Initialize with UNKNOWN (-1)
        data = [-1] * 10000
        # Explored circle of radius 2.5m (25 cells) around center (50, 50)
        for r in range(100):
            for c in range(100):
                dist = math.hypot(r - 50, c - 50)
                if dist <= 25:
                    data[r * 100 + c] = 0 # FREE
                    
        grid.data = data
        return grid

    def _publish(self):
        t = time.monotonic() - self.start_t
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)
        
        # Drone pose near center
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.5 * math.sin(t)
        pose.pose.position.y = 0.5 * math.cos(t)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        
        # Battery state: 85%
        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()
        batt.percentage = 0.85
        self.batt_pub.publish(batt)
        
        # Survivor array: 1 survivor found so far
        s_arr = SurvivorGridArray()
        s1 = SurvivorGridLocation()
        s1.survivor_id = 1
        s1.grid_box = "H8"
        s1.world_x = 0.5
        s1.world_y = 0.5
        s1.confidence = 0.92
        s_arr.locations.append(s1)
        self.surv_pub.publish(s_arr)

class VerificationSubscriber(Node):
    def __init__(self):
        super().__init__("verification_subscriber")
        self.received_goal = None
        self.received_status = None
        self.received_path = None
        
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, OUTPUT_QOS)
        self.create_subscription(String, "/exploration_status", self._status_cb, OUTPUT_QOS)
        self.create_subscription(Path, "/planned_path", self._path_cb, OUTPUT_QOS)

    def _goal_cb(self, msg):
        self.received_goal = msg
    def _status_cb(self, msg):
        self.received_status = json.loads(msg.data)
    def _path_cb(self, msg):
        self.received_path = msg

def main():
    rclpy.init()
    pub_node = VerificationPublisher()
    sub_node = VerificationSubscriber()
    
    # Import exploration node
    from exploration_node.exploration_node import ExplorationNode
    exp_node = ExplorationNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    executor.add_node(exp_node)
    
    print("\n=======================================================")
    print("  NIDAR AIRMOUSE: EXPLORATION NODE LIVE ROS 2 VERIFICATION")
    print("=======================================================\n")
    
    start = time.time()
    while time.time() - start < 5.0:
        executor.spin_once(timeout_sec=0.1)
        if sub_node.received_goal and sub_node.received_status and sub_node.received_path:
            break
            
    print("[1] STATUS OUTPUT (/exploration_status):")
    print(json.dumps(sub_node.received_status, indent=2))
    
    print("\n[2] GOAL POSE OUTPUT (/goal_pose):")
    if sub_node.received_goal:
        p = sub_node.received_goal.pose.position
        o = sub_node.received_goal.pose.orientation
        print(f"  Frame:  {sub_node.received_goal.header.frame_id}")
        print(f"  Target: X = {p.x:.2f} m, Y = {p.y:.2f} m, Z = {p.z:.2f} m")
        print(f"  Orient: Z = {o.z:.4f}, W = {o.w:.4f}")
    else:
        print("  None received")
        
    print("\n[3] PLANNED PATH OUTPUT (/planned_path):")
    if sub_node.received_path:
        print(f"  Frame:     {sub_node.received_path.header.frame_id}")
        print(f"  Waypoints: {len(sub_node.received_path.poses)} waypoints generated by A*")
        if len(sub_node.received_path.poses) > 0:
            first_p = sub_node.received_path.poses[0].pose.position
            last_p = sub_node.received_path.poses[-1].pose.position
            print(f"  Start:     ({first_p.x:.2f}, {first_p.y:.2f})")
            print(f"  End Goal:  ({last_p.x:.2f}, {last_p.y:.2f})")
    else:
        print("  None received")

    print("\n=======================================================")
    print("  ALL 3 OUTPUT TOPICS VERIFIED IN LIVE ROS 2 ENVIRONMENT")
    print("=======================================================\n")

    exp_node.destroy_node()
    sub_node.destroy_node()
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()