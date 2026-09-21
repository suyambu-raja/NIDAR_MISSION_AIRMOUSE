# NIDAR M2 — Complete Software Inventory

## 1. System Inventory Table

| Subsystem / Capability | Node / Module | Implementation Status | ROS 2 Interface | GCS Interface | Tested & Verified |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **RGB Object Detection** | `detection_node` / YOLOv8 | Verified Active | `/camera/image_raw` (`sensor_msgs/Image`) | `camera_frame` (RGB feed) | **YES** (Test 1) |
| **Survivor Detection** | `detection_node` / `tracker_node` | Verified Active | `/tracked_survivors` (`nidar_msgs/SurvivorArray`) | `survivor_detected` | **YES** (Test 1) |
| **ByteTrack Trajectory** | `tracker_node` (`test_byte_tracker.py`) | Verified Active | `/tracked_survivors` (track continuity) | Persistent Track IDs (`S01`..`S06`) | **YES** (Test 1, 3) |
| **Thermal Detection** | `thermal_node` / YOLO11n | Verified Active | `/thermal/image_raw` (`sensor_msgs/Image`) | `camera_frame` (Thermal FLIR) | **YES** (Test 2) |
| **Thermal Radiometric Processing** | `thermal_node` / `video_bridge` | Verified Active | Colormap / Radiometric normalization | FLIR Palette rendering | **YES** (Test 2) |
| **RGB + Thermal Fusion** | `fusion_node` | Verified Active | `/confirmed_survivors` (`nidar_msgs/SurvivorArray`) | Multi-modal badge (`RGB+THERMAL`) | **YES** (Test 3) |
| **Survivor Confidence Fusion** | `fusion_node` (`SpatialAligner`) | Verified Active | Running EMA Confidence calculation | Percentage Score (`94%`) | **YES** (Test 3) |
| **Survivor Deduplication** | `fusion_node` (`Deduplicator`) | Single Owner (Authoritative) | Euclidean 0.5m gate, 6 max survivors | Deduplicated cards (`S01`..`S06+`) | **YES** (Audit & E2E) |
| **Survivor Localization (Arena Grid)**| `grid_mapper_node` (`SurvivorRegistry`) | Verified Active | `/survivor_grid_locations` (`nidar_msgs/SurvivorGridArray`) | Discrete Grid Cell (`F4`) | **YES** (Test 4) |
| **2D LiDAR SLAM** | `slam_node` / `slam_toolbox` | Verified Active | `/map` (`nav_msgs/OccupancyGrid`, 0.05m) | `map_update` (2D Canvas SLAM) | **YES** (Test 5) |
| **GPS-Denied Odometry & TF** | `slam_node` (`map` $\to$ `base_link`) | Verified Active | `/drone_pose` (`geometry_msgs/PoseStamped`) | Vehicle Pose & Heading Vector | **YES** (Test 6) |
| **EKF State Estimation** | Pixhawk / `mavros_bridge` | Verified Active | `/mavros/local_position/pose` | Live Kinematics (Alt, Vel, Att) | **YES** (Test 6) |
| **Frontier Detection & Ranking** | `exploration_node` (`FrontierDetector`) | Verified Active | BFS Frontier Clusters & Information Gain | Active Frontier Centroids | **YES** (Test 7) |
| **Corridor Semantic Classification** | `corridor_classifier` | Single Owner (Authoritative) | `/map_regions` (`std_msgs/String` JSON) | `map_regions` (Topology Polygons) | **YES** (Audit & E2E) |
| **Exploration State Machine** | `exploration_node` | Verified Active | `/exploration_status` (`std_msgs/String` JSON) | `mission_status` (Progress, Clock)| **YES** (Test 7) |
| **Authoritative A\* Path Planner** | `exploration_node.path_planner` | Single Owner (Authoritative) | `/planned_path` (`nav_msgs/Path`) | Planned Path Trajectory (Cyan) | **YES** (Test 8) |
| **Obstacle Avoidance / DWA** | `failsafe_node` / local navigation | Verified Active | `/failsafe/status` (Obstacle distance) | Clearance readout (`Obstacle: 1.25m`)| **YES** (Test 9) |
| **Emergency Obstacle Braking** | `failsafe_node` | Verified Active | Reactive zero-velocity threshold ($0.45\text{m}$) | Failsafe Banner (`EMERGENCY BRAKE`)| **YES** (Test 10) |
| **Flight Control Bridge (MAVROS)** | `mavros_bridge` | Verified Active | `/mavros/cmd/arming`, `/mavros/set_mode` | Motor armed status & mode tags | **YES** (Test 11, 12) |
| **GUIDED_NOGPS Flight Mode** | ArduPilot / `mavros_bridge` | Verified Active | `/mavros/setpoint_position/local` | GPS-Denied status indicator | **YES** (Test 6, 11) |
| **Multi-Stage Failsafe Supervisor** | `failsafe_node` | Verified Active | `/failsafe/status`, `/failsafe/planned_path` | Emergency Alert Banner & Safe Path | **YES** (Test 10, 15) |
| **Battery Monitoring & Alarm** | `mavros_bridge` / `failsafe_node` | Verified Active | `/battery_status` (`sensor_msgs/BatteryState`) | Battery Bar, Voltage, Low Alarm | **YES** (Test 14) |
| **Communication Watchdog (3s)** | GCS Client & Server Engine | Verified Active | WebSocket Heartbeat & Ring Buffer | Connection Loss Warning Banner | **YES** (Test 13) |
| **Command Bridge & Interlocks** | `command_bridge` | Verified Active | `/mavros/cmd/arming`, `/mavros/cmd/land` | Operator Flight Override Modal | **YES** (Test 11, 12) |
| **Tactical Rescue Mission GCS** | `Custom GCS` (HTML5/Canvas/CSS/JS) | Fully Integrated | WebSockets (Port 8765), HTTP (Port 8080) | 3-Column Rescue Mission Terminal | **YES** (All 46 GCS tests) |
