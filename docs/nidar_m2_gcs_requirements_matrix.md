# NIDAR M2 — GCS Requirements Traceability Matrix

## 1. Traceability Methodology

Every functional requirement for the NIDAR M2 GPS-denied autonomous rescue drone is mapped across its complete end-to-end operational pipeline:
$$\text{Requirement} \longrightarrow \text{Implementation} \longrightarrow \text{ROS 2 Interface} \longrightarrow \text{Bridge} \longrightarrow \text{GCS Data Model} \longrightarrow \text{GCS UI} \longrightarrow \text{Verification Test}$$

If any link in the chain is incomplete, the capability is flagged as `NOT INTEGRATED`.

---

## 2. Requirements Traceability Matrix

### REQ-01: RGB Survivor Detection
* **Requirement**: Detect human victims in daylight/illuminated indoor environments.
* **Implementation**: YOLOv8 neural network inference inside `detection_node`.
* **ROS 2 Interface**: `/camera/image_raw` (`sensor_msgs/Image`), `/camera_frame`.
* **Bridge**: `VideoBridgeNode.process_rgb_image` (Base64 JPEG conversion).
* **GCS Data Model**: `camera_frame` payload with `camera_id="rgb"`.
* **GCS UI**: Live RGB camera viewport (`#pane-rgb-feed`, `#img-rgb-stream`).
* **Verification Test**: `test_01_rgb_detection_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-02: Thermal Heat-Signature Detection
* **Requirement**: Detect victims in zero-lux, smoke, or obscured disaster rubble using thermal signatures.
* **Implementation**: YOLO11n radiometric detection inside `thermal_node`.
* **ROS 2 Interface**: `/thermal/image_raw` (`sensor_msgs/Image`), `/thermal_frame`.
* **Bridge**: `VideoBridgeNode.process_thermal_image` (Inferno colormap + JPEG encoding).
* **GCS Data Model**: `camera_frame` payload with `camera_id="thermal"`.
* **GCS UI**: Live Thermal camera viewport (`#pane-thermal-feed`, `#img-thermal-stream`).
* **Verification Test**: `test_02_thermal_detection_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-03: Multi-Modal Fusion & Persistent Tracking
* **Requirement**: Associate RGB and thermal detections, calculate confidence, and assign persistent survivor IDs.
* **Implementation**: `fusion_node` (`SpatialAligner`, `Deduplicator`).
* **ROS 2 Interface**: `/confirmed_survivors` (`nidar_msgs/SurvivorArray`).
* **Bridge**: `ROS2BridgeNode.convert_confirmed_survivors`.
* **GCS Data Model**: `survivor_detected` / `survivor_update` JSON events.
* **GCS UI**: Dedicated Survivor Panel (`S01`..`S06+` cards, modality badges).
* **Verification Test**: `test_03_survivor_fusion_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-04: Arena 1-Meter Search Grid Localization
* **Requirement**: Localize confirmed victims into discrete 1-meter arena search cells (e.g. `F4`).
* **Implementation**: `grid_mapper_node` (`CoordinateTransformer`).
* **ROS 2 Interface**: `/survivor_grid_locations` (`nidar_msgs/SurvivorGridArray`).
* **Bridge**: `ROS2BridgeNode` + `GridManager.metric_to_grid`.
* **GCS Data Model**: `grid_cell` attribute attached to `survivor_detected`.
* **GCS UI**: Survivor card grid tag (`GRID: F4`) + 2D map survivor pin icon.
* **Verification Test**: `test_04_survivor_location_to_map` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-05: 2D LiDAR SLAM Occupancy Mapping
* **Requirement**: Generate a real-time $0.05\text{ m}$ occupancy grid map of unknown indoor environments.
* **Implementation**: `slam_node` / `slam_toolbox`.
* **ROS 2 Interface**: `/map` (`nav_msgs/OccupancyGrid`).
* **Bridge**: `ROS2BridgeNode.convert_map`.
* **GCS Data Model**: `map_update` payload (`occupied_cells`, `free_cells`, `resolution`).
* **GCS UI**: Center 2D interactive canvas (`#map-canvas`).
* **Verification Test**: `test_05_lidar_slam_to_gcs_map` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-06: GPS-Denied Kinematic State Estimation
* **Requirement**: Estimate drone position, altitude, velocity, and orientation without GPS.
* **Implementation**: Pixhawk EKF (Optical Flow + Laser Ranger + IMU) + `slam_node`.
* **ROS 2 Interface**: `/drone_pose` (`geometry_msgs/PoseStamped`).
* **Bridge**: `ROS2BridgeNode.convert_drone_pose`.
* **GCS Data Model**: `telemetry` payload (`x, y, z, heading, altitude, velocity`).
* **GCS UI**: Canvas drone symbol, heading needle (`#compass-needle`), attitude indicator.
* **Verification Test**: `test_06_ekf_to_drone_pose_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-07: Frontier Autonomous Exploration
* **Requirement**: Detect open frontiers, cluster them via BFS, and autonomously direct search.
* **Implementation**: `exploration_node` (`FrontierDetector`, `FrontierGrouper`).
* **ROS 2 Interface**: `/exploration_status` (`std_msgs/String` JSON).
* **Bridge**: `ROS2BridgeNode.convert_exploration_status`.
* **GCS Data Model**: `mission_status` payload (`progress`, `state`, `survivor_count`).
* **GCS UI**: Mission overview card, exploration progress bar, active frontier count.
* **Verification Test**: `test_07_frontier_exploration_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-08: Authoritative A* Path Planning
* **Requirement**: Plan Euclidean shortest collision-free path to frontiers and safe staging.
* **Implementation**: `exploration_node.path_planner.PathPlanner`.
* **ROS 2 Interface**: `/planned_path` (`nav_msgs/Path`).
* **Bridge**: `ROS2BridgeNode.convert_planned_path`.
* **GCS Data Model**: `path_overlay` array of `[x, y]` coordinates.
* **GCS UI**: Cyan path trajectory polyline drawn on `#map-canvas`.
* **Verification Test**: `test_08_astar_path_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-09: Obstacle Clearance & Local Avoidance
* **Requirement**: Monitor obstacle clearance in real time and execute dynamic avoidance.
* **Implementation**: `failsafe_node` / local navigation.
* **ROS 2 Interface**: `/failsafe/status` (`std_msgs/String` JSON).
* **Bridge**: `ROS2BridgeNode.convert_failsafe_status`.
* **GCS Data Model**: `failsafe_status` payload (`obstacle_dist`).
* **GCS UI**: Autonomy panel obstacle metric (`Obstacle: 1.25m`).
* **Verification Test**: `test_09_obstacle_avoidance_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-10: Emergency Obstacle Braking (< 0.45m)
* **Requirement**: Intercept flight controls and halt drone when an obstacle is within $0.45\text{ m}$.
* **Implementation**: `failsafe_node` emergency braking trigger.
* **ROS 2 Interface**: `/failsafe/status` (`state="BRAKE"`).
* **Bridge**: `ROS2BridgeNode.convert_failsafe_status`.
* **GCS Data Model**: High-priority alert payload dispatched to queue.
* **GCS UI**: Emergency banner (`#failsafe-alert-banner`) + brake status tag.
* **Verification Test**: `test_10_emergency_obstacle_brake_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-11: Operator Motor Arming Command
* **Requirement**: Dispatch motor arm command with safety pre-checks.
* **Implementation**: GCS `#btn-arm` $\to$ `command_bridge` $\to$ `/mavros/cmd/arming`.
* **ROS 2 Interface**: `/mavros/cmd/arming` (`std_srvs/srv/SetBool` / MAVROS Bool).
* **Bridge**: `CommandBridgeNode.handle_command`.
* **GCS Data Model**: `cmd_ack` with `action="ARM"`.
* **GCS UI**: Motor status badge switches from `DISARMED` to `ARMED`.
* **Verification Test**: `test_11_gcs_arm_to_backend_to_ardupilot` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-12: Operator Vertical Landing Command
* **Requirement**: Dispatch controlled descent and touchdown command.
* **Implementation**: GCS `#btn-land` $\to$ `command_bridge` $\to$ `/mavros/cmd/land`.
* **ROS 2 Interface**: `/mavros/set_mode` (`mode="LAND"`).
* **Bridge**: `CommandBridgeNode.handle_command`.
* **GCS Data Model**: `cmd_ack` with `action="LAND"`.
* **GCS UI**: Flight mode badge switches to `LAND`.
* **Verification Test**: `test_12_gcs_land_to_backend_to_ardupilot` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-13: Telemetry Loss Watchdog (3s Timeout)
* **Requirement**: Warn operator if communication link drops without halting onboard autonomy.
* **Implementation**: Client-side JS watchdog timer in `app.js` checking telemetry timestamp.
* **ROS 2 Interface**: WebSocket heartbeat and message stream.
* **Bridge**: `PriorityMessageQueue` connection state handling.
* **GCS Data Model**: Real-time timestamp tracking.
* **GCS UI**: Flashing warning banner (`#connection-loss-banner`).
* **Verification Test**: `test_13_communication_loss_to_gcs_watchdog` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-14: Battery Voltage Monitoring & Low Battery Alarm
* **Requirement**: Monitor battery level and raise alarm when below 20%.
* **Implementation**: `mavros_bridge` $\to$ `/battery_status` $\to$ `failsafe_node`.
* **ROS 2 Interface**: `/battery_status` (`sensor_msgs/BatteryState`).
* **Bridge**: `ROS2BridgeNode.convert_battery`.
* **GCS Data Model**: `battery_status` (`voltage`, `percentage`).
* **GCS UI**: Dynamic colored battery bar (`#battery-bar-fill`) and percentage readout.
* **Verification Test**: `test_14_battery_warning_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**

### REQ-15: Localization Degradation Safety Interlock
* **Requirement**: Alert operator and enter position-hold if optical flow/SLAM quality drops.
* **Implementation**: `failsafe_node` sensor quality monitor.
* **ROS 2 Interface**: `/failsafe/status` (`nav_ok=False`, `ekf_ok=False`).
* **Bridge**: `ROS2BridgeNode.convert_failsafe_status`.
* **GCS Data Model**: `failsafe_status` details structure.
* **GCS UI**: Failsafe banner displaying `Reason: Optical flow quality degraded`.
* **Verification Test**: `test_15_localization_degradation_to_gcs` (**PASS**).
* **Status**: **INTEGRATED & VERIFIED**
