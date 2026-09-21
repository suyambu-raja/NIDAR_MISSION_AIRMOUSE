# NIDAR M2 Autonomous Rescue Drone — Architecture Specification

## 1. System Overview

The **NIDAR M2** is an autonomous indoor rescue micro-aerial vehicle designed to operate in severe disaster scenarios (collapsed structures, earthquakes, industrial ruins) where GNSS signals are completely denied. The system discovers, localizes, tracks, and confirms human survivors using an integrated multi-modal sensor suite (RGB camera, thermal micro-bolometer, 2D LiDAR, optical flow, and rangefinder) linked to an onboard ROS 2 autonomy pipeline and a ground control station (Custom GCS).

```
 +-----------------------------------------------------------------------------------+
 |                             NIDAR M2 AVIONICS (Onboard)                           |
 |                                                                                   |
 |  [Sensors]                [Perception]                      [State Estimation]    |
 |  • 2D LiDAR (10Hz)    --> YOLOv8 RGB + ByteTrack       -->  2D LiDAR SLAM         |
 |  • RGB Camera (30Hz)  --> YOLO11n Thermal              -->  EKF (Flow+Ranger+IMU) |
 |  • Thermal Cam (9Hz)  --> Spatial Alignment (0.5m)          (0.05m Occupancy Map) |
 |  • Optical Flow (50Hz)--> Confidence Fusion (EMA)                                 |
 |  • Rangefinder (50Hz) --> Deduplicator (max 6)                                    |
 |                           (/confirmed_survivors)                                  |
 |                                    │                                 │            |
 |                                    ▼                                 ▼            |
 |                           [Mission Planning]               [Safety & Control]     |
 |                           • Frontier Detector              • Failsafe Node        |
 |                           • Frontier Grouper (BFS)         • MAVROS Bridge        |
 |                           • Corridor Classifier            • ArduPilot            |
 |                           • PathPlanner (A*)                 (GUIDED_NOGPS)       |
 |                           • Battery Strategy                                      |
 +------------------------------------│---------------------------------│-------------+
                                      │                                 │
                                      ▼                                 ▼
                     +───────────────────────────────────────────────────+
                     |         NIDAR GCS Authoritative Bridge            |
                     |  - ROS 2 Bridge (Telemetry, Map, Paths, Survivors)|
                     |  - Command Bridge (Arm, Takeoff, Land, RTL, Pause)|
                     |  - High-Speed Async WebSocket Engine (Port 8765)  |
                     +─────────────────────────▲─────────────────────────+
                                               │
                                               ▼
                     +───────────────────────────────────────────────────+
                     |                NIDAR M2 Custom GCS                |
                     |  - Real-time 2D Canvas Rescue Map (Grid, SLAM)   |
                     |  - Survivor Panel (S01..S06+, Modality, Confidence)|
                     |  - Autonomous Search Diagnostics (Frontiers, A*)  |
                     |  - GPS-Denied Avionics Health & Sensor Matrices   |
                     |  - Emergency Safety Banner & Operator Overrides   |
                     +───────────────────────────────────────────────────+
```

---

## 2. Core Functional Layers

### Layer 1: Perception & Multi-Modal Fusion
* **RGB Detection & Tracking**: YOLOv8 extracts human bounding boxes; ByteTrack assigns continuous short-term trajectory IDs.
* **Thermal Detection**: YOLO11n operates on radiometric thermal frames to detect human heat signatures regardless of zero-lux or smoky conditions.
* **Spatial Alignment & Confidence Fusion**: Matches detections within Euclidean threshold ($d \le 0.5\text{ m}$), updates running average confidence, and applies modality weights.
* **Survivor Deduplicator (`fusion_node`)**: Sole authoritative source of survivor identification. Assigns sequential integer IDs (`1..6`), rejects spatial duplicates within 0.5 m, and publishes `/confirmed_survivors`.

### Layer 2: GPS-Denied Mapping & Localization
* **2D LiDAR SLAM**: Continuously generates a $0.05\text{ m}$ resolution occupancy grid and estimated vehicle pose in the `map` coordinate frame.
* **EKF Sensor Fusion**: Fuses optical flow ($v_x, v_y$), downward time-of-flight laser rangefinder ($z$), and onboard IMU angular rates/accelerations for fast attitude and velocity estimation.
* **Arena Grid Mapping (`grid_mapper_node`)**: Consumes `/confirmed_survivors` and the metric `/map` to convert continuous coordinate poses into discrete 1-meter search grid cells (e.g. `A1` to `F6`) for rescue personnel dispatch.

### Layer 3: Autonomy & Path Planning
* **Corridor & Room Classification (`corridor_classifier`)**: Authoritative geometric analyzer. Partitions free-space occupancy into `corridor`, `room`, `junction`, and `unclassified` zones, published on `/map_regions`.
* **Exploration Engine (`exploration_node`)**: Detects open frontiers between explored free space and unknown space, groups them via breadth-first search (BFS), computes an information-gain utility, and selects optimal target frontiers.
* **Authoritative Path Planner (`PathPlanner`)**: Unified A* grid planner residing in `exploration_node.path_planner`. Computes Euclidean shortest collision-free paths over costmaps with inflation buffers. Used by exploration and reused by `failsafe_node`.

### Layer 4: Safety & Flight Control
* **MAVROS Bridge (`mavros_bridge`)**: Translates high-level velocity commands (`geometry_msgs/Twist`) and setpoints into MAVLink packets for ArduPilot running in `GUIDED_NOGPS` mode.
* **Failsafe Supervisor (`failsafe_node`)**: Constantly monitors battery voltage, sensor health, obstacle clearance, and communication timeouts. Executes graded responses: `NORMAL` $\to$ `WARN` $\to$ `HOLD` $\to$ `RETURN_TO_SAFE_ZONE` $\to$ `EMERGENCY_LAND`.

### Layer 5: Mission Control & Custom GCS
* **Communication Pipeline**: Single authoritative bridge in `NIDAR-AIR-Mouse-GCS/communication/` bridging ROS 2 topics/services to an asynchronous WebSocket interface.
* **Custom Rescue Dashboard**: Dark-mode aeronautical GCS providing tactical map visualization, survivor cards, sensor diagnostics, flight telemetry, and emergency controls.
