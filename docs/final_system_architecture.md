# NIDAR M2 — Final Integrated System Architecture

## 1. Global System Architectural Diagram

```
 +───────────────────────────────────────────────────────────────────────────────────+
 |                                  NIDAR M2 AVIONICS                                |
 |                                                                                   |
 |  [Sensors & SLAM]               [Perception & Fusion]        [Planning & Safety]  |
 |  • RPLIDAR A2M8 (10Hz)          • YOLOv8 RGB Detection       • Frontier Detector  |
 |  • OAK-D Lite RGB (30Hz)        • ByteTrack Spatial Tracker  • Frontier Grouper   |
 |  • FLIR Lepton 3.5 (9Hz)        • YOLO11n Thermal Detection  • Corridor Classifier|
 |  • Matek Flow + Rangefinder     • Multi-Modal Fusion Engine    (/map_regions)     |
 |  • Pixhawk 6C EKF (50Hz)        • Authoritative Deduplicator • PathPlanner (A*)   |
 |  • slam_node (/map, /drone_pose)    (/confirmed_survivors)     • Failsafe Supervisor|
 |                                                                                   |
 |  [Flight Control Layer]                                                           |
 |  • mavros_bridge_node (ArduPilot GUIDED_NOGPS / MAVLink / Setpoints / Telemetry)   |
 +──────────────────────────────────────────│────────────────────────────────────────+
                                            │
                                            ▼ ROS 2 Topics / Services
 +───────────────────────────────────────────────────────────────────────────────────+
 |                        AUTHORITATIVE GCS BRIDGE ENGINE                            |
 |                                                                                   |
 |  • ros2_bridge.py: High-frequency telemetry, 2D SLAM maps, paths, and health     |
 |  • video_bridge.py: Dual-stream base64 JPEG encoding with drop-oldest policies    |
 |  • command_bridge.py: Request-ACK validation & ROS 2 service/topic command calls |
 |  • message_queue.py: Bounded Priority Queue (HIGH: Alerts/Survivors, NORMAL: Map) |
 +──────────────────────────────────────────▲────────────────────────────────────────+
                                            │
                                            ▼ WebSocket (Port 8765) / HTTP (Port 8080)
 +───────────────────────────────────────────────────────────────────────────────────+
 |                          CUSTOM RESCUE MISSION GCS                                |
 |                                                                                   |
 |  [Left Column]                [Center Canvas]               [Right Column]        |
 |  • Flight State & Modes       • 2D Dynamic SLAM Map         • Dual Video Feed     |
 |  • GPS-Denied Health          • Drone Symbol & Heading Arrow  (RGB / Thermal Tab) |
 |  • Battery Percentage & Bar   • Planned A* Path (Cyan)      • Survivor Panel      |
 |  • Flight Controls:           • Failsafe Return Path (Red)    (S01..S06+ Cards)   |
 |    (Arm/Disarm/Land/Takeoff)  • Corridor & Room Polygons    • 12-Subsystem Sensor |
 |                               • Frontier Centroids            Health Grid         |
 |                                                                                   |
 |  [Bottom Bar]                                                                     |
 |  • Autonomy Pipeline Metrics: Corridors, A* Planner, DWA Clearances, Brakes       |
 |  [Top Alert Banners]                                                              |
 |  • Connection Loss Watchdog (3s Timeout) & Emergency Failsafe Trigger Banners     |
 +───────────────────────────────────────────────────────────────────────────────────+
```

---

## 2. Core Architectural Pillars

1. **Strict Single-Ownership**:
   * A* Path Planning: Owned strictly by `exploration_node.path_planner.PathPlanner`.
   * Survivor Identity: Owned strictly by `fusion_node` (`Deduplicator`).
   * Corridor Classification: Owned strictly by `corridor_classifier`.
   * GCS Bridge: Owned strictly by `NIDAR-AIR-Mouse-GCS/communication/`.
2. **Deterministic Data Freshness**:
   * Telemetry updates at 10 Hz.
   * SLAM occupancy grids update at 1–5 Hz.
   * Video streams throttle dynamically using drop-oldest queues to prevent socket stalls.
   * Telemetry older than 3 seconds triggers an immediate operator warning.
3. **Fail-Safe Independence**:
   * Onboard ROS 2 autonomy and ArduPilot failsafes operate completely autonomously.
   * Loss of ground station communication does NOT cause vehicle crash; the drone safely pauses, returns, or lands per onboard configuration.
