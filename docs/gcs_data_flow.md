# NIDAR M2 — GCS Data Flow Architecture

## 1. End-to-End Data Pipeline

The Custom GCS operates strictly as a **Mission Control, Real-Time Visualization, and Safety Override Terminal**. It does not perform autonomous navigation, SLAM, or sensor fusion.

```
+-------------------------------------------------------------------------------+
|                                ONBOARD ROS 2 STACK                            |
|                                                                               |
|  [SLAM & EKF]        [Perception & Fusion]      [Autonomy & Path Planner]     |
|   /map                   /confirmed_survivors       /planned_path             |
|   /drone_pose            /survivor_grid_locations   /exploration_status       |
|                                                     /map_regions              |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
|                 AUTHORITATIVE GCS BRIDGE (ros2_bridge.py)                     |
|                                                                               |
|  • Subscription Managers & Topic Translators                                  |
|  • Throttling & Priority Serialization (Telemetry 10Hz, Map 5Hz, Video 12Hz)   |
|  • In-Memory Fast Ring Buffers (Prevent UI Stalls)                            |
|  • JSON / Binary WebSocket Encoders                                           |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼ WebSocket (Port 8765)
+-------------------------------------------------------------------------------+
|                           CUSTOM RESCUE GCS (Browser)                         |
|                                                                               |
|  +-------------------------+  +--------------------------+  +---------------+ |
|  |       Left Column       |  |       Center Canvas      |  | Right Column  | |
|  | - Mission State Machine |  | - 2D Dynamic SLAM Map    |  | - Survivor    | |
|  | - FCU Armed / Mode      |  | - Drone & Heading Arrow  |  |   Registry    | |
|  | - GPS-Denied Avionics   |  | - Planned A* Path (Cyan) |  |   (S01..S06+) | |
|  | - Battery Telemetry     |  | - Failsafe Path (Red)    |  | - Sensor      | |
|  | - Flight Overrides      |  | - Corridors / Rooms      |  |   Health Grid | |
|  |                         |  | - Frontier Markers       |  | - Video Feeds | |
|  +-------------------------+  +--------------------------+  +---------------+ |
|                                                                               |
|  +──────────────────────────────────────────────────────────────────────────+ |
|  | Bottom Bar: Autonomy Metrics (Corridors, A*, DWA, Obstacles, Brakes)     | |
|  +──────────────────────────────────────────────────────────────────────────+ |
+-------------------------------------------------------------------------------+
```

---

## 2. Inbound Telemetry Specifications

1. **`telemetry` (10 Hz)**:
   * Attributes: `x`, `y`, `z`, `heading`, `velocity`, `battery_voltage`, `battery_percentage`, `flight_mode`, `armed`.
   * Action: Updates map vehicle position, heading vector, HUD velocity, attitude indicator, and left-column flight metrics.
2. **`map_update` (5 Hz)**:
   * Attributes: `width`, `height`, `resolution`, `origin_x`, `origin_y`, `data` (occupancy grid).
   * Action: Renders high-contrast 2D tactical canvas with free space (white), obstacles (black), and unexplored zones (neutral gray).
3. **`survivor_detected` / `survivor_update` (Event-driven / 2 Hz)**:
   * Attributes: `id`, `grid_cell`, `x`, `y`, `confidence`, `status`, `modality`, `first_seen`, `last_seen`, `distance`.
   * Action: Appends or refreshes card in right-hand Survivor Panel, and renders distinct numbered icon on 2D map.
4. **`path_update` (1 Hz)**:
   * Attributes: `path` (array of `[x, y]` coordinates), `frontiers` (list of active cluster centroids).
   * Action: Draws authoritative cyan A* path and glowing frontier points on map canvas.
5. **`failsafe_status` (2 Hz)**:
   * Attributes: `active`, `state`, `reason`, `action`, `battery_ok`, `nav_ok`, `comm_ok`.
   * Action: Updates bottom health indicators; if `active=True`, immediately raises top failsafe banner.
6. **`camera_frame` (12 FPS)**:
   * Attributes: `camera_id` (`"rgb"` or `"thermal"`), `frame_base64`, `status`.
   * Action: Updates live video canvas under selected camera tab.
