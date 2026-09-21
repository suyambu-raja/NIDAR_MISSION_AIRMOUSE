# NIDAR M2 — ROS 2 Topic & Interface Contract

## 1. Authoritative ROS 2 Topic Registry

The following table defines the verified, non-conflicting topic contract for the NIDAR M2 system. Every semantic data flow has exactly one owning publisher.

| Topic Name | Message Type | Owning Publisher | Consumers | Rate | QoS Reliability | Description |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `/drone_pose` | `geometry_msgs/PoseStamped` | `slam_node` / EKF | `grid_mapper_node`, `exploration_node`, GCS Bridge | 10 Hz | BEST_EFFORT | Estimated vehicle 2D/3D pose in `map` frame |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_node` | `grid_mapper_node`, `corridor_classifier`, `exploration_node`, GCS Bridge | 1–5 Hz | RELIABLE / TRANSIENT_LOCAL | Global occupancy grid (0.05 m resolution) |
| `/confirmed_survivors` | `nidar_msgs/SurvivorArray` | `fusion_node` | `grid_mapper_node`, GCS Bridge | On Event | RELIABLE | Multi-modal confirmed survivor list with unique IDs |
| `/survivor_grid_locations` | `nidar_msgs/SurvivorGridArray`| `grid_mapper_node` | `exploration_node`, GCS Bridge | 2 Hz | RELIABLE | Arena 1m discrete grid assignments (e.g. `F4`) |
| `/grid_map_overlay` | `nav_msgs/OccupancyGrid` | `grid_mapper_node` | GCS Bridge | 2 Hz | BEST_EFFORT | 1m arena grid visualization layer |
| `/map_regions` | `std_msgs/String` (JSON) | `corridor_classifier` | `exploration_node`, GCS Bridge | 1 Hz | RELIABLE / TRANSIENT_LOCAL | Geometric topology regions (`corridor`, `room`, `junction`) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | `exploration_node` | `navigation_node` / Path Follower | 1 Hz | RELIABLE | Current autonomous exploration waypoint target |
| `/planned_path` | `nav_msgs/Path` | `exploration_node` | GCS Bridge | 1 Hz | RELIABLE | Authoritative A* planned exploration trajectory |
| `/exploration_status` | `std_msgs/String` (JSON) | `exploration_node` | GCS Bridge | 1 Hz | BEST_EFFORT | Active exploration metrics, frontier count, target |
| `/failsafe/status` | `std_msgs/String` (JSON) | `failsafe_node` | GCS Bridge | 2 Hz | RELIABLE | Failsafe state (`NORMAL`, `WARN`, `HOLD`, `RETURN`) |
| `/failsafe/planned_path` | `nav_msgs/Path` | `failsafe_node` | GCS Bridge | On Event | RELIABLE | Emergency return-to-safe-zone A* path |
| `/battery_status` | `sensor_msgs/BatteryState` | `mavros_bridge` | `failsafe_node`, `exploration_node`, GCS Bridge | 1 Hz | BEST_EFFORT | Battery voltage, current, percentage, health |
| `/mavros_bridge/state` | `std_msgs/String` (JSON) | `mavros_bridge` | GCS Bridge | 1 Hz | RELIABLE | Flight controller connectivity and armed state |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Camera Driver | `detection_node`, GCS Bridge | 30 Hz | BEST_EFFORT | Raw or compressed RGB video frames |
| `/camera/thermal/image_raw`| `sensor_msgs/Image` | Thermal Driver | `thermal_node`, GCS Bridge | 9 Hz | BEST_EFFORT | Radiometric or colormapped thermal frames |

---

## 2. QoS Policy Guidelines

* **Sensor Streams (Camera, IMU, Pose)**: `BEST_EFFORT` reliability with depth 5–10 to minimize transport latency and prevent buffer congestion.
* **Map & Topology Layers (`/map`, `/map_regions`)**: `RELIABLE` with `TRANSIENT_LOCAL` durability so late-joining GCS clients receive immediate state without waiting for the next periodic cycle.
* **Mission Critical Events (`/confirmed_survivors`, `/failsafe/status`)**: `RELIABLE` with `VOLATILE` or `TRANSIENT_LOCAL` to guarantee zero message drop.

---

## 3. Prohibited Legacy / Duplicate Topics

* `/tracked_survivors` as a direct GCS feed: **Deprecated/Disallowed**. GCS must subscribe to fused, authoritative `/confirmed_survivors`.
* Multiple competing publishers on `/planned_path`: **Disallowed**. `exploration_node` is the sole exploration planner; `failsafe_node` publishes on dedicated `/failsafe/planned_path`.
* Duplicate corridor classification: **Disallowed**. Only `corridor_classifier` publishes `/map_regions`.
