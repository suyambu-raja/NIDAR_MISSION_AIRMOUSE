# NIDAR AirMouse — ROS 2 Interface Contracts

This document is the **single source of truth** for all ROS 2 topic names, message types, frame IDs, and publish rates used in the NIDAR AirMouse system.

> [!IMPORTANT]
> All team members **must** use the exact topic names listed here.
> If you need to rename or add a topic, update this document first and notify the team.

---

## Topic Map

```
RPLIDAR A2M8
    │
    │  /scan  (sensor_msgs/LaserScan, ~10 Hz, frame: laser)
    ▼
slam_node  ◄──────── slam_toolbox (internal)
    │
    ├──── /map          (nav_msgs/OccupancyGrid,    ~2 Hz,  frame: map)
    │         │
    │         ├──► grid_mapper_node
    │         ├──► exploration_node
    │         └──► GCS dashboard
    │
    └──── /drone_pose   (geometry_msgs/PoseStamped, ~10 Hz, frame: map)
              │
              ├──► grid_mapper_node
              ├──► exploration_node
              ├──► fusion_node
              └──► GCS dashboard

fusion_node  ◄── /tracked_survivors, /thermal_detections, /drone_pose, /camera_frame
    │
    ├──── /confirmed_survivors  (nidar_msgs/SurvivorArray,  ~5 Hz)
    │         ├──► grid_mapper_node
    │         └──► GCS dashboard
    │
    └──── /fusion_status        (std_msgs/String JSON,       ~2 Hz)
              └──► GCS dashboard

grid_mapper_node  ◄── /map, /drone_pose, /confirmed_survivors
    │
    ├──── /survivor_grid_locations  (nidar_msgs/SurvivorGridArray, ~2 Hz)
    │         ├──► fusion_node
    │         ├──► exploration_node
    │         └──► GCS dashboard
    │
    └──── /grid_map_overlay         (nav_msgs/OccupancyGrid,      ~2 Hz)
              └──► GCS dashboard

exploration_node  ◄── /map, /drone_pose, /survivor_grid_locations, /battery_status
    │
    ├──── /goal_pose           (geometry_msgs/PoseStamped, ~1 Hz)
    │         └──► mavros_bridge
    │
    ├──── /exploration_status  (std_msgs/String JSON,      ~1 Hz)
    │         └──► GCS dashboard
    │
    └──── /planned_path        (nav_msgs/Path,             ~1 Hz)
              └──► GCS dashboard

mavros_bridge  ◄── /goal_pose, /cmd_vel, MAVLink (SITL / Pixhawk 6C)
    │
    ├──── /battery_status      (sensor_msgs/BatteryState,   ~1 Hz)
    │         ├──► exploration_node (battery_monitor)
    │         └──► GCS dashboard
    │
    ├──── /drone_pose          (geometry_msgs/PoseStamped,  ~10 Hz, frame: map)
    │         ├──► grid_mapper_node
    │         ├──► exploration_node
    │         └──► fusion_node
    │
    └──── /mavros_bridge/state (std_msgs/String JSON,       ~2 Hz)
              └──► GCS dashboard
```

---

## Topic Reference

### `/scan`

| Field       | Value                          |
|-------------|--------------------------------|
| **Type**    | `sensor_msgs/LaserScan`        |
| **Publisher** | `rplidar_ros` driver node    |
| **Subscriber** | `slam_node`               |
| **Frame**   | `laser`                        |
| **Rate**    | ~10 Hz (RPLIDAR A2M8 hardware) |
| **QoS**     | Best-effort, Volatile          |

**Field notes:**
- `angle_min` / `angle_max`: 0 → 2π (full 360°)
- `range_min`: 0.15 m (near-field blind spot)
- `range_max`: 8.0 m (RPLIDAR A2M8 spec maximum)
- `ranges[]`: distance per beam in metres; `inf` = no return (open space beyond range)

**Hardware:** ⚠️ Port configured via `.env` → `RPLIDAR_PORT` (default `/dev/ttyUSB0`), baud `115200`.

---

### `/map`

| Field       | Value                                                  |
|-------------|--------------------------------------------------------|
| **Type**    | `nav_msgs/OccupancyGrid`                               |
| **Publisher** | `slam_node`                                          |
| **Subscribers** | `grid_mapper_node`, `exploration_node`, GCS dashboard |
| **Frame**   | `map`                                                  |
| **Rate**    | ~2 Hz (configurable via `map_update_interval` in `slam_params.yaml`) |
| **QoS**     | Reliable, Transient-Local (latched — late subscribers receive the last map) |

**Field notes:**
- `info.resolution`: `0.05` m/cell (5 cm)
- `info.width` / `info.height`: dynamic, starts at 300×300 (15×15 m), grows as drone explores
- `info.origin`: pose of cell `(0, 0)` in the `map` frame
- `data[]`: flattened row-major array of `int8`
  - `0`   = FREE
  - `100` = OCCUPIED (wall / obstacle)
  - `-1`  = UNKNOWN (unexplored)

**Consumer contract for `grid_mapper_node`:**
Subscribe with `TRANSIENT_LOCAL` + `RELIABLE` QoS or you will miss the map if your node starts after the first publish.

---

### `/drone_pose`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `geometry_msgs/PoseStamped`                             |
| **Publisher** | `slam_node`                                           |
| **Subscribers** | `grid_mapper_node`, `fusion_node`, GCS dashboard    |
| **Frame**   | `map`                                                   |
| **Rate**    | ~10 Hz                                                  |
| **QoS**     | Best-effort, Volatile                                   |

**Field notes:**
- `header.frame_id`: always `"map"`
- `pose.position.x` / `.y`: drone position in metres, map frame
- `pose.position.z`: always `0.0` (2D SLAM — z is not estimated)
- `pose.orientation`: quaternion representing yaw (heading) of the drone; roll and pitch are always `0`

**Consumer contract for `fusion_node`:**
The pose is derived from the TF tree (`map → base_link`) at the moment of publish.
If you need covariance, subscribe to slam_toolbox's `/pose` topic (PoseWithCovarianceStamped) directly — `/drone_pose` does not carry covariance to save bandwidth.

---

### `/tracked_survivors`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `nidar_msgs/SurvivorArray`                              |
| **Publisher** | `tracker_node`                                        |
| **Subscribers** | `fusion_node`                                     |
| **Frame**   | `map`                                                   |
| **Rate**    | Event-driven — published when a new confirmed survivor is detected |
| **QoS**     | Reliable, Volatile                                      |

**Field notes:**
- `detections[]`: array of `SurvivorDetection` — consumed by `fusion_node` for RGB evidence
- `position.x` / `.y`: survivor position in SLAM map frame (metres)
- `confidence`: detection confidence from tracker (0.0–1.0)
- `survivor_id`: tracker-assigned ID (-1 if not yet tracked)

---

### `/thermal_detections`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `nidar_msgs/SurvivorArray`                              |
| **Publisher** | `thermal_node` (YOLO11n)                              |
| **Subscribers** | `fusion_node`                                     |
| **Frame**   | `map`                                                   |
| **Rate**    | ~8 Hz                                                   |
| **QoS**     | Reliable, Volatile                                      |

**Field notes:**
- `detections[]`: array of `SurvivorDetection` from thermal YOLO11n inference
- `detection_source`: always `"thermal"`
- `position.x` / `.y`: world position if available, else `(0, 0)` and `fusion_node` projects from bbox + depth
- `confidence`: thermal detection confidence (0.0–1.0)

**Thermal detection strategy:**
```
thermal = parallel/conditional channel
        = primary in DEGRADED visibility
        = secondary in NORMAL visibility
        = NOT a confirmation gate
```

---

### `/camera_frame`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `sensor_msgs/Image`                                     |
| **Publisher** | OAK-D camera driver                                  |
| **Subscribers** | `fusion_node`                                     |
| **Frame**   | `camera_link`                                           |
| **Rate**    | ~30 Hz                                                  |
| **QoS**     | Best-effort, Volatile                                   |

**Field notes:**
- Encoding: `rgb8` or `bgr8` (3-channel colour) or `mono8` (grayscale)
- Used by `fusion_node` for **visibility estimation only** (average brightness)
- Not stored or forwarded — only brightness metric is extracted

---

### `/confirmed_survivors`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `nidar_msgs/SurvivorArray`                              |
| **Publisher** | `fusion_node`                                         |
| **Subscribers** | `grid_mapper_node`, GCS dashboard                 |
| **Frame**   | `map`                                                   |
| **Rate**    | ~5 Hz                                                   |
| **QoS**     | Reliable, Volatile                                      |

**Field notes:**
- `detections[]`: array of `SurvivorDetection` — all confirmed survivors found so far
- `is_confirmed`: always `True` (only confirmed detections are published here)
- `detection_source`: `"rgb"`, `"thermal"`, or `"fused"` (indicates which sensor(s) contributed)
- `confidence`: fused confidence after weighted evidence fusion + EMA smoothing
- `survivor_id`: unique ID assigned by fusion deduplicator (1-based)
- Maximum 6 entries (competition limit — enforced by `fusion_node`)

---

### `/fusion_status`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `std_msgs/String` (JSON payload)                        |
| **Publisher** | `fusion_node`                                         |
| **Subscriber** | GCS dashboard                                       |
| **Frame**   | N/A                                                     |
| **Rate**    | ~2 Hz                                                   |
| **QoS**     | Reliable, Volatile                                      |

**JSON payload schema:**
```json
{
  "visibility_state":     "NORMAL",
  "active_detector":      "Both",
  "brightness":           142.5,
  "pending_rgb":          3,
  "pending_thermal":      1,
  "tracked_positions":    2,
  "confirmed_survivors":  1,
  "max_survivors":        6,
  "confidence_threshold": 0.65,
  "pose_available":       true
}
```

**Visibility state values:** `NORMAL` (brightness > 50), `DEGRADED` (brightness ≤ 50)
**Active detector values:** `RGB`, `Thermal`, `Both`, `None`

---

### `/survivor_grid_locations`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `nidar_msgs/SurvivorGridArray`                          |
| **Publisher** | `grid_mapper_node`                                    |
| **Subscribers** | `fusion_node`, GCS dashboard                      |
| **Frame**   | `map`                                                   |
| **Rate**    | ~2 Hz                                                   |
| **QoS**     | Reliable, Transient-Local (latched — late subscribers receive last message) |

**Field notes:**
- `locations[]`: array of `SurvivorGridLocation` messages
  - `survivor_id`: unique integer ID assigned by `grid_mapper_node` registry
  - `grid_box`: arena grid box string, e.g. `"F4"` (column letter A–O + row number 1–15)
  - `world_x` / `world_y`: SLAM map-frame position of the survivor (metres)
  - `confidence`: running average of all merged detections (0.0–1.0)
- This is the **running list** of all confirmed unique survivors found so far — not just the latest batch.
- Maximum 6 entries (competition limit). Once 6 survivors are confirmed, new unique detections are silently dropped.

**Grid naming convention:**
```
Column: A (x=0–1m) → B (1–2m) → … → O (14–15m)   [left to right, X axis]
Row:    1 (y=0–1m) → 2 (1–2m) → … → 15 (14–15m)  [bottom to top, Y axis]
Example: survivor at map (6.3m, 4.7m) → grid box "F4"
```

---

### `/grid_map_overlay`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `nav_msgs/OccupancyGrid`                                |
| **Publisher** | `grid_mapper_node`                                    |
| **Subscribers** | GCS dashboard                                     |
| **Frame**   | `map`                                                   |
| **Rate**    | ~2 Hz                                                   |
| **QoS**     | Reliable, Transient-Local                               |

**Field notes:**
- Same dimensions and resolution as `/map` (copied from the base map).
- Additional cell values drawn on top of `/map`:
  - `75` = 1×1 m **grid line** boundary
  - `50` = **survivor marker** (3×3 cell cross at each confirmed survivor position)
  - `0` = FREE (unchanged from /map)
  - `100` = OCCUPIED wall (unchanged from /map — never overwritten)
  - `-1` = UNKNOWN (unchanged from /map)
- Designed for direct consumption by the GCS dashboard (rosbridge → roslibjs).
- The base `/map` is **never mutated** — the overlay is always a fresh copy.

---

### `/goal_pose`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `geometry_msgs/PoseStamped`                             |
| **Publisher** | `exploration_node`                                    |
| **Subscriber** | `mavros_bridge`                                      |
| **Frame**   | `map`                                                   |
| **Rate**    | ~1 Hz                                                   |
| **QoS**     | Reliable, Transient-Local                               |

**Field notes:**
- `pose.position.x` / `.y`: target world position for the drone to fly to (metres, map frame)
- `pose.position.z`: always `0.0` (2D exploration)
- `pose.orientation`: quaternion representing the yaw the drone should face when it arrives
- Updated every 1 second by the exploration pipeline; only published when a valid A* path exists
- Navigation stops publishing when all 6 survivors are found or battery enters EXIT state

---

### `/exploration_status`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `std_msgs/String` (JSON payload)                        |
| **Publisher** | `exploration_node`                                    |
| **Subscriber** | GCS dashboard                                       |
| **Frame**   | N/A                                                     |
| **Rate**    | ~1 Hz                                                   |
| **QoS**     | Reliable, Transient-Local                               |

**JSON payload schema:**
```json
{
  "state":           "exploring",
  "survivors_found": 3,
  "frontiers":       12,
  "coverage_pct":    47.3,
  "goal":            [5.2, 3.1],
  "strategy":        "EXPLORE",
  "battery_pct":     68.4,
  "ticks":           42
}
```

**State values:**
- `waiting_for_map` — no `/map` received yet
- `waiting_for_pose` — no `/drone_pose` received yet
- `exploring` — normal frontier exploration in progress
- `fully_explored` — no frontier cells remain (arena mapped)
- `no_valid_groups` — all frontier groups below minimum size
- `no_path_found` — A* could not plan a safe route to any frontier
- `exiting_battery_critical` — battery < 15%, navigating to exit (0, 0)
- `mission_complete` — all 6 survivors found, exploration stopped

**Strategy values:** `EXPLORE` (battery > 30%), `RETURN` (15–30%), `EXIT` (< 15%)

---

### `/planned_path`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `nav_msgs/Path`                                         |
| **Publisher** | `exploration_node`                                    |
| **Subscriber** | GCS dashboard                                       |
| **Frame**   | `map`                                                   |
| **Rate**    | ~1 Hz                                                   |
| **QoS**     | Reliable, Transient-Local                               |

**Field notes:**
- `poses[]`: ordered list of `PoseStamped` waypoints from the A* planner
- First pose = drone's current position; last pose = selected frontier center
- All poses have `z = 0.0` and `orientation.w = 1.0` (heading not specified per waypoint)
- Published alongside `/goal_pose` for GCS map visualisation only — mavros_bridge uses `/goal_pose`
- Not published if A* fails to find a path

---

### `/battery_status`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `sensor_msgs/BatteryState`                              |
| **Publisher** | `mavros_bridge` (relayed from `/mavros/battery`)      |
| **Subscribers** | `exploration_node` (`battery_monitor`), GCS dashboard |
| **Frame**   | `base_link`                                             |
| **Rate**    | ~1 Hz                                                   |
| **QoS**     | Best-effort, Volatile                                   |

**Field notes:**
- `percentage`: float [0.0, 1.0] representing battery charge level (e.g. `0.65` = 65%)
- `voltage`: current battery pack voltage (e.g. `11.4` V)
- `current`: instantaneous current draw in amperes
- Consumed directly by `exploration_node`'s `battery_monitor.py` to trigger `EXPLORE` (>30%), `RETURN` (15-30%), or `EXIT` (<15%) strategies

---

### `/mavros_bridge/state`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `std_msgs/String` (JSON payload)                        |
| **Publisher** | `mavros_bridge`                                       |
| **Subscriber** | GCS dashboard                                       |
| **Frame**   | N/A                                                     |
| **Rate**    | ~2 Hz                                                   |
| **QoS**     | Reliable, Transient-Local                               |

**JSON payload schema:**
```json
{
  "connected":       true,
  "armed":           true,
  "mode":            "GUIDED",
  "guided":          true,
  "in_air":          true,
  "battery_pct":     78.4,
  "battery_voltage": 11.8,
  "altitude":        2.5,
  "position":        [2.1, 4.5, 2.5],
  "goal":            [5.0, 3.0],
  "fcu_url":         "udp://127.0.0.1:14550@"
}
```

---

### `/cmd_vel`

| Field       | Value                                                   |
|-------------|---------------------------------------------------------|
| **Type**    | `geometry_msgs/Twist`                                   |
| **Publisher** | Manual teleop / GCS control / emergency override        |
| **Subscriber** | `mavros_bridge`                                      |
| **Frame**   | `base_link`                                             |
| **Rate**    | ~10 Hz (when active)                                    |
| **QoS**     | Best-effort, Volatile                                   |

---

## Service Reference (`mavros_bridge`)

| Service Name         | Type                   | Purpose                                                |
|----------------------|------------------------|--------------------------------------------------------|
| `/arm`               | `std_srvs/srv/SetBool` | Arm (`data: true`) or disarm (`data: false`) the drone |
| `/takeoff`           | `std_srvs/srv/Trigger` | Switch to GUIDED mode, arm, and takeoff to cruise alt  |
| `/land`              | `std_srvs/srv/Trigger` | Switch to LAND mode and land                           |
| `/rtl`               | `std_srvs/srv/Trigger` | Switch to RTL (Return To Launch) mode                  |
| `/set_mode_guided`   | `std_srvs/srv/Trigger` | Switch flight mode to `GUIDED`                         |
| `/set_mode_rtl`      | `std_srvs/srv/Trigger` | Switch flight mode to `RTL`                            |
| `/set_mode_land`     | `std_srvs/srv/Trigger` | Switch flight mode to `LAND`                           |
| `/set_mode_loiter`   | `std_srvs/srv/Trigger` | Switch flight mode to `LOITER`                         |
| `/set_mode_stabilize`| `std_srvs/srv/Trigger` | Switch flight mode to `STABILIZE`                      |

---

## TF Tree

```
map
 └── odom              (managed by slam_toolbox — do not publish this yourself)
      └── base_link    (drone body centre)
           └── laser   (RPLIDAR A2M8 — offset: x=0, y=0, z=0.05m above base_link)
```

Static transforms are published by `rplidar_launch.py` at startup.

---

## Mock Publisher

During development, use [`tools/mock_publishers/mock_slam.py`](../tools/mock_publishers/mock_slam.py) to get `/map` and `/drone_pose` without any hardware:

```bash
source /opt/ros/humble/setup.bash
python3 tools/mock_publishers/mock_slam.py
```

The mock publishes:
- `/map`: empty 15×15 m arena (border walls, all interior FREE)
- `/drone_pose`: figure-8 trajectory over a 10×6 m area, 30-second loop

> [!WARNING]
> `mock_slam.py` is **development-only**. Never launch it alongside `rplidar_launch.py` — both publish on `/map` and `/drone_pose` and will conflict.

---

## Dependency Graph (Node → Topics)

| Node              | Publishes                                               | Subscribes                                          |
|-------------------|---------------------------------------------------------|-----------------------------------------------------|
| `rplidar_ros`     | `/scan`                                                 | —                                                   |
| `slam_node`       | `/map`, `/drone_pose`                                   | `/scan`, `/map` (from slam_toolbox)                 |
| `grid_mapper_node`| `/survivor_grid_locations`, `/grid_map_overlay`         | `/map`, `/drone_pose`, `/confirmed_survivors`        |
| `tracker_node`    | `/tracked_survivors`                                    | `/detected_survivors`                                |
| `thermal_node`    | `/thermal_detections`                                   | *(thermal camera driver)*                            |
| `fusion_node`     | `/confirmed_survivors`, `/fusion_status`                | `/tracked_survivors`, `/thermal_detections`, `/drone_pose`, `/camera_frame` |
| `exploration_node`| `/goal_pose`, `/exploration_status`, `/planned_path`    | `/map`, `/drone_pose`, `/survivor_grid_locations`, `/battery_status` |
| `mavros_bridge`   | `/drone_pose`, `/battery_status`, `/mavros_bridge/state`, `/mavros/setpoint_position/local`, `/mavros/setpoint_velocity/cmd_vel_unstamped` | `/goal_pose`, `/cmd_vel`, `/mavros/state`, `/mavros/battery`, `/mavros/local_position/pose`, `/mavros/local_position/velocity_local` |
| GCS dashboard     | `/cmd_vel` (teleop)                                     | `/map`, `/drone_pose`, `/survivor_grid_locations`, `/grid_map_overlay`, `/exploration_status`, `/planned_path`, `/mavros_bridge/state` |

---

## Versioning

| Version | Date       | Change                                  | Author      |
|---------|------------|-----------------------------------------|-------------|
| 0.1.0   | 2026-08-16 | Initial interface contract              | NIDAR Team  |
| 0.2.0   | 2026-08-17 | Add `/survivor_grid_locations`, `/grid_map_overlay`, `/tracked_survivors`; update dependency graph | NIDAR Team  |
| 0.3.0   | 2026-08-17 | Add `/goal_pose`, `/exploration_status`, `/planned_path`; promote `exploration_node` from future to active; update topic map and dependency graph | NIDAR Team  |
| 0.4.0   | 2026-08-19 | Add `/confirmed_survivors`, `/fusion_status`, `/thermal_detections`, `/camera_frame`; promote `fusion_node` from TBD to active; add `thermal_node` to dependency graph; thermal strategy documentation | NIDAR Team  |
| 0.5.0   | 2026-08-20 | Add `mavros_bridge` topic (`/battery_status`, `/mavros_bridge/state`, `/cmd_vel`) and service contracts (`/arm`, `/takeoff`, `/land`, `/rtl`, `/set_mode_*`); update dependency graph | NIDAR Team  |

> [!NOTE]
> Update the version row whenever a topic name, type, frame, or QoS changes.
> Bump MINOR for additive changes, MAJOR for breaking changes.
