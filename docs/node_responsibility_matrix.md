# NIDAR M2 — Node Responsibility & Ownership Matrix

## 1. Responsibility Matrix

| Subsystem / Function | Authoritative ROS 2 Node | Secondary / Consumer Nodes | Prohibited Duplications |
| :--- | :--- | :--- | :--- |
| **RGB Person Detection** | `detection_node` | `tracker_node` | No secondary RGB detectors |
| **RGB Track Association** | `tracker_node` | `fusion_node` | No secondary tracker |
| **Thermal Person Detection** | `thermal_node` | `fusion_node` | No secondary thermal detector |
| **Multi-Modal Evidence Fusion** | `fusion_node` | GCS Bridge | No fusion in GCS or navigation |
| **Survivor Identity & Spatial Deduplication** | `fusion_node` (`Deduplicator`) | `grid_mapper_node`, GCS Bridge | No competing survivor registry in `grid_mapper_node` or GCS |
| **2D LiDAR SLAM & Map Building** | `slam_node` | `grid_mapper_node`, `exploration_node`, `corridor_classifier`, GCS | No SLAM calculations in GCS |
| **GPS-Denied Odometry & State Estimation** | `ekf_node` / MAVROS | All navigation & control nodes | No GPS dependencies |
| **Arena Search Grid (1m Cells)** | `grid_mapper_node` | GCS Bridge, `exploration_node` | Single owner of discrete cell labelling |
| **Corridor & Room Topology** | `corridor_classifier` | GCS Bridge, `exploration_node` | No duplicate corridor topology in `navigation_node` |
| **Frontier Detection & Grouping** | `exploration_node` (`FrontierDetector`, `FrontierGrouper`) | GCS Bridge (`/exploration_status`) | No frontier detection in GCS |
| **A\* Path Planning** | `exploration_node` (`PathPlanner`) | `failsafe_node` (direct import reuse) | No competing A* planner in `navigation_node` or GCS |
| **Local Obstacle Avoidance / Braking** | `navigation_node` / `failsafe_node` | `mavros_bridge` | Deterministic priority rules |
| **Flight Control Interface** | `mavros_bridge` | ArduPilot FCU (via MAVLink) | Direct MAVLink writes from other nodes prohibited |
| **Safety & Failsafe Supervisor** | `failsafe_node` | `mavros_bridge`, GCS Bridge | Failsafe actions must route through `failsafe_node` |
| **GCS Communication Bridge** | `NIDAR-AIR-Mouse-GCS/communication` (`ros2_bridge.py`, `command_bridge.py`) | Custom GCS Web App | Exactly one communication pipeline (no `gcs_bridge_node`) |
| **GCS Visualization & Control** | `Custom GCS` (HTML5/Canvas/CSS/JS) | Human Operator | Read-only telemetry consumer + command dispatcher |

---

## 2. Key Architecture Enforcement Rules

1. **Survivor Identity Single Source of Truth**:
   * `fusion_node` owns survivor IDs (`1..6`).
   * `grid_mapper_node` consumes `/confirmed_survivors` and decorates them with arena grid IDs (e.g. `F4`). It never mutates survivor IDs or performs independent spatial re-clustering.
2. **Path Planning Consolidation**:
   * `exploration_node.path_planner.PathPlanner` is the sole authoritative A* implementation.
   * `failsafe_node` directly imports and utilizes `PathPlanner` for emergency safe-zone routes, ensuring mathematical parity between exploration and emergency recovery paths.
3. **Corridor Topology Authority**:
   * `corridor_classifier` analyzes `/map` and publishes `/map_regions`.
   * GCS and autonomy nodes consume `/map_regions` directly; no local node may re-classify corridor geometries.
4. **Single GCS Bridge Pipeline**:
   * `NIDAR-AIR-Mouse-GCS/communication/` is the sole validated bridge (31/31 unit & integration tests passing).
