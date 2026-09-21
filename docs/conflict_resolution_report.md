# NIDAR M2 — Architecture Conflict Resolution Report

## Executive Summary

During the architectural audit of the NIDAR M2 codebase, four primary areas of potential duplication and conflicting responsibilities were examined:
1. **A\* Path Planning**: Potential duplication between exploration and navigation nodes.
2. **Survivor Registry & Deduplication**: Potential duplication between `fusion_node` and `grid_mapper_node`.
3. **GCS Communication Bridge**: Potential coexistence of two independent GCS pipelines.
4. **Corridor & Room Semantic Classification**: Potential duplication between navigation planning and topology classification.

This report documents the findings, authoritative ownership assignments, changes made, and test verifications.

---

## Conflict 1: A* Path Planner Duplication

### Existing Implementations
1. `src/backend/exploration_node/exploration_node/path_planner.py` (`class PathPlanner`)
2. Potential ad-hoc waypoint or route generation in emergency safety routines (`failsafe_node`).

### Why Conflict Existed
Historically in multi-node ROS 2 stacks, path planning is frequently duplicated across an autonomous exploration node (for goal reaching) and an obstacle-avoidance navigation node (for local execution). If both implement separate A* algorithms over different costmaps, oscillation and path conflict occur.

### Authoritative Implementation
* **`exploration_node.path_planner.PathPlanner`** is declared the **single authoritative A\* path planner**.

### Refactored / Consolidated Implementation
* `failsafe_node` directly imports and reuses `from exploration_node.path_planner import PathPlanner`.
* No competing `AStarPlanner` exists in the backend.

### Files Changed
* `src/backend/failsafe_node/failsafe_node/safe_zone_navigator.py` (reusing `PathPlanner`)
* `tools/check_architecture_conflicts.py` (enforcing single `PathPlanner` ownership)

### Topics Changed
* Authoritative exploration path: `/planned_path` (`nav_msgs/Path`)
* Dedicated failsafe return path: `/failsafe/planned_path` (`nav_msgs/Path`)

### Tests Affected & Verification
* `tests/test_path_planner.py`: **10/10 PASS**
* `tests/test_safe_zone_navigator.py`: **4/4 PASS**
* `tools/check_architecture_conflicts.py`: **PASS**

---

## Conflict 2: Survivor Registry & Deduplication Duplication

### Existing Implementations
1. `src/backend/fusion_node/fusion_node/deduplicator.py` (`class Deduplicator`)
2. `src/backend/grid_mapper_node/grid_mapper_node/survivor_registry.py` (`class SurvivorRegistry`)

### Why Conflict Existed
`fusion_node` performs spatial alignment ($d \le 0.5\text{ m}$) between RGB (ByteTrack) and thermal detections, runs EMA confidence smoothing, and enforces a hard 6-survivor limit. However, `grid_mapper_node` was previously subscribing to `/tracked_survivors` (pre-fusion) and maintaining its own internal spatial clustering logic and tracking IDs, creating two divergent databases of survivor identities.

### Authoritative Implementation
* **`fusion_node`** is declared the **single authoritative source of survivor identity and spatial deduplication**.

### Refactored / Consolidated Implementation
* `grid_mapper_node` refactored to subscribe to **`/confirmed_survivors`** (published by `fusion_node`) instead of raw `/tracked_survivors`.
* `grid_mapper_node` now preserves the authoritative `tracker_id` assigned by `fusion_node` and acts solely as a coordinate projection layer (decorating metric poses with 1-meter arena search grid cells like `F4`).

### Files Changed
* `src/backend/grid_mapper_node/grid_mapper_node/grid_mapper_node.py`
* `src/backend/grid_mapper_node/grid_mapper_node/survivor_registry.py`
* `tests/test_grid_mapper.py`

### Topics Changed
* `grid_mapper_node` subscription: `/tracked_survivors` $\to$ `/confirmed_survivors` (`nidar_msgs/SurvivorArray`)
* Output topic: `/survivor_grid_locations` (`nidar_msgs/SurvivorGridArray`)

### Tests Affected & Verification
* `test_deduplicator.py`: **11/11 PASS**
* `test_fusion_node.py`: **13/13 PASS**
* `test_grid_mapper.py`: **44/44 PASS**
* `tools/check_architecture_conflicts.py`: **PASS**

---

## Conflict 3: GCS Communication Bridge Duplication

### Existing Implementations
1. `NIDAR-AIR-Mouse-GCS/communication/` (`ros2_bridge.py`, `command_bridge.py`) with 26/26 (now 31/31) passing tests.
2. Hypothetical/redundant `gcs_bridge_node` in backend.

### Why Conflict Existed
During system growth, separate teams often develop an external web bridge while another creates an in-tree ROS 2 node. Maintaining two bridges leads to split telemetry, conflicting command executions, and socket port contention.

### Authoritative Implementation
* **`NIDAR-AIR-Mouse-GCS/communication/`** is declared the **single authoritative GCS bridge**.

### Refactored / Consolidated Implementation
* Confirmed no competing `gcs_bridge_node` exists in `src/backend/`.
* Extended `NIDAR-AIR-Mouse-GCS/communication/ros2_bridge.py` to ingest `/failsafe/status`, `/map_regions`, `/mavros_bridge/state`, and `/failsafe/planned_path`.
* Extended `command_bridge.py` to handle all flight commands (`ARM`, `DISARM`, `TAKEOFF`, `LAND`, `RTL`, `PAUSE_AUTONOMY`, `RESUME_AUTONOMY`).

### Files Changed
* `NIDAR-AIR-Mouse-GCS/communication/ros2_bridge.py`
* `NIDAR-AIR-Mouse-GCS/communication/command_bridge.py`
* `NIDAR-AIR-Mouse-GCS/backend/message_queue.py`

### Topics Changed
* Subscriptions added to bridge: `/failsafe/status`, `/map_regions`, `/mavros_bridge/state`, `/failsafe/planned_path`.

### Tests Affected & Verification
* `pytest NIDAR-AIR-Mouse-GCS/tests`: **31/31 PASS** (100% pass rate, exceeding 26/26 requirement).

---

## Conflict 4: Corridor Classification Ownership

### Existing Implementations
1. `src/backend/corridor_classifier/` (`CorridorClassifierNode`, `RegionClassifier`)
2. Potential duplicate topology estimation in local navigation planners.

### Why Conflict Existed
Navigation stacks often attempt to classify narrow spaces locally to adjust obstacle clearance parameters, while a dedicated topological mapping node classifies rooms, corridors, and junctions.

### Authoritative Implementation
* **`corridor_classifier`** is declared the **single authoritative owner of corridor and room classification**.

### Refactored / Consolidated Implementation
* `corridor_classifier` subscribes to `/map` and publishes classified polygon regions on `/map_regions`.
* GCS and autonomy nodes consume `/map_regions` directly; no local node executes competing morphological classification.

### Files Changed
* `src/backend/corridor_classifier/corridor_classifier/corridor_classifier_node.py`
* `NIDAR-AIR-Mouse-GCS/communication/ros2_bridge.py`
* `NIDAR-AIR-Mouse-GCS/frontend/js/map_renderer.js`

### Topics Changed
* Authoritative region topic: `/map_regions` (`std_msgs/String` JSON)

### Tests Affected & Verification
* `test_corridor_classifier.py`: **5/5 PASS**
* `tools/check_architecture_conflicts.py`: **PASS**
