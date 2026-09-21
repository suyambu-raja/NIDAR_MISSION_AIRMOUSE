# NIDAR M2 — Canonical Telemetry Schema Specification

## 1. Schema Overview

The NIDAR M2 Custom GCS employs a canonical JSON schema transmitted across the high-speed asynchronous WebSocket transport (Port 8765). Every message includes an explicit `type` discriminator and timestamp.

---

## 2. Telemetry Entity Schemas

### A. DroneKinematics (`type: "telemetry"`)
* **Frequency**: 10 Hz
* **Source**: `slam_node` / EKF (`/drone_pose`)

| Field | Type | Unit | Range / Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `timestamp` | `float` | seconds | Unix Epoch | Message creation time |
| `x` | `float` | meters | $-50.0 \dots +50.0$ | Vehicle X coordinate in `map` frame |
| `y` | `float` | meters | $-50.0 \dots +50.0$ | Vehicle Y coordinate in `map` frame |
| `z` | `float` | meters | $0.0 \dots 10.0$ | Altitude above launch origin |
| `altitude` | `float` | meters | $0.0 \dots 10.0$ | Vertical distance |
| `heading` | `float` | degrees | $0.0 \dots 359.9$ | Compass heading ($0^\circ = \text{North}$) |
| `roll` | `float` | degrees | $-180.0 \dots +180.0$| Vehicle body roll |
| `pitch` | `float` | degrees | $-90.0 \dots +90.0$ | Vehicle body pitch |
| `velocity` | `float` | m/s | $0.0 \dots 15.0$ | Ground speed magnitude |
| `grid_cell` | `string` | — | `"A1"` $\dots$ `"F6"` | Discrete 1-meter arena search grid cell |
| `flight_mode` | `string` | — | `"GUIDED"`, `"LOITER"`| Active ArduPilot flight mode |
| `armed` | `boolean`| — | `true` / `false` | Motor arming state |
| `battery_v` | `float` | volts | $10.0 \dots 16.8$ | Main flight pack voltage |
| `battery_pct`| `float` | percent | $0.0 \dots 100.0$ | Estimated remaining battery charge |

### B. MapState (`type: "map_update"`)
* **Frequency**: 1–5 Hz
* **Source**: `slam_node` (`/map`)

| Field | Type | Unit | Range / Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `timestamp` | `float` | seconds | Unix Epoch | Map generation time |
| `resolution` | `float` | meters/cell | $0.05$ | Metric size of one grid cell |
| `width` | `integer`| cells | $20 \dots 1000$ | Map grid width |
| `height` | `integer`| cells | $20 \dots 1000$ | Map grid height |
| `origin_x` | `float` | meters | Relative to map | Lower-left origin X |
| `origin_y` | `float` | meters | Relative to map | Lower-left origin Y |
| `data` | `array` | occupancy | $-1 \dots 100$ | Row-major occupancy probabilities |
| `occupied_cells` | `array` | meters | `[[x, y], ...]` | Filtered obstacle coordinates for canvas |
| `free_cells` | `array` | meters | `[[x, y], ...]` | Subsampled explored free space points |
| `slam_mode` | `string` | — | `"REALTIME"`, `"SIM"` | Map generation mode |

### C. SurvivorData (`type: "survivor_detected"` / `"survivor_update"`)
* **Frequency**: Event-driven / 2 Hz
* **Source**: `fusion_node` (`/confirmed_survivors`) + `grid_mapper_node`

| Field | Type | Unit | Range / Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `id` | `string` | — | `"S01"` $\dots$ `"S06+"` | Monotonic unique survivor identifier |
| `status` | `string` | — | Enum | `CONFIRMED`, `TRACKING`, `DETECTED`, `LOST` |
| `x` | `float` | meters | World map frame | Metric X coordinate |
| `y` | `float` | meters | World map frame | Metric Y coordinate |
| `grid_cell` | `string` | — | `"A1"` $\dots$ `"F6"` | Arena 1-meter cell |
| `confidence` | `float` | probability | $0.00 \dots 1.00$ | Fused multi-modal confidence |
| `source` | `string` | — | Enum | `"RGB"`, `"THERMAL"`, `"RGB + THERMAL"` |
| `timestamp` | `string` | — | `HH:MM:SS` | Detection time string |

### D. MissionState (`type: "mission_status"`)
* **Frequency**: 1 Hz
* **Source**: `exploration_node` (`/exploration_status`)

| Field | Type | Unit | Range / Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `state` | `string` | — | Enum | `IDLE`, `EXPLORING`, `EXITING`, `COMPLETE` |
| `elapsed_time`| `string` | — | `MM:SS` | Time elapsed since mission start |
| `progress` | `float` | ratio | $0.00 \dots 1.00$ | Explored arena fraction |
| `survivor_count`| `integer`| count | $0 \dots 6$ | Total confirmed survivors discovered |

### E. FailsafeState (`type: "failsafe_status"`)
* **Frequency**: 2 Hz
* **Source**: `failsafe_node` (`/failsafe/status`)

| Field | Type | Unit | Range / Format | Description |
| :--- | :--- | :--- | :--- | :--- |
| `active` | `boolean`| — | `true` / `false` | True if emergency failsafe is engaged |
| `state` | `string` | — | Enum | `NORMAL`, `WARN`, `HOLD`, `BRAKE`, `RETURN` |
| `reason` | `string` | — | Text | Trigger justification |
| `action` | `string` | — | Text | Autonomous recovery procedure commanded |
| `details` | `object` | — | Key-value pairs | Sensor health diagnostics, obstacle distance |
