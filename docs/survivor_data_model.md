# NIDAR M2 — Survivor Data Model & Lifecycle Specification

## 1. Single Source of Truth Architecture

Survivor detection, identification, and spatial deduplication are strictly governed by `fusion_node` (`Deduplicator`). No downstream node (including `grid_mapper_node` or the Custom GCS) is permitted to modify survivor IDs, re-cluster positions, or create competing registries.

```
       [ RGB Camera ]           [ Thermal Micro-bolometer ]
             │                                │
             ▼                                ▼
       [ YOLOv8 + ByteTrack ]        [ YOLO11n Thermal ]
             │                                │
             └───► [ Multi-Modal Fusion Engine ] ◄───┘
                           │
             Spatial Alignment (d <= 0.5 m)
             Confidence Fusion (EMA: w_rgb * c_rgb + w_th * c_th)
                           │
                           ▼
             [ Survivor Deduplicator (fusion_node) ]
             • Assigns Monotonic Sequential Survivor ID (1..6)
             • Suppresses Duplicates within Euclidean 0.5 m
             • Updates Running Average Confidence
                           │
                           ▼ Published on /confirmed_survivors
      +────────────────────┴─────────────────────+
      │                                          │
      ▼                                          ▼
[ grid_mapper_node ]                    [ GCS Bridge ]
• Projects metric (x,y) to 1m grid                 │
• Labels arena cell (e.g. "F4")                   ▼
• Never re-indexes IDs                 [ Custom Rescue GCS ]
      │                                • Renders S01..S06+ Cards
      ▼ Published on                   • Renders Map Pin
 /survivor_grid_locations              • Preserves Status & Timestamps
```

---

## 2. Survivor Lifecycle States

The NIDAR M2 backend and GCS implement the following formal survivor lifecycle states:

```
                  ┌──────────────┐
                  │   DETECTED   │ (Single-modality candidate detection)
                  └──────┬───────┘
                         │ Multi-frame observation
                         ▼
                  ┌──────────────┐
                  │   TRACKING   │ (ByteTrack persistent spatial track active)
                  └──────┬───────┘
                         │ Spatial alignment (d <= 0.5m) + Multi-modal agreement
                         ▼
                  ┌──────────────┐
                  │  CONFIRMED   │ (Fused RGB + Thermal, Deduplicated, ID assigned)
                  └──────┬───────┘
                         │
        ┌────────────────┴────────────────┐
        ▼ Target out of FOV > 5s          ▼ Rescuer on-site
 ┌──────────────┐                  ┌──────────────┐
 │     LOST     │                  │   RESCUED    │
 └──────┬───────┘                  └──────────────┘
        │ Re-detected within 0.5m
        ▼
 ┌──────────────┐
 │  REACQUIRED  │
 └──────────────┘
```

| State | Definition | Backend Origin | GCS Badge Color |
| :--- | :--- | :--- | :--- |
| `DETECTED` | Single-modality candidate identified (confidence $\ge 0.50$). | `tracker_node` / `thermal_node` | Amber (`#f59e0b`) |
| `TRACKING` | Candidate tracked continuously across consecutive frames. | `tracker_node` | Blue (`#3b82f6`) |
| `CONFIRMED` | Fused RGB + Thermal confirmation, passed $0.5\text{ m}$ spatial dedup gate. | `fusion_node` | Green (`#10b981`) |
| `LOST` | Previously confirmed survivor no longer visible in current camera FOV. | GCS Watchdog / Tracker | Gray (`#6b7280`) |
| `REACQUIRED` | Confirmed survivor re-observed within $0.5\text{ m}$ of known location. | `fusion_node` | Cyan (`#06b6d4`) |
| `RESCUED` | First responders have reached the victim and extracted them. | Operator GCS action | Purple (`#8b5cf6`) |

---

## 3. Survivor Data Schema (WebSocket & ROS 2)

```json
{
  "id": 1,
  "label": "S01",
  "status": "CONFIRMED",
  "confidence": 0.94,
  "confidence_pct": 94,
  "grid_cell": "F4",
  "x": 4.52,
  "y": 2.18,
  "z": 0.0,
  "modality": "RGB + THERMAL",
  "detection_count": 48,
  "track_status": "ACTIVE",
  "first_seen": "08:21",
  "last_seen": "08:39",
  "distance_m": 3.8
}
```
