# NIDAR M2 — GCS Command Traceability Specification

## 1. Command Execution Flow

All operator commands initiated from the Custom GCS follow a secure, validated bidirectional pipeline:

```
[ GCS UI Button Click ]
          │
          ▼ JavaScript Event Handler (controls.js)
[ WebSocket JSON Frame ]
          │  Payload: {"action": "<CMD>", "params": {...}}
          ▼ Port 8765
[ Python GCSServer (server.py) ]
          │  Client message routing & permission guard
          ▼
[ ROS2IntegrationManager (ros2_integration_manager.py) ]
          │
          ▼
[ CommandBridgeNode (command_bridge.py) ]
          │  Translation, parameter clamping, and topic/service dispatch
          ▼
[ ROS 2 Service / Topic ]
          │
          ▼
[ Target Node (mavros_bridge / failsafe_node / exploration_node) ]
          │  MAVLink Setpoint / Mode / State Machine
          ▼
[ ArduPilot FCU (Pixhawk 6C) ]
          │
          ▼ Actuator Output (Motors / Servos)
[ Physical Drone Hardware ]
```

---

## 2. Command Traceability Registry

| Command | GCS Button | Frontend Handler | WebSocket Message | Bridge Handler | Target ROS 2 Interface | Target Backend Node | FCU Action |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **ARM** | `#btn-arm` | `controls.js:btnArm` | `{"action": "ARM"}` | `CommandBridgeNode.handle_command` | `/mavros/cmd/arming` (`Bool(True)`) | `mavros_bridge_node` | Pixhawk arms motor ESCs |
| **DISARM** | `#btn-disarm` | `controls.js:btnDisarm` | `{"action": "DISARM"}` | `CommandBridgeNode.handle_command` | `/mavros/cmd/arming` (`Bool(False)`) | `mavros_bridge_node` | Pixhawk disarms motor ESCs |
| **TAKEOFF** | `#btn-takeoff` | `controls.js:btnTakeoff` | `{"action": "TAKEOFF", "params": {"altitude": 1.5}}` | `CommandBridgeNode.handle_command` | `/takeoff` + `/mavros/set_mode` (`"GUIDED"`) | `mavros_bridge_node` | Ascends vertically to 1.5m and hovers |
| **LAND** | `#btn-land` | `controls.js:btnLand` | `{"action": "LAND"}` | `CommandBridgeNode.handle_command` | `/land` + `/mavros/set_mode` (`"LAND"`) | `mavros_bridge_node` | Decends vertically to ground, auto-disarms |
| **RTL / SAFE** | `#btn-rtl` | `controls.js:btnRtl` | `{"action": "RTL"}` | `CommandBridgeNode.handle_command` | `/rtl` + `/mavros/set_mode` (`"RTL"`) | `mavros_bridge_node` | Climbs to safe clearance and returns to launch pad |
| **PAUSE AUTONOMY**| `#btn-pause` | `controls.js:btnPause` | `{"action": "PAUSE"}` | `CommandBridgeNode.handle_command` | `/mavros/set_mode` (`"LOITER"`) | `mavros_bridge_node` | Freezes exploration goals; enters position hold |
| **RESUME AUTONOMY**| `#btn-resume` | `controls.js:btnResume` | `{"action": "RESUME"}` | `CommandBridgeNode.handle_command` | `/mavros/set_mode` (`"GUIDED"`) | `mavros_bridge_node` | Re-engages exploration target tracking |
| **EMERGENCY ABORT**| `#btn-abort-mission`| `controls.js:modalConfirm` | `{"action": "ABORT_MISSION", "params": {"reason": "..."}}` | `CommandBridgeNode.handle_command` | `/abort` + `/failsafe/abort` | `failsafe_node` / `mavros_bridge` | Immediately transitions to failsafe emergency state |
| **RESET MISSION** | `#btn-reset-mission`| `controls.js:btnReset` | `{"action": "RESET"}` | `CommandBridgeNode.handle_command` | `/failsafe/reset` | Internal State | Resets map canvas, clears survivor caches |
| **SET SLAM MODE** | `#slam-mode-select` | `controls.js:selectSlamMode` | `{"action": "CHANGE_SLAM_MODE", "params": {"mode": "..."}}`| `CommandBridgeNode.handle_command` | `/slam_mode_change` | GCS & SLAM nodes | Toggles live ROS 2 vs simulation playback |

---

## 3. Safety Interlocks & Operator Confirmation

1. **Modal Confirmation Guard**: High-consequence commands (`EMERGENCY ABORT`, `DISARM IN FLIGHT`, `RESET`) require explicit user confirmation through the tactile modal dialog (`#abort-modal`) to prevent accidental clicks.
2. **Deterministic Feedback Loop**: Upon command dispatch, the bridge returns immediate execution status (`"SUCCESS"` or `"FAILED"`) with diagnostic details. Commands never fail silently.
