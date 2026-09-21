# NIDAR M2 — GCS Command & Control Contract

## 1. Command Dispatch Model

Every operator action initiated from the Custom GCS follows a deterministic Request-Acknowledge-Execute-Status loop. Commands are validated at the bridge before being translated to ROS 2 services or topics.

```
[ GCS Operator Action ]
          │  WebSocket JSON: {"type": "command", "name": "<CMD>", "params": {...}}
          ▼
[ GCS Command Bridge ]  ──(Schema validation & Parameter Clamp)──> [ REJECT if Invalid ]
          │
          ├──> Sends immediate ACK to GCS: {"type": "cmd_ack", "cmd": "<CMD>", "status": "PENDING"}
          │
          ▼ ROS 2 Service Call / Publisher
[ Target Onboard Subsystem ] (mavros_bridge / failsafe_node / exploration_node)
          │
          ├──> Service Response / Action Feedback
          ▼
[ GCS Command Bridge ]
          │
          └──> Sends Final Execution Status: {"type": "cmd_result", "cmd": "<CMD>", "status": "SUCCESS" | "FAILED"}
```

---

## 2. Command Reference Table

| GCS Command | Parameters | ROS 2 Target Interface | Interface Type | Safety Interlocks & Prerequisites | Flight Action |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `ARM` | None | `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Pre-arm checks pass; EKF healthy; Disarmed | Motors spin at idle |
| `DISARM` | None | `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Vehicle landed or emergency override confirmed | Motors stop immediately |
| `TAKEOFF` | `altitude: 1.5` | `/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | Vehicle Armed; Mode = `GUIDED_NOGPS` | Climbs vertically to target altitude (1.5 m) and hovers |
| `LAND` | None | `/mavros/cmd/land` | `mavros_msgs/srv/CommandTOL` | In flight | Initiates vertical descent and auto-disarms upon touchdown |
| `RTL` / `RETURN` | None | `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | In flight; Mode parameter = `"RTL"` | Climbs to safe clearance and returns to launch staging |
| `PAUSE_AUTONOMY` | None | `/exploration/pause` | `std_srvs/srv/Trigger` | Autonomy currently active | Freezes exploration goals; commands zero velocity hover |
| `RESUME_AUTONOMY` | None | `/exploration/resume` | `std_srvs/srv/Trigger` | Autonomy paused; vehicle healthy | Resumes frontier selection and A* trajectory execution |
| `ABORT_MISSION` | `reason: str` | `/failsafe/trigger` | `std_srvs/srv/Trigger` | Operator emergency abort confirmation | Triggers emergency failsafe state machine |
| `RESET_MISSION` | None | Internal GCS State | Internal Reset | Disarmed | Clears map cache, resets survivor display to staging |
| `SET_SLAM_MODE` | `mode: "REALTIME" \| "SIMULATION"` | WebSocket State | Internal Mode Switch | None | Toggles between live hardware and simulated feeds |

---

## 3. Communication Reliability & Timeouts

* **Command Acknowledgement Timeout**: 3000 ms. If no response is received from the onboard ROS 2 service within 3.0 seconds, the bridge generates an automatic `TIMEOUT` error notification to the GCS event log.
* **Never Silent Failure**: Any failed service response returns a descriptive human-readable failure reason that is immediately displayed in the GCS top banner and audit log.
* **Loss-of-Link Behavior**: If the WebSocket or telemetry stream disconnects for $\ge 3.0$ seconds, onboard autonomy continues uninterrupted according to onboard mission parameters. The drone does not depend on the GCS to maintain stability or avoid collisions.
