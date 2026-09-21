# NIDAR M2 — Target Hardware Integration Status

## 1. Hardware Readiness Classification

Each hardware subsystem is strictly categorized according to verified operational readiness:
* **SOFTWARE READY**: ROS 2 driver nodes, interfaces, telemetry parsers, and GCS UI pipelines are fully implemented and verified with mock/SITL test fixtures.
* **HARDWARE VERIFIED**: Validated on physical hardware under bench or flight conditions.
* **HARDWARE PENDING**: Physical wiring, optical bench calibration, or flight range tuning is required.

---

## 2. Hardware Subsystem Status Matrix

| Subsystem / Device | Role in NIDAR M2 | Onboard Interface | Driver / ROS Node | Readiness Level | Current Verification Detail |
| :--- | :--- | :--- | :--- | :---: | :--- |
| **Pixhawk 6C** | Primary Flight Controller (ArduPilot) | UART / USB (`/dev/ttyACM0`) | `mavros_bridge_node` | **SOFTWARE READY** | MAVLink message translation, GUIDED mode setpoints, arm/disarm, and battery telemetry verified. Physical flight tuning pending. |
| **Matek Optical Flow (3901-L0X)** | Lateral Velocity State Estimation ($v_x, v_y$) | MSP / I2C to Pixhawk | ArduPilot EKF3 Optical Flow | **SOFTWARE READY** | Velocity conversion and quality metrics piped to EKF and `/drone_pose`. Physical indoor ground texture testing pending. |
| **VL53L0X Laser Rangefinder** | Downward Height Above Ground ($z$) | I2C / Serial to Pixhawk | ArduPilot Rangefinder | **SOFTWARE READY** | Altitude telemetry and ground distance thresholds verified in software. Optical surface reflectivity calibration pending. |
| **RPLIDAR A2M8** | 2D LiDAR SLAM & Obstacle Avoidance | USB (`/dev/ttyUSB0`) | `slam_node` / `rplidar_launch.py` | **SOFTWARE READY** | Occupancy grid generation, obstacle extraction ($0.05\text{m}$), and scan dropout monitoring implemented. Bench verified. |
| **Luxonis OAK-D Lite** | Primary RGB Camera & Depth Sensor | USB 3.0 Type-C | `detection_node` (YOLOv8) | **SOFTWARE READY** | RGB camera frames, bounding boxes, and ByteTrack continuous IDs verified. Bench stream validated. |
| **FLIR Lepton 3.5** | Radiometric Thermal Micro-bolometer | SPI + I2C (VoSPI) | `thermal_node` (YOLO11n) | **SOFTWARE READY** | Radiometric heat-signature parsing and Inferno colormap generation verified. Sensor calibration in heated chamber pending. |
| **Raspberry Pi 4 / 5 (8GB)** | Companion Autonomy Computer | Onboard Carrier / Header | Linux / ROS 2 Humble | **SOFTWARE READY** | Software stack runs autonomously in containerized environment. Thermal dissipation bench testing pending. |
| **RFD900x Telemetry Modem** | Long-Range Point-to-Point Telemetry | UART Serial (`57600/115200`) | GCS Radio Manager | **SOFTWARE READY** | RSSI/SNR signal diagnostics and packet serialization verified. Reinforced concrete penetration testing pending. |
| **ExpressLRS (ELRS 2.4G)** | Manual Safety Pilot Radio Link | CRSF / UART | ArduPilot RC In | **SOFTWARE READY** | Manual pilot override channel interlock verified in software. Field transmitter bind pending. |

---

## 3. Hardware Integration Action Plan

1. **Step 1 (Benchtop Bringup)**: Mount Raspberry Pi and Pixhawk on test stand; power via bench supply; verify bidirectional MAVLink communication at 921600 baud.
2. **Step 2 (Sensor Calibration)**: Calibrate optical flow focal length scale factor using moving textured ground target. Calibrate thermal flat-field correction (FFC).
3. **Step 3 (Tethered Flight)**: Conduct tethered hover in low-illumination GPS-denied arena; verify velocity hold stability and obstacle emergency braking at $0.45\text{ m}$.
