"""
Simulated Telemetry Provider for NIDAR AirMouse GCS.
Generates realistic 10 Hz telemetry for GPS-denied indoor autonomous navigation.
"""
import math
import random
import time
from typing import List, Tuple, Optional
from PySide6.QtCore import QObject, Signal, QTimer
from communication.message_models import TelemetryData
from mission.mission_state import MissionState


class SimulatedTelemetryProvider(QObject):
    """
    Simulates Pixhawk 6C MAVLink telemetry and companion computer odometry.
    Follows an indoor waypoint trajectory through corridors and rooms.
    """
    telemetry_updated = Signal(object)  # Emits TelemetryData

    def __init__(
        self,
        speed_mps: float = 0.7,
        battery_drain_rate_pct_per_min: float = 3.5,
        parent=None,
    ):
        super().__init__(parent)
        self.speed_mps = speed_mps
        self.battery_drain_rate = battery_drain_rate_pct_per_min

        # Indoor Waypoint Path (X, Y in meters)
        self.waypoints: List[Tuple[float, float]] = [
            (1.25, 1.25),    # 0: Entrance / Staging
            (2.50, 2.50),    # 1: Corridor 1
            (2.50, 7.50),    # 2: Hallway Junction
            (3.75, 6.25),    # 3: Room 1 (Survivor S1)
            (2.50, 7.50),    # 4: Return to Junction
            (2.50, 12.50),   # 5: North Corridor
            (7.50, 12.50),   # 6: Central Crossover
            (7.50, 7.50),    # 7: Central Hall
            (12.50, 7.50),   # 8: East Junction
            (13.75, 11.25),  # 9: Room 2 (Survivor S2)
            (12.50, 12.50),  # 10: East Corridor North
            (17.50, 12.50),  # 11: Exit Corridor
            (17.50, 17.50),  # 12: Approach Exit
            (18.75, 18.75),  # 13: Exit Area
        ]

        self._curr_wp_idx = 0
        self._x = self.waypoints[0][0]
        self._y = self.waypoints[0][1]
        self._z = 0.0
        self._altitude = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._ground_speed = 0.0

        self._battery_pct = 98.0
        self._battery_v = 16.6
        self._battery_a = 0.5
        self._armed = False
        self._flight_mode = "DISARMED"

        self._mission_state = MissionState.IDLE
        self._is_running = False

        # 10 Hz Telemetry Loop
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._step_physics)

    def set_mission_state(self, state: MissionState):
        """Updates internal flight dynamics based on current mission state."""
        self._mission_state = state

        if state == MissionState.TAKEOFF:
            self._armed = True
            self._flight_mode = "AUTO_TAKEOFF"
        elif state in {MissionState.ENTERING, MissionState.EXPLORING, MissionState.SURVIVOR_DETECTED}:
            self._armed = True
            self._flight_mode = "AUTO_EXPLORE"
        elif state == MissionState.EXITING:
            self._armed = True
            self._flight_mode = "AUTO_EXIT"
        elif state == MissionState.MISSION_COMPLETE:
            self._flight_mode = "LANDED"
            self._armed = False
            self._ground_speed = 0.0
        elif state == MissionState.ABORTED:
            self._flight_mode = "EMERGENCY_HOLD"
            self._ground_speed = 0.0

    def start(self):
        self._is_running = True
        self._timer.start()

    def stop(self):
        self._is_running = False
        self._timer.stop()

    def reset(self):
        """Resets drone to initial staging coordinate."""
        self._curr_wp_idx = 0
        self._x = self.waypoints[0][0]
        self._y = self.waypoints[0][1]
        self._z = 0.0
        self._altitude = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._ground_speed = 0.0
        self._battery_pct = 98.0
        self._battery_v = 16.6
        self._battery_a = 0.5
        self._armed = False
        self._flight_mode = "DISARMED"
        self._mission_state = MissionState.IDLE

    def _step_physics(self):
        """Advances simulated drone kinematics by dt = 0.1s."""
        dt = 0.1

        # Battery consumption simulation
        if self._armed:
            drain_per_sec = (self.battery_drain_rate / 60.0)
            self._battery_pct = max(0.0, self._battery_pct - drain_per_sec * dt)
            self._battery_v = 13.5 + (self._battery_pct / 100.0) * 3.3  # 13.5V to 16.8V
            self._battery_a = 14.5 + random.uniform(-0.5, 0.5)
        else:
            self._battery_a = 0.8 + random.uniform(-0.05, 0.05)

        # Altitude Dynamics
        target_alt = 0.0
        if self._mission_state in {
            MissionState.TAKEOFF,
            MissionState.ENTERING,
            MissionState.EXPLORING,
            MissionState.SURVIVOR_DETECTED,
            MissionState.EXITING,
        }:
            target_alt = 1.25  # 1.25m standard indoor cruising altitude
        elif self._mission_state == MissionState.ABORTED:
            target_alt = 0.0  # Land on abort
        elif self._mission_state == MissionState.MISSION_COMPLETE:
            target_alt = 0.0  # Land on completion

        # Smooth vertical climb/descent
        alt_err = target_alt - self._altitude
        self._altitude += max(-0.6 * dt, min(0.6 * dt, alt_err * 0.5))
        self._z = self._altitude

        # Horizontal Trajectory Dynamics (Only during active navigation)
        if self._mission_state in {
            MissionState.TAKEOFF,
            MissionState.ENTERING,
            MissionState.EXPLORING,
            MissionState.SURVIVOR_DETECTED,
            MissionState.EXITING,
        } and self._altitude > 0.4:
            target_wp = self.waypoints[self._curr_wp_idx]
            dx = target_wp[0] - self._x
            dy = target_wp[1] - self._y
            dist = math.hypot(dx, dy)

            if dist < 0.25:
                # Advance to next waypoint
                if self._curr_wp_idx < len(self.waypoints) - 1:
                    self._curr_wp_idx += 1
            else:
                target_yaw_rad = math.atan2(dy, dx)
                target_yaw_deg = math.degrees(target_yaw_rad) % 360.0

                # Smooth heading rotation
                yaw_diff = (target_yaw_deg - self._yaw + 180.0) % 360.0 - 180.0
                self._yaw = (self._yaw + max(-90.0 * dt, min(90.0 * dt, yaw_diff * 0.8))) % 360.0

                # Move forward along target vector
                move_dist = min(self.speed_mps * dt, dist)
                self._x += math.cos(math.radians(self._yaw)) * move_dist
                self._y += math.sin(math.radians(self._yaw)) * move_dist
                self._ground_speed = self.speed_mps + random.uniform(-0.02, 0.02)

                # Tilt (Roll / Pitch) slight natural bank
                self._pitch = -3.5 + random.uniform(-0.5, 0.5)
                self._roll = math.sin(time.time() * 2.0) * 1.5
        else:
            self._ground_speed = 0.0
            self._pitch = random.uniform(-0.2, 0.2)
            self._roll = random.uniform(-0.2, 0.2)

        # Build Telemetry snapshot
        telem = TelemetryData(
            timestamp=time.time(),
            armed=self._armed,
            flight_mode=self._flight_mode,
            x_m=self._x,
            y_m=self._y,
            z_m=self._z,
            altitude_m=self._altitude + (random.uniform(-0.015, 0.015) if self._armed else 0.0),
            ground_speed_mps=self._ground_speed,
            heading_deg=self._yaw,
            roll_deg=self._roll,
            pitch_deg=self._pitch,
            yaw_deg=self._yaw,
            battery_voltage_v=self._battery_v,
            battery_current_a=self._battery_a,
            battery_percentage=self._battery_pct,
            ekf_healthy=True,
            optical_flow_healthy=True,
            rangefinder_healthy=True,
            lidar_healthy=True,
            oak_d_healthy=True,
            thermal_healthy=True,
            mavlink_connected=True,
            video_connected=True,
        )

        self.telemetry_updated.emit(telem)

    def get_current_pose(self) -> Tuple[float, float, float]:
        """Returns (x_m, y_m, yaw_deg)."""
        return self._x, self._y, self._yaw

    def is_at_exit(self) -> bool:
        """Returns True if drone reached the final waypoint near exit."""
        return self._curr_wp_idx >= len(self.waypoints) - 1 and math.hypot(
            self.waypoints[-1][0] - self._x,
            self.waypoints[-1][1] - self._y
        ) < 0.4
