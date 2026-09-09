"""
Safety Manager for NIDAR AirMouse GCS.
Continuously monitors drone health, communications link liveness, battery levels,
and autonomous safety constraints.
"""
import time
from typing import Optional
from PySide6.QtCore import QObject, Signal, QTimer
from communication.message_models import TelemetryData, MissionEvent
from mission.mission_state import MissionState
from config.config_manager import SafetyConfig


class SafetyManager(QObject):
    """
    Dedicated safety supervisor for GPS-denied indoor operations.
    Monitors failsafes, link timeouts, battery reserves, and handles emergency abort triggers.
    """
    safety_alert = Signal(str, str, str)  # (level, category, message)
    emergency_abort_requested = Signal(str)  # (reason)

    def __init__(self, safety_config: Optional[SafetyConfig] = None, parent=None):
        super().__init__(parent)
        self.config = safety_config or SafetyConfig()

        self._last_telemetry_time: float = 0.0
        self._last_map_time: float = 0.0
        self._last_video_time: float = 0.0
        self._mission_state = MissionState.IDLE

        self._battery_warned = False
        self._battery_critical_warned = False
        self._comm_lost_warned = False

        # Watchdog check timer (runs at 2 Hz)
        self._watchdog_timer = QTimer(self)
        self._watchdog_timer.setInterval(500)
        self._watchdog_timer.timeout.connect(self._run_watchdog_checks)
        self._watchdog_timer.start()

    def set_mission_state(self, state: MissionState):
        """Updates internal state reference."""
        self._mission_state = state

    def update_telemetry(self, telem: TelemetryData):
        """Called upon receiving new telemetry."""
        now = time.time()
        self._last_telemetry_time = now

        # Check battery
        if telem.battery_percentage <= self.config.battery_critical_percent:
            if not self._battery_critical_warned:
                self._battery_critical_warned = True
                self.safety_alert.emit(
                    "CRITICAL",
                    "SAFETY",
                    f"CRITICAL BATTERY: {telem.battery_percentage:.1f}% ({telem.battery_voltage_v:.2f}V) - Immediate Land/Abort required!"
                )
                if self._mission_state in {
                    MissionState.TAKEOFF,
                    MissionState.ENTERING,
                    MissionState.EXPLORING,
                    MissionState.SURVIVOR_DETECTED,
                }:
                    self.emergency_abort_requested.emit("Critical battery threshold reached")
        elif telem.battery_percentage <= self.config.battery_warning_percent:
            if not self._battery_warned:
                self._battery_warned = True
                self.safety_alert.emit(
                    "WARNING",
                    "SAFETY",
                    f"LOW BATTERY WARNING: {telem.battery_percentage:.1f}% ({telem.battery_voltage_v:.2f}V) remaining."
                )

        # Check Estimator / EKF
        if not telem.ekf_healthy:
            self.safety_alert.emit(
                "ERROR",
                "SAFETY",
                "EKF / State Estimator report UNHEALTHY! GPS-denied position drift risk."
            )

        # Clear comm lost alert if reconnected
        if self._comm_lost_warned:
            self._comm_lost_warned = False
            self.safety_alert.emit("SUCCESS", "COMM", "MAVLink Telemetry link restored.")

    def update_map_received(self):
        """Called when a valid 2D map update is received."""
        self._last_map_time = time.time()

    def update_video_received(self):
        """Called when a new camera frame is received."""
        self._last_video_time = time.time()

    def trigger_emergency_abort(self, reason: str = "Operator Initiated Emergency Abort"):
        """Triggers immediate emergency abort."""
        self.safety_alert.emit("CRITICAL", "SAFETY", f"EMERGENCY ABORT TRIGGERED: {reason}")
        self.emergency_abort_requested.emit(reason)

    def _run_watchdog_checks(self):
        """Periodic watchdog verifying communication freshness during mission."""
        now = time.time()

        # Only check telemetry timeouts if we've received at least one packet
        if self._last_telemetry_time > 0:
            telem_age = now - self._last_telemetry_time
            if telem_age > self.config.telemetry_timeout_seconds:
                if not self._comm_lost_warned:
                    self._comm_lost_warned = True
                    self.safety_alert.emit(
                        "ERROR",
                        "COMM",
                        f"TELEMETRY TIMEOUT: No MAVLink packets received for {telem_age:.1f}s!"
                    )

        # Check map updates if actively exploring
        if self._mission_state in {MissionState.EXPLORING, MissionState.ENTERING}:
            if self._last_map_time > 0:
                map_age = now - self._last_map_time
                if map_age > self.config.map_timeout_seconds:
                    self.safety_alert.emit(
                        "WARNING",
                        "SLAM",
                        f"SLAM Map stream stale: no updates for {map_age:.1f}s."
                    )
