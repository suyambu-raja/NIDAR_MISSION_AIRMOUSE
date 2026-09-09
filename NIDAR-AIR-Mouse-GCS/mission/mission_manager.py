"""
Mission Manager for NIDAR AirMouse GCS.
Manages the deterministic mission state transitions, execution timeline, and progress calculation.
"""
import time
from typing import Optional
from PySide6.QtCore import QObject, Signal, QTimer
from mission.mission_state import MissionState, VALID_TRANSITIONS
from communication.message_models import MissionEvent


class MissionManager(QObject):
    """
    Central Mission State Machine Controller.
    Ensures safe transitions, tracks mission clock, and computes mission progress.
    """
    state_changed = Signal(str, str)  # (old_state_str, new_state_str)
    progress_updated = Signal(float)   # progress 0.0 to 1.0
    timer_updated = Signal(str, float) # (formatted_time "MM:SS", raw_seconds)
    event_logged = Signal(object)      # MissionEvent

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_state = MissionState.IDLE
        self._start_time: Optional[float] = None
        self._elapsed_time: float = 0.0
        self._survivor_count: int = 0
        self._progress: float = 0.0

        # High-frequency timer for mission clock
        self._clock_timer = QTimer(self)
        self._clock_timer.setInterval(1000)  # 1 Hz tick
        self._clock_timer.timeout.connect(self._on_clock_tick)

    @property
    def current_state(self) -> MissionState:
        return self._current_state

    @property
    def is_active(self) -> bool:
        """Returns True if mission is currently in flight/executing."""
        return self._current_state in {
            MissionState.TAKEOFF,
            MissionState.ENTERING,
            MissionState.EXPLORING,
            MissionState.SURVIVOR_DETECTED,
            MissionState.EXITING,
        }

    def transition_to(self, new_state: MissionState, reason: str = "") -> bool:
        """
        Attempts a state transition according to transition rules.
        
        Args:
            new_state: Target MissionState.
            reason: Optional explanation string.
            
        Returns:
            True if transition succeeded, False if rejected.
        """
        old_state = self._current_state
        if old_state == new_state:
            return True

        # Check validity (Allow ABORTED from any active state)
        allowed = VALID_TRANSITIONS.get(old_state, set())
        if new_state != MissionState.ABORTED and new_state not in allowed:
            self._log_event(
                "WARNING",
                "MISSION",
                f"Illegal state transition rejected: {old_state.value} -> {new_state.value}"
            )
            return False

        self._current_state = new_state

        # Handle timer start/stop
        if new_state == MissionState.TAKEOFF and self._start_time is None:
            self._start_time = time.time()
            self._clock_timer.start()
        elif new_state in {MissionState.MISSION_COMPLETE, MissionState.ABORTED, MissionState.IDLE}:
            self._clock_timer.stop()

        # Update progress estimation based on state
        self._update_progress_for_state(new_state)

        # Log event
        msg = f"State: {old_state.value} -> {new_state.value}"
        if reason:
            msg += f" ({reason})"
        severity = "CRITICAL" if new_state == MissionState.ABORTED else "INFO"
        self._log_event(severity, "MISSION", msg)

        # Emit Qt signal
        self.state_changed.emit(old_state.value, new_state.value)
        return True

    def start_mission(self) -> bool:
        """Operator command to commence autonomous search mission."""
        if self._current_state not in {MissionState.READY, MissionState.SYSTEM_CHECK, MissionState.IDLE}:
            self._log_event("WARNING", "MISSION", f"Cannot start mission from state {self._current_state.value}")
            return False

        self._start_time = time.time()
        self._elapsed_time = 0.0
        self._survivor_count = 0
        self._progress = 0.0
        self._clock_timer.start()

        return self.transition_to(MissionState.TAKEOFF, "Operator initiated mission start")

    def abort_mission(self, reason: str = "Manual Emergency Abort") -> bool:
        """Operator or safety system emergency abort."""
        self._log_event("CRITICAL", "SAFETY", f"EMERGENCY ABORT TRIGGERED: {reason}")
        return self.transition_to(MissionState.ABORTED, reason)

    def reset_mission(self):
        """Resets mission manager back to IDLE state."""
        self._clock_timer.stop()
        self._start_time = None
        self._elapsed_time = 0.0
        self._survivor_count = 0
        self._progress = 0.0
        self._current_state = MissionState.IDLE
        self.state_changed.emit("RESET", MissionState.IDLE.value)
        self.progress_updated.emit(0.0)
        self.timer_updated.emit("00:00", 0.0)
        self._log_event("INFO", "MISSION", "Mission state reset to IDLE")

    def increment_survivor_count(self):
        """Called when a new survivor is registered."""
        self._survivor_count += 1
        self._log_event("SUCCESS", "SURVIVOR", f"Survivor count updated: {self._survivor_count}")

    def set_progress(self, progress: float):
        """Explicitly sets mission progress percentage (0.0 to 1.0)."""
        self._progress = max(0.0, min(1.0, float(progress)))
        self.progress_updated.emit(self._progress)

    def _update_progress_for_state(self, state: MissionState):
        """Heuristic progress estimate based on mission phase."""
        progress_map = {
            MissionState.IDLE: 0.0,
            MissionState.SYSTEM_CHECK: 0.05,
            MissionState.READY: 0.10,
            MissionState.TAKEOFF: 0.15,
            MissionState.ENTERING: 0.25,
            MissionState.EXPLORING: max(self._progress, 0.35),
            MissionState.SURVIVOR_DETECTED: max(self._progress, 0.60),
            MissionState.EXITING: 0.85,
            MissionState.MISSION_COMPLETE: 1.0,
        }
        if state in progress_map:
            self.set_progress(progress_map[state])

    def _on_clock_tick(self):
        """Updates elapsed mission time every second."""
        if self._start_time is not None:
            self._elapsed_time = time.time() - self._start_time
            mins = int(self._elapsed_time // 60)
            secs = int(self._elapsed_time % 60)
            time_str = f"{mins:02d}:{secs:02d}"
            self.timer_updated.emit(time_str, self._elapsed_time)

    def _log_event(self, level: str, category: str, message: str):
        evt = MissionEvent(
            timestamp=time.time(),
            level=level,
            category=category,
            message=message,
        )
        self.event_logged.emit(evt)
