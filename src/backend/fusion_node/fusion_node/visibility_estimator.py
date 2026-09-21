#!/usr/bin/env python3
"""
NIDAR AirMouse — Visibility Estimator
Determines whether the arena has NORMAL or DEGRADED visibility based on
the average brightness of the RGB camera frame.

Algorithm:
    average_brightness = mean(all pixel values in grayscale frame)
    if average_brightness > threshold → NORMAL
    if average_brightness <= threshold → DEGRADED
    update every N frames only (saves CPU on Raspberry Pi 4)
"""

from enum import Enum
import numpy as np


class VisibilityState(Enum):
    """Current visibility condition in the arena."""
    NORMAL = "NORMAL"        # Adequate lighting — RGB is primary detector
    DEGRADED = "DEGRADED"    # Smoke/darkness — thermal becomes primary detector


class VisibilityEstimator:
    """
    Estimates arena visibility from camera frame brightness.

    Design rationale:
    - Runs every N frames (not every frame) to save CPU on RPi 4
    - Uses simple mean brightness — fast O(pixels) computation
    - Hysteresis could be added later but not needed for 15×15m arena
      where lighting transitions are abrupt (smoke grenade, lights off)
    """

    def __init__(self, brightness_threshold: int = 50, update_interval: int = 10):
        """
        Args:
            brightness_threshold: Pixel brightness cutoff (0-255).
                                  Above → NORMAL, at or below → DEGRADED.
            update_interval: Only recompute brightness every N frames.
                             Saves ~90% CPU vs per-frame computation.
        """
        # Configurable threshold — tunable via ROS 2 parameter
        self.brightness_threshold = brightness_threshold

        # Frame skip counter — only recompute every N frames
        self.update_interval = update_interval

        # Internal state
        self._frame_counter = 0                          # Counts frames since last update
        self._current_state = VisibilityState.NORMAL     # Default to NORMAL until first frame
        self._last_brightness = 255.0                    # Default brightness (assume lit)

    def update(self, frame: np.ndarray) -> VisibilityState:
        """
        Process an incoming camera frame and return the current visibility state.

        Only recalculates brightness every `update_interval` frames.
        On skipped frames, returns the cached state.

        Args:
            frame: RGB or grayscale camera frame as numpy array.
                   Shape: (H, W, 3) for RGB or (H, W) for grayscale.
                   dtype: uint8

        Returns:
            VisibilityState.NORMAL or VisibilityState.DEGRADED
        """
        # Increment frame counter
        self._frame_counter += 1

        # Only recompute on every Nth frame to save CPU
        if self._frame_counter < self.update_interval:
            return self._current_state

        # Reset counter — time to recompute
        self._frame_counter = 0

        # Convert RGB to grayscale if needed (weighted luminance)
        # Using simple mean instead of cv2.cvtColor to avoid OpenCV dependency
        # in the estimator itself — keeps it testable without cv2
        if frame.ndim == 3:
            # Average across color channels — fast approximation of luminance
            grayscale = np.mean(frame, axis=2)
        else:
            # Already grayscale
            grayscale = frame.astype(np.float64)

        # Calculate mean brightness across entire frame
        self._last_brightness = float(np.mean(grayscale))

        # Threshold decision: above → NORMAL, at or below → DEGRADED
        if self._last_brightness > self.brightness_threshold:
            self._current_state = VisibilityState.NORMAL
        else:
            self._current_state = VisibilityState.DEGRADED

        return self._current_state

    @property
    def current_state(self) -> VisibilityState:
        """Return the last computed visibility state without processing a new frame."""
        return self._current_state

    @property
    def last_brightness(self) -> float:
        """Return the last computed average brightness value (0-255)."""
        return self._last_brightness

    def reset(self):
        """Reset estimator to initial state. Useful for testing."""
        self._frame_counter = 0
        self._current_state = VisibilityState.NORMAL
        self._last_brightness = 255.0
