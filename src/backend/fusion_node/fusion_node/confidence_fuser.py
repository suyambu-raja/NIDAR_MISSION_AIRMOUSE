#!/usr/bin/env python3
"""
NIDAR AirMouse — Confidence Fuser
Implements Weighted Evidence Fusion + Exponential Moving Average (EMA)
to calculate a combined confidence score for each detection.

Algorithm (Weighted Evidence Fusion):
    NORMAL visibility:
        matched pair:       fused = (RGB_conf × 0.7) + (thermal_conf × 0.3)
        unmatched RGB:      fused = RGB_conf × 0.8
        unmatched thermal:  fused = thermal_conf × 0.4   (low trust in NORMAL)

    DEGRADED visibility:
        matched pair:       fused = (RGB_conf × 0.3) + (thermal_conf × 0.7)
        unmatched thermal:  fused = thermal_conf × 0.8
        unmatched RGB:      fused = RGB_conf × 0.4       (low trust in DEGRADED)

Algorithm (EMA Smoothing):
    smoothed = (alpha × new_score) + ((1 - alpha) × previous_score)
    alpha = 0.3 (default) — heavy smoothing to reduce jitter

Design rationale:
    - Visibility-dependent weights reflect sensor reliability
    - Penalty factor (0.8) for unmatched primary sensor
    - Heavy penalty (0.4) for unmatched secondary sensor
    - EMA smoothing prevents single-frame spikes from triggering false confirms
"""

from typing import Optional, Tuple
from .visibility_estimator import VisibilityState


class ConfidenceFuser:
    """
    Fuses confidence scores from RGB and thermal detections using
    visibility-dependent weighting and EMA smoothing.
    """

    def __init__(
        self,
        rgb_weight_normal: float = 0.7,
        thermal_weight_normal: float = 0.3,
        rgb_weight_degraded: float = 0.3,
        thermal_weight_degraded: float = 0.7,
        unmatched_primary_factor: float = 0.8,
        unmatched_secondary_factor: float = 0.4,
        ema_alpha: float = 0.3,
        position_quantize: float = 0.3
    ):
        """
        Args:
            rgb_weight_normal: RGB weight when visibility is NORMAL
            thermal_weight_normal: Thermal weight when visibility is NORMAL
            rgb_weight_degraded: RGB weight when visibility is DEGRADED
            thermal_weight_degraded: Thermal weight when visibility is DEGRADED
            unmatched_primary_factor: Multiplier for unmatched primary sensor
                                      (RGB in NORMAL, thermal in DEGRADED)
            unmatched_secondary_factor: Multiplier for unmatched secondary sensor
                                         (thermal in NORMAL, RGB in DEGRADED)
            ema_alpha: Weight for new observation in EMA (0-1).
                       Lower = more smoothing. 0.3 means 30% new, 70% history.
            position_quantize: Grid size for EMA history keys (metres).
                               Detections within this distance share EMA state.
        """
        # Visibility-dependent fusion weights
        self.rgb_weight_normal = rgb_weight_normal
        self.thermal_weight_normal = thermal_weight_normal
        self.rgb_weight_degraded = rgb_weight_degraded
        self.thermal_weight_degraded = thermal_weight_degraded

        # Penalty factors for single-sensor detections
        self.unmatched_primary_factor = unmatched_primary_factor
        self.unmatched_secondary_factor = unmatched_secondary_factor

        # EMA smoothing parameter
        self.ema_alpha = ema_alpha

        # Position quantization for EMA history lookup
        self.position_quantize = position_quantize

        # EMA history: maps quantized (x, y) → previous smoothed confidence
        # This allows EMA to track confidence evolution per spatial location
        self._ema_history: dict[Tuple[int, int], float] = {}

    def _quantize_position(self, x: float, y: float) -> Tuple[int, int]:
        """
        Quantize a world position to a grid cell for EMA history lookup.
        Detections within `position_quantize` metres share the same EMA state.

        Args:
            x: World X position (metres)
            y: World Y position (metres)

        Returns:
            (grid_x, grid_y) integer grid cell
        """
        return (
            int(round(x / self.position_quantize)),
            int(round(y / self.position_quantize))
        )

    def fuse_matched(
        self,
        rgb_confidence: float,
        thermal_confidence: float,
        visibility: VisibilityState
    ) -> float:
        """
        Fuse confidence from a matched RGB + thermal detection pair.

        Formula:
            NORMAL:   fused = (RGB × 0.7) + (thermal × 0.3)
            DEGRADED: fused = (RGB × 0.3) + (thermal × 0.7)

        Args:
            rgb_confidence: RGB detection confidence (0.0–1.0)
            thermal_confidence: Thermal detection confidence (0.0–1.0)
            visibility: Current visibility state

        Returns:
            Fused confidence score (0.0–1.0)
        """
        if visibility == VisibilityState.NORMAL:
            return (rgb_confidence * self.rgb_weight_normal +
                    thermal_confidence * self.thermal_weight_normal)
        else:
            return (rgb_confidence * self.rgb_weight_degraded +
                    thermal_confidence * self.thermal_weight_degraded)

    def fuse_unmatched(
        self,
        confidence: float,
        source: str,
        visibility: VisibilityState
    ) -> float:
        """
        Calculate fused confidence for an unmatched single-sensor detection.

        Unmatched primary sensor (RGB in NORMAL, thermal in DEGRADED):
            fused = confidence × 0.8
        Unmatched secondary sensor (thermal in NORMAL, RGB in DEGRADED):
            fused = confidence × 0.4

        Args:
            confidence: Raw detection confidence (0.0–1.0)
            source: Detection source — "rgb" or "thermal"
            visibility: Current visibility state

        Returns:
            Fused confidence score (0.0–1.0)
        """
        # Determine if this source is primary or secondary for current visibility
        is_primary = (
            (source == "rgb" and visibility == VisibilityState.NORMAL) or
            (source == "thermal" and visibility == VisibilityState.DEGRADED)
        )

        if is_primary:
            # Primary sensor — moderate penalty (still fairly reliable alone)
            return confidence * self.unmatched_primary_factor
        else:
            # Secondary sensor — heavy penalty (low trust without corroboration)
            return confidence * self.unmatched_secondary_factor

    def apply_ema(
        self,
        raw_confidence: float,
        world_x: float,
        world_y: float
    ) -> float:
        """
        Apply Exponential Moving Average smoothing to a confidence score.

        Formula: smoothed = (alpha × new) + ((1 - alpha) × previous)

        EMA state is tracked per quantized world position, so the same
        physical location accumulates smoothing history across frames.

        Args:
            raw_confidence: New (un-smoothed) fused confidence score
            world_x: World X position (for EMA history lookup)
            world_y: World Y position (for EMA history lookup)

        Returns:
            EMA-smoothed confidence score
        """
        # Quantize position to find matching EMA history entry
        key = self._quantize_position(world_x, world_y)

        if key in self._ema_history:
            # Apply EMA: blend new score with historical score
            previous = self._ema_history[key]
            smoothed = (self.ema_alpha * raw_confidence +
                        (1.0 - self.ema_alpha) * previous)
        else:
            # First observation at this position — no history to blend with
            smoothed = raw_confidence

        # Store updated EMA value for next frame
        self._ema_history[key] = smoothed

        return smoothed

    def fuse(
        self,
        rgb_confidence: Optional[float],
        thermal_confidence: Optional[float],
        source: str,
        visibility: VisibilityState,
        world_x: float,
        world_y: float
    ) -> float:
        """
        Complete fusion pipeline: weighted fusion → EMA smoothing.

        This is the main entry point. Handles matched pairs and
        unmatched single-sensor detections.

        Args:
            rgb_confidence: RGB confidence (None if unmatched thermal)
            thermal_confidence: Thermal confidence (None if unmatched RGB)
            source: "fused" for matched pair, "rgb" or "thermal" for unmatched
            visibility: Current visibility state
            world_x: World X position (for EMA history)
            world_y: World Y position (for EMA history)

        Returns:
            Final EMA-smoothed fused confidence score
        """
        # Step 1: Weighted evidence fusion
        if source == "fused" and rgb_confidence is not None and thermal_confidence is not None:
            # Matched pair — use weighted combination
            raw = self.fuse_matched(rgb_confidence, thermal_confidence, visibility)
        elif rgb_confidence is not None:
            # Unmatched RGB detection
            raw = self.fuse_unmatched(rgb_confidence, "rgb", visibility)
        elif thermal_confidence is not None:
            # Unmatched thermal detection
            raw = self.fuse_unmatched(thermal_confidence, "thermal", visibility)
        else:
            # No confidence data — should not happen, but handle gracefully
            raw = 0.0

        # Step 2: EMA smoothing across frames
        smoothed = self.apply_ema(raw, world_x, world_y)

        return smoothed

    def reset(self):
        """Clear EMA history. Useful for testing or mission restart."""
        self._ema_history.clear()
