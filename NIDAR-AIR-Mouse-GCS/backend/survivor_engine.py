"""
Survivor Detection, De-duplication, and RGB/Thermal Correlation Engine for NIDAR AirMouse GCS.
Manages the autonomous identification and evidence linking for up to 6 survivors.
"""
import math
import time
from typing import Dict, List, Optional, Tuple
from backend.schemas import SurvivorPayload
from mapping.grid_manager import GridManager


class SurvivorEngine:
    """
    Validates autonomous detection packets from OAK-D / Thermal vision,
    links multimodal evidence, eliminates duplicates, and maintains persistent survivor state.
    """

    def __init__(self, grid_manager: GridManager, max_survivors: Optional[int] = None):
        self.grid_manager = grid_manager
        self.max_survivors = max_survivors
        self.survivors: Dict[str, SurvivorPayload] = {}
        self.duplicate_distance_threshold_m = 1.8  # Merge detections within 1.8m

    def reset(self):
        self.survivors.clear()

    @property
    def survivor_count(self) -> int:
        return len(self.survivors)

    def process_detection(
        self,
        local_x: float,
        local_y: float,
        confidence: float,
        source: str = "THERMAL",
        survivor_id_hint: Optional[str] = None,
        rgb_time: Optional[str] = None,
        thermal_time: Optional[str] = None,
    ) -> Tuple[SurvivorPayload, bool]:
        """
        Processes a raw detection event.
        
        Returns:
            Tuple of (SurvivorPayload, is_new: bool).
        """
        now_str = time.strftime("%H:%M:%S")

        # 1. Check for spatial duplicate / existing survivor nearby
        for s_id, existing in self.survivors.items():
            dist = math.hypot(existing.local_x - local_x, existing.local_y - local_y)
            if dist <= self.duplicate_distance_threshold_m or (survivor_id_hint and s_id == survivor_id_hint):
                # Update existing survivor (increase confidence or fuse source)
                if confidence > existing.confidence:
                    existing.confidence = confidence
                if existing.source != source:
                    existing.source = "FUSED_AI"
                if source == "RGB" and not existing.rgb_evidence_time:
                    existing.rgb_evidence_time = rgb_time or now_str
                elif source == "THERMAL" and not existing.thermal_evidence_time:
                    existing.thermal_evidence_time = thermal_time or now_str

                existing.status = "CONFIRMED"
                return existing, False

        # 2. Assign next available Survivor ID (S01, S02, S03...)
        if survivor_id_hint and survivor_id_hint not in self.survivors:
            assigned_id = survivor_id_hint
        else:
            idx = len(self.survivors) + 1
            if self.max_survivors and idx > self.max_survivors:
                idx = self.max_survivors
            assigned_id = f"S{idx:02d}"

        # 3. Calculate Search Grid coordinates
        grid_cell = self.grid_manager.metric_to_grid(local_x, local_y)
        # Parse grid column and row number
        col_char = grid_cell[0] if len(grid_cell) >= 1 else "A"
        row_str = grid_cell[1:] if len(grid_cell) >= 2 else "1"
        try:
            grid_x = ord(col_char.upper()) - ord('A') + 1
            grid_y = int(row_str)
        except Exception:
            grid_x, grid_y = 1, 1

        payload = SurvivorPayload(
            id=assigned_id,
            local_x=round(local_x, 2),
            local_y=round(local_y, 2),
            grid_x=grid_x,
            grid_y=grid_y,
            grid_cell=grid_cell,
            confidence=round(confidence, 2),
            source=source.upper(),
            status="CONFIRMED",
            timestamp=now_str,
            rgb_evidence_time=rgb_time or (now_str if source == "RGB" else None),
            thermal_evidence_time=thermal_time or (now_str if source == "THERMAL" else None),
        )

        self.survivors[assigned_id] = payload
        return payload, True

    def get_all_survivors(self) -> List[SurvivorPayload]:
        return list(self.survivors.values())
