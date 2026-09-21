"""
Simulated Survivor Provider for NIDAR AirMouse GCS.
Simulates autonomous survivor detection by OAK-D / thermal vision when drone explores target rooms.
"""
import math
import time
from typing import List, Dict, Set
from vision.survivor_provider import BaseSurvivorProvider
from communication.message_models import SurvivorDetection
from mapping.grid_manager import GridManager


class SimulatedSurvivorProvider(BaseSurvivorProvider):
    """
    Simulates onboard computer vision AI detecting survivors in designated rooms.
    """

    def __init__(self, grid_manager: GridManager, parent=None):
        super().__init__(parent)
        self.grid_manager = grid_manager

        # Ground truth survivor targets inside the maze
        self._ground_truth_survivors = [
            {
                "id": "S1",
                "x": 3.75,
                "y": 6.25,
                "confidence": 0.95,
                "source": "AI_OAKD_RGB",
                "detection_radius": 2.8,
            },
            {
                "id": "S2",
                "x": 13.75,
                "y": 11.25,
                "confidence": 0.92,
                "source": "FLIR_THERMAL",
                "detection_radius": 2.8,
            },
        ]

        self._detected_ids: Set[str] = set()

    def reset(self):
        """Clears detected survivors list for new mission run."""
        self._detected_ids.clear()

    def update_drone_pose(self, drone_x: float, drone_y: float) -> List[SurvivorDetection]:
        """
        Checks proximity between drone and simulated survivors.
        Emits `survivor_detected` if a new survivor comes within sensor range.
        """
        if not self._is_running:
            return []

        new_detections = []
        for target in self._ground_truth_survivors:
            s_id = target["id"]
            if s_id in self._detected_ids:
                continue

            dist = math.hypot(target["x"] - drone_x, target["y"] - drone_y)
            if dist <= target["detection_radius"]:
                self._detected_ids.add(s_id)
                grid_cell = self.grid_manager.metric_to_grid(target["x"], target["y"])

                detection = SurvivorDetection(
                    survivor_id=s_id,
                    x=target["x"],
                    y=target["y"],
                    grid_cell=grid_cell,
                    confidence=target["confidence"],
                    timestamp=time.time(),
                    status="CONFIRMED",
                    source=target["source"],
                    bbox=(180, 120, 320, 360),  # Synthetic bounding box in camera frame
                )
                new_detections.append(detection)
                self.survivor_detected.emit(detection)

        return new_detections

    def get_active_survivor_in_view(self, drone_x: float, drone_y: float) -> Optional[Dict]:
        """Returns the survivor target currently in view of the camera (if any)."""
        for target in self._ground_truth_survivors:
            dist = math.hypot(target["x"] - drone_x, target["y"] - drone_y)
            if dist <= target["detection_radius"]:
                return target
        return None
