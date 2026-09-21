#!/usr/bin/env python3
"""
region_classifier.py — Geometric Segmentation and Classification Engine
========================================================================
Standalone utility (NOT a ROS 2 node) used by corridor_classifier_node.

Performs:
  1. Connected Component Analysis on OccupancyGrid FREE cells (0).
  2. Geometric feature extraction: Bounding box, width, length, area, aspect ratio, centroid.
  3. Rule-based classification matching mission arena dimensions:
     - Room: ~2m x 2m, aspect ratio ~1.0 (roughly square).
     - Corridor: width ~1m, length >> width (aspect ratio >= 1.8).
     - Junction: multi-branching intersection zones.
     - Unclassified: ambiguous or partial geometries.
  4. Grid box tagging (e.g. A1–O15) matching NIDAR CoordinateTransformer conventions.
"""

import math
from collections import deque
from typing import Dict, List, Optional, Tuple, Any

# Occupancy grid constants
CELL_FREE = 0
CELL_OCCUPIED = 100
CELL_UNKNOWN = -1

# Arena grid box conventions (15x15m, 1m/box)
GRID_BOX_SIZE_M: float = 1.0
ARENA_COLS: int = 15
ARENA_ROWS: int = 15


class RegionClassifier:
    """
    Performs connected component segmentation and geometric classification on an OccupancyGrid.
    """

    def __init__(
        self,
        room_min_area_sqm: float = 2.5,
        room_max_area_sqm: float = 6.0,
        room_min_aspect_ratio: float = 0.70,
        room_max_aspect_ratio: float = 1.45,
        corridor_min_aspect_ratio: float = 1.8,
        corridor_min_width_m: float = 0.75,
        corridor_max_width_m: float = 2.2,
        corridor_min_area_sqm: float = 2.0,
        min_region_cells: int = 20,
    ) -> None:
        self.room_min_area = float(room_min_area_sqm)
        self.room_max_area = float(room_max_area_sqm)
        self.room_min_aspect = float(room_min_aspect_ratio)
        self.room_max_aspect = float(room_max_aspect_ratio)
        self.corridor_min_aspect = float(corridor_min_aspect_ratio)
        self.corridor_min_width = float(corridor_min_width_m)
        self.corridor_max_width = float(corridor_max_width_m)
        self.corridor_min_area = float(corridor_min_area_sqm)
        self.min_region_cells = int(min_region_cells)

    def segment_and_classify(self, map_msg) -> List[Dict[str, Any]]:
        """
        Extracts and classifies all connected free-space regions in the map.

        Parameters
        ----------
        map_msg : nav_msgs.msg.OccupancyGrid
            Current occupancy grid from SLAM.

        Returns
        -------
        List of dicts, each representing a classified region.
        """
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        data = map_msg.data

        if width <= 0 or height <= 0 or len(data) != width * height:
            return []

        # Find connected components of FREE cells
        raw_components = self._find_connected_components(data, width, height)

        classified_regions = []
        room_count = 0
        corridor_count = 0
        junction_count = 0
        unclassified_count = 0

        for idx, cells in enumerate(raw_components):
            if len(cells) < self.min_region_cells:
                continue

            # Compute geometric metrics
            metrics = self._compute_region_metrics(cells, origin_x, origin_y, resolution)
            if metrics is None:
                continue

            # Classify region based on metrics
            region_type, confidence = self._classify_geometry(metrics)

            if region_type == "room":
                room_count += 1
                label = f"Room {room_count}"
            elif region_type == "corridor":
                corridor_count += 1
                label = f"Corridor {corridor_count}"
            elif region_type == "junction":
                junction_count += 1
                label = f"Junction {junction_count}"
            else:
                unclassified_count += 1
                label = f"Unclassified {unclassified_count}"

            # Compute overlapping arena grid boxes (e.g. ["D4", "E4"])
            grid_boxes = self._compute_grid_boxes(metrics["bounds"], origin_x, origin_y)

            classified_regions.append({
                "region_id": idx + 1,
                "type": region_type,
                "label": label,
                "confidence": round(confidence, 2),
                "grid_boxes": grid_boxes,
                "centroid": [round(metrics["centroid"][0], 2), round(metrics["centroid"][1], 2)],
                "bounds": {
                    "min_x": round(metrics["bounds"]["min_x"], 2),
                    "max_x": round(metrics["bounds"]["max_x"], 2),
                    "min_y": round(metrics["bounds"]["min_y"], 2),
                    "max_y": round(metrics["bounds"]["max_y"], 2),
                    "width": round(metrics["width_m"], 2),
                    "length": round(metrics["length_m"], 2),
                },
                "area_sqm": round(metrics["area_sqm"], 2),
                "aspect_ratio": round(metrics["aspect_ratio"], 2),
                "fill_ratio": round(metrics["fill_ratio"], 2),
                "cell_count": len(cells),
            })

        return classified_regions

    def _find_connected_components(self, data, width: int, height: int) -> List[List[Tuple[int, int]]]:
        """
        Groups contiguous FREE (0) cells into connected regions using BFS flood-fill.
        """
        visited = bytearray(width * height)
        components = []

        for r in range(height):
            for c in range(width):
                cell_idx = r * width + c
                if visited[cell_idx] or data[cell_idx] != CELL_FREE:
                    continue

                # BFS for new region
                component = []
                queue = deque([(r, c)])
                visited[cell_idx] = 1

                while queue:
                    curr_r, curr_c = queue.popleft()
                    component.append((curr_r, curr_c))

                    # 4-connected neighbors
                    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nr, nc = curr_r + dr, curr_c + dc
                        if 0 <= nr < height and 0 <= nc < width:
                            n_idx = nr * width + nc
                            if not visited[n_idx] and data[n_idx] == CELL_FREE:
                                visited[n_idx] = 1
                                queue.append((nr, nc))

                if len(component) >= self.min_region_cells:
                    components.append(component)

        return components

    def _compute_region_metrics(
        self, cells: List[Tuple[int, int]], origin_x: float, origin_y: float, resolution: float
    ) -> Optional[Dict[str, Any]]:
        """
        Computes bounding box, width, length, area, aspect ratio, centroid, and fill ratio.
        """
        if not cells:
            return None

        rows = [r for r, c in cells]
        cols = [c for r, c in cells]

        min_r, max_r = min(rows), max(rows)
        min_c, max_c = min(cols), max(cols)

        min_x = origin_x + min_c * resolution
        max_x = origin_x + (max_c + 1) * resolution
        min_y = origin_y + min_r * resolution
        max_y = origin_y + (max_r + 1) * resolution

        width_m = max(0.01, max_x - min_x)
        length_m = max(0.01, max_y - min_y)

        # Centroid in world coordinates
        mean_r = sum(rows) / len(rows)
        mean_c = sum(cols) / len(cols)
        centroid_x = origin_x + (mean_c + 0.5) * resolution
        centroid_y = origin_y + (mean_r + 0.5) * resolution

        area_sqm = len(cells) * (resolution * resolution)
        bbox_area = width_m * length_m
        fill_ratio = max(0.0, min(1.0, area_sqm / max(0.01, bbox_area)))

        dim_major = max(width_m, length_m)
        dim_minor = min(width_m, length_m)
        aspect_ratio = dim_major / max(0.01, dim_minor)

        return {
            "bounds": {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y},
            "width_m": width_m,
            "length_m": length_m,
            "dim_major": dim_major,
            "dim_minor": dim_minor,
            "area_sqm": area_sqm,
            "aspect_ratio": aspect_ratio,
            "fill_ratio": fill_ratio,
            "centroid": (centroid_x, centroid_y),
        }

    def _classify_geometry(self, metrics: Dict[str, Any]) -> Tuple[str, float]:
        """
        Applies geometric classification rules:
        - Room: area in [2.5, 6.0] sqm, aspect ratio in [0.70, 1.45], roughly square.
        - Corridor: aspect ratio >= 1.8, minor dimension ~1m (0.75m to 2.2m).
        - Junction: multi-way junction or hub area.
        - Unclassified: ambiguous shapes.
        """
        area = metrics["area_sqm"]
        aspect = metrics["aspect_ratio"]
        dim_minor = metrics["dim_minor"]
        dim_major = metrics["dim_major"]
        fill = metrics["fill_ratio"]

        # 1. Room Check (~2m x 2m)
        if (self.room_min_area <= area <= self.room_max_area and
            self.room_min_aspect <= aspect <= self.room_max_aspect and
            1.4 <= dim_minor <= 3.2 and 1.4 <= dim_major <= 3.2 and fill >= 0.60):
            # High confidence if close to 4.0 sqm and 1.0 aspect ratio
            area_dev = abs(area - 4.0) / 4.0
            aspect_dev = abs(aspect - 1.0)
            confidence = max(0.70, min(0.98, 1.0 - 0.25 * area_dev - 0.2 * aspect_dev))
            return "room", confidence

        # 2. Corridor Check (width ~1m, length >> width)
        if (aspect >= self.corridor_min_aspect and
            self.corridor_min_width <= dim_minor <= self.corridor_max_width and
            area >= self.corridor_min_area):
            # Elongated hallway
            confidence = max(0.70, min(0.98, 0.75 + min(0.20, (aspect - 1.8) * 0.1)))
            return "corridor", confidence

        # 3. Junction Check (wide branching area / hub between corridors)
        if (aspect < 1.6 and area >= 5.5 and fill < 0.65) or (aspect < 1.5 and 1.8 <= dim_minor <= 4.0 and fill < 0.55):
            return "junction", 0.80

        # 4. Fallback: Unclassified
        return "unclassified", 0.50

    def _compute_grid_boxes(self, bounds: Dict[str, float], origin_x: float, origin_y: float) -> List[str]:
        """
        Returns list of 1x1m arena grid box IDs (e.g. A1–O15) overlapping this region.
        """
        min_col = max(0, min(ARENA_COLS - 1, int((bounds["min_x"] - origin_x) / GRID_BOX_SIZE_M)))
        max_col = max(0, min(ARENA_COLS - 1, int((bounds["max_x"] - origin_x) / GRID_BOX_SIZE_M)))
        min_row = max(0, min(ARENA_ROWS - 1, int((bounds["min_y"] - origin_y) / GRID_BOX_SIZE_M)))
        max_row = max(0, min(ARENA_ROWS - 1, int((bounds["max_y"] - origin_y) / GRID_BOX_SIZE_M)))

        boxes = []
        for c in range(min_col, max_col + 1):
            col_letter = chr(65 + c)
            for r in range(min_row, max_row + 1):
                row_num = r + 1
                boxes.append(f"{col_letter}{row_num}")

        return boxes
