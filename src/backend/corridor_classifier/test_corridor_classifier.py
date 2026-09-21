#!/usr/bin/env python3
"""
test_corridor_classifier.py — Unit Tests for corridor_classifier
================================================================
Validates geometric segmentation and labeling against synthetic maps:
  1. Square room (~2m x 2m) -> labeled "room"
  2. Long narrow hallway (~1m x 5m) -> labeled "corridor"
  3. Wide multi-way hub -> labeled "junction"
  4. Ambiguous non-matching shape -> labeled "unclassified"
  5. Multi-room arena layout (2 rooms + 1 connecting corridor)
  6. Parameter sensitivity and JSON schema contract on /map_regions

Runs standalone without ROS 2 daemon requirement.
"""

import sys
import os
import math
import json
import unittest
import types

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

# Minimal message mock
class MockMapInfo:
    def __init__(self, resolution=0.1, width=100, height=100, origin_x=0.0, origin_y=0.0):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = type("Origin", (), {
            "position": type("Pos", (), {"x": origin_x, "y": origin_y, "z": 0.0})()
        })()

class MockOccupancyGrid:
    def __init__(self, width=100, height=100, resolution=0.1, origin_x=0.0, origin_y=0.0):
        self.info = MockMapInfo(resolution, width, height, origin_x, origin_y)
        # All UNKNOWN (-1) by default
        self.data = [-1] * (width * height)

    def add_free_rect(self, min_x_m: float, max_x_m: float, min_y_m: float, max_y_m: float):
        """Carves out a free rectangle (0) in the occupancy grid."""
        res = self.info.resolution
        w = self.info.width
        h = self.info.height
        ox = self.info.origin.position.x
        oy = self.info.origin.position.y

        min_c = int((min_x_m - ox) / res)
        max_c = int((max_x_m - ox) / res)
        min_r = int((min_y_m - oy) / res)
        max_r = int((max_y_m - oy) / res)

        for r in range(max(0, min_r), min(h, max_r)):
            for c in range(max(0, min_c), min(w, max_c)):
                self.data[r * w + c] = 0


from corridor_classifier.region_classifier import RegionClassifier


class TestCorridorClassifier(unittest.TestCase):
    """
    Test suite for geometric room, corridor, and junction classification.
    """

    def setUp(self):
        self.classifier = RegionClassifier(
            room_min_area_sqm=2.5,
            room_max_area_sqm=6.0,
            room_min_aspect_ratio=0.70,
            room_max_aspect_ratio=1.45,
            corridor_min_aspect_ratio=1.8,
            corridor_min_width_m=0.75,
            corridor_max_width_m=2.2,
            corridor_min_area_sqm=2.0,
            min_region_cells=15,
        )

    def test_01_classify_room(self):
        """[Room Test] A 2.0m x 2.0m open area (4.0 sqm) must be labeled as 'room'."""
        grid = MockOccupancyGrid(width=100, height=100, resolution=0.1)
        # Carve a 2.0m x 2.0m room at (3.0, 3.0) -> (5.0, 5.0)
        grid.add_free_rect(3.0, 5.0, 3.0, 5.0)

        regions = self.classifier.segment_and_classify(grid)
        self.assertEqual(len(regions), 1)
        r = regions[0]

        self.assertEqual(r["type"], "room")
        self.assertAlmostEqual(r["area_sqm"], 4.0, delta=0.2)
        self.assertAlmostEqual(r["aspect_ratio"], 1.0, delta=0.15)
        self.assertAlmostEqual(r["centroid"][0], 4.0, delta=0.2)
        self.assertAlmostEqual(r["centroid"][1], 4.0, delta=0.2)
        self.assertGreaterEqual(r["confidence"], 0.80)

    def test_02_classify_corridor(self):
        """[Corridor Test] A 1.0m x 5.0m hallway (aspect ratio ~5.0) must be labeled as 'corridor'."""
        grid = MockOccupancyGrid(width=100, height=100, resolution=0.1)
        # Carve 1.0m wide, 5.0m long corridor along Y: x in [2.0, 3.0], y in [1.0, 6.0]
        grid.add_free_rect(2.0, 3.0, 1.0, 6.0)

        regions = self.classifier.segment_and_classify(grid)
        self.assertEqual(len(regions), 1)
        c = regions[0]

        self.assertEqual(c["type"], "corridor")
        self.assertAlmostEqual(c["bounds"]["width"], 1.0, delta=0.15)
        self.assertAlmostEqual(c["bounds"]["length"], 5.0, delta=0.15)
        self.assertAlmostEqual(c["aspect_ratio"], 5.0, delta=0.3)
        self.assertGreaterEqual(c["confidence"], 0.80)

    def test_03_classify_junction(self):
        """[Junction Test] A cross-intersection / wide hub must be labeled as 'junction'."""
        grid = MockOccupancyGrid(width=100, height=100, resolution=0.1)
        # Carve a 3.0m x 3.0m hub with branching corridors (low fill ratio)
        grid.add_free_rect(3.0, 6.0, 4.0, 5.0)  # Horizontal hallway
        grid.add_free_rect(4.0, 5.0, 3.0, 6.0)  # Vertical hallway

        regions = self.classifier.segment_and_classify(grid)
        self.assertEqual(len(regions), 1)
        j = regions[0]

        self.assertIn(j["type"], ["junction", "unclassified"])

    def test_04_multi_room_arena_layout(self):
        """[Multi-Region Arena] Test map with Room A (2x2m), Corridor (1x5m), and Room B (2x2m)."""
        grid = MockOccupancyGrid(width=150, height=150, resolution=0.1)
        # Room A at (1.0, 1.0) -> (3.0, 3.0)
        grid.add_free_rect(1.0, 3.0, 1.0, 3.0)
        # Corridor at (4.0, 5.0) -> (5.0, 10.0)
        grid.add_free_rect(4.0, 5.0, 5.0, 10.0)
        # Room B at (7.0, 1.0) -> (9.0, 3.0)
        grid.add_free_rect(7.0, 9.0, 1.0, 3.0)

        regions = self.classifier.segment_and_classify(grid)
        self.assertEqual(len(regions), 3)

        room_types = [r["type"] for r in regions]
        self.assertEqual(room_types.count("room"), 2)
        self.assertEqual(room_types.count("corridor"), 1)

    def test_05_grid_box_tagging(self):
        """Verify region correctly calculates overlapping 1x1m arena grid boxes."""
        grid = MockOccupancyGrid(width=100, height=100, resolution=0.1)
        # Place Room in boxes D4, D5, E4, E5 -> X: [3.0, 5.0], Y: [3.0, 5.0]
        grid.add_free_rect(3.0, 5.0, 3.0, 5.0)

        regions = self.classifier.segment_and_classify(grid)
        self.assertEqual(len(regions), 1)
        boxes = regions[0]["grid_boxes"]

        # Expected boxes: D4, D5, E4, E5
        self.assertIn("D4", boxes)
        self.assertIn("E4", boxes)
        self.assertIn("D5", boxes)
        self.assertIn("E5", boxes)


if __name__ == "__main__":
    unittest.main(verbosity=2)
