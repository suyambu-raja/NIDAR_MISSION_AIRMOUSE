#!/usr/bin/env python3
"""
test_grid_mapper.py — NIDAR AirMouse: Grid Mapper Unit Tests
=============================================================
Standalone test suite — runs WITHOUT a live ROS 2 environment.

All tests use plain Python objects as mock inputs.
No rclpy.init() is called.  No real hardware required.

Test categories
---------------
1.  CoordinateTransformer — basic conversion (known positions → grid box)
2.  CoordinateTransformer — edge cases (boundary, out-of-bounds, negative)
3.  CoordinateTransformer — map origin offset handling
4.  CoordinateTransformer — reverse lookup (grid_to_world)
5.  SurvivorRegistry — nearby detections → same survivor (dedup)
6.  SurvivorRegistry — far detections → different survivors
7.  SurvivorRegistry — maximum 6 survivor cap
8.  SurvivorRegistry — confidence and position averaging
9.  GridVisualizer — grid lines drawn at correct cell indices
10. GridVisualizer — survivor markers placed at correct cells
11. GridVisualizer — base map is NOT mutated (copy check)
12. Integration — full pipeline (position → grid box → registry → message)

Run:
    python3 src/backend/grid_mapper_node/test_grid_mapper.py
    # or:
    pytest src/backend/grid_mapper_node/test_grid_mapper.py -v
"""

import math
import sys
import os
import unittest

# ---------------------------------------------------------------------------
# Make the grid_mapper_node package importable without colcon install.
# We add the inner package directory to sys.path so imports work standalone.
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# Adds .../grid_mapper_node/ (the outer package dir) to the path so that
# `from grid_mapper_node.xxx import Xxx` resolves correctly.
sys.path.insert(0, _THIS_DIR)

from grid_mapper_node.coordinate_transformer import CoordinateTransformer, ARENA_COLS, ARENA_ROWS
from grid_mapper_node.survivor_registry import SurvivorRegistry, MAX_SURVIVORS, DEDUP_THRESHOLD_M


# =============================================================================
# Mock OccupancyGrid helpers (no ROS 2 needed)
# =============================================================================

class MockPose:
    """Minimal mock for geometry_msgs/Point."""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockOrientation:
    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.z = 0.0; self.w = 1.0


class MockPoseFull:
    def __init__(self, x=0.0, y=0.0):
        self.position = MockPose(x, y)
        self.orientation = MockOrientation()


class MockMapInfo:
    def __init__(self, width=300, height=300, resolution=0.05,
                 origin_x=-7.5, origin_y=-7.5):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = MockPoseFull(origin_x, origin_y)
        self.map_load_time = None


class MockHeader:
    def __init__(self, frame_id="map"):
        self.frame_id = frame_id
        self.stamp = None


class MockOccupancyGrid:
    """Minimal mock for nav_msgs/OccupancyGrid."""
    def __init__(self, width=300, height=300, resolution=0.05,
                 origin_x=-7.5, origin_y=-7.5):
        self.header = MockHeader()
        self.info = MockMapInfo(width, height, resolution, origin_x, origin_y)
        # All-free map data
        self.data = [0] * (width * height)


# =============================================================================
# Test 1-4: CoordinateTransformer
# =============================================================================

class TestCoordinateTransformer(unittest.TestCase):

    def setUp(self):
        """Fresh transformer with default (0,0) grid origin for each test."""
        # Grid origin at (0,0), no map origin offset — clean slate
        self.tf = CoordinateTransformer(
            grid_origin_x=0.0,
            grid_origin_y=0.0,
            map_origin_x=0.0,
            map_origin_y=0.0,
        )

    # -------------------------------------------------------------------------
    # 1. Basic coordinate transformation (known positions)
    # -------------------------------------------------------------------------

    def test_mission_spec_example(self):
        """
        Algorithm check: survivor at (6.3, 4.7) m.
        grid_column = int(6.3 / 1.0) = 6 → chr(65+6) = 'G'
        grid_row    = int(4.7 / 1.0) + 1 = 5
        Expected: 'G5'

        Note: the prompt narrative says 'F4' but the algorithm
        (which is the authoritative spec) yields 'G5'.
        int(6.3)=6 → 7th letter = G; int(4.7)+1=5 → row 5.
        """
        result = self.tf.world_to_grid(6.3, 4.7)
        self.assertEqual(result, "G5",
            f"Expected 'G5' for (6.3, 4.7) per algorithm, got '{result}'")

    def test_origin_box_a1(self):
        """(0.5, 0.5) is inside the A1 grid box."""
        result = self.tf.world_to_grid(0.5, 0.5)
        self.assertEqual(result, "A1")

    def test_box_a1_near_origin(self):
        """(0.0, 0.0) exactly at origin → A1."""
        result = self.tf.world_to_grid(0.0, 0.0)
        self.assertEqual(result, "A1")

    def test_first_column_various_rows(self):
        """First column (A) across several rows."""
        self.assertEqual(self.tf.world_to_grid(0.1, 0.1), "A1")
        self.assertEqual(self.tf.world_to_grid(0.9, 1.5), "A2")
        self.assertEqual(self.tf.world_to_grid(0.5, 14.9), "A15")

    def test_last_column_o(self):
        """Column O = index 14 = world_x in [14.0, 15.0)."""
        result = self.tf.world_to_grid(14.0, 0.5)
        self.assertEqual(result, "O1")

        result = self.tf.world_to_grid(14.99, 14.99)
        self.assertEqual(result, "O15")

    def test_middle_of_arena(self):
        """Centre of arena: (7.5, 7.5) → H8 (col H=7, row 8)."""
        result = self.tf.world_to_grid(7.5, 7.5)
        self.assertEqual(result, "H8")

    def test_f4_grid_box(self):
        """Several positions all inside F4."""
        for x in [5.0, 5.5, 5.99]:
            for y in [3.0, 3.5, 3.99]:
                result = self.tf.world_to_grid(x, y)
                self.assertEqual(result, "F4",
                    f"Expected F4 for ({x}, {y}), got {result}")

    # -------------------------------------------------------------------------
    # 2. Edge cases
    # -------------------------------------------------------------------------

    def test_exactly_on_grid_line_x(self):
        """Exactly on x=5.0 boundary → F-column (index 5), not E."""
        # int(5.0 / 1.0) = 5 → chr(65+5) = 'F'
        result = self.tf.world_to_grid(5.0, 0.5)
        self.assertEqual(result, "F1")

    def test_exactly_on_grid_line_y(self):
        """Exactly on y=4.0 boundary → row 5 (int(4.0)+1=5)."""
        result = self.tf.world_to_grid(0.5, 4.0)
        self.assertEqual(result, "A5")

    def test_far_edge_of_arena(self):
        """Exactly at the far arena edge (15.0, 15.0) → clamped to O15."""
        result = self.tf.world_to_grid(15.0, 15.0)
        self.assertEqual(result, "O15",
            "Far edge (15.0, 15.0) should snap to last box O15")

    def test_outside_arena_positive(self):
        """Beyond far edge (16.0, 5.0) → None."""
        result = self.tf.world_to_grid(16.0, 5.0)
        self.assertIsNone(result, "Position outside arena must return None")

    def test_outside_arena_negative(self):
        """Negative coordinates → None."""
        self.assertIsNone(self.tf.world_to_grid(-1.0, 5.0))
        self.assertIsNone(self.tf.world_to_grid(5.0, -0.1))
        self.assertIsNone(self.tf.world_to_grid(-1.0, -1.0))

    def test_outside_arena_y(self):
        """y=20.0 way beyond the 15m arena → None."""
        result = self.tf.world_to_grid(7.0, 20.0)
        self.assertIsNone(result)

    # -------------------------------------------------------------------------
    # 3. Map origin offset handling
    # -------------------------------------------------------------------------

    def test_with_map_origin_offset(self):
        """
        mock_slam.py sets map.info.origin = (-7.5, -7.5).
        Grid origin (drone start) = (0.0, 0.0) in map frame.
        A survivor at map-frame (6.3, 4.7) maps to arena-frame (6.3, 4.7).
        Algorithm: int(6.3)=6→G, int(4.7)+1=5 → 'G5'
        (map_origin is used by GridVisualizer for cell math, not by world_to_grid)
        """
        tf = CoordinateTransformer(
            grid_origin_x=0.0,
            grid_origin_y=0.0,
            map_origin_x=-7.5,   # mock_slam map origin
            map_origin_y=-7.5,
        )
        result = tf.world_to_grid(6.3, 4.7)
        self.assertEqual(result, "G5")

    def test_grid_origin_shift(self):
        """
        If the drone enters the arena at map-frame (2.0, 3.0),
        a survivor at map-frame (8.3, 7.7) should be at
        arena-frame (6.3, 4.7) → "F4".
        """
        tf = CoordinateTransformer(
            grid_origin_x=2.0,
            grid_origin_y=3.0,
        )
        # arena_x = 8.3 - 2.0 = 6.3 → col F
        # arena_y = 7.7 - 3.0 = 4.7 → row 5... wait: int(4.7)+1=5 → "F5"
        result = tf.world_to_grid(8.3, 7.7)
        # int(6.3)=6→G, int(4.7)+1=5 → "G5"
        self.assertEqual(result, "G5")

    def test_set_grid_origin_method(self):
        """set_grid_origin() updates the origin correctly."""
        tf = CoordinateTransformer()
        tf.set_grid_origin(1.0, 2.0)
        self.assertAlmostEqual(tf.grid_origin_x, 1.0)
        self.assertAlmostEqual(tf.grid_origin_y, 2.0)
        # survivor at (4.5, 5.5) in map frame → arena (3.5, 3.5) → D4
        result = tf.world_to_grid(4.5, 5.5)
        self.assertEqual(result, "D4")

    # -------------------------------------------------------------------------
    # 4. Reverse lookup: grid_to_world
    # -------------------------------------------------------------------------

    def test_reverse_lookup_f4(self):
        """grid_to_world("F4") → centre of F4 box = (5.5, 3.5)."""
        pos = self.tf.grid_to_world("F4")
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos[0], 5.5, places=6)
        self.assertAlmostEqual(pos[1], 3.5, places=6)

    def test_reverse_lookup_a1(self):
        """grid_to_world("A1") → (0.5, 0.5)."""
        pos = self.tf.grid_to_world("A1")
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos[0], 0.5, places=6)
        self.assertAlmostEqual(pos[1], 0.5, places=6)

    def test_reverse_lookup_invalid(self):
        """grid_to_world("Z99") → None (out of range)."""
        self.assertIsNone(self.tf.grid_to_world("Z99"))
        self.assertIsNone(self.tf.grid_to_world(""))
        self.assertIsNone(self.tf.grid_to_world("X"))

    def test_round_trip(self):
        """world → grid → world centre should be consistent."""
        # (6.3, 4.7) → G5  (int(6.3)=6→G, int(4.7)+1=5)
        world_x, world_y = 6.3, 4.7
        grid_id = self.tf.world_to_grid(world_x, world_y)   # "G5"
        self.assertEqual(grid_id, "G5")

        centre = self.tf.grid_to_world(grid_id)
        # Centre of G5: col_idx=6, row_num=5 → (6.5, 4.5)
        self.assertAlmostEqual(centre[0], 6.5, places=6)
        self.assertAlmostEqual(centre[1], 4.5, places=6)


# =============================================================================
# Test 5-8: SurvivorRegistry
# =============================================================================

class TestSurvivorRegistry(unittest.TestCase):

    def setUp(self):
        """Fresh registry for each test."""
        self.registry = SurvivorRegistry()

    # -------------------------------------------------------------------------
    # 5. Nearby detections → same survivor (deduplication)
    # -------------------------------------------------------------------------

    def test_same_survivor_exact_position(self):
        """Two detections at identical positions → 1 survivor."""
        self.registry.add_or_update(5.0, 5.0, 0.9, "F6")
        self.registry.add_or_update(5.0, 5.0, 0.8, "F6")
        self.assertEqual(self.registry.count(), 1)

    def test_same_survivor_within_threshold(self):
        """Two detections 0.3 m apart → same survivor (< 0.5 m threshold)."""
        self.registry.add_or_update(5.0, 5.0, 0.9, "F6")
        # Second detection 0.3 m away (within DEDUP_THRESHOLD_M=0.5)
        self.registry.add_or_update(5.3, 5.0, 0.8, "F6")
        self.assertEqual(self.registry.count(), 1,
            "Detections within 0.5m must be merged as same survivor")

    def test_same_survivor_exactly_at_threshold(self):
        """
        Exactly at 0.5 m: the spec says distance < 0.5 → same.
        So 0.5 m exactly → different survivor.
        """
        self.registry.add_or_update(0.0, 0.0, 0.9, "A1")
        self.registry.add_or_update(0.5, 0.0, 0.8, "A1")
        # distance = 0.5, threshold = 0.5 → NOT same (< not ≤)
        self.assertEqual(self.registry.count(), 2,
            "Exactly 0.5m apart should be treated as different survivors")

    def test_detection_count_increments(self):
        """Merging N detections should increment detection_count."""
        self.registry.add_or_update(5.0, 5.0, 0.9, "F6")
        self.registry.add_or_update(5.1, 5.0, 0.85, "F6")
        self.registry.add_or_update(4.95, 5.05, 0.8, "F6")
        survivors = self.registry.get_all()
        self.assertEqual(len(survivors), 1)
        self.assertEqual(survivors[0].detection_count, 3)

    # -------------------------------------------------------------------------
    # 6. Far detections → different survivors
    # -------------------------------------------------------------------------

    def test_different_survivors_far_apart(self):
        """Two detections 5 m apart → 2 different survivors."""
        self.registry.add_or_update(2.0, 2.0, 0.9, "C3")
        self.registry.add_or_update(7.0, 7.0, 0.85, "H8")
        self.assertEqual(self.registry.count(), 2)

    def test_three_different_survivors(self):
        """Three survivors in completely different grid boxes."""
        self.registry.add_or_update(1.5, 1.5, 0.9, "B2")
        self.registry.add_or_update(8.5, 8.5, 0.85, "I9")
        self.registry.add_or_update(14.5, 14.5, 0.8, "O15")
        self.assertEqual(self.registry.count(), 3)
        ids = {r.grid_box for r in self.registry.get_all()}
        self.assertEqual(ids, {"B2", "I9", "O15"})

    # -------------------------------------------------------------------------
    # 7. Maximum 6 survivor cap
    # -------------------------------------------------------------------------

    def test_max_survivors_cap(self):
        """Registry must hard-stop at MAX_SURVIVORS (6)."""
        for i in range(10):
            # Each survivor is >0.5m from the previous ones
            result = self.registry.add_or_update(
                world_x=float(i * 2),   # 0, 2, 4, 6, ... metres apart
                world_y=0.0,
                confidence=0.9,
                grid_box=f"{chr(65+i)}1",
            )
            if i < MAX_SURVIVORS:
                self.assertIsNotNone(result,
                    f"Survivor {i+1} should be accepted (below cap)")
            else:
                self.assertIsNone(result,
                    f"Survivor {i+1} should be rejected (cap={MAX_SURVIVORS})")

        self.assertEqual(self.registry.count(), MAX_SURVIVORS,
            f"Registry must never exceed {MAX_SURVIVORS} survivors")
        self.assertTrue(self.registry.is_full())

    def test_full_registry_still_updates_existing(self):
        """Even when full, existing survivors should still be updated."""
        # Fill to max
        for i in range(MAX_SURVIVORS):
            self.registry.add_or_update(float(i * 2), 0.0, 0.9, f"{chr(65+i)}1")
        self.assertTrue(self.registry.is_full())

        # Update the first survivor (position within 0.5m of original)
        result = self.registry.add_or_update(0.1, 0.0, 0.95, "A1")
        self.assertIsNotNone(result,
            "Existing survivors must be updatable even when registry is full")
        self.assertEqual(self.registry.count(), MAX_SURVIVORS,
            "Count must not increase beyond cap")

    # -------------------------------------------------------------------------
    # 8. Confidence and position averaging
    # -------------------------------------------------------------------------

    def test_position_averaging(self):
        """Running-average position should converge toward true position."""
        # Add a cluster of detections with small noise around (5.0, 5.0)
        detections = [(5.0, 5.0), (5.1, 4.9), (4.9, 5.1), (5.0, 5.0)]
        for x, y in detections:
            self.registry.add_or_update(x, y, 0.9, "F6")

        record = self.registry.get_all()[0]
        # Average X: (5.0+5.1+4.9+5.0)/4 = 5.0
        self.assertAlmostEqual(record.world_x, 5.0, places=5)
        self.assertAlmostEqual(record.world_y, 5.0, places=5)

    def test_confidence_averaging(self):
        """Confidence must be a running average of all merged detections."""
        self.registry.add_or_update(5.0, 5.0, 0.8, "F6")  # first
        self.registry.add_or_update(5.1, 5.0, 0.6, "F6")  # second
        record = self.registry.get_all()[0]
        # Running average: (0.8 + 0.6) / 2 = 0.7
        self.assertAlmostEqual(record.confidence, 0.7, places=5)

    def test_survivor_ids_unique(self):
        """Each survivor must get a unique, auto-incremented ID."""
        for i in range(MAX_SURVIVORS):
            self.registry.add_or_update(float(i * 2), 0.0, 0.9, f"{chr(65+i)}1")
        ids = [r.survivor_id for r in self.registry.get_all()]
        self.assertEqual(len(ids), len(set(ids)), "Survivor IDs must be unique")
        self.assertEqual(sorted(ids), list(range(MAX_SURVIVORS)))

    def test_out_of_bounds_position_not_added(self):
        """grid_box=None (out-of-bounds) → record not added."""
        result = self.registry.add_or_update(5.0, 5.0, 0.9, grid_box=None)
        self.assertIsNone(result)
        self.assertEqual(self.registry.count(), 0)

    def test_clear_resets_registry(self):
        """clear() must reset the registry and ID counter."""
        self.registry.add_or_update(1.0, 1.0, 0.9, "B2")
        self.registry.clear()
        self.assertEqual(self.registry.count(), 0)
        # After clear, new entries start from id=0 again
        self.registry.add_or_update(1.0, 1.0, 0.9, "B2")
        self.assertEqual(self.registry.get_all()[0].survivor_id, 0)


# =============================================================================
# Test 9-11: GridVisualizer
# =============================================================================

class TestGridVisualizer(unittest.TestCase):
    """
    GridVisualizer tests use MockOccupancyGrid (no ROS 2 import needed).

    We patch the OccupancyGrid import inside grid_visualizer to use our mock.
    Since the module is already imported (via coordinate_transformer which
    doesn't import it), we manually monkeypatch the module attribute.
    """

    def setUp(self):
        """Patch OccupancyGrid in grid_visualizer with our mock, then import."""
        import grid_mapper_node.grid_visualizer as gv_module

        # Inject the mock class so _copy_map() works
        gv_module.OccupancyGrid = MockOccupancyGrid
        gv_module._ROS_AVAILABLE = True

        from grid_mapper_node.grid_visualizer import GridVisualizer
        self.GridVisualizer = GridVisualizer
        self.gv_module = gv_module

    def _make_map(self, width=300, height=300, resolution=0.05,
                  origin_x=0.0, origin_y=0.0):
        """Create a fresh all-free MockOccupancyGrid."""
        m = MockOccupancyGrid(width, height, resolution, origin_x, origin_y)
        m.data = [0] * (width * height)
        return m

    # -------------------------------------------------------------------------
    # 9. Grid lines drawn at correct cell indices
    # -------------------------------------------------------------------------

    def test_grid_lines_vertical_position(self):
        """
        With resolution=0.05m/cell and 1m grid boxes:
        cells_per_box = 1.0 / 0.05 = 20.
        With grid_origin at map_origin (0,0), first vertical line at col=0,
        next at col=20, col=40, …
        """
        vis = self.GridVisualizer(grid_box_size_m=1.0, arena_size_m=15.0)
        base = self._make_map(width=300, height=300, resolution=0.05,
                              origin_x=0.0, origin_y=0.0)
        overlay = vis.render(base, survivor_world_positions=None,
                             grid_origin_x=0.0, grid_origin_y=0.0)

        data = list(overlay.data)
        width = overlay.info.width
        cells_per_box = 20  # 1.0m / 0.05m

        # Check several vertical grid line columns: 0, 20, 40, 60, …
        for col_idx in range(0, min(300, 15 * cells_per_box + 1), cells_per_box):
            # All rows at this column should be grid lines (75) or walls (100)
            mid_row = width // 2
            cell_val = data[mid_row * width + col_idx]
            self.assertIn(cell_val, [75, 100],
                f"Expected grid line at col={col_idx}, got {cell_val}")

    def test_grid_lines_horizontal_position(self):
        """Horizontal lines appear at every cells_per_box rows."""
        vis = self.GridVisualizer(grid_box_size_m=1.0, arena_size_m=15.0)
        base = self._make_map(width=300, height=300, resolution=0.05,
                              origin_x=0.0, origin_y=0.0)
        overlay = vis.render(base, survivor_world_positions=None,
                             grid_origin_x=0.0, grid_origin_y=0.0)

        data = list(overlay.data)
        width = overlay.info.width
        cells_per_box = 20

        for row_idx in range(0, min(300, 15 * cells_per_box + 1), cells_per_box):
            mid_col = width // 2
            cell_val = data[row_idx * width + mid_col]
            self.assertIn(cell_val, [75, 100],
                f"Expected grid line at row={row_idx}, got {cell_val}")

    def test_non_grid_cells_unchanged(self):
        """Cells that are NOT on grid lines should remain FREE (0)."""
        vis = self.GridVisualizer(grid_box_size_m=1.0, arena_size_m=15.0)
        base = self._make_map(width=300, height=300, resolution=0.05,
                              origin_x=0.0, origin_y=0.0)
        overlay = vis.render(base, survivor_world_positions=None,
                             grid_origin_x=0.0, grid_origin_y=0.0)

        data = list(overlay.data)
        width = overlay.info.width
        cells_per_box = 20

        # Cell (row=10, col=10) is inside the first grid box (not on any line)
        # row=10: not a multiple of 20, col=10: not a multiple of 20
        val = data[10 * width + 10]
        self.assertEqual(val, 0, "Interior cells should remain FREE (0)")

    # -------------------------------------------------------------------------
    # 10. Survivor markers
    # -------------------------------------------------------------------------

    def test_survivor_marker_placed(self):
        """Survivor at (5.5, 5.5) → cell (110, 110) for 0.05m/cell, origin=(0,0)."""
        vis = self.GridVisualizer()
        base = self._make_map(width=300, height=300, resolution=0.05,
                              origin_x=0.0, origin_y=0.0)
        overlay = vis.render(base,
                             survivor_world_positions=[(5.5, 5.5)],
                             grid_origin_x=0.0, grid_origin_y=0.0)

        data = list(overlay.data)
        width = overlay.info.width
        # cell_col = int((5.5 - 0.0) / 0.05) = 110
        # cell_row = int((5.5 - 0.0) / 0.05) = 110
        cell_col = int(5.5 / 0.05)  # 110
        cell_row = int(5.5 / 0.05)  # 110
        centre_val = data[cell_row * width + cell_col]
        self.assertEqual(centre_val, 50,
            f"Survivor marker (50) expected at ({cell_col}, {cell_row})")

    def test_multiple_survivor_markers(self):
        """Multiple survivors should all get markers."""
        vis = self.GridVisualizer()
        positions = [(2.0, 2.0), (8.0, 8.0), (14.0, 14.0)]
        base = self._make_map(width=300, height=300, resolution=0.05,
                              origin_x=0.0, origin_y=0.0)
        overlay = vis.render(base,
                             survivor_world_positions=positions,
                             grid_origin_x=0.0, grid_origin_y=0.0)
        data = list(overlay.data)
        width = overlay.info.width

        for sx, sy in positions:
            col = int(sx / 0.05)
            row = int(sy / 0.05)
            if 0 <= row < 300 and 0 <= col < 300:
                val = data[row * width + col]
                self.assertEqual(val, 50,
                    f"Expected survivor marker at ({sx},{sy}) → cell ({col},{row}), got {val}")

    # -------------------------------------------------------------------------
    # 11. Base map not mutated
    # -------------------------------------------------------------------------

    def test_base_map_not_mutated(self):
        """render() must not modify the original base_map data."""
        vis = self.GridVisualizer()
        base = self._make_map(width=100, height=100, resolution=0.05,
                              origin_x=0.0, origin_y=0.0)
        original_data = list(base.data)  # snapshot

        vis.render(base,
                   survivor_world_positions=[(2.5, 2.5)],
                   grid_origin_x=0.0, grid_origin_y=0.0)

        self.assertEqual(list(base.data), original_data,
            "Base map data must not be mutated by render()")


# =============================================================================
# Test 12: Integration
# =============================================================================

class TestIntegration(unittest.TestCase):
    """
    End-to-end pipeline test:
        world position → grid box → registry → survivor record fields
    """

    def test_full_pipeline_single_survivor(self):
        """Single survivor: position → correct grid box → registry entry."""
        tf = CoordinateTransformer(grid_origin_x=0.0, grid_origin_y=0.0)
        reg = SurvivorRegistry()

        # Algorithm: int(6.3)=6→G, int(4.7)+1=5 → "G5"
        grid_box = tf.world_to_grid(6.3, 4.7)
        self.assertEqual(grid_box, "G5")

        record = reg.add_or_update(
            world_x=6.3, world_y=4.7, confidence=0.92, grid_box=grid_box
        )
        self.assertIsNotNone(record)
        self.assertEqual(record.grid_box, "G5")
        self.assertAlmostEqual(record.world_x, 6.3)
        self.assertAlmostEqual(record.world_y, 4.7)
        self.assertAlmostEqual(record.confidence, 0.92)
        self.assertEqual(record.survivor_id, 0)
        self.assertEqual(reg.count(), 1)

    def test_full_pipeline_dedup(self):
        """Two nearby detections of same survivor → 1 registry entry."""
        tf = CoordinateTransformer(grid_origin_x=0.0, grid_origin_y=0.0)
        reg = SurvivorRegistry()

        for world_x, world_y in [(6.3, 4.7), (6.35, 4.72)]:
            grid_box = tf.world_to_grid(world_x, world_y)
            reg.add_or_update(world_x, world_y, 0.9, grid_box)

        self.assertEqual(reg.count(), 1)
        self.assertEqual(reg.get_all()[0].detection_count, 2)

    def test_full_pipeline_six_survivors(self):
        """Exactly 6 well-separated survivors are all registered."""
        tf = CoordinateTransformer(grid_origin_x=0.0, grid_origin_y=0.0)
        reg = SurvivorRegistry()

        positions = [
            (1.5, 1.5), (3.5, 1.5), (5.5, 1.5),
            (1.5, 5.5), (3.5, 5.5), (5.5, 5.5),
        ]
        for x, y in positions:
            grid_box = tf.world_to_grid(x, y)
            reg.add_or_update(x, y, 0.9, grid_box)

        self.assertEqual(reg.count(), 6)
        self.assertTrue(reg.is_full())

    def test_full_pipeline_rejects_seventh(self):
        """7th survivor must be rejected when registry is full."""
        tf = CoordinateTransformer(grid_origin_x=0.0, grid_origin_y=0.0)
        reg = SurvivorRegistry()

        # Add 6 survivors
        for i in range(6):
            grid_box = tf.world_to_grid(float(i * 2), 0.5)
            reg.add_or_update(float(i * 2), 0.5, 0.9, grid_box)

        self.assertEqual(reg.count(), 6)

        # Try to add a 7th
        result = reg.add_or_update(12.0, 8.0, 0.9, tf.world_to_grid(12.0, 8.0))
        self.assertIsNone(result, "7th survivor must be rejected")
        self.assertEqual(reg.count(), 6)

    def test_euclidean_distance_formula(self):
        """
        Verify the exact distance calculation matches the spec formula:
        distance = sqrt((x2-x1)² + (y2-y1)²)
        """
        x1, y1 = 3.0, 4.0
        x2, y2 = 3.4, 4.3  # distance ≈ 0.5 m

        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx * dx + dy * dy)

        # dist ≈ 0.5 → just at the boundary
        # Let's use positions definitively < 0.5 m apart
        reg = SurvivorRegistry()
        reg.add_or_update(x1, y1, 0.9, "D5")

        # Move 0.4 m → same survivor
        reg.add_or_update(x1 + 0.3, y1 + 0.265, 0.8, "D5")
        dist_actual = math.sqrt(0.3**2 + 0.265**2)
        self.assertLess(dist_actual, DEDUP_THRESHOLD_M)
        self.assertEqual(reg.count(), 1, "0.4m detections must merge")


# =============================================================================
# Test runner
# =============================================================================

def run_tests():
    """Run all tests and print a formatted summary."""
    print("=" * 65)
    print("NIDAR AirMouse — Grid Mapper Node Test Suite")
    print("=" * 65)
    print()

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test classes
    for cls in [
        TestCoordinateTransformer,
        TestSurvivorRegistry,
        TestGridVisualizer,
        TestIntegration,
    ]:
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(suite)

    print()
    print("=" * 65)
    if result.wasSuccessful():
        print(f"✅  ALL {result.testsRun} TESTS PASSED")
    else:
        print(f"❌  FAILURES: {len(result.failures)}, "
              f"ERRORS: {len(result.errors)}, "
              f"PASSED: {result.testsRun - len(result.failures) - len(result.errors)}")
    print("=" * 65)

    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(run_tests())
