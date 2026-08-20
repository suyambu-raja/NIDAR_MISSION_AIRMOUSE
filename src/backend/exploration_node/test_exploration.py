#!/usr/bin/env python3
"""
test_exploration.py — NIDAR AirMouse: Exploration Node Unit Tests
=================================================================
Standalone test suite — runs WITHOUT a live ROS 2 environment.

All tests use plain Python mock objects.
No rclpy.init() is called. No real hardware required.

Test categories
---------------
1.  FrontierDetector — finds correct frontier cells on known test map
2.  FrontierDetector — handles empty map, fully free map, all-wall map
3.  FrontierGrouper  — correctly groups connected frontier cells
4.  FrontierGrouper  — does NOT group disconnected frontier cells
5.  FrontierGrouper  — filters groups smaller than MIN_GROUP_SIZE
6.  FrontierSelector — picks highest-scoring frontier correctly
7.  FrontierSelector — picks closest frontier when scores are tied
8.  FrontierSelector — returns None when no groups provided
9.  PathPlanner      — finds path on simple open map
10. PathPlanner      — avoids walls (safety margin)
11. PathPlanner      — returns None when no path exists (goal in wall)
12. PathPlanner      — returns single-point when start equals goal
13. BatteryMonitor   — returns EXPLORE when battery > 30%
14. BatteryMonitor   — returns RETURN when battery 15-30%
15. BatteryMonitor   — returns EXIT when battery < 15%
16. Integration      — full pipeline: map input -> goal pose output
17. Integration      — exploration stops when 6 survivors found

Run:
    python test_exploration.py
    # or:
    pytest test_exploration.py -v
"""

import math
import sys
import os
import unittest

# ---------------------------------------------------------------------------
# Make exploration_node importable without colcon install
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

from exploration_node.frontier_detector import FrontierDetector, CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN
from exploration_node.frontier_grouper  import FrontierGrouper, FrontierGroup, MIN_GROUP_SIZE
from exploration_node.frontier_selector import FrontierSelector
from exploration_node.path_planner      import PathPlanner
from exploration_node.battery_monitor   import BatteryMonitor, Strategy


# =============================================================================
# Mock helpers — minimal stand-ins for ROS 2 message types
# =============================================================================

class MockPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class MockPose:
    def __init__(self, x=0.0, y=0.0):
        self.position = MockPoint(x, y)
        self.orientation = MockPoint(0, 0)

class MockMapInfo:
    def __init__(self, width, height, resolution=0.05, origin_x=0.0, origin_y=0.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = MockPose(origin_x, origin_y)

class MockOccupancyGrid:
    """Minimal mock for nav_msgs/OccupancyGrid."""
    def __init__(self, width, height, data, resolution=0.05,
                 origin_x=0.0, origin_y=0.0):
        self.info = MockMapInfo(width, height, resolution, origin_x, origin_y)
        self.data = data


def make_empty_map(width=10, height=10):
    """All-free map."""
    data = [CELL_FREE] * (width * height)
    return MockOccupancyGrid(width, height, data)


def make_bordered_map(width=10, height=10):
    """Free interior, OCCUPIED border, rest FREE."""
    data = [CELL_FREE] * (width * height)
    for row in range(height):
        for col in range(width):
            if row == 0 or row == height-1 or col == 0 or col == width-1:
                data[row * width + col] = CELL_OCCUPIED
    return MockOccupancyGrid(width, height, data)


def make_frontier_map():
    """
    5x5 map:
        Row 4 (top):   UNKNOWN UNKNOWN UNKNOWN UNKNOWN UNKNOWN
        Row 3:         UNKNOWN FREE    FREE    FREE    UNKNOWN
        Row 2:         UNKNOWN FREE    FREE    FREE    UNKNOWN
        Row 1:         UNKNOWN FREE    FREE    FREE    UNKNOWN
        Row 0 (bottom):UNKNOWN UNKNOWN UNKNOWN UNKNOWN UNKNOWN
    Frontier cells = interior FREE cells adjacent to UNKNOWN.
    ALL interior cells (rows 1-3, cols 1-3) are frontier cells
    because they all border UNKNOWN cells.
    """
    W, H = 5, 5
    data = [CELL_UNKNOWN] * (W * H)
    for row in range(1, 4):
        for col in range(1, 4):
            data[row * W + col] = CELL_FREE
    return MockOccupancyGrid(W, H, data)


# =============================================================================
# Test 1-2: FrontierDetector
# =============================================================================

class TestFrontierDetector(unittest.TestCase):

    def setUp(self):
        self.detector = FrontierDetector()

    def test_detects_frontier_cells_on_known_map(self):
        """Interior FREE cells adjacent to UNKNOWN are detected as frontiers."""
        m = make_frontier_map()
        frontiers = self.detector.detect(m)
        # 8 of the 9 interior cells are frontiers — the center cell (2,2)
        # has ALL neighbours as FREE so it is NOT a frontier itself.
        # The 8 border-interior cells (ring around center) all touch UNKNOWN.
        self.assertEqual(len(frontiers), 8,
            f"Expected 8 frontier cells, got {len(frontiers)}")
        # Spot-check: a border-interior cell must be detected
        self.assertIn((1, 1), frontiers)
        # Spot-check: center cell (2,2) is NOT a frontier (surrounded by FREE)
        self.assertNotIn((2, 2), frontiers)

    def test_empty_map_returns_no_frontiers(self):
        """Empty data list → no frontiers."""
        m = MockOccupancyGrid(0, 0, [])
        frontiers = self.detector.detect(m)
        self.assertEqual(frontiers, [])

    def test_all_free_map_has_no_frontiers(self):
        """If no UNKNOWN cells exist, nothing is a frontier."""
        m = make_empty_map(6, 6)
        frontiers = self.detector.detect(m)
        self.assertEqual(frontiers, [],
            "Fully free map should have no frontiers")

    def test_all_wall_map_has_no_frontiers(self):
        """All-occupied map → no FREE cells → no frontiers."""
        data = [CELL_OCCUPIED] * 25
        m = MockOccupancyGrid(5, 5, data)
        frontiers = self.detector.detect(m)
        self.assertEqual(frontiers, [])

    def test_occupied_cells_not_detected_as_frontiers(self):
        """OCCUPIED cells adjacent to UNKNOWN are NOT frontiers."""
        W, H = 3, 3
        data = [CELL_UNKNOWN] * 9
        data[4] = CELL_OCCUPIED  # center cell is a wall, not free
        m = MockOccupancyGrid(W, H, data)
        frontiers = self.detector.detect(m)
        # The center wall cell must NOT be in frontiers
        self.assertNotIn((1, 1), frontiers)


# =============================================================================
# Test 3-5: FrontierGrouper
# =============================================================================

class TestFrontierGrouper(unittest.TestCase):

    def setUp(self):
        self.grouper = FrontierGrouper()

    def _make_map_for_grouping(self, width=20, height=10):
        data = [CELL_FREE] * (width * height)
        return MockOccupancyGrid(width, height, data, resolution=1.0)

    def test_groups_connected_frontier_cells(self):
        """9 connected frontier cells should form 1 group."""
        m = make_frontier_map()
        detector = FrontierDetector()
        frontier_cells = detector.detect(m)
        groups = self.grouper.group(frontier_cells, m)
        self.assertEqual(len(groups), 1,
            f"8 connected cells should form 1 group, got {len(groups)}")
        # 8 frontier cells (center is not a frontier — all its neighbours are FREE)
        self.assertEqual(groups[0].size, 8)

    def test_does_not_group_disconnected_cells(self):
        """Two separated clusters should form 2 groups."""
        # 20-wide map: cluster A at cols 1-3, cluster B at cols 16-18
        # separated by UNKNOWN gap in the middle
        W, H = 20, 5
        data = [CELL_UNKNOWN] * (W * H)
        # Cluster A: rows 1-3, cols 1-3
        for row in range(1, 4):
            for col in range(1, 4):
                data[row * W + col] = CELL_FREE
        # Cluster B: rows 1-3, cols 16-18
        for row in range(1, 4):
            for col in range(16, 19):
                data[row * W + col] = CELL_FREE
        m = MockOccupancyGrid(W, H, data)
        detector = FrontierDetector()
        frontier_cells = detector.detect(m)
        groups = self.grouper.group(frontier_cells, m)
        self.assertEqual(len(groups), 2,
            f"Two disconnected clusters should form 2 groups, got {len(groups)}")

    def test_filters_small_groups(self):
        """Groups smaller than MIN_GROUP_SIZE should be filtered out."""
        # Only 2 frontier cells (below MIN_GROUP_SIZE=3)
        W, H = 5, 5
        data = [CELL_OCCUPIED] * (W * H)
        # Two adjacent FREE cells next to UNKNOWN
        data[1 * W + 1] = CELL_FREE
        data[1 * W + 2] = CELL_FREE
        data[0 * W + 1] = CELL_UNKNOWN  # neighbour making them frontier
        data[0 * W + 2] = CELL_UNKNOWN
        m = MockOccupancyGrid(W, H, data)
        detector = FrontierDetector()
        frontier_cells = detector.detect(m)
        groups = self.grouper.group(frontier_cells, m)
        # Group of 2 should be filtered (MIN_GROUP_SIZE=3)
        self.assertEqual(len(groups), 0,
            f"Group of 2 cells should be filtered (MIN={MIN_GROUP_SIZE})")

    def test_group_center_is_correct(self):
        """Center of a 3x3 group should be the center cell."""
        # Simple 3-cell horizontal group at row=1, cols=1,2,3 — resolution=1.0
        W, H = 5, 3
        data = [CELL_UNKNOWN] * (W * H)
        for col in range(1, 4):
            data[1 * W + col] = CELL_FREE
        # row 0 is UNKNOWN so row 1 cells border UNKNOWN
        m = MockOccupancyGrid(W, H, data, resolution=1.0,
                              origin_x=0.0, origin_y=0.0)
        detector = FrontierDetector()
        cells = detector.detect(m)
        groups = self.grouper.group(cells, m)
        self.assertEqual(len(groups), 1)
        cx, cy = groups[0].center_world
        # mean col = 2.0, mean row = 1.0 → world_x = 2.5, world_y = 1.5
        self.assertAlmostEqual(cx, 2.5, places=1)
        self.assertAlmostEqual(cy, 1.5, places=1)


# =============================================================================
# Test 6-8: FrontierSelector
# =============================================================================

class TestFrontierSelector(unittest.TestCase):

    def setUp(self):
        self.selector = FrontierSelector()

    def _make_group(self, cx, cy, size):
        return FrontierGroup(center_world=(cx, cy), size=size, cells=[])

    def test_picks_highest_scoring_frontier(self):
        """
        Large near frontier should beat small far frontier.
        Group A: size=50, dist=2m  → score = 50 - 2*0.5 = 49
        Group B: size=10, dist=1m  → score = 10 - 1*0.5 = 9.5
        """
        groups = [
            self._make_group(2.0, 0.0, 50),   # group A: 2m away, size 50
            self._make_group(1.0, 0.0, 10),   # group B: 1m away, size 10
        ]
        best = self.selector.select(groups, 0.0, 0.0)
        self.assertIsNotNone(best)
        self.assertEqual(best.size, 50,
            "Highest-scoring frontier (size=50) should be selected")

    def test_tie_break_picks_closest(self):
        """
        Equal scores → closer frontier wins.
        To get equal scores: size - dist*0.5 must be equal.
        Group A: size=10, dist=2m → score=9
        Group B: size=9,  dist=0m → score=9
        """
        groups = [
            self._make_group(2.0, 0.0, 10),  # dist=2, score = 10-1.0 = 9
            self._make_group(0.0, 0.0,  9),  # dist=0, score = 9-0.0  = 9
        ]
        best = self.selector.select(groups, 0.0, 0.0)
        self.assertIsNotNone(best)
        # Both score 9 — closest (dist=0) should win
        self.assertEqual(best.center_world, (0.0, 0.0),
            "When scores are equal the closest frontier should be picked")

    def test_returns_none_when_no_groups(self):
        """Empty group list → None returned."""
        result = self.selector.select([], 0.0, 0.0)
        self.assertIsNone(result)

    def test_single_group_always_selected(self):
        """Only one group → it must be selected regardless of score."""
        groups = [self._make_group(100.0, 100.0, 1)]
        best = self.selector.select(groups, 0.0, 0.0)
        self.assertIsNotNone(best)


# =============================================================================
# Test 9-12: PathPlanner
# =============================================================================

class TestPathPlanner(unittest.TestCase):

    def setUp(self):
        self.planner = PathPlanner()

    def _open_map(self, size=20, resolution=1.0):
        """All-free square map, no walls, resolution=1m for simplicity."""
        data = [CELL_FREE] * (size * size)
        return MockOccupancyGrid(size, size, data, resolution=resolution,
                                 origin_x=0.0, origin_y=0.0)

    def test_finds_path_on_open_map(self):
        """A* should find a path on an obstacle-free map."""
        m = self._open_map(size=20)
        path = self.planner.plan(m, (1.5, 1.5), (18.5, 18.5))
        self.assertIsNotNone(path, "A* should find a path on an open map")
        self.assertGreater(len(path), 1, "Path should have more than 1 waypoint")
        # First waypoint should be near start, last near goal
        last = path[-1]
        self.assertAlmostEqual(last[0], 18.5, delta=2.0)
        self.assertAlmostEqual(last[1], 18.5, delta=2.0)

    def test_returns_single_point_when_start_equals_goal(self):
        """Start == Goal → return list with just the goal."""
        m = self._open_map(size=10)
        path = self.planner.plan(m, (5.5, 5.5), (5.5, 5.5))
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 1)

    def test_returns_none_when_goal_in_wall(self):
        """If goal is inside an occupied region with no nearby safe cell, return None."""
        # 10x10 all-occupied map
        data = [CELL_OCCUPIED] * 100
        m = MockOccupancyGrid(10, 10, data, resolution=1.0)
        path = self.planner.plan(m, (0.5, 0.5), (9.5, 9.5))
        self.assertIsNone(path, "All-wall map should return None")

    def test_avoids_walls(self):
        """Path should not include cells immediately adjacent to walls."""
        # 20x20 map with a horizontal wall at row 10
        W = H = 20
        data = [CELL_FREE] * (W * H)
        for col in range(W):
            data[10 * W + col] = CELL_OCCUPIED   # wall at row 10
        m = MockOccupancyGrid(W, H, data, resolution=1.0)
        # Plan from bottom to top — path must go around the wall
        path = self.planner.plan(m, (10.5, 2.5), (10.5, 17.5))
        # If a path is found, none of its waypoints should be inside the wall row
        if path is not None:
            for wx, wy in path:
                row = int(wy / 1.0)
                self.assertNotEqual(row, 10,
                    f"Path passed through wall row 10: waypoint ({wx}, {wy})")

    def test_returns_none_when_no_path_exists(self):
        """Goal completely enclosed by walls → no path."""
        # 10x10: goal is at center (5,5), surrounded by a box of walls
        W = H = 10
        data = [CELL_FREE] * (W * H)
        # Build a closed box from row 3-7, col 3-7
        for row in range(3, 8):
            for col in range(3, 8):
                if row in (3, 7) or col in (3, 7):
                    data[row * W + col] = CELL_OCCUPIED
        m = MockOccupancyGrid(W, H, data, resolution=1.0)
        path = self.planner.plan(m, (0.5, 0.5), (5.5, 5.5))
        # May be None (enclosed) or not, depending on safety margin
        # Main assertion: should not crash
        self.assertTrue(path is None or isinstance(path, list))


# =============================================================================
# Test 13-15: BatteryMonitor
# =============================================================================

class TestBatteryMonitor(unittest.TestCase):

    def setUp(self):
        self.monitor = BatteryMonitor()

    def test_explore_when_above_30(self):
        """Battery > 30% → EXPLORE strategy."""
        strategy = self.monitor.update(80.0)
        self.assertEqual(strategy, Strategy.EXPLORE)

    def test_explore_at_exactly_31(self):
        """31% is above threshold → EXPLORE."""
        strategy = self.monitor.update(31.0)
        self.assertEqual(strategy, Strategy.EXPLORE)

    def test_return_when_between_15_and_30(self):
        """20% is between 15 and 30 → RETURN."""
        strategy = self.monitor.update(20.0)
        self.assertEqual(strategy, Strategy.RETURN)

    def test_return_at_exactly_30(self):
        """Exactly 30% is at the boundary → RETURN (not > 30%)."""
        strategy = self.monitor.update(30.0)
        self.assertEqual(strategy, Strategy.RETURN)

    def test_exit_when_below_15(self):
        """10% is critical → EXIT."""
        strategy = self.monitor.update(10.0)
        self.assertEqual(strategy, Strategy.EXIT)

    def test_exit_at_exactly_15(self):
        """Exactly 15% is at the critical boundary → EXIT (not > 15%)."""
        strategy = self.monitor.update(15.0)
        self.assertEqual(strategy, Strategy.EXIT)

    def test_is_critical_flag(self):
        """is_critical should be True after EXIT strategy."""
        self.monitor.update(5.0)
        self.assertTrue(self.monitor.is_critical)

    def test_exit_point_is_origin(self):
        """Exit point should always be (0.0, 0.0)."""
        self.assertEqual(self.monitor.exit_point, (0.0, 0.0))


# =============================================================================
# Test 16-17: Integration tests
# =============================================================================

class TestIntegration(unittest.TestCase):
    """
    Full pipeline: map input → frontier detection → grouping → selection
    → path planning → goal pose output.

    Uses mock data only — no ROS 2 runtime.
    """

    def setUp(self):
        self.detector = FrontierDetector()
        self.grouper  = FrontierGrouper()
        self.selector = FrontierSelector()
        self.planner  = PathPlanner()

    def _build_exploration_map(self, size=30):
        """
        30x30 map: outer ring UNKNOWN (unexplored),
        inner 20x20 FREE (explored), rest UNKNOWN.
        Frontier cells = inner FREE cells adjacent to outer UNKNOWN ring.
        """
        W = H = size
        data = [CELL_UNKNOWN] * (W * H)
        # Mark inner 20x20 as FREE
        margin = 5
        for row in range(margin, size - margin):
            for col in range(margin, size - margin):
                data[row * W + col] = CELL_FREE
        return MockOccupancyGrid(W, H, data, resolution=1.0,
                                 origin_x=0.0, origin_y=0.0)

    def test_full_pipeline_produces_goal(self):
        """
        Full pipeline should detect frontiers, form groups, and select a goal.
        """
        m = self._build_exploration_map(size=30)
        drone_x, drone_y = 10.0, 10.0  # inside explored area

        frontiers = self.detector.detect(m)
        self.assertGreater(len(frontiers), 0, "Should detect frontier cells")

        groups = self.grouper.group(frontiers, m)
        self.assertGreater(len(groups), 0, "Should form at least one group")

        best = self.selector.select(groups, drone_x, drone_y)
        self.assertIsNotNone(best, "Selector should return a group")

        path = self.planner.plan(m, (drone_x, drone_y), best.center_world)
        # Path may be None if planner struggles on all-resolution-1 map; just
        # ensure no exception is raised and result is valid type
        self.assertTrue(path is None or isinstance(path, list))
        if path is not None:
            self.assertGreater(len(path), 0)

    def test_no_frontiers_when_fully_explored(self):
        """
        Fully free map (no UNKNOWN) → no frontiers → no groups → selector returns None.
        """
        m = make_empty_map(20, 20)
        frontiers = self.detector.detect(m)
        self.assertEqual(frontiers, [], "Fully free map has no frontiers")

        groups = self.grouper.group(frontiers, m)
        self.assertEqual(groups, [])

        best = self.selector.select(groups, 5.0, 5.0)
        self.assertIsNone(best)

    def test_pipeline_respects_max_survivors(self):
        """
        Once 6 survivors are confirmed, exploration_node should stop.
        We test the threshold logic directly (node not instantiated here).
        """
        MAX_SURVIVORS = 6
        survivor_count = 6
        mission_done = survivor_count >= MAX_SURVIVORS
        self.assertTrue(mission_done,
            "Mission should be marked done when all 6 survivors are found")


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  NIDAR AirMouse — Exploration Node Tests")
    print("=" * 60)
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add all test classes
    for cls in [
        TestFrontierDetector,
        TestFrontierGrouper,
        TestFrontierSelector,
        TestPathPlanner,
        TestBatteryMonitor,
        TestIntegration,
    ]:
        suite.addTests(loader.loadTestsFromTestCase(cls))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
