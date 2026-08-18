"""
test_mock_slam_logic.py — Standalone validation of mock_slam.py logic
======================================================================
Tests everything that doesn't require a live ROS 2 runtime:
  1. Quaternion math — euler_to_quaternion correctness
  2. Figure-8 trajectory — x/y range, yaw computation
  3. Map grid construction — dimensions, border walls, data length
  4. Message field contracts — frame_id, resolution, cell values

Run with:
    python3 test_mock_slam_logic.py
No ROS 2 install needed.
"""

import sys
import math
import types

# =============================================================================
# Minimal ROS 2 message stubs so we can import mock_slam.py without rclpy
# =============================================================================

def _make_stub_module(name):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod

# Stub: rclpy
rclpy_mod = _make_stub_module("rclpy")
rclpy_mod.init = lambda args=None: None
rclpy_mod.spin = lambda node: None
rclpy_mod.shutdown = lambda: None

# Stub: rclpy.node
node_mod = _make_stub_module("rclpy.node")
class FakeNode:
    def __init__(self, name): self._name = name
    def get_logger(self): return self
    def warn(self, m): print(f"[WARN]  {m}")
    def info(self, m): print(f"[INFO]  {m}")
    def get_clock(self): return self
    def now(self): return self
    def to_msg(self): return None
    def create_publisher(self, *a, **kw): return None
    def create_timer(self, *a, **kw): return None
    def destroy_node(self): pass
node_mod.Node = FakeNode

# Stub: rclpy.qos
qos_mod = _make_stub_module("rclpy.qos")

class _QoSProfile:
    RELIABLE = 1; BEST_EFFORT = 2; TRANSIENT_LOCAL = 3; VOLATILE = 4; KEEP_LAST = 5
    def __init__(self, **kwargs): pass

class _Policy:
    RELIABLE = 1; BEST_EFFORT = 2; TRANSIENT_LOCAL = 3; VOLATILE = 4; KEEP_LAST = 5

qos_mod.QoSProfile = _QoSProfile
qos_mod.QoSDurabilityPolicy = _Policy
qos_mod.QoSReliabilityPolicy = _Policy
qos_mod.QoSHistoryPolicy = _Policy

# Stub: nav_msgs
nav_mod = _make_stub_module("nav_msgs")
nav_msg_mod = _make_stub_module("nav_msgs.msg")
class OccupancyGrid:
    def __init__(self):
        self.header = type("H", (), {"frame_id": "", "stamp": None})()
        self.info = type("I", (), {
            "resolution": 0, "width": 0, "height": 0,
            "map_load_time": None,
            "origin": type("O", (), {
                "position": type("P", (), {"x":0,"y":0,"z":0})(),
                "orientation": None
            })()
        })()
        self.data = []
class MapMetaData:
    def __init__(self):
        self.resolution = 0; self.width = 0; self.height = 0
        self.map_load_time = None
        self.origin = type("O",(),{
            "position":type("P",(),{"x":0,"y":0,"z":0})(),
            "orientation":None})()
nav_msg_mod.OccupancyGrid = OccupancyGrid
nav_msg_mod.MapMetaData = MapMetaData

# Stub: geometry_msgs
geom_mod = _make_stub_module("geometry_msgs")
geom_msg_mod = _make_stub_module("geometry_msgs.msg")
class PoseStamped:
    def __init__(self):
        self.header = type("H",(),{"frame_id":"","stamp":None})()
        self.pose = type("P",(),{
            "position":type("Pos",(),{"x":0.0,"y":0.0,"z":0.0})(),
            "orientation":None})()
class Quaternion:
    def __init__(self): self.x=0.0; self.y=0.0; self.z=0.0; self.w=1.0
geom_msg_mod.PoseStamped = PoseStamped
geom_msg_mod.Quaternion = Quaternion

# Stub: std_msgs
std_mod = _make_stub_module("std_msgs")
std_msg_mod = _make_stub_module("std_msgs.msg")
class Header:
    def __init__(self): self.frame_id=""; self.stamp=None
std_msg_mod.Header = Header

# Stub: builtin_interfaces
bi_mod = _make_stub_module("builtin_interfaces")
bi_msg_mod = _make_stub_module("builtin_interfaces.msg")
bi_msg_mod.Time = type("Time",(),{})

# =============================================================================
# Now import the functions we want to test from mock_slam
# =============================================================================
sys.path.insert(0, "tools/mock_publishers")
from mock_slam import (
    _euler_to_quaternion,
    _build_empty_map,
    ARENA_SIZE_M,
    MAP_RESOLUTION,
    MAP_CELLS,
    FIGURE8_AMPLITUDE_X,
    FIGURE8_AMPLITUDE_Y,
    FIGURE8_PERIOD_SEC,
    MAP_FRAME,
)


# =============================================================================
# Test helpers
# =============================================================================
PASS = 0
FAIL = 0

def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        print(f"  ✅ PASS  {name}")
        PASS += 1
    else:
        print(f"  ❌ FAIL  {name}" + (f" — {detail}" if detail else ""))
        FAIL += 1

def approx(a, b, tol=1e-6):
    return abs(a - b) < tol


# =============================================================================
# Test 1 — Quaternion math
# =============================================================================
print("\n── Test 1: euler_to_quaternion ──────────────────────────────")

# Identity: roll=pitch=yaw=0 → w=1, xyz=0
q = _euler_to_quaternion(0.0, 0.0, 0.0)
check("Identity: w=1",    approx(q.w, 1.0))
check("Identity: x=0",   approx(q.x, 0.0))
check("Identity: y=0",   approx(q.y, 0.0))
check("Identity: z=0",   approx(q.z, 0.0))

# Yaw 90° (π/2) → z=sin(π/4), w=cos(π/4)
q90 = _euler_to_quaternion(0.0, 0.0, math.pi / 2)
check("Yaw90: w=cos(π/4)",  approx(q90.w, math.cos(math.pi / 4)))
check("Yaw90: z=sin(π/4)",  approx(q90.z, math.sin(math.pi / 4)))
check("Yaw90: x=0",         approx(q90.x, 0.0))
check("Yaw90: y=0",         approx(q90.y, 0.0))

# Yaw 180° → w≈0, z≈1
q180 = _euler_to_quaternion(0.0, 0.0, math.pi)
check("Yaw180: w≈0",   approx(q180.w, 0.0, tol=1e-6))
check("Yaw180: z≈1",   approx(abs(q180.z), 1.0, tol=1e-6))

# Unit norm check for all rotations
for yaw in [0, 0.5, 1.0, math.pi/2, math.pi]:
    qn = _euler_to_quaternion(0, 0, yaw)
    norm = math.sqrt(qn.x**2 + qn.y**2 + qn.z**2 + qn.w**2)
    check(f"Unit norm at yaw={yaw:.2f}",  approx(norm, 1.0, tol=1e-9))


# =============================================================================
# Test 2 — Figure-8 trajectory range
# =============================================================================
print("\n── Test 2: Figure-8 trajectory ──────────────────────────────")
import time as time_mod

omega = (2.0 * math.pi) / FIGURE8_PERIOD_SEC
xs, ys, yaws = [], [], []

# Sample 1000 points across one full loop
N = 1000
for i in range(N):
    t = i * FIGURE8_PERIOD_SEC / N
    x = FIGURE8_AMPLITUDE_X * math.sin(omega * t)
    y = FIGURE8_AMPLITUDE_Y * math.sin(2.0 * omega * t) / 2.0
    dx = FIGURE8_AMPLITUDE_X * omega * math.cos(omega * t)
    dy = FIGURE8_AMPLITUDE_Y * omega * math.cos(2.0 * omega * t)
    yaw = math.atan2(dy, dx)
    xs.append(x); ys.append(y); yaws.append(yaw)

check("X range within ±AMPLITUDE_X",
      max(xs) <= FIGURE8_AMPLITUDE_X + 0.01 and min(xs) >= -FIGURE8_AMPLITUDE_X - 0.01,
      f"max={max(xs):.3f}, amp={FIGURE8_AMPLITUDE_X}")

check("Y range within ±AMPLITUDE_Y/2",
      max(ys) <= FIGURE8_AMPLITUDE_Y / 2 + 0.01 and min(ys) >= -FIGURE8_AMPLITUDE_Y / 2 - 0.01,
      f"max={max(ys):.3f}, amp/2={FIGURE8_AMPLITUDE_Y/2}")

check("Yaw stays in [-π, π]",
      all(-math.pi - 0.01 <= y <= math.pi + 0.01 for y in yaws))

check("Trajectory is periodic (start ≈ end)",
      approx(xs[0], xs[-1], tol=0.15) and approx(ys[0], ys[-1], tol=0.15),
      f"start=({xs[0]:.3f},{ys[0]:.3f}) end=({xs[-1]:.3f},{ys[-1]:.3f})")

# Drone stays within arena bounds
check("X stays within arena (±7.5m)",
      all(abs(x) <= ARENA_SIZE_M / 2 for x in xs))
check("Y stays within arena (±7.5m)",
      all(abs(y) <= ARENA_SIZE_M / 2 for y in ys))


# =============================================================================
# Test 3 — Map construction
# =============================================================================
print("\n── Test 3: OccupancyGrid map construction ───────────────────")

fake_node = FakeNode("test")
grid = _build_empty_map(fake_node)

expected_cells = int(ARENA_SIZE_M / MAP_RESOLUTION)   # 300

check(f"Width = {expected_cells} cells",       grid.info.width == expected_cells)
check(f"Height = {expected_cells} cells",      grid.info.height == expected_cells)
check(f"Resolution = {MAP_RESOLUTION}",        approx(grid.info.resolution, MAP_RESOLUTION))
check("frame_id = 'map'",                      grid.header.frame_id == MAP_FRAME)

total = expected_cells * expected_cells
check(f"data length = {total}",                len(grid.data) == total,
      f"got {len(grid.data)}")

check("All values are 0, 100 only",
      all(v in (0, 100) for v in grid.data),
      f"unexpected values: {set(v for v in grid.data if v not in (0,100))}")

# Count border (occupied) vs interior (free) cells
occupied = sum(1 for v in grid.data if v == 100)
free_cells = sum(1 for v in grid.data if v == 0)
border_expected = 4 * (expected_cells - 1)   # perimeter of a square grid
check(f"Border wall cells = {border_expected}",
      occupied == border_expected,
      f"got {occupied}")
check(f"Interior free cells = {total - border_expected}",
      free_cells == total - border_expected,
      f"got {free_cells}")

# Check origin is centred
check("Origin X = -7.5m",  approx(grid.info.origin.position.x, -ARENA_SIZE_M / 2))
check("Origin Y = -7.5m",  approx(grid.info.origin.position.y, -ARENA_SIZE_M / 2))
check("Origin Z = 0.0",    approx(grid.info.origin.position.z, 0.0))

# Spot check: corner cells are walls
def cell(row, col): return grid.data[row * expected_cells + col]
check("Top-left corner is wall (100)",      cell(0, 0) == 100)
check("Top-right corner is wall (100)",     cell(0, expected_cells-1) == 100)
check("Bottom-left corner is wall (100)",   cell(expected_cells-1, 0) == 100)
check("Bottom-right corner is wall (100)",  cell(expected_cells-1, expected_cells-1) == 100)
check("Interior centre is free (0)",        cell(expected_cells//2, expected_cells//2) == 0)
check("Interior [1][1] is free (0)",        cell(1, 1) == 0)


# =============================================================================
# Test 4 — Interface contract constants
# =============================================================================
print("\n── Test 4: Interface contract constants ─────────────────────")
check("MAP_FRAME = 'map'",                   MAP_FRAME == "map")
check("ARENA_SIZE_M = 15.0",                 approx(ARENA_SIZE_M, 15.0))
check("MAP_RESOLUTION = 0.05",               approx(MAP_RESOLUTION, 0.05))
check("MAP_CELLS = 300",                     MAP_CELLS == 300)
check("FIGURE8_AMPLITUDE_X > 0",             FIGURE8_AMPLITUDE_X > 0)
check("FIGURE8_AMPLITUDE_Y > 0",             FIGURE8_AMPLITUDE_Y > 0)
check("FIGURE8_PERIOD_SEC > 0",              FIGURE8_PERIOD_SEC > 0)
check("Drone never exits 15m arena",
      FIGURE8_AMPLITUDE_X <= ARENA_SIZE_M / 2 and
      FIGURE8_AMPLITUDE_Y / 2 <= ARENA_SIZE_M / 2)


# =============================================================================
# Summary
# =============================================================================
total_tests = PASS + FAIL
print(f"\n{'='*55}")
print(f"  Results: {PASS}/{total_tests} passed  |  {FAIL} failed")
print(f"{'='*55}")
sys.exit(0 if FAIL == 0 else 1)
