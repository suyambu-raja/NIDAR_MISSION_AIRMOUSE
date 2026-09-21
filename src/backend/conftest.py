# conftest.py — ROS 2 mock stub environment for src/backend
import os
import sys
import time
import types
from unittest.mock import MagicMock

def _make_stub_module(fullname):
    parts = fullname.split(".")
    for i in range(1, len(parts) + 1):
        sub = ".".join(parts[:i])
        if sub not in sys.modules:
            mod = types.ModuleType(sub)
            sys.modules[sub] = mod
    return sys.modules[fullname]

# ---------------------------------------------------------------------------
# Setup ROS 2 stubs if not present
# ---------------------------------------------------------------------------
if "rclpy" not in sys.modules:
    rclpy_mod = _make_stub_module("rclpy")
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.shutdown = lambda: None
    rclpy_mod.ok = lambda: True
    rclpy_mod.spin_once = lambda node, timeout_sec=0: None

if "rclpy.node" not in sys.modules:
    node_mod = _make_stub_module("rclpy.node")

    class FakeClock:
        def now(self):
            return self
        def to_msg(self):
            t = time.time()
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            return type("TimeMsg", (), {"sec": sec, "nanosec": nanosec})()

    class FakeLogger:
        def info(self, m): pass
        def debug(self, m): pass
        def warn(self, m): pass
        def error(self, m): pass

    class FakeParameter:
        def __init__(self, name, val):
            self._name = name
            self.value = val
            self.string_value = str(val) if val is not None else ""
            self.integer_value = int(val) if isinstance(val, (int, float)) else 0
            self.double_value = float(val) if isinstance(val, (int, float)) else 0.0
            self.bool_value = bool(val) if val is not None else False
        def get_parameter_value(self):
            return self

    class FakePublisher:
        def __init__(self, topic, msg_type):
            self.topic = topic
            self.msg_type = msg_type
            self.published = []
            self.published_messages = self.published
        def publish(self, msg):
            self.published.append(msg)

    class FakeSubscription:
        def __init__(self, topic, msg_type, cb):
            self.topic = topic
            self.msg_type = msg_type
            self.callback = cb
            self.cb = cb

    class FakeClient:
        def __init__(self, srv_type, srv_name):
            self.srv_type = srv_type
            self.srv_name = srv_name
            self.name = srv_name
            self._ready = True
        def service_is_ready(self):
            return self._ready
        def wait_for_service(self, timeout_sec=None):
            return self._ready
        def call_async(self, req):
            fut = MagicMock()
            fut.done.return_value = True
            resp = MagicMock()
            resp.success = True
            resp.message = "OK"
            fut.result.return_value = resp
            return fut

    class FakeNode:
        def __init__(self, node_name):
            self._name = node_name
            self.node_name = node_name
            self._params = {}
            self._publishers = []
            self._subscriptions = []
            self._services = []
            self._clients = []
            self._timers = []
            self._logger = FakeLogger()
            self._clock = FakeClock()

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return self._clock

        def declare_parameter(self, name, default):
            self._params[name] = default
            return FakeParameter(name, default)

        def get_parameter(self, name):
            return FakeParameter(name, self._params.get(name))

        def create_publisher(self, msg_type, topic, qos):
            pub = FakePublisher(topic, msg_type)
            self._publishers.append(pub)
            return pub

        def create_subscription(self, msg_type, topic, cb, qos):
            sub = FakeSubscription(topic, msg_type, cb)
            self._subscriptions.append(sub)
            return sub

        def create_service(self, srv_type, srv_name, cb):
            srv = (srv_type, srv_name, cb)
            self._services.append(srv)
            return srv

        def create_client(self, srv_type, srv_name):
            cli = FakeClient(srv_type, srv_name)
            self._clients.append(cli)
            return cli

        def create_timer(self, period, cb):
            self._timers.append((period, cb))
            return cb

        def destroy_node(self): pass

    node_mod.Node = FakeNode

if "rclpy.qos" not in sys.modules:
    qos_mod = _make_stub_module("rclpy.qos")
    class QoSProfile:
        def __init__(self, **kw): pass
    class _Policy:
        RELIABLE = 1; BEST_EFFORT = 2; TRANSIENT_LOCAL = 3; VOLATILE = 4; KEEP_LAST = 5
    qos_mod.QoSProfile = QoSProfile
    qos_mod.QoSDurabilityPolicy = _Policy
    qos_mod.QoSReliabilityPolicy = _Policy
    qos_mod.QoSHistoryPolicy = _Policy

if "geometry_msgs.msg" not in sys.modules:
    geom_msg_mod = _make_stub_module("geometry_msgs.msg")
else:
    geom_msg_mod = sys.modules["geometry_msgs.msg"]

class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x); self.y = float(y); self.z = float(z)

class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x); self.y = float(y); self.z = float(z)

class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = float(x); self.y = float(y); self.z = float(z); self.w = float(w)

class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()

class Header:
    def __init__(self, frame_id="map"):
        t = time.time()
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        self.stamp = type("Stamp", (), {"sec": sec, "nanosec": nanosec})()
        self.frame_id = frame_id

class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()

class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()

class TwistStamped:
    def __init__(self):
        self.header = Header()
        self.twist = Twist()

geom_msg_mod.Point = Point
geom_msg_mod.Vector3 = Vector3
geom_msg_mod.Quaternion = Quaternion
geom_msg_mod.Pose = Pose
geom_msg_mod.PoseStamped = PoseStamped
geom_msg_mod.Header = Header
geom_msg_mod.Twist = Twist
geom_msg_mod.TwistStamped = TwistStamped

if "nav_msgs.msg" not in sys.modules:
    nav_msg_mod = _make_stub_module("nav_msgs.msg")
    class OccupancyGrid:
        def __init__(self):
            self.header = Header()
            self.info = type("MapInfo", (), {
                "resolution": 0.1,
                "width": 100,
                "height": 100,
                "origin": Pose()
            })()
            self.data = []
    class Path:
        def __init__(self):
            self.header = Header()
            self.poses = []
    nav_msg_mod.OccupancyGrid = OccupancyGrid
    nav_msg_mod.Path = Path

if "sensor_msgs.msg" not in sys.modules:
    sensor_msg_mod = _make_stub_module("sensor_msgs.msg")
else:
    sensor_msg_mod = sys.modules["sensor_msgs.msg"]

class BatteryState:
    def __init__(self):
        self.header = Header()
        self.voltage = 12.6
        self.percentage = 1.0
sensor_msg_mod.BatteryState = BatteryState

class Image:
    def __init__(self):
        self.header = Header()
        self.height = 480
        self.width = 640
        self.encoding = "bgr8"
        self.data = b""
sensor_msg_mod.Image = Image

if "std_msgs.msg" not in sys.modules:
    std_msg_mod = _make_stub_module("std_msgs.msg")
else:
    std_msg_mod = sys.modules["std_msgs.msg"]

class String:
    def __init__(self):
        self.data = ""
class Bool:
    def __init__(self):
        self.data = False
std_msg_mod.String = String
std_msg_mod.Bool = Bool
std_msg_mod.Header = Header

if "std_srvs.srv" not in sys.modules:
    std_srv_mod = _make_stub_module("std_srvs.srv")
    class Trigger:
        class Request: pass
        class Response:
            def __init__(self): self.success = False; self.message = ""
    class SetBool:
        class Request:
            def __init__(self): self.data = False
        class Response:
            def __init__(self): self.success = False; self.message = ""
    std_srv_mod.Trigger = Trigger
    std_srv_mod.SetBool = SetBool
