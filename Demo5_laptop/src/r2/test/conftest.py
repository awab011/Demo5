"""Stub out ROS-only deps so pure-logic tests can run on a dev machine
that has neither rclpy nor a working cv_bridge / vision_msgs install.

Real ROS env (robot, CI with ros-humble) imports the real modules and
skips this stubbing entirely.
"""
import sys
import types


def _ensure(modname: str, attrs: dict | None = None):
    if modname in sys.modules:
        return sys.modules[modname]
    mod = types.ModuleType(modname)
    for k, v in (attrs or {}).items():
        setattr(mod, k, v)
    sys.modules[modname] = mod
    return mod


# rclpy + submodules — only used at runtime, never executed during tests.
class _NodeStub:
    def __init__(self, *a, **kw):
        pass


_ensure("rclpy", {"init": lambda *a, **kw: None,
                  "shutdown": lambda *a, **kw: None,
                  "spin": lambda *a, **kw: None})
_ensure("rclpy.node", {"Node": _NodeStub})
_ensure("rclpy.qos", {
    "QoSProfile": lambda **kw: None,
    "HistoryPolicy": types.SimpleNamespace(KEEP_LAST=0),
    "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1),
})


class _TimeStub:
    @classmethod
    def from_msg(cls, _stamp):
        return types.SimpleNamespace(nanoseconds=0)


_ensure("rclpy.time", {"Time": _TimeStub})

# Message packages
_ensure("geometry_msgs", {})
_ensure("geometry_msgs.msg", {"PoseStamped": object})
_ensure("sensor_msgs", {})
_ensure("sensor_msgs.msg", {"Image": object, "PointCloud2": object})
_ensure("std_msgs", {})
_ensure("std_msgs.msg", {"String": object})
_ensure("vision_msgs", {})
_ensure("vision_msgs.msg", {"Detection2DArray": object})
_ensure("sensor_msgs_py", {})
_ensure("sensor_msgs_py.point_cloud2", {"read_points": lambda *a, **kw: []})

# cv_bridge
class _CvBridgeStub:
    def imgmsg_to_cv2(self, *a, **kw): raise NotImplementedError
    def cv2_to_imgmsg(self, *a, **kw): raise NotImplementedError


_ensure("cv_bridge", {"CvBridge": _CvBridgeStub})
