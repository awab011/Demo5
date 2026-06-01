"""
Safety behaviours — emergency stop and R1 clearance guard.

EmergencyStop fires when:
  - explicit /r2/estop is True, OR
  - H7 telemetry stale > H7_LINK_TIMEOUT_MS (link loss)
When it succeeds it publishes a STOP high-level command and blocks the
mission subtree (it's the highest-priority child of the root selector).

R1ClearanceGuard is a Condition behaviour intended for use inside a
Parallel decorator wrapping motion-action leaves. It returns SUCCESS while
R1 stays farther than R1_CLEARANCE_MIN_M from R2's current path. When R1
encroaches, it returns FAILURE — the Parallel will tear down the action
and the leaf is responsible for emitting a STOP command.

Skip the guard around lifting actions: during "Get ready for lifting" R2
*intends* to dock with R1, so close-proximity is required.
"""

import json
import math
import time

import py_trees
from std_msgs.msg import String

from .topics import (
    BB_ESTOP,
    BB_H7_STATE,
    BB_LAST_CMD_TS,
    BB_POSE,
    BB_R1_STATUS,
    H7_LINK_TIMEOUT_MS,
    R1_CLEARANCE_MIN_M,
    TOPIC_CMD_HIGH_LEVEL,
    bb_get,
)


def _now_ns() -> int:
    return time.monotonic_ns()


class EmergencyStop(py_trees.behaviour.Behaviour):
    """Returns SUCCESS while estop should be active. Publishes STOP cmd."""

    def __init__(self, node, name: str = 'EmergencyStop'):
        super().__init__(name)
        self._node = node
        self._pub = node.create_publisher(String, TOPIC_CMD_HIGH_LEVEL, 10)
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(BB_ESTOP, access=py_trees.common.Access.READ)
        self._bb.register_key(BB_H7_STATE, access=py_trees.common.Access.READ)
        self._last_published_ns = 0

    def update(self) -> py_trees.common.Status:
        estop = bool(bb_get(self._bb, BB_ESTOP, default=False))
        h7 = bb_get(self._bb, BB_H7_STATE, default=None)

        link_dead = False
        if h7 is None:
            # No telemetry ever received — treat as dead only after grace period.
            link_dead = (_now_ns() // 1_000_000) > H7_LINK_TIMEOUT_MS * 5
        else:
            age_ms = (_now_ns() - h7.get('rx_ns', 0)) / 1e6
            link_dead = age_ms > H7_LINK_TIMEOUT_MS

        if estop or link_dead:
            self._publish_stop(reason='estop' if estop else 'h7_link_lost')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def _publish_stop(self, reason: str):
        # Throttle so we don't spam the bus
        now = _now_ns()
        if now - self._last_published_ns < 100_000_000:  # 100 ms
            return
        self._last_published_ns = now
        msg = String()
        msg.data = json.dumps({'cmd': 'STOP', 'reason': reason})
        self._pub.publish(msg)


class R1ClearanceGuard(py_trees.behaviour.Behaviour):
    """SUCCESS while clearance to R1 ≥ R1_CLEARANCE_MIN_M, else FAILURE."""

    def __init__(self, name: str = 'R1Clearance', min_dist_m: float = R1_CLEARANCE_MIN_M):
        super().__init__(name)
        self._min = min_dist_m
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(BB_POSE, access=py_trees.common.Access.READ)
        self._bb.register_key(BB_R1_STATUS, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        pose = bb_get(self._bb, BB_POSE, default=None)
        r1   = bb_get(self._bb, BB_R1_STATUS, default=None)
        if pose is None or r1 is None:
            # Without info, fail safe (motion blocked rather than allowed).
            return py_trees.common.Status.FAILURE
        try:
            dx = pose['x'] - r1['x']
            dy = pose['y'] - r1['y']
        except (KeyError, TypeError):
            return py_trees.common.Status.FAILURE
        dist = math.hypot(dx, dy)
        return (py_trees.common.Status.SUCCESS
                if dist >= self._min
                else py_trees.common.Status.FAILURE)
