"""
r2_brain root node — owns the BT, the ROS subscriptions, and the tick loop.

Tree shape:

    Root (Selector, no memory)
      ├─ EmergencyStop          (succeeds when estop / link lost → blocks below)
      ├─ MissionGate            (Sequence, NO memory: re-checks the gate every tick)
      │    ├─ MissionActiveGuard (operator gate — SUCCESS only when mission_active)
      │    └─ MissionRunning     (Sequence, memory: MFF → MC → Arena)
      └─ Idle                   (always Success — keeps tree happy when nothing else matches)

The gate sits in a NO-memory sequence so 'stop' (mission_active → False) halts
an in-flight mission on the very next tick: the guard re-ticks first, fails,
and the no-memory sequence invalidates the running MissionRunning subtree.
MissionRunning keeps its own memory so MFF→MC→Arena progress is preserved
while the gate stays open. Gate is CLOSED at startup (robot idle until the
operator sends 'start' on /r2/mission/cmd) unless `mission_autostart` is set.

Inputs feed a py_trees blackboard via ROS subscriptions on this node;
behaviours read the blackboard rather than holding their own subscribers.

Outputs: every motion / placement leaf publishes a JSON command on
/r2/cmd/high_level. AggregatorNode forwards onto a CAN ID in the
0x110-0x11F laptop→H7 command range (parser TODO on H7 side).
"""

import json
import time
from typing import Optional

import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped
from r2_msgs.msg import ArenaStatus
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .arena import build_arena_subtree
from .phases import build_mc_subtree, build_mff_subtree
from .safety import EmergencyStop
from .state import SnapshotBuilder, find_running_path
from .web_dashboard import WebDashboard
from .topics import (
    BB_ARENA_MODE,
    BB_ARENA_RACK,
    BB_ESTOP,
    BB_FOREST_GRID,
    BB_FOREST_PLAN,
    BB_H7_STATE,
    BB_INVENTORY,
    BB_MANUAL_GRID,
    BB_MISSION_ACTIVE,
    BB_POSE,
    BB_R1_STATUS,
    INTEGRATION_STATUS_HZ,
    MISSION_CMD_TIMEOUT_S,
    SNAPSHOT_HZ,
    TICK_HZ,
    TOPIC_ARENA_MODE,
    TOPIC_ARENA_STATUS,
    TOPIC_BRAIN_SNAPSHOT,
    TOPIC_CMD_HIGH_LEVEL,
    TOPIC_ESTOP,
    TOPIC_FOREST_GRID,
    TOPIC_FOREST_PLAN,
    TOPIC_H7_STATE,
    TOPIC_INTEGRATION_STATUS,
    TOPIC_INVENTORY,
    TOPIC_MANUAL_GRID,
    TOPIC_MISSION_CMD,
    TOPIC_POSE,
    TOPIC_R1_STATUS,
)


def _now_ns() -> int:
    return time.monotonic_ns()


# ─────────────────────────────────────────────────────────────────────────────
#  Mission gate
# ─────────────────────────────────────────────────────────────────────────────

class MissionActiveGuard(py_trees.behaviour.Behaviour):
    """
    Operator gate for MissionRunning. SUCCESS only while the operator has
    opened the mission (blackboard `mission_active` is True). Closed by
    default so the robot stays idle on boot until 'start' arrives on
    /r2/mission/cmd. Read-only; never blocks (returns immediately each tick).
    """

    def __init__(self, name: str = 'MissionActiveGuard'):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(BB_MISSION_ACTIVE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            active = bool(self._bb.get(BB_MISSION_ACTIVE))
        except KeyError:
            active = False
        return (py_trees.common.Status.SUCCESS if active
                else py_trees.common.Status.FAILURE)


# ─────────────────────────────────────────────────────────────────────────────
#  Tree assembly
# ─────────────────────────────────────────────────────────────────────────────

def build_tree(node: Node) -> py_trees.trees.BehaviourTree:
    # MissionRunning keeps its own memory: MFF→MC→Arena progress is preserved.
    mission = py_trees.composites.Sequence('MissionRunning', memory=True)
    mission.add_children([
        build_mff_subtree(node),
        build_mc_subtree(node),
        build_arena_subtree(node),
    ])

    # The gate sequence has NO memory so the guard is re-evaluated every tick;
    # closing the gate ('stop') invalidates the running mission immediately.
    gated_mission = py_trees.composites.Sequence('MissionGate', memory=False)
    gated_mission.add_children([MissionActiveGuard(), mission])

    idle = py_trees.behaviours.Success(name='Idle')

    root = py_trees.composites.Selector('Root', memory=False)
    root.add_children([EmergencyStop(node), gated_mission, idle])

    tree = py_trees.trees.BehaviourTree(root)
    return tree


# ─────────────────────────────────────────────────────────────────────────────
#  ROS node — wires subscriptions into the blackboard, ticks the tree
# ─────────────────────────────────────────────────────────────────────────────

class R2BrainNode(Node):

    def __init__(self):
        super().__init__('r2_brain')

        self._tree = build_tree(self)
        self._tree.setup(timeout=5.0)
        self.get_logger().info(py_trees.display.unicode_tree(self._tree.root))

        _BB_KEYS = (BB_POSE, BB_FOREST_GRID, BB_FOREST_PLAN, BB_ARENA_RACK,
                    BB_R1_STATUS, BB_H7_STATE, BB_ARENA_MODE, BB_INVENTORY,
                    BB_ESTOP, BB_MISSION_ACTIVE, BB_MANUAL_GRID)

        self._bb = py_trees.blackboard.Client(name='r2_brain_writer')
        for key in _BB_KEYS:
            self._bb.register_key(key, access=py_trees.common.Access.WRITE)

        self._bb_reader = py_trees.blackboard.Client(name='r2_brain_reader')
        for key in _BB_KEYS:
            self._bb_reader.register_key(key, access=py_trees.common.Access.READ)
        self._snapshot_builder = SnapshotBuilder()
        self._snapshot_pub = self.create_publisher(String, TOPIC_BRAIN_SNAPSHOT, 10)

        # Operator-stop publishes the same STOP shape EmergencyStop emits.
        self._cmd_pub = self.create_publisher(String, TOPIC_CMD_HIGH_LEVEL, 10)
        self._integration_pub = self.create_publisher(
            String, TOPIC_INTEGRATION_STATUS, 10)

        # Integration / mission-command bookkeeping.
        self._tick_count = 0
        self._last_snapshot_pub_t = 0.0   # throttles /r2/brain/snapshot to SNAPSHOT_HZ
        self._last_mission_cmd: Optional[str] = None
        self._last_mission_cmd_ns = 0
        self._mission_cmd_seen = False
        self._can_tx_count = None   # filled only once aggregator publishes it (see _publish_integration_status)

        # Web dashboard — declared as a parameter so you can disable
        # by setting `web_dashboard_port: 0` in launch / params.
        self.declare_parameter('web_dashboard_port', 8080)
        self._web_port = int(self.get_parameter('web_dashboard_port').value)
        self._web: Optional[WebDashboard] = None
        if self._web_port > 0:
            try:
                self._web = WebDashboard(port=self._web_port)
                self._web.start()
                self.get_logger().info(f'web dashboard at {self._web.url}')
            except RuntimeError as e:
                self.get_logger().warn(f'web dashboard disabled: {e}')

        # mission_autostart: open the gate at boot for bench testing without the
        # app. Default False — competition-safe (robot idle until 'start').
        self.declare_parameter('mission_autostart', False)
        autostart = bool(self.get_parameter('mission_autostart').value)

        # Sensible defaults so leaves don't crash before first message
        self._bb.set(BB_ARENA_MODE, 'attack')
        self._bb.set(BB_INVENTORY, [])
        self._bb.set(BB_ESTOP, False)
        self._bb.set(BB_ARENA_RACK, {})
        self._bb.set(BB_MISSION_ACTIVE, autostart)
        self._bb.set(BB_MANUAL_GRID, None)
        if autostart:
            self.get_logger().warn('mission_autostart=True — MissionRunning gate OPEN at boot')

        self.create_subscription(PoseStamped, TOPIC_POSE,         self._on_pose,    10)
        self.create_subscription(String,      TOPIC_FOREST_GRID,  self._on_forest_grid, 10)
        self.create_subscription(String,      TOPIC_FOREST_PLAN,  self._on_forest_plan, 10)
        self.create_subscription(ArenaStatus, TOPIC_ARENA_STATUS, self._on_arena,   10)
        self.create_subscription(String,      TOPIC_R1_STATUS,    self._on_r1,      10)
        self.create_subscription(String,      TOPIC_H7_STATE,     self._on_h7,      10)
        self.create_subscription(String,      TOPIC_ARENA_MODE,   self._on_mode,    10)
        self.create_subscription(String,      TOPIC_INVENTORY,    self._on_inv,     10)
        self.create_subscription(Bool,        TOPIC_ESTOP,        self._on_estop,   10)
        self.create_subscription(String,      TOPIC_MANUAL_GRID,  self._on_manual_grid, 10)
        self.create_subscription(String,      TOPIC_MISSION_CMD,  self._on_mission_cmd, 10)

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(1.0 / INTEGRATION_STATUS_HZ, self._publish_integration_status)
        # One-shot: warn (do not block) if no operator command after startup.
        self._mission_cmd_timer = self.create_timer(
            MISSION_CMD_TIMEOUT_S, self._mission_cmd_timeout_check)
        self.get_logger().info('r2_brain ready.')

    # ── Subscription callbacks ───────────────────────────────────────────────

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self._bb.set(BB_POSE, {'x': p.x, 'y': p.y, 'z': p.z, 'rx_ns': _now_ns()})

    def _on_forest_grid(self, msg: String):
        self._bb.set(BB_FOREST_GRID, _safe_json(msg.data))

    def _on_forest_plan(self, msg: String):
        self._bb.set(BB_FOREST_PLAN, _safe_json(msg.data))

    def _on_arena(self, msg: ArenaStatus):
        rack = {s: None for s in range(1, 10)}
        for slot in msg.occupied_slots:
            rack[slot.slot_num] = {
                'kfs_type':   slot.kfs_type,
                'confidence': slot.confidence,
                'distance_m': slot.distance_m,
                'col':        slot.col,
                'row':        slot.row,
            }
        self._bb.set(BB_ARENA_RACK, rack)

    def _on_r1(self, msg: String):
        self._bb.set(BB_R1_STATUS, _safe_json(msg.data))

    def _on_h7(self, msg: String):
        d = _safe_json(msg.data) or {}
        d['rx_ns'] = _now_ns()
        self._bb.set(BB_H7_STATE, d)

    def _on_mode(self, msg: String):
        mode = (msg.data or '').strip().lower()
        if mode in ('attack', 'defense'):
            self._bb.set(BB_ARENA_MODE, mode)

    def _on_inv(self, msg: String):
        d = _safe_json(msg.data)
        if isinstance(d, list):
            self._bb.set(BB_INVENTORY, d)
        elif isinstance(d, dict) and 'kfs' in d:
            self._bb.set(BB_INVENTORY, d['kfs'])

    def _on_estop(self, msg: Bool):
        self._bb.set(BB_ESTOP, bool(msg.data))

    def _on_manual_grid(self, msg: String):
        """Operator grid intent from the app. Parsed into BB_MANUAL_GRID; the
        forest planner (phases.py) consumes it. Guarded so a malformed payload
        is logged, never crashes the node."""
        grid = _safe_json(msg.data)
        if grid is None:
            self.get_logger().warn(f'manual_grid: malformed JSON ignored: {msg.data!r}')
            return
        self._bb.set(BB_MANUAL_GRID, grid)
        self.get_logger().info(f'manual_grid: {grid}')

    def _on_mission_cmd(self, msg: String):
        """Operator mission command: 'start' | 'stop' | 'retry'."""
        cmd = (msg.data or '').strip().lower()
        self._last_mission_cmd = cmd
        self._last_mission_cmd_ns = _now_ns()
        self._mission_cmd_seen = True

        if cmd == 'start':
            self._bb.set(BB_MISSION_ACTIVE, True)
            self.get_logger().info('mission: START — MissionRunning gate opened')
        elif cmd == 'stop':
            # Close the gate (halts mission next tick) and emit an immediate
            # STOP. NOTE: this is an operator mission-stop, NOT the safety
            # e-stop — that is the separate /r2/estop path (BB_ESTOP).
            self._bb.set(BB_MISSION_ACTIVE, False)
            self._publish_stop('operator_stop')
            self.get_logger().warn('mission: STOP — gate closed, STOP cmd published')
        elif cmd == 'retry':
            # Reset the whole tree to INVALID (clears MissionRunning memory) and
            # leave the gate closed so it re-ticks from Idle; operator resends
            # 'start' to run again.
            self._bb.set(BB_MISSION_ACTIVE, False)
            try:
                self._tree.root.stop(py_trees.common.Status.INVALID)
            except Exception as e:
                self.get_logger().error(f'mission: retry reset failed: {e}')
            self.get_logger().info('mission: RETRY — BT reset to INVALID, awaiting START')
        else:
            self.get_logger().warn(f'mission: ignoring unknown cmd "{cmd}"')

    def _publish_stop(self, reason: str):
        self._cmd_pub.publish(String(data=json.dumps({'cmd': 'STOP', 'reason': reason})))

    def _mission_cmd_timeout_check(self):
        # One-shot warning. Does NOT block: the tree keeps ticking (EmergencyStop
        # + Idle keep it healthy) and a tester can open the gate via the
        # mission_autostart param or by publishing 'start'.
        self._mission_cmd_timer.cancel()
        if not self._mission_cmd_seen:
            self.get_logger().warn(
                f'no /r2/mission/cmd within {MISSION_CMD_TIMEOUT_S:.0f}s of start — '
                'MissionRunning gate is CLOSED (robot idle). Publish "start" or set '
                'mission_autostart:=true to run. Not blocking.')

    def _publish_integration_status(self):
        """Integration health on /r2/debug/integration_status (1 Hz)."""
        try:
            path = find_running_path(self._tree.root)
            active_subtree = path[1] if len(path) > 1 else (path[0] if path else 'none')
            age_s = ((_now_ns() - self._last_mission_cmd_ns) / 1e9
                     if self._last_mission_cmd_ns else None)
            status = {
                'last_cmd_received': self._last_mission_cmd,
                'last_cmd_age_s':    round(age_s, 2) if age_s is not None else None,
                'bt_tick_count':     self._tick_count,
                'active_subtree':    active_subtree,
                'mission_active':    bool(self._bb_safe_get(BB_MISSION_ACTIVE, False)),
                'estop_state':       bool(self._bb_safe_get(BB_ESTOP, False)),
                # TODO: aggregator does not publish a CAN TX count yet. Wire a
                # publisher in AggregatorNode and subscribe here to fill this.
                'can_tx_count':      self._can_tx_count,
            }
            self._integration_pub.publish(String(data=json.dumps(status)))
        except Exception as e:
            self.get_logger().debug(f'integration_status skipped: {e}')

    # ── Tick ─────────────────────────────────────────────────────────────────

    def _bb_safe_get(self, key, default=None):
        try:
            return self._bb_reader.get(key)
        except KeyError:
            return default

    def _tick(self):
        self._tick_count += 1
        try:
            self._tree.tick()
        except Exception as e:
            self.get_logger().error(f'BT tick failed: {e}')
            return
        try:
            # Build every tick so tick stats + the local web dashboard stay
            # accurate, but throttle the ROS dashboard feed to SNAPSHOT_HZ — the
            # BT ticks at TICK_HZ (20), far faster than any operator UI needs.
            snap = self._snapshot_builder.build(self._tree, self._bb_safe_get)
            if self._web is not None:
                self._web.update(snap)
            now = time.monotonic()
            if now - self._last_snapshot_pub_t >= 1.0 / SNAPSHOT_HZ:
                self._last_snapshot_pub_t = now
                self._snapshot_pub.publish(String(data=snap.to_json()))
        except Exception as e:
            self.get_logger().debug(f'snapshot publish skipped: {e}')


def _safe_json(s: str):
    try:
        return json.loads(s) if s else None
    except (ValueError, TypeError):
        return None


def main():
    rclpy.init()
    node = R2BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._web is not None:
            node._web.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
