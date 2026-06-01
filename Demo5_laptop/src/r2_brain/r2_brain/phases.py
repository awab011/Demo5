"""
Pre-arena phases: MFF (Meihua Forest) and MC (forest→arena transit).

Both were Success() stubs. They are now real:

  MFF — reads the operator grid intent (BB_MANUAL_GRID), runs the forest
        planner (forest_planner, a 1:1 port of the app's cal_path.dart,
        parity-checked by tools/forest_parity_check.py), and publishes the
        resulting R2Path on /r2_path. AggregatorNode packs it onto CAN 0x100
        (r2can_pack).

  MC  — publishes a high-level "GO to Arena" transit command on
        /r2/cmd/high_level (AggregatorNode → CRITICAL_CMD / ZONES frames).

Both leaves are TIME-BOUNDED so neither can block the tree indefinitely.

KNOWN BLOCKER (HANDOFF §6.2): H7→laptop telemetry does not exist yet, so phase
completion falls back to an optimistic timeout — see the TODOs below. Do NOT
stub fake feedback; replace the timeouts with real /r2/h7_state checks when the
H7 telemetry frame is live.
"""

import json
import time

import py_trees
from std_msgs.msg import String

# R2Path lives in the r2_navigation package (restored separately — it is the
# same message AggregatorNode subscribes to on /r2_path). The field names set
# below are the contract AggregatorNode.r2can_pack already relies on. If your
# R2Path.msg names the per-step type differently, fix this import only.
from r2_navigation.msg import R2Path, R2Step

from .forest_planner import plan_r2_path
from .topics import (
    BB_FOREST_GRID,
    BB_MANUAL_GRID,
    TOPIC_CMD_HIGH_LEVEL,
    TOPIC_R2_PATH,
    bb_get,
)

# Phase timeouts (seconds). Bound every leaf — no indefinite blocking.
MFF_GRID_WAIT_S   = 10.0   # fail the phase if no usable grid arrives in this window
MFF_FOREST_DONE_S = 30.0   # optimistic forest-complete fallback (TODO: real H7 telemetry)
MC_TRANSIT_DONE_S = 5.0    # optimistic transit-complete fallback (TODO: real H7 telemetry)


def _extract_intent(grid):
    """Pull (states, team, kfs_count, already_picked) from the manual-grid dict.

    Shape (published by the app on /r2/forest/manual_grid):
        {team, kfs_count, states[12], retry, already_picked}
    Returns None if the payload is missing or malformed.
    """
    if not isinstance(grid, dict):
        return None
    states = grid.get('states')
    if not isinstance(states, list) or len(states) != 12:
        return None
    try:
        states = [int(x) for x in states]
    except (TypeError, ValueError):
        return None
    team = grid.get('team', 'red')
    if team not in ('red', 'blue'):
        team = 'red'
    try:
        kfs = int(grid.get('kfs_count', 3))
    except (TypeError, ValueError):
        kfs = 3
    picked = grid.get('already_picked') or [] if grid.get('retry') else []
    if not isinstance(picked, list):
        picked = []
    return states, team, kfs, picked


def _to_r2path(plan: dict):
    """Convert a forest_planner R2CanPath dict to an R2Path ROS message.

    Only the fields AggregatorNode.r2can_pack reads are populated; any other
    R2Path/R2Step fields keep their message defaults.
    """
    msg = R2Path()
    msg.total_cost  = int(plan['total_cost'])
    msg.entry_block = int(plan['entry_block'])
    msg.exit_block  = int(plan['exit_block'])
    msg.grid_state  = [int(x) for x in plan['grid_state']]
    # r2can_pack only uses len(pre_entry_pickups); populate with the target
    # blocks of the leading pre-entry steps so the length is correct.
    pre_count = int(plan['pre_entry_count'])
    msg.pre_entry_pickups = [int(s['target_block']) for s in plan['steps'][:pre_count]]

    steps = []
    for s in plan['steps']:
        st = R2Step()
        st.action_type       = int(s['action_type'])
        st.current_block     = int(s['current_block'])
        st.target_block      = int(s['target_block'])
        st.direction         = int(s['direction'])
        st.collected_after   = int(s['collected_after'])
        st.requires_r1_clear = bool(s['requires_r1_clear'])
        st.auto_pickup       = bool(s['auto_pickup'])
        st.height_delta_enc  = int(s['height_delta_enc'])
        steps.append(st)
    msg.steps = steps
    return msg


class ForestPlanAndExecute(py_trees.behaviour.Behaviour):
    """MFF leaf: plan the forest route once and publish it as an R2Path.

    Named 'MFF' so the dashboard's operator-text template fires.
    """

    def __init__(self, node, grid_wait_s=MFF_GRID_WAIT_S,
                 forest_done_s=MFF_FOREST_DONE_S):
        super().__init__('MFF')
        self._node = node
        self._pub = node.create_publisher(R2Path, TOPIC_R2_PATH, 10)
        self._bb = self.attach_blackboard_client(name='MFF')
        self._bb.register_key(BB_MANUAL_GRID, access=py_trees.common.Access.READ)
        self._bb.register_key(BB_FOREST_GRID, access=py_trees.common.Access.READ)
        self._grid_wait_s = grid_wait_s
        self._forest_done_s = forest_done_s
        self._entered_at = None
        self._sent_at = None
        self._published = False

    def initialise(self):
        self._entered_at = time.monotonic()
        self._sent_at = None
        self._published = False

        # Operator grid (has team) preferred; perception's confirmed grid lacks
        # a team field so it is not a planning source on its own.
        intent = _extract_intent(bb_get(self._bb, BB_MANUAL_GRID, default=None))
        if intent is None:
            self._node.get_logger().warn('MFF: no valid manual grid yet — waiting')
            return

        states, team, kfs, picked = intent
        try:
            plan = plan_r2_path(states, team, kfs, picked)
        except Exception as e:
            self._node.get_logger().error(f'MFF: planner failed: {e}')
            return
        if plan['step_count'] == 0:
            self._node.get_logger().warn(
                f'MFF: empty plan for grid {states} (team={team}, kfs={kfs})')
            return

        self._pub.publish(_to_r2path(plan))
        self._sent_at = time.monotonic()
        self._published = True
        self._node.get_logger().info(
            f"MFF: published R2Path entry=B{plan['entry_block']} "
            f"exit=B{plan['exit_block']} steps={plan['step_count']} "
            f"(team={team}, kfs={kfs}, retry={bool(picked)})")

    def update(self):
        now = time.monotonic()
        if not self._published:
            # Bounded wait for a usable grid — never block forever.
            if now - (self._entered_at or now) > self._grid_wait_s:
                self._node.get_logger().warn(
                    'MFF: no usable grid within wait window — failing phase')
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # Path sent. Until H7 telemetry exists we cannot detect forest-exit.
        # TODO: replace with real /r2/h7_state feedback when H7 telemetry is live
        #       (e.g. h7_state['completed'] or current_navi_state == ARENA).
        if now - self._sent_at > self._forest_done_s:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class TransitToArena(py_trees.behaviour.Behaviour):
    """MC leaf: publish a high-level GO-to-Arena transit command, then wait.

    Named 'MC' so the dashboard's operator-text template fires.
    """

    def __init__(self, node, ack_timeout_s=MC_TRANSIT_DONE_S):
        super().__init__('MC')
        self._node = node
        self._pub = node.create_publisher(String, TOPIC_CMD_HIGH_LEVEL, 10)
        self._ack_timeout_s = ack_timeout_s
        self._sent_at = None

    def initialise(self):
        # Uses the existing AggregatorNode high-level contract: CMD_TO_INT['GO'],
        # ZONE_TO_INT['Arena'].
        cmd = {'cmd': 'GO', 'r2_h_cmd': 'GO', 'r2_zone': 'Arena',
               'seq': int(time.monotonic_ns() & 0xFF)}
        self._pub.publish(String(data=json.dumps(cmd)))
        self._sent_at = time.monotonic()
        self._node.get_logger().info(f'MC: transit cmd sent: {cmd}')

    def update(self):
        # TODO: replace with real /r2/h7_state feedback when H7 telemetry is live
        #       (e.g. current_navi_state == ARENA / arrival confirmation).
        if self._sent_at and (time.monotonic() - self._sent_at) > self._ack_timeout_s:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


def build_mff_subtree(node) -> py_trees.behaviour.Behaviour:
    """Meihua Forest — plan + publish R2 forest path."""
    return ForestPlanAndExecute(node)


def build_mc_subtree(node) -> py_trees.behaviour.Behaviour:
    """Mission Control transit phase between Forest and Arena."""
    return TransitToArena(node)
