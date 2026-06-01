"""
Standalone tester for r2_brain — drives the BT with synthetic state and
renders the dashboard. No ROS / no robot required.

Usage:

    python -m r2_brain.tester                    # run default 'demo' scenario
    python -m r2_brain.tester --list             # list scenarios
    python -m r2_brain.tester --scenario attack_basic
    python -m r2_brain.tester --scenario all     # run every scenario back-to-back
    python -m r2_brain.tester --no-dashboard     # headless (just print snapshots)

How it works:
  - Stubs out rclpy + std_msgs + geometry_msgs so the BT can be built
    without a ROS install.
  - Captures every command published on /r2/cmd/high_level into the log.
  - When the BT publishes a PLACE_KFS, schedules a simulated rack update
    + inventory decrement after the action's ack timeout — so the BT
    naturally re-picks the next placement on the following tick.
  - Scenarios are timeline scripts: list of (t_seconds, kind, arg) events
    applied to the blackboard at the right moment.
"""

from __future__ import annotations

# ── ROS stubs MUST come before any r2_brain.* import ────────────────────────
import sys
import types as _types


def _stub_ros():
    for mod in ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg',
                'geometry_msgs', 'geometry_msgs.msg'):
        if mod not in sys.modules:
            sys.modules[mod] = _types.ModuleType(mod)
    sys.modules['rclpy'].init     = lambda *a, **k: None
    sys.modules['rclpy'].shutdown = lambda *a, **k: None
    sys.modules['rclpy'].spin     = lambda *a, **k: None

    class _Node: ...
    sys.modules['rclpy.node'].Node = _Node

    class _StringMsg:
        def __init__(self, data=''): self.data = data
    sys.modules['std_msgs.msg'].String = _StringMsg

    class _BoolMsg:
        def __init__(self, data=False): self.data = data
    sys.modules['std_msgs.msg'].Bool = _BoolMsg

    class _PoseStamped: ...
    sys.modules['geometry_msgs.msg'].PoseStamped = _PoseStamped


_stub_ros()

# ── Real imports ────────────────────────────────────────────────────────────
import argparse  # noqa: E402
import json      # noqa: E402
import time      # noqa: E402
from dataclasses import dataclass, field  # noqa: E402
from typing import Any, Callable, List, Optional  # noqa: E402

import py_trees  # noqa: E402

from .arena import build_arena_subtree  # noqa: E402
from .dashboard import open_dashboard   # noqa: E402
from .phases import build_mc_subtree, build_mff_subtree  # noqa: E402
from .safety import EmergencyStop  # noqa: E402
from .state import SnapshotBuilder  # noqa: E402
from .web_dashboard import WebDashboard  # noqa: E402
from .topics import (  # noqa: E402
    BB_ARENA_MODE,
    BB_ARENA_RACK,
    BB_ESTOP,
    BB_FOREST_GRID,
    BB_FOREST_PLAN,
    BB_H7_STATE,
    BB_INVENTORY,
    BB_POSE,
    BB_R1_STATUS,
)


# ─────────────────────────────────────────────────────────────────────────────
#  Mock ROS node
# ─────────────────────────────────────────────────────────────────────────────

class _Logger:
    def __init__(self, sink): self._sink = sink
    def info(self, m):  self._sink('INFO  ' + str(m))
    def warn(self, m):  self._sink('WARN  ' + str(m))
    def error(self, m): self._sink('ERROR ' + str(m))
    def debug(self, m): pass


class _Publisher:
    def __init__(self, on_publish): self._on_publish = on_publish
    def publish(self, msg): self._on_publish(msg)


class MockNode:
    """Quacks like rclpy.node.Node enough for r2_brain leaves."""

    def __init__(self):
        self.cmd_log: List[dict] = []
        self.log_lines: List[str] = []
        self._on_cmd: Optional[Callable[[dict], None]] = None

    def set_cmd_callback(self, cb): self._on_cmd = cb

    def create_publisher(self, _msg_type, topic, _qos):
        if topic == '/r2/cmd/high_level':
            return _Publisher(self._capture_cmd)
        return _Publisher(lambda _m: None)

    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_logger(self): return _Logger(self.log_lines.append)
    def get_clock(self):
        class _C:
            def now(self_inner):
                class _T: nanoseconds = time.monotonic_ns()
                return _T()
        return _C()

    def _capture_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except (ValueError, AttributeError):
            return
        self.cmd_log.append(cmd)
        if self._on_cmd:
            self._on_cmd(cmd)


# ─────────────────────────────────────────────────────────────────────────────
#  Scenario timeline
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Event:
    t: float
    kind: str
    arg: Any = None
    note: str = ''


@dataclass
class Scenario:
    name: str
    description: str
    duration_s: float
    events: List[Event] = field(default_factory=list)
    initial_inventory: list = field(default_factory=list)
    initial_mode: str = 'attack'
    initial_rack: dict = field(default_factory=dict)
    initial_pose: dict = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.0})
    initial_r1: dict = field(default_factory=lambda: {'x': 1.0, 'y': 0.0, 'mode': 'IDLE'})


def _kfs(t='TEAM_KFS', conf=0.95, source='sim'):
    return {'kfs_type': t, 'confidence': conf, 'source': source}


SCENARIOS = {

    'attack_basic': Scenario(
        name='attack_basic',
        description='Empty rack, 3 R2 KFS in inventory, attack mode. BT places by priority.',
        duration_s=20.0,
        initial_inventory=['R2_KFS', 'R2_KFS', 'R2_KFS'],
        initial_mode='attack',
        initial_rack={},
        events=[],
    ),

    'r1_collaboration': Scenario(
        name='r1_collaboration',
        description='R1 gradually places bottom row; R2 stacks middle row above.',
        duration_s=25.0,
        initial_inventory=['R2_KFS', 'R2_KFS', 'R2_KFS'],
        initial_mode='attack',
        initial_rack={},
        events=[
            Event(2.0, 'rack_set', (1, _kfs('TEAM_KFS')), 'R1 places slot 1'),
            Event(6.0, 'rack_set', (2, _kfs('TEAM_KFS')), 'R1 places slot 2'),
            Event(10.0, 'rack_set', (3, _kfs('TEAM_KFS')), 'R1 places slot 3'),
        ],
    ),

    'opp_blocking': Scenario(
        name='opp_blocking',
        description='Opponent KFS clog the middle row. R1 must use weapons to clear one before R2 can place.',
        duration_s=22.0,
        initial_inventory=['R2_KFS', 'R2_KFS'],
        initial_mode='attack',
        initial_rack={
            4: _kfs('OPP_KFS'),
            5: _kfs('OPP_KFS'),
            6: _kfs('OPP_KFS'),
            1: _kfs('TEAM_KFS'),
        },
        initial_r1={'x': 0.5, 'y': 0.0, 'mode': 'CLEARING'},
        events=[
            Event(4.0, 'r1_status', {'x': 0.5, 'y': 0.0, 'mode': 'CLEARING', 'cleared_slot': 5}, 'R1 finishes clearing slot 5'),
            Event(4.1, 'rack_clear', 5, 'opp KFS removed from slot 5'),
        ],
    ),

    'defense_flow': Scenario(
        name='defense_flow',
        description='Mode switches to defense; R1 declares defended slot; R2 reinforces.',
        duration_s=18.0,
        initial_inventory=['R2_KFS', 'R2_KFS', 'R2_KFS'],
        initial_mode='defense',
        initial_rack={
            5: _kfs('OPP_KFS'),
        },
        initial_r1={'x': 1.0, 'y': 0.0, 'mode': 'IDLE'},
        events=[
            Event(3.0, 'r1_status', {'x': 1.0, 'y': 0.0, 'mode': 'DEFEND', 'defended_slot': 4}, 'R1 declares defending slot 4'),
        ],
    ),

    'win_diagonal': Scenario(
        name='win_diagonal',
        description='Slot 1 (R1) + slot 9 (top via lift) already ours; placing slot 5 closes the 1-5-9 diagonal.',
        duration_s=15.0,
        initial_inventory=['R2_KFS'],
        initial_mode='attack',
        initial_rack={
            1: _kfs('TEAM_KFS'),
            9: _kfs('TEAM_KFS'),
        },
    ),
}


# ─────────────────────────────────────────────────────────────────────────────
#  Tester
# ─────────────────────────────────────────────────────────────────────────────

class Tester:

    def __init__(self, scenario: Scenario, dashboard=True, tick_hz: float = 15.0,
                 sim_place_delay_s: float = 1.7, web_port: int = 0):
        self._scenario = scenario
        self._tick_dt = 1.0 / tick_hz
        self._dashboard = dashboard
        self._web_port = web_port
        self._web: Optional[WebDashboard] = None
        self._sim_place_delay = sim_place_delay_s
        self._scheduled: list = []  # (fire_t, callable)

        self._node = MockNode()
        self._node.set_cmd_callback(self._on_cmd)

        self._tree = self._build_tree()
        self._tree.setup(timeout=2.0)
        self._writer = self._make_writer()
        self._reader = self._make_reader()
        self._builder = SnapshotBuilder()

        self._t0 = time.monotonic()
        self._fired_event_idxs: set = set()
        self._apply_initial_state()

    # ── tree assembly (mirror parent.py but minus ROS subs) ─────────────────
    def _build_tree(self) -> py_trees.trees.BehaviourTree:
        mission = py_trees.composites.Sequence('MissionRunning', memory=True)
        mission.add_children([
            build_mff_subtree(self._node),
            build_mc_subtree(self._node),
            build_arena_subtree(self._node),
        ])
        idle = py_trees.behaviours.Success(name='Idle')
        root = py_trees.composites.Selector('Root', memory=False)
        root.add_children([EmergencyStop(self._node), mission, idle])
        return py_trees.trees.BehaviourTree(root)

    # ── blackboard helpers ──────────────────────────────────────────────────
    def _make_writer(self):
        bb = py_trees.blackboard.Client(name='tester_writer')
        for k in (BB_POSE, BB_FOREST_GRID, BB_FOREST_PLAN, BB_ARENA_RACK,
                  BB_R1_STATUS, BB_H7_STATE, BB_ARENA_MODE, BB_INVENTORY,
                  BB_ESTOP):
            bb.register_key(k, access=py_trees.common.Access.WRITE)
        return bb

    def _make_reader(self):
        bb = py_trees.blackboard.Client(name='tester_reader')
        for k in (BB_POSE, BB_FOREST_GRID, BB_FOREST_PLAN, BB_ARENA_RACK,
                  BB_R1_STATUS, BB_H7_STATE, BB_ARENA_MODE, BB_INVENTORY,
                  BB_ESTOP):
            bb.register_key(k, access=py_trees.common.Access.READ)
        return bb

    def _bb_get(self, key, default=None):
        try:
            return self._reader.get(key)
        except KeyError:
            return default

    def _apply_initial_state(self):
        s = self._scenario
        self._writer.set(BB_INVENTORY, list(s.initial_inventory))
        self._writer.set(BB_ARENA_MODE, s.initial_mode)
        self._writer.set(BB_ARENA_RACK, dict(s.initial_rack))
        self._writer.set(BB_POSE, dict(s.initial_pose))
        self._writer.set(BB_R1_STATUS, dict(s.initial_r1))
        # Fake H7 telemetry so the EmergencyStop doesn't fire on link loss
        self._writer.set(BB_H7_STATE, {'hsm_mode': 'IDLE', 'rx_ns': time.monotonic_ns()})
        self._writer.set(BB_ESTOP, False)
        self._builder.append_log(f'scenario "{s.name}" started')

    # ── simulated outcomes from BT commands ─────────────────────────────────
    def _on_cmd(self, cmd: dict):
        self._builder.record_command(cmd)
        kind = cmd.get('cmd')
        if kind == 'PLACE_KFS':
            slot = cmd.get('slot')
            self._schedule(self._sim_place_delay, lambda: self._sim_place(slot))
        elif kind == 'DOCK_FOR_LIFT':
            # Nothing to commit — leaf will time out into SUCCESS naturally
            pass
        elif kind == 'STOP':
            pass

    def _sim_place(self, slot):
        rack = dict(self._bb_get(BB_ARENA_RACK, {}) or {})
        rack[slot] = _kfs('TEAM_KFS')
        self._writer.set(BB_ARENA_RACK, rack)
        inv = list(self._bb_get(BB_INVENTORY, []) or [])
        if inv:
            inv.pop(0)
        self._writer.set(BB_INVENTORY, inv)
        self._builder.append_log(f'sim → placed in slot {slot}, inv now {inv}')

    def _schedule(self, after_s, fn):
        self._scheduled.append((time.monotonic() + after_s, fn))

    def _run_scheduled(self):
        now = time.monotonic()
        keep = []
        for t, fn in self._scheduled:
            if now >= t:
                try: fn()
                except Exception as e: self._builder.append_log(f'scheduled err: {e}')
            else:
                keep.append((t, fn))
        self._scheduled = keep

    # ── scenario timeline application ───────────────────────────────────────
    def _apply_due_events(self):
        elapsed = time.monotonic() - self._t0
        for i, ev in enumerate(self._scenario.events):
            if i in self._fired_event_idxs:
                continue
            if elapsed >= ev.t:
                self._apply_event(ev)
                self._fired_event_idxs.add(i)

    def _apply_event(self, ev: Event):
        if ev.kind == 'rack_set':
            slot, info = ev.arg
            rack = dict(self._bb_get(BB_ARENA_RACK, {}) or {})
            rack[slot] = info
            self._writer.set(BB_ARENA_RACK, rack)
        elif ev.kind == 'rack_clear':
            slot = ev.arg
            rack = dict(self._bb_get(BB_ARENA_RACK, {}) or {})
            rack.pop(slot, None)
            self._writer.set(BB_ARENA_RACK, rack)
        elif ev.kind == 'set_inventory':
            self._writer.set(BB_INVENTORY, list(ev.arg))
        elif ev.kind == 'set_mode':
            self._writer.set(BB_ARENA_MODE, ev.arg)
        elif ev.kind == 'r1_status':
            self._writer.set(BB_R1_STATUS, dict(ev.arg))
        elif ev.kind == 'pose':
            self._writer.set(BB_POSE, dict(ev.arg))
        elif ev.kind == 'estop':
            self._writer.set(BB_ESTOP, bool(ev.arg))
        else:
            self._builder.append_log(f'unknown event kind: {ev.kind}')
            return
        self._builder.append_log(f'event @ {ev.t:.1f}s : {ev.kind} {ev.note}'.rstrip())

    # ── main loop ────────────────────────────────────────────────────────────
    def _tick_once(self, dash):
        self._apply_due_events()
        self._run_scheduled()
        # Refresh H7 telemetry so EmergencyStop doesn't think the link died
        self._writer.set(BB_H7_STATE, {'hsm_mode': 'IDLE', 'rx_ns': time.monotonic_ns()})
        self._tree.tick()
        snap = self._builder.build(self._tree, self._bb_get)
        if dash is not None:
            dash.render(snap)
        if self._web is not None:
            self._web.update(snap)
        return snap

    def run(self):
        if self._web_port:
            self._web = WebDashboard(port=self._web_port)
            self._web.start()
            print(f'\nWeb dashboard:  {self._web.url}\n')
        try:
            if self._dashboard:
                with open_dashboard(refresh_per_second=10) as dash:
                    self._main_loop(dash)
            else:
                self._main_loop(None)
        finally:
            if self._web is not None:
                self._web.stop()

    def _main_loop(self, dash):
        end = self._t0 + self._scenario.duration_s
        # When the web UI is up, the user drives the test interactively —
        # keep ticking until Ctrl+C regardless of scenario duration.
        forever = self._web is not None
        while forever or time.monotonic() < end:
            t_iter = time.monotonic()
            snap = self._tick_once(dash)
            if dash is None:
                self._headless_print(snap)
            sleep_s = self._tick_dt - (time.monotonic() - t_iter)
            if sleep_s > 0:
                time.sleep(sleep_s)

    def _headless_print(self, snap):
        if snap.tick_count % 5 == 1:
            print(f'[t={time.monotonic() - self._t0:5.1f}s] '
                  f'mode={snap.arena_mode} inv={len(snap.inventory)} '
                  f'us={snap.estimated_score} opp={snap.opp_score_estimate} '
                  f'→ {snap.r2_next_action}')


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def _build_demo_chain() -> List[Scenario]:
    return [SCENARIOS[name] for name in
            ('attack_basic', 'r1_collaboration', 'opp_blocking', 'defense_flow', 'win_diagonal')]


def main(argv=None):
    parser = argparse.ArgumentParser(description='r2_brain tester / simulator')
    parser.add_argument('--scenario', default='attack_basic',
                        help='scenario name, "all", or "demo"')
    parser.add_argument('--list', action='store_true', help='list scenarios and exit')
    parser.add_argument('--no-dashboard', action='store_true', help='headless text output')
    parser.add_argument('--web', nargs='?', const=8080, type=int, default=0, metavar='PORT',
                        help='also serve an HTML dashboard on PORT (default 8080)')
    parser.add_argument('--web-only', action='store_true',
                        help='shorthand for --web --no-dashboard (just the browser UI)')
    parser.add_argument('--tick-hz', type=float, default=15.0)
    parser.add_argument('--sim-place-delay', type=float, default=1.7,
                        help='seconds before a PLACE_KFS command shows up in the simulated rack')
    args = parser.parse_args(argv)

    if args.list:
        for name, s in SCENARIOS.items():
            print(f'  {name:20s}  {s.description}')
        return

    if args.scenario in ('all', 'demo'):
        scenarios = _build_demo_chain()
    elif args.scenario in SCENARIOS:
        scenarios = [SCENARIOS[args.scenario]]
    else:
        print(f'Unknown scenario: {args.scenario}. Use --list to see options.')
        return

    if args.web_only:
        args.web = args.web or 8080
        args.no_dashboard = True

    for sc in scenarios:
        # Reset blackboard between scenarios
        py_trees.blackboard.Blackboard.clear()
        Tester(sc,
               dashboard=not args.no_dashboard,
               tick_hz=args.tick_hz,
               sim_place_delay_s=args.sim_place_delay,
               web_port=args.web).run()


if __name__ == '__main__':
    main()
