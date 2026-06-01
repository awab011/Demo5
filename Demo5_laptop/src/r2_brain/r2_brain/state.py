"""
BrainSnapshot — frozen dataclass capturing everything the dashboard needs.

The dashboard is a pure renderer. Whoever runs the BT (parent.py for ROS,
tester.py for sim) builds a snapshot once per tick and hands it to the
dashboard. This keeps the rendering stack decoupled from the data source.

A snapshot can be JSON-serialised (so parent.py can publish it on
/r2/brain/snapshot for a remote dashboard subscriber).
"""

from __future__ import annotations

import collections
import json
import time
from dataclasses import asdict, dataclass, field
from typing import Optional

import py_trees

from .arena import (
    SLOT_MIDDLE,
    SLOTS_BOTTOM_ROW,
    SLOTS_MIDDLE_ROW,
    SLOTS_SIDES,
    SLOTS_TOP_ROW,
    TOP_ABOVE_BOTTOM,
    _is_empty,
    _r1_bottom_slots,
    _slot_owner,
)


# Scoring per row from the rulebook
SCORE_BOTTOM = 30
SCORE_MIDDLE = 40
SCORE_TOP    = 80


@dataclass
class BrainSnapshot:
    # ── World state ────────────────────────────────────────────────────────
    rack: dict          = field(default_factory=dict)   # {slot:int → {kfs_type, ...}|None}
    inventory: list     = field(default_factory=list)   # ['R2_KFS', ...]
    arena_mode: str     = 'attack'
    pose: Optional[dict] = None                          # {x, y, z}
    r1_status: Optional[dict] = None                     # {x, y, mode, defended_slot, cleared_slot, holding}
    h7_state: Optional[dict] = None                      # {hsm_mode, sub_state, current_block, faults, hb, rx_ns}
    estop: bool         = False

    # ── BT state ───────────────────────────────────────────────────────────
    tree_render: str    = ''                             # py_trees.display.ascii_tree(show_status=True)
    running_path: list  = field(default_factory=list)    # ['Root', 'MissionRunning', 'Arena', 'AttackBranch', 'PlaceOneKFS', 'TakeMiddle', 'PlaceInSlot[5]']
    running_leaf: Optional[str] = None                   # tip of running path
    last_status_summary: str    = ''                     # 'Arena → Attack → Place in slot 5 (RUNNING)'

    # ── Last command ───────────────────────────────────────────────────────
    last_command: Optional[dict] = None                  # JSON dict actually published
    last_command_age_s: float    = 0.0

    # ── Stats ──────────────────────────────────────────────────────────────
    tick_count: int     = 0
    tick_hz: float      = 0.0
    placements_total: int = 0
    estimated_score: int = 0
    opp_score_estimate: int = 0
    won_lines: list     = field(default_factory=list)    # winning 3-in-a-line tuples for us
    blocked_by_opp: list = field(default_factory=list)   # opp 2-in-a-line that we should block

    # ── Operator-facing summary (R1 + R2) ──────────────────────────────────
    r2_next_action: str = ''                             # 'PLACE in slot 5 (middle column)'
    r1_advice: str      = ''                             # 'Stay clear of column 2'

    # ── Log tail ───────────────────────────────────────────────────────────
    log: list           = field(default_factory=list)    # latest N strings

    timestamp: float    = field(default_factory=time.time)

    def to_json(self) -> str:
        return json.dumps(asdict(self), default=str)


# ─────────────────────────────────────────────────────────────────────────────
#  Tree introspection
# ─────────────────────────────────────────────────────────────────────────────

def find_running_path(root: py_trees.behaviour.Behaviour) -> list:
    """Walk the tree depth-first; return name path of the RUNNING tip."""
    path = []

    def _walk(node):
        if node.status != py_trees.common.Status.RUNNING:
            return False
        path.append(node.name)
        for child in node.children if hasattr(node, 'children') else []:
            if _walk(child):
                return True
        return True

    _walk(root)
    return path


def summarise_status(root: py_trees.behaviour.Behaviour) -> str:
    path = find_running_path(root)
    if not path:
        # Look for last terminated tip
        last = None
        for node in root.iterate():
            if not getattr(node, 'children', None):
                last = node
        if last is None:
            return '(no behaviours)'
        return f'{last.name} → {last.status.name}'
    return ' → '.join(path) + ' (RUNNING)'


# ─────────────────────────────────────────────────────────────────────────────
#  Score / line analysis
# ─────────────────────────────────────────────────────────────────────────────

# Winning lines per the rulebook: vertical and diagonal only (NOT horizontal).
# A horizontal row never spans more than one row tier and therefore never wins.
LINES = [
    (1, 4, 7), (2, 5, 8), (3, 6, 9),     # vertical (columns)
    (1, 5, 9), (3, 5, 7),                # diagonals
]

# Per-slot point value (for raw scoring, ignoring lines)
SLOT_POINTS = {
    1: SCORE_BOTTOM, 2: SCORE_BOTTOM, 3: SCORE_BOTTOM,
    4: SCORE_MIDDLE, 5: SCORE_MIDDLE, 6: SCORE_MIDDLE,
    7: SCORE_TOP,    8: SCORE_TOP,    9: SCORE_TOP,
}


def score_for_owner(rack: dict, owner: str) -> int:
    total = 0
    for slot, pts in SLOT_POINTS.items():
        if _slot_owner(rack, slot) == owner:
            total += pts
    return total


def winning_lines_for(rack: dict, owner: str) -> list:
    """Lines where `owner` already holds all 3 slots (instant win)."""
    out = []
    for line in LINES:
        if all(_slot_owner(rack, s) == owner for s in line):
            out.append(line)
    return out


def threatening_lines_for(rack: dict, owner: str) -> list:
    """Lines where `owner` holds 2/3 with the third slot empty (one-move-from-win)."""
    out = []
    for line in LINES:
        owners = [_slot_owner(rack, s) for s in line]
        if owners.count(owner) == 2 and 'EMPTY' in owners:
            out.append(line)
    return out


# ─────────────────────────────────────────────────────────────────────────────
#  Operator-facing translation
# ─────────────────────────────────────────────────────────────────────────────

_R2_ACTION_TEMPLATES = {
    'PlaceInSlot':                 lambda s: f'PLACE in slot {s} ({_slot_zone(s)})',
    'PutOnSide':                   lambda s: 'PLACE in middle row, side column',
    'PutOverThere':                lambda s: 'PLACE in nearest empty middle slot',
    'PlaceInFreedSlot':            lambda s: f'PLACE in slot {s or "?"} (just freed by R1)',
    'ReinforceDefendedColumn':     lambda s: 'REINFORCE the defended column',
    'GetReadyForLifting':          lambda s: 'DOCK with R1 for lift sequence',
    'LiftAndPlaceOnTopOfNearestR1':lambda s: 'LIFT + PLACE on top of nearest R1 KFS',
    'WaitForR1ToFreeSlot':         lambda s: 'WAIT — R1 must use weapons to clear an opp KFS',
    'WaitForR1ToDefend':           lambda s: 'WAIT — R1 to declare defended slot',
    'MFF':                         lambda s: 'IN MEIHUA FOREST — collecting KFS',
    'MC':                          lambda s: 'IN TRANSIT — moving forest → arena',
    'EmergencyStop':               lambda s: 'EMERGENCY STOP — link / safety triggered',
    'Idle':                        lambda s: 'IDLE',
}


def _slot_zone(slot: int) -> str:
    if slot == SLOT_MIDDLE: return 'middle column, middle row — center'
    if slot in SLOTS_SIDES: return 'middle row, side column'
    if slot in SLOTS_TOP_ROW: return 'top row — requires lift'
    if slot in SLOTS_BOTTOM_ROW: return 'bottom row — R1 placement'
    return f'slot {slot}'


def describe_r2_next_action(running_path: list) -> str:
    """Translate the running BT path into one operator-facing sentence."""
    if not running_path:
        return '(no active action)'
    leaf = running_path[-1]
    # Strip slot index suffix from PlaceInSlot[N] for matching
    base = leaf.split('[', 1)[0]
    arg = None
    if '[' in leaf and ']' in leaf:
        try:
            arg = int(leaf[leaf.index('[') + 1:leaf.rindex(']')])
        except ValueError:
            arg = None
    template = _R2_ACTION_TEMPLATES.get(base)
    if template:
        return template(arg)
    return f'{leaf}'


def describe_r1_advice(running_path: list, rack: dict, mode: str) -> str:
    """One-line guidance for the R1 operator, derived from BT state + rack."""
    if not running_path:
        return '—'
    leaf = running_path[-1]
    base = leaf.split('[', 1)[0]

    if base == 'PlaceInSlot' and '[' in leaf:
        slot = int(leaf[leaf.index('[') + 1:leaf.rindex(']')])
        col = ((slot - 1) % 3) + 1
        return f'Stay clear of column {col} — R2 placing in slot {slot}'
    if base in ('PutOnSide', 'PutOverThere'):
        return 'Stay clear of middle row — R2 placing now'
    if base == 'GetReadyForLifting' or base == 'LiftAndPlaceOnTopOfNearestR1':
        return 'MOVE TO LIFT POSITION — R2 docking for top-row placement'
    if base == 'WaitForR1ToFreeSlot':
        # Find an opp slot in middle row to clear
        opp_slots = [s for s in SLOTS_MIDDLE_ROW
                     if _slot_owner(rack, s) == 'OPP_KFS']
        if opp_slots:
            return f'USE WEAPONS to clear opp KFS in slot(s) {opp_slots}'
        return 'CLEAR an opp KFS to free a slot for R2'
    if base == 'WaitForR1ToDefend':
        return 'DECLARE the slot you are defending'
    if base == 'ReinforceDefendedColumn':
        return 'KEEP DEFENDING — R2 reinforcing your column'
    if base == 'EmergencyStop':
        return 'HALT — wait for r2_brain to recover'
    if base in ('MFF', 'MC'):
        return 'R2 not in arena yet — proceed with your own plan'
    return '—'


# ─────────────────────────────────────────────────────────────────────────────
#  Snapshot builder
# ─────────────────────────────────────────────────────────────────────────────

class SnapshotBuilder:
    """Owns the rolling log + tick stats; builds a BrainSnapshot per tick."""

    def __init__(self, log_capacity: int = 12):
        self.log = collections.deque(maxlen=log_capacity)
        self.tick_count = 0
        self._last_tick_t = time.monotonic()
        self._tick_hz = 0.0
        self._last_command: Optional[dict] = None
        self._last_command_t: float = 0.0
        self._placements_total = 0
        self._last_running_leaf: Optional[str] = None

    def append_log(self, line: str):
        ts = time.strftime('%H:%M:%S')
        self.log.appendleft(f'{ts} {line}')

    def record_command(self, cmd: dict):
        self._last_command = cmd
        self._last_command_t = time.monotonic()
        self.append_log(f"cmd → {cmd.get('cmd', '?')}  {dict(k for k in cmd.items() if k[0] != 'cmd')}")

    def build(self,
              tree: py_trees.trees.BehaviourTree,
              bb_get,
              ) -> BrainSnapshot:
        """`bb_get(key, default)` reads from whatever blackboard the caller uses."""
        self.tick_count += 1
        now = time.monotonic()
        dt = now - self._last_tick_t
        self._last_tick_t = now
        if dt > 0:
            inst = 1.0 / dt
            self._tick_hz = self._tick_hz * 0.9 + inst * 0.1 if self._tick_hz else inst

        rack       = bb_get('arena_rack', {}) or {}
        inventory  = bb_get('inventory', []) or []
        mode       = bb_get('arena_mode', 'attack') or 'attack'
        pose       = bb_get('pose', None)
        r1         = bb_get('r1_status', None)
        h7         = bb_get('h7_state', None)
        estop      = bool(bb_get('estop', False))

        path  = find_running_path(tree.root)
        leaf  = path[-1] if path else None
        if leaf and leaf != self._last_running_leaf:
            self.append_log(f'BT → {leaf}')
            self._last_running_leaf = leaf

        # Detect successful placements by counting our middle/top KFS in rack
        our_placed = sum(
            1 for s in SLOTS_MIDDLE_ROW + SLOTS_TOP_ROW
            if _slot_owner(rack, s) == 'TEAM_KFS'
        )
        self._placements_total = max(self._placements_total, our_placed)

        snap = BrainSnapshot(
            rack=rack,
            inventory=list(inventory),
            arena_mode=mode,
            pose=pose,
            r1_status=r1,
            h7_state=h7,
            estop=estop,
            tree_render=py_trees.display.unicode_tree(tree.root, show_status=True),
            running_path=path,
            running_leaf=leaf,
            last_status_summary=summarise_status(tree.root),
            last_command=self._last_command,
            last_command_age_s=(now - self._last_command_t) if self._last_command else 0.0,
            tick_count=self.tick_count,
            tick_hz=self._tick_hz,
            placements_total=self._placements_total,
            estimated_score=score_for_owner(rack, 'TEAM_KFS'),
            opp_score_estimate=score_for_owner(rack, 'OPP_KFS'),
            won_lines=winning_lines_for(rack, 'TEAM_KFS'),
            blocked_by_opp=threatening_lines_for(rack, 'OPP_KFS'),
            r2_next_action=describe_r2_next_action(path),
            r1_advice=describe_r1_advice(path, rack, mode),
            log=list(self.log),
        )
        return snap
