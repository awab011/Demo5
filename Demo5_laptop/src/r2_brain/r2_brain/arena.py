"""
Arena (Tic-Tac-Toe rack) subtree.

Rack layout (slot numbers — see lidar_camera_fusion.TikTacTeoRackGrid):

    [7][8][9]   top row    (R2 places only when lifted by R1, 80 pts)
    [4][5][6]   middle row (R2 places, 40 pts)
    [1][2][3]   bottom row (R1 places, 30 pts)

Columns: col0 = {1,4,7} (left), col1 = {2,5,8} (middle), col2 = {3,6,9} (right).

The top-level Arena selector chooses Defense or Attack based on
blackboard `arena_mode`. The Attack subtree is a priority Selector that
ticks once per cycle, picking the best next placement; when an action
leaf returns SUCCESS the BT inventory shrinks and the next tick re-picks.

Action leaves publish a JSON command on /r2/cmd/high_level. AggregatorNode
forwards them onto a CAN ID in the 0x110-0x11F laptop→H7 command range
(NOT yet wired — see HANDOFF §6.1).

NOTE: H7 → laptop telemetry does not exist yet (HANDOFF §6.2). Action
leaves currently fall back to an optimistic timeout-based ack. When
/r2/h7_state is wired, replace `_wait_for_ack` with a real state check.
"""

import json
import time

import py_trees
from std_msgs.msg import String

from .safety import R1ClearanceGuard
from .topics import (
    BB_ARENA_MODE,
    BB_ARENA_RACK,
    BB_INVENTORY,
    BB_R1_STATUS,
    TOPIC_CMD_HIGH_LEVEL,
    bb_get,
)


# Slot constants
SLOT_MIDDLE         = 5
SLOTS_MIDDLE_ROW    = (4, 5, 6)
SLOTS_SIDES         = (4, 6)
SLOTS_TOP_ROW       = (7, 8, 9)
SLOTS_BOTTOM_ROW    = (1, 2, 3)
TOP_ABOVE_BOTTOM    = {1: 4, 2: 5, 3: 6}   # bottom-row slot → mid-row slot directly above

# All 3-in-a-row lines on the rack (columns + diagonals).
WINNING_LINES = (
    (1, 4, 7), (2, 5, 8), (3, 6, 9),       # columns
    (1, 5, 9), (3, 5, 7),                  # diagonals
)


# ─────────────────────────────────────────────────────────────────────────────
#  Rack helpers
# ─────────────────────────────────────────────────────────────────────────────

def _slot_owner(rack: dict, slot: int) -> str:
    """Return 'TEAM_KFS' / 'OPP_KFS' / 'EMPTY'. Rack: {slot: {kfs_type, ...}|None}"""
    if rack is None:
        return 'UNKNOWN'
    info = rack.get(slot) or rack.get(str(slot))
    if info is None:
        return 'EMPTY'
    return info.get('kfs_type', 'UNKNOWN')


def _is_empty(rack: dict, slot: int) -> bool:
    return _slot_owner(rack, slot) == 'EMPTY'


def _r1_bottom_slots(rack: dict) -> list:
    """Bottom-row slots currently occupied by an R1 KFS (ours)."""
    return [s for s in SLOTS_BOTTOM_ROW if _slot_owner(rack, s) == 'TEAM_KFS']


def _find_completion_slot(rack: dict, owner: str):
    """
    Empty slot that would complete a 3-in-a-row for `owner`, or None.

    owner='TEAM_KFS' → R2's winning placement.
    owner='OPP_KFS'  → slot we must fill to block opponent's win.
    """
    for line in WINNING_LINES:
        owners = [_slot_owner(rack, s) for s in line]
        if owners.count(owner) == 2 and owners.count('EMPTY') == 1:
            return line[owners.index('EMPTY')]
    return None


def _r2_can_reach(rack: dict, slot: int) -> bool:
    """
    True if R2 can place into `slot`:
      - Mid-row    (4-6): always (autonomous placement).
      - Top-row    (7-9): always — R1 docks under R2 wherever R1 is and
                          lifts; the lift is a generic mechanism so R1's
                          column is irrelevant.
      - Bottom-row (1-3): never (R1's territory).
    """
    return slot in SLOTS_MIDDLE_ROW or slot in SLOTS_TOP_ROW


# ─────────────────────────────────────────────────────────────────────────────
#  Conditions
# ─────────────────────────────────────────────────────────────────────────────

class _BBCondition(py_trees.behaviour.Behaviour):
    """Base for read-only blackboard conditions."""

    def __init__(self, name: str, keys: tuple):
        super().__init__(name)
        self._bb = self.attach_blackboard_client(name=name)
        for k in keys:
            self._bb.register_key(k, access=py_trees.common.Access.READ)


class ModeIs(_BBCondition):
    def __init__(self, mode: str):
        super().__init__(f'Mode={mode}', (BB_ARENA_MODE,))
        self._mode = mode

    def update(self):
        cur = bb_get(self._bb, BB_ARENA_MODE, default='attack')
        return (py_trees.common.Status.SUCCESS if cur == self._mode
                else py_trees.common.Status.FAILURE)


class HasKFS(_BBCondition):
    def __init__(self, at_least: int = 1):
        super().__init__(f'HasKFS≥{at_least}', (BB_INVENTORY,))
        self._at_least = at_least

    def update(self):
        inv = bb_get(self._bb, BB_INVENTORY, default=[]) or []
        return (py_trees.common.Status.SUCCESS if len(inv) >= self._at_least
                else py_trees.common.Status.FAILURE)


class KFSCountIs(_BBCondition):
    def __init__(self, count: int):
        super().__init__(f'KFS=={count}', (BB_INVENTORY,))
        self._count = count

    def update(self):
        inv = bb_get(self._bb, BB_INVENTORY, default=[]) or []
        return (py_trees.common.Status.SUCCESS if len(inv) == self._count
                else py_trees.common.Status.FAILURE)


class MiddleSlotEmpty(_BBCondition):
    def __init__(self):
        super().__init__('MiddleSlotEmpty', (BB_ARENA_RACK,))

    def update(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={})
        return (py_trees.common.Status.SUCCESS if _is_empty(rack, SLOT_MIDDLE)
                else py_trees.common.Status.FAILURE)


class MiddleSlotIsOurs(_BBCondition):
    """True iff the centre slot (5) holds a TEAM_KFS."""
    def __init__(self):
        super().__init__('MiddleSlotIsOurs', (BB_ARENA_RACK,))

    def update(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={})
        return (py_trees.common.Status.SUCCESS
                if _slot_owner(rack, SLOT_MIDDLE) == 'TEAM_KFS'
                else py_trees.common.Status.FAILURE)


class SidesBothEmpty(_BBCondition):
    def __init__(self):
        super().__init__('SidesBothEmpty', (BB_ARENA_RACK,))

    def update(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={})
        return (py_trees.common.Status.SUCCESS
                if all(_is_empty(rack, s) for s in SLOTS_SIDES)
                else py_trees.common.Status.FAILURE)


class AnyMiddleRowSlotEmpty(_BBCondition):
    def __init__(self):
        super().__init__('AnyMiddleRowEmpty', (BB_ARENA_RACK,))

    def update(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={})
        return (py_trees.common.Status.SUCCESS
                if any(_is_empty(rack, s) for s in SLOTS_MIDDLE_ROW)
                else py_trees.common.Status.FAILURE)


class TopOfAnyR1Empty(_BBCondition):
    """At least one of R1's bottom-row KFS has an empty middle-row slot above it."""
    def __init__(self):
        super().__init__('TopOfAnyR1Empty', (BB_ARENA_RACK,))

    def update(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={})
        for b in _r1_bottom_slots(rack):
            if _is_empty(rack, TOP_ABOVE_BOTTOM[b]):
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class NoOppInstantWin(_BBCondition):
    """True iff the opponent has no 2-in-a-row threat (no slot completes opp's line)."""
    def __init__(self):
        super().__init__('NoOppInstantWin', (BB_ARENA_RACK,))

    def update(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={}) or {}
        return (py_trees.common.Status.SUCCESS
                if _find_completion_slot(rack, 'OPP_KFS') is None
                else py_trees.common.Status.FAILURE)


# ─────────────────────────────────────────────────────────────────────────────
#  Action leaves — publish high-level cmd, wait for ack
# ─────────────────────────────────────────────────────────────────────────────

class _PublishAndWait(py_trees.behaviour.Behaviour):
    """
    Publishes a high-level command once on entry, then waits for ack.

    TODO: replace _ack_received() with a real check against /r2/h7_state when
    the H7 telemetry frame lands (HANDOFF §6.2). Today we optimistically
    succeed after `ack_timeout_s` so the tree keeps moving in sim/dry-run.
    """

    def __init__(self, node, name: str, ack_timeout_s: float = 1.5):
        super().__init__(name)
        self._node = node
        self._pub = node.create_publisher(String, TOPIC_CMD_HIGH_LEVEL, 10)
        self._sent_at = None
        self._ack_timeout_s = ack_timeout_s

    def _build_cmd(self) -> dict:
        raise NotImplementedError

    def initialise(self):
        cmd = self._build_cmd()
        cmd.setdefault('seq', int(time.monotonic_ns() & 0xFF))
        self._pub.publish(String(data=json.dumps(cmd)))
        self._sent_at = time.monotonic()
        self._node.get_logger().info(f"[{self.name}] sent: {cmd}")

    def _ack_received(self) -> bool:
        # Placeholder until H7 telemetry exists
        return (time.monotonic() - (self._sent_at or 0)) > self._ack_timeout_s

    def update(self):
        if self._ack_received():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class PlaceInSlot(_PublishAndWait):
    """
    'Go drop a KFS in slot N.' N must be in middle or top row.

    Closes the loop on the rack: succeeds the moment the fusion node
    reports our KFS in the target slot. Falls back to ack_timeout if no
    sensor confirmation arrives.
    """

    def __init__(self, node, slot: int):
        super().__init__(node, f'PlaceInSlot[{slot}]', ack_timeout_s=2.5)
        self._slot = slot
        self._bb.register_key(BB_ARENA_RACK, access=py_trees.common.Access.READ) \
            if hasattr(self, '_bb') else None
        self._rack_bb = self.attach_blackboard_client(name=f'{self.name}.rack')
        self._rack_bb.register_key(BB_ARENA_RACK, access=py_trees.common.Access.READ)

    def _build_cmd(self):
        return {'cmd': 'PLACE_KFS', 'slot': self._slot}

    def _ack_received(self) -> bool:
        rack = bb_get(self._rack_bb, BB_ARENA_RACK, default={}) or {}
        if _slot_owner(rack, self._slot) == 'TEAM_KFS':
            return True
        return super()._ack_received()


class PickBestMiddleSideAndPlace(py_trees.behaviour.Behaviour):
    """
    Picks the nearest empty middle-row side slot at entry and delegates to
    PlaceInSlot. Used by 'Put on Both Sides' / 'Put over there'.
    """

    def __init__(self, node, name: str = 'PlaceInNearestMiddleRowSide'):
        super().__init__(name)
        self._node = node
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(BB_ARENA_RACK, access=py_trees.common.Access.READ)
        self._delegate = None

    def initialise(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={})
        empties = [s for s in SLOTS_MIDDLE_ROW if _is_empty(rack, s)]
        if not empties:
            self._delegate = None
            return
        # Prefer middle slot 5 then sides
        empties.sort(key=lambda s: (0 if s == SLOT_MIDDLE else 1, s))
        self._delegate = PlaceInSlot(self._node, empties[0])
        self._delegate.setup()
        self._delegate.initialise()

    def update(self):
        if self._delegate is None:
            return py_trees.common.Status.FAILURE
        return self._delegate.update()

    def terminate(self, new_status):
        if self._delegate is not None:
            self._delegate.terminate(new_status)


class GetReadyForLifting(_PublishAndWait):
    """Position R2 to be picked up by R1 (R1-clearance guard MUST be skipped here)."""

    def __init__(self, node):
        super().__init__(node, 'GetReadyForLifting', ack_timeout_s=2.0)

    def _build_cmd(self):
        return {'cmd': 'DOCK_FOR_LIFT'}


class LiftAndPlaceOnTopOfNearestR1(py_trees.behaviour.Behaviour):
    """
    Compound: dock with R1, get lifted, place into top-row slot above the
    nearest R1 bottom-row KFS that still has an empty slot above it.
    """

    def __init__(self, node):
        super().__init__('LiftAndPlaceOnTopOfNearestR1')
        self._node = node
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(BB_ARENA_RACK, access=py_trees.common.Access.READ)
        self._stage = None
        self._dock = GetReadyForLifting(node)
        self._place = None

    def initialise(self):
        self._stage = 'dock'
        self._dock.setup()
        self._dock.initialise()

    def update(self):
        if self._stage == 'dock':
            s = self._dock.update()
            if s == py_trees.common.Status.SUCCESS:
                rack = bb_get(self._bb, BB_ARENA_RACK, default={})
                bottoms = [b for b in _r1_bottom_slots(rack)
                           if _is_empty(rack, TOP_ABOVE_BOTTOM[b])]
                if not bottoms:
                    return py_trees.common.Status.FAILURE
                target = TOP_ABOVE_BOTTOM[bottoms[0]]
                self._place = PlaceInSlot(self._node, target)
                self._place.setup()
                self._place.initialise()
                self._stage = 'place'
                return py_trees.common.Status.RUNNING
            return s
        if self._stage == 'place':
            return self._place.update()
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self._dock is not None:
            self._dock.terminate(new_status)
        if self._place is not None:
            self._place.terminate(new_status)


class LiftAndPlaceInTopSlot(py_trees.behaviour.Behaviour):
    """
    Compound: dock with R1, get lifted, place into the *specified* top-row
    slot. Same flow as LiftAndPlaceOnTopOfNearestR1 but the target slot is
    fixed at construction time rather than recomputed after docking — used
    by PlaceToCompleteLine when the win/block target is in the top row.
    """

    def __init__(self, node, top_slot: int):
        super().__init__(f'LiftAndPlaceInTopSlot[{top_slot}]')
        self._node = node
        self._top_slot = top_slot
        self._dock = GetReadyForLifting(node)
        self._place = None
        self._stage = None

    def initialise(self):
        self._stage = 'dock'
        self._dock.setup()
        self._dock.initialise()

    def update(self):
        if self._stage == 'dock':
            s = self._dock.update()
            if s == py_trees.common.Status.SUCCESS:
                self._place = PlaceInSlot(self._node, self._top_slot)
                self._place.setup()
                self._place.initialise()
                self._stage = 'place'
                return py_trees.common.Status.RUNNING
            return s
        if self._stage == 'place':
            return self._place.update()
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self._dock is not None:
            self._dock.terminate(new_status)
        if self._place is not None:
            self._place.terminate(new_status)


class PlaceToCompleteLine(py_trees.behaviour.Behaviour):
    """
    Strategic placement that completes (or denies) a 3-in-a-row.

      owner='TEAM_KFS' → take the winning placement.
      owner='OPP_KFS'  → block the opponent's pending win.

    Routes the chosen slot to PlaceInSlot (mid-row) or LiftAndPlaceInTopSlot
    (top-row, dock+lift). Returns FAILURE when no completion exists, so the
    parent Selector falls through to the next priority.
    """

    def __init__(self, node, owner: str, name: str):
        super().__init__(name)
        self._node = node
        self._owner = owner
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(BB_ARENA_RACK, access=py_trees.common.Access.READ)
        self._delegate = None

    def initialise(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={}) or {}
        slot = _find_completion_slot(rack, self._owner)
        if slot is None or not _r2_can_reach(rack, slot):
            self._delegate = None
            return
        if slot in SLOTS_MIDDLE_ROW:
            self._delegate = PlaceInSlot(self._node, slot)
        else:
            self._delegate = LiftAndPlaceInTopSlot(self._node, slot)
        self._delegate.setup()
        self._delegate.initialise()

    def update(self):
        if self._delegate is None:
            return py_trees.common.Status.FAILURE
        return self._delegate.update()

    def terminate(self, new_status):
        if self._delegate is not None:
            self._delegate.terminate(new_status)


class LiftAndPlaceBestTopSlot(py_trees.behaviour.Behaviour):
    """
    Dock with R1 and place into the highest-value free top-row slot:

      preference 1 — top-row slot that completes a 3-in-a-row (pure win).
      preference 2 — center-top (slot 8), max scoring potential.
      preference 3 — any free top slot.

    Used by Flow 4 (last-KFS-saved-for-lift) where a lift placement is
    desired even when no immediate win is set up. FAILURE if every top
    slot is occupied.
    """

    _TOP_PREFERENCE = (8, 7, 9)

    def __init__(self, node):
        super().__init__('LiftAndPlaceBestTopSlot')
        self._node = node
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(BB_ARENA_RACK, access=py_trees.common.Access.READ)
        self._delegate = None

    def initialise(self):
        rack = bb_get(self._bb, BB_ARENA_RACK, default={}) or {}
        target = self._pick_top_slot(rack)
        if target is None:
            self._delegate = None
            return
        self._delegate = LiftAndPlaceInTopSlot(self._node, target)
        self._delegate.setup()
        self._delegate.initialise()

    def _pick_top_slot(self, rack):
        win_slot = _find_completion_slot(rack, 'TEAM_KFS')
        if win_slot in SLOTS_TOP_ROW:
            return win_slot
        for top in self._TOP_PREFERENCE:
            if _is_empty(rack, top):
                return top
        return None

    def update(self):
        if self._delegate is None:
            return py_trees.common.Status.FAILURE
        return self._delegate.update()

    def terminate(self, new_status):
        if self._delegate is not None:
            self._delegate.terminate(new_status)


class WaitForR1To(_BBCondition):
    """Generic wait-for-R1-state condition. Returns RUNNING until predicate true."""

    def __init__(self, name: str, predicate, keys=(BB_R1_STATUS,)):
        super().__init__(name, keys)
        self._predicate = predicate

    def update(self):
        try:
            ok = self._predicate(self._bb)
        except Exception:
            ok = False
        return (py_trees.common.Status.SUCCESS if ok
                else py_trees.common.Status.RUNNING)


def _r1_freed_a_slot(bb) -> bool:
    r1 = bb.get(BB_R1_STATUS, default=None) or {}
    return bool(r1.get('cleared_slot'))


def _r1_is_defending(bb) -> bool:
    r1 = bb.get(BB_R1_STATUS, default=None) or {}
    return r1.get('mode') == 'DEFEND' and r1.get('defended_slot') is not None


# ─────────────────────────────────────────────────────────────────────────────
#  Subtree builders
# ─────────────────────────────────────────────────────────────────────────────

def build_attack_priority(node) -> py_trees.behaviour.Behaviour:
    """
    Priority Selector for one placement cycle. Re-ticked next round when
    inventory drops. Tactical (win/block) always come first; the rest is
    inventory-aware so the plan adapts as KFS are consumed.

      1. Win now              — complete our 3-in-a-row (column or diagonal).
      2. Block opp            — fill empty slot of opp's pending 3-in-a-row.
      3. Save-for-lift        — last KFS + opp has no instant threat
                                → wait for lift, place top-row (80 pts).
      4. Take middle (slot 5).
      5. Mid-row above an R1  — middle already ours AND ≥2 KFS remaining.
                                Sets up a side-column threat we finish from
                                the top row next cycle. Fails for 2-KFS
                                budgets (only 1 left after taking middle),
                                so they fall through to save-for-lift (#3).
      6. Take nearest side    — middle full + sides empty + ≥3 KFS.
      7. Any mid-row empty    — generic fallback.
      8. Wait for R1 freed    — nothing reachable; wait until R1 clears one.

    Lines considered for win/block are columns and diagonals only — full-row
    wins are unreachable because R2 is capped at ≤2 KFS per row.
    """
    sel = py_trees.composites.Selector('PlaceOneKFS', memory=False)

    # 1-2: Tactical priorities — never miss a win or an opp-block opportunity.
    win_now   = PlaceToCompleteLine(node, 'TEAM_KFS', 'WinIfPossible')
    block_opp = PlaceToCompleteLine(node, 'OPP_KFS',  'BlockOppWin')

    # 3: Last KFS + no opp threat → save it for the high-scoring top-row slot.
    save_for_lift = py_trees.composites.Sequence('SaveLastKFSForLift', memory=True)
    save_for_lift.add_children([
        KFSCountIs(1),
        NoOppInstantWin(),
        LiftAndPlaceBestTopSlot(node),
    ])

    # 4: Middle slot is open — best opening / recovery move.
    take_middle = py_trees.composites.Sequence('TakeMiddle', memory=True)
    take_middle.add_children([MiddleSlotEmpty(), PlaceInSlot(node, SLOT_MIDDLE)])

    # 5: Middle is already ours and we still have ≥2 KFS — place mid-row
    #    directly above an R1 bottom KFS to set up a side-column threat we
    #    can finish from the top row next cycle. With only 1 KFS left after
    #    taking middle (i.e., 2-KFS budget) this branch fails the HasKFS(2)
    #    gate, leaving save_for_lift (#3) to take the lift-to-win path.
    place_mid_above_r1 = py_trees.composites.Sequence('PlaceMidAboveR1', memory=True)
    place_mid_above_r1.add_children([
        HasKFS(at_least=2),
        MiddleSlotIsOurs(),
        TopOfAnyR1Empty(),
        LiftAndPlaceOnTopOfNearestR1(node),
    ])

    # 6: Middle full, both sides empty, ≥3 KFS → take nearest side.
    take_nearest_side = py_trees.composites.Sequence('TakeNearestSide', memory=True)
    take_nearest_side.add_children([
        HasKFS(at_least=3),
        SidesBothEmpty(),
        PickBestMiddleSideAndPlace(node, 'NearestSide'),
    ])

    # 7: Any mid-row slot empty (catch-all when none of the above fit).
    take_overthere = py_trees.composites.Sequence('PutOverThere', memory=True)
    take_overthere.add_children([
        AnyMiddleRowSlotEmpty(),
        PickBestMiddleSideAndPlace(node, 'PutOverThere'),
    ])

    # 8: Mid-row fully occupied → wait for R1 to clear a slot, then place.
    wait_freed = py_trees.composites.Sequence('PlaceInFreedSlot', memory=True)
    wait_freed.add_children([
        WaitForR1To('WaitForR1ToFreeSlot', _r1_freed_a_slot),
        PickBestMiddleSideAndPlace(node, 'PlaceInFreedSlot'),
    ])

    sel.add_children([
        win_now, block_opp,
        save_for_lift,
        take_middle,
        place_mid_above_r1,
        take_nearest_side,
        take_overthere,
        wait_freed,
    ])
    return sel


def build_defense_subtree(node) -> py_trees.behaviour.Behaviour:
    """
    Defense: wait for R1 to declare a defended slot, then if we hold ≥3 KFS
    reinforce it by placing one in the column of the defended slot.
    """
    seq = py_trees.composites.Sequence('Defense', memory=True)
    seq.add_children([
        WaitForR1To('WaitForR1ToDefend', _r1_is_defending),
        HasKFS(at_least=3),
        PickBestMiddleSideAndPlace(node, 'ReinforceDefendedColumn'),
    ])
    return seq


# ─────────────────────────────────────────────────────────────────────────────
#  Pure strategy helper — no BT / ROS, used by the tester dashboard
# ─────────────────────────────────────────────────────────────────────────────

def compute_next_move(rack: dict, inventory: int, mode: str) -> dict:
    """
    Return {'move': str, 'slot': int|None, 'priority': str} for the current
    rack state.  Mirrors the attack-priority order in build_attack_priority().
    rack: {slot_num (int or str) -> {'kfs_type': ...} | None}
    inventory: number of KFS R2 currently holds
    mode: 'attack' | 'defense'
    """
    if mode == 'defense':
        return {'move': 'DEFENSE MODE — waiting for R1 to declare defended slot',
                'slot': None, 'priority': 'defense'}

    if inventory == 0:
        return {'move': 'No KFS in inventory — cannot place', 'slot': None, 'priority': 'idle'}

    # 1. Win now
    slot = _find_completion_slot(rack, 'TEAM_KFS')
    if slot and _r2_can_reach(rack, slot):
        row = 'top — needs lift' if slot in SLOTS_TOP_ROW else 'middle'
        return {'move': f'WIN — place in slot {slot} ({row})', 'slot': slot, 'priority': 'win'}

    # 2. Block opponent
    slot = _find_completion_slot(rack, 'OPP_KFS')
    if slot and _r2_can_reach(rack, slot):
        row = 'top — needs lift' if slot in SLOTS_TOP_ROW else 'middle'
        return {'move': f'BLOCK OPP — place in slot {slot} ({row})', 'slot': slot, 'priority': 'block'}

    # 3. Save last KFS for top-row lift
    if inventory == 1 and _find_completion_slot(rack, 'OPP_KFS') is None:
        top = next((s for s in (8, 7, 9) if _is_empty(rack, s)), None)
        if top:
            return {'move': f'SAVE FOR LIFT — place in top slot {top}', 'slot': top, 'priority': 'save_lift'}

    # 4. Take centre (slot 5)
    if _is_empty(rack, SLOT_MIDDLE):
        return {'move': 'TAKE MIDDLE — place in slot 5', 'slot': 5, 'priority': 'middle'}

    # 5. Place mid-row above an R1 bottom KFS (≥2 KFS required)
    if inventory >= 2:
        for b in _r1_bottom_slots(rack):
            above = TOP_ABOVE_BOTTOM[b]
            if _is_empty(rack, above):
                return {'move': f'ABOVE R1 — slot {above} (above R1 slot {b})',
                        'slot': above, 'priority': 'above_r1'}

    # 6. Take nearest empty side (≥3 KFS, both sides empty)
    if inventory >= 3 and all(_is_empty(rack, s) for s in SLOTS_SIDES):
        side = next((s for s in SLOTS_SIDES if _is_empty(rack, s)), None)
        if side:
            return {'move': f'TAKE SIDE — slot {side}', 'slot': side, 'priority': 'side'}

    # 7. Any mid-row empty
    mid = next((s for s in SLOTS_MIDDLE_ROW if _is_empty(rack, s)), None)
    if mid:
        return {'move': f'ANY MID — slot {mid}', 'slot': mid, 'priority': 'any_mid'}

    # 8. Nothing reachable
    return {'move': 'WAIT — mid-row full, waiting for R1 to clear a slot',
            'slot': None, 'priority': 'wait'}


def build_arena_subtree(node) -> py_trees.behaviour.Behaviour:
    """
    Top of the Arena subtree.

      Arena (Selector)
        ├─ Defense (Sequence: ModeIs(defense) → defense flow)
        └─ Attack  (Sequence: ModeIs(attack)  → HasKFS → priority placement)
    """
    arena = py_trees.composites.Selector('Arena', memory=False)

    defense_branch = py_trees.composites.Sequence('DefenseBranch', memory=True)
    defense_branch.add_children([ModeIs('defense'), build_defense_subtree(node)])

    attack_branch = py_trees.composites.Sequence('AttackBranch', memory=True)
    attack_branch.add_children([ModeIs('attack'), HasKFS(1), build_attack_priority(node)])

    arena.add_children([defense_branch, attack_branch])
    return arena
