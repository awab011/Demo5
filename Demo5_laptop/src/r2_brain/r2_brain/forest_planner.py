"""
Forest (Meihua) R2 path planner — pure Python, no ROS / py_trees.

This is a faithful port of the R2 planner in the Flutter app's cal_path.dart
(``CalPath.calculate`` → ``_planR2`` → ``_aStar`` → ``buildCanPath``). The app
used to run this on-device and ship the resulting CAN path to the robot; the
app is now operator-console-only and r2_brain is the single planner, so the
logic lives here. phases.py wraps ``plan_r2_path`` in a BT leaf that publishes
the result as an R2Path on /r2_path (AggregatorNode packs it onto CAN 0x100).

Output shape mirrors cal_path.dart's ``R2CanPath.toJson`` /
``R2CanStep.toJson`` exactly, so the byte layout AggregatorNode.r2can_pack
produces is unchanged. A parity harness (tools/forest_parity_check.py) diffs
this against the Dart implementation over a grid corpus.

Grid encoding (matches the app): ``states[i]`` is block ``i+1``'s class —
0=empty, 1=R1_KFS, 2=R2_KFS, 3=FAKE (treated as a blocked cell).

The A* manual binary heap, neighbour iteration order, and tie-breaking are
reproduced 1:1 from the Dart so the chosen path matches in equal-cost cases —
do not "optimise" them to heapq without re-running the parity check.
"""

from __future__ import annotations

from typing import Optional

# ── Action / direction / cost constants (cal_path.dart top) ──────────────────
_ACT_MOVE         = 0
_ACT_PICKUP       = 1
_ACT_MOVE_PICKUP  = 2   # move onto KFS cell and pick up atomically

_DIR_DOWN  = 0
_DIR_RIGHT = 1
_DIR_UP    = 2
_DIR_LEFT  = 3

_C_MOVE2     = 2
_C_TURN90_2  = 2
_C_TURN180_2 = 4
_C_BACK_PEN2 = 6
_C_ARM2      = 1
_C_R1_WAIT2  = 6

_ROWS, _COLS, _NUM_BLOCKS = 4, 3, 12

_BLOCK_HEIGHT_MM = {
    1: 400, 2: 200, 3: 400, 4: 200, 5: 400, 6: 600,
    7: 400, 8: 600, 9: 400, 10: 200, 11: 400, 12: 200,
}

# block -> [row, col]
_R2_RED = {
    12: [0, 0], 11: [0, 1], 10: [0, 2],
    9:  [1, 0], 8:  [1, 1], 7:  [1, 2],
    6:  [2, 0], 5:  [2, 1], 4:  [2, 2],
    3:  [3, 0], 2:  [3, 1], 1:  [3, 2],
}
_R2_BLUE = {
    10: [0, 0], 11: [0, 1], 12: [0, 2],
    7:  [1, 0], 8:  [1, 1], 9:  [1, 2],
    4:  [2, 0], 5:  [2, 1], 6:  [2, 2],
    1:  [3, 0], 2:  [3, 1], 3:  [3, 2],
}

# ── R2CanStep enums (cal_path.dart) ──────────────────────────────────────────
_CAN_ACT_MOVE      = 0
_CAN_ACT_PICKUP    = 1
_CAN_ACT_PRE_ENTRY = 2

_CAN_DIR_DOWN  = 0
_CAN_DIR_RIGHT = 1
_CAN_DIR_UP    = 2
_CAN_DIR_LEFT  = 3

_HD_NEG400 = 0
_HD_NEG200 = 1
_HD_ZERO   = 2
_HD_POS200 = 3
_HD_POS400 = 4
_HD_POS600 = 5
_HD_INVALID = 7


# ── Geometry / cost helpers ──────────────────────────────────────────────────

def _row_of(i: int) -> int:
    return i // _COLS


def _col_of(i: int) -> int:
    return i % _COLS


def _idx(r: int, c: int) -> int:
    return r * _COLS + c


def _block1(cell0: int, gp: dict) -> int:
    """Grid index (0-11) -> block number (1-12)."""
    for k, v in gp.items():
        if v[0] * _COLS + v[1] == cell0:
            return k
    return cell0 + 1


def _step(i: int, d: int) -> int:
    r, c = _row_of(i), _col_of(i)
    if d == _DIR_DOWN:
        r += 1
    elif d == _DIR_UP:
        r -= 1
    elif d == _DIR_RIGHT:
        c += 1
    elif d == _DIR_LEFT:
        c -= 1
    if r < 0 or r >= _ROWS or c < 0 or c >= _COLS:
        return -1
    return _idx(r, c)


def _rot_cost2(cur: int, tgt: int) -> int:
    d = (tgt - cur + 4) % 4
    if d == 0:
        return 0
    return _C_TURN180_2 if d == 2 else _C_TURN90_2


def _move_cost2(cf: int, d: int) -> int:
    return _rot_cost2(cf, d) + _C_MOVE2 + (_C_BACK_PEN2 if d == _DIR_DOWN else 0)


def _h2(cell0: int, collected: int, target_count: int) -> int:
    return _row_of(cell0) * 2 + (target_count - collected) * 4


# ── A* state ─────────────────────────────────────────────────────────────────

def _state_key(block: int, collected: int, facing: int, picked: int) -> int:
    return ((block & 0xF)
            | ((collected & 0x3) << 4)
            | ((facing & 0x3) << 6)
            | ((picked & 0xFF) << 8))


class _Act:
    __slots__ = ('type', 'direction', 'target_cell')

    def __init__(self, type_: int, direction: int, target_cell: int):
        self.type = type_
        self.direction = direction
        self.target_cell = target_cell


class _Plan:
    __slots__ = ('total_cost2', 'states', 'actions')

    def __init__(self, total_cost2, states, actions):
        self.total_cost2 = total_cost2
        self.states = states      # list of (block, collected, facing, picked)
        self.actions = actions    # list of _Act | None


# ── Manual binary min-heap on f (ported 1:1 to preserve tie-breaking) ────────
# Each node is a list [f, g, state_tuple] where state_tuple = (block, collected,
# facing, picked). Only f (index 0) participates in heap ordering.

def _heap_push(heap, node):
    heap.append(node)
    i = len(heap) - 1
    while i > 0:
        p = (i - 1) // 2
        if heap[p][0] <= heap[i][0]:
            break
        heap[p], heap[i] = heap[i], heap[p]
        i = p


def _heap_pop(heap):
    top = heap[0]
    last = heap.pop()
    if heap:
        heap[0] = last
        i = 0
        n = len(heap)
        while True:
            s, l, r = i, 2 * i + 1, 2 * i + 2
            if l < n and heap[l][0] < heap[s][0]:
                s = l
            if r < n and heap[r][0] < heap[s][0]:
                s = r
            if s == i:
                break
            heap[s], heap[i] = heap[i], heap[s]
            i = s
    return top


def _a_star(boxes, r2c0, entry0, start_facing, init_picked, init_collected,
            target_count) -> Optional[_Plan]:
    g_score = {}
    parent_k = {}
    parent_a = {}

    start = (entry0, init_collected, start_facing, init_picked)
    sk = _state_key(*start)
    g_score[sk] = 0
    parent_a[sk] = None

    heap = [[_h2(entry0, init_collected, target_count), 0, start]]

    while heap:
        cur = _heap_pop(heap)
        cur_g = cur[1]
        cur_state = cur[2]
        cs_block, cs_collected, cs_facing, cs_picked = cur_state
        cs_key = _state_key(*cur_state)
        g_cur = g_score.get(cs_key)
        if g_cur is None or cur_g > g_cur:
            continue

        # Goal: collected enough and back on the top row (row 0).
        if cs_collected >= target_count and _row_of(cs_block) == 0:
            keys = []
            c = cs_key
            while c in parent_k:
                keys.append(c)
                c = parent_k[c]
            keys.append(sk)
            ordered = list(reversed(keys))
            sts = [((k & 0xF), (k >> 4) & 0x3, (k >> 6) & 0x3, (k >> 8) & 0xFF)
                   for k in ordered]
            acs = [None]
            for i in range(1, len(ordered)):
                acs.append(parent_a[ordered[i]])
            return _Plan(cur_g, sts, acs)

        # --- Side-arm pickup: pick up an adjacent KFS without moving ---
        if cs_collected < target_count:
            for d in range(4):
                adj = _step(cs_block, d)
                if adj < 0:
                    continue
                bit = r2c0.index(adj) if adj in r2c0 else -1
                if bit < 0 or (cs_picked & (1 << bit)) != 0:
                    continue
                pc = _C_ARM2
                ns = (cs_block, cs_collected + 1, cs_facing, cs_picked | (1 << bit))
                nk = _state_key(*ns)
                ng = cur_g + pc
                if nk not in g_score or ng < g_score[nk]:
                    g_score[nk] = ng
                    parent_k[nk] = cs_key
                    parent_a[nk] = _Act(_ACT_PICKUP, d, adj)
                    _heap_push(heap, [ng + _h2(cs_block, cs_collected + 1, target_count),
                                      ng, ns])

        # --- Movement (with optional simultaneous KFS pickup) ---
        for d in range(4):
            nxt = _step(cs_block, d)
            if nxt < 0:
                continue
            if boxes[nxt] == 3:
                continue

            bit = r2c0.index(nxt) if nxt in r2c0 else -1

            # Already have enough and this is an unpicked KFS ahead — don't
            # walk past KFS we don't need.
            if bit >= 0 and (cs_picked & (1 << bit)) == 0 and cs_collected >= target_count:
                continue

            mc = _move_cost2(cs_facing, d)
            if boxes[nxt] == 1:
                mc += _C_R1_WAIT2

            is_kfs_ahead = (bit >= 0 and (cs_picked & (1 << bit)) == 0
                            and cs_collected < target_count)
            nc = cs_collected + 1 if is_kfs_ahead else cs_collected
            np_ = cs_picked | (1 << bit) if is_kfs_ahead else cs_picked
            act_type = _ACT_MOVE_PICKUP if is_kfs_ahead else _ACT_MOVE

            ns = (nxt, nc, d, np_)
            nk = _state_key(*ns)
            ng = cur_g + mc
            if nk not in g_score or ng < g_score[nk]:
                g_score[nk] = ng
                parent_k[nk] = cs_key
                parent_a[nk] = _Act(act_type, d, nxt)
                _heap_push(heap, [ng + _h2(nxt, nc, target_count), ng, ns])

    return None


def _is_better(res: _Plan, best: _Plan) -> bool:
    if res.total_cost2 < best.total_cost2:
        return True
    if res.total_cost2 > best.total_cost2:
        return False
    res_moves = sum(1 for a in res.actions
                    if a is not None and a.type in (_ACT_MOVE, _ACT_MOVE_PICKUP))
    best_moves = sum(1 for a in best.actions
                     if a is not None and a.type in (_ACT_MOVE, _ACT_MOVE_PICKUP))
    return res_moves < best_moves


# ── R2 path result (mirrors cal_path PathResult fields we use) ───────────────

class _R2Result:
    __slots__ = ('waypoints', 'full_sequence', 'chosen_target', 'entry_kfs',
                 'side_targets', 'using_side_arm')

    def __init__(self, waypoints, full_sequence, chosen_target, entry_kfs,
                 side_targets, using_side_arm):
        self.waypoints = waypoints
        self.full_sequence = full_sequence
        self.chosen_target = chosen_target
        self.entry_kfs = entry_kfs
        self.side_targets = side_targets
        self.using_side_arm = using_side_arm

    @property
    def is_empty(self) -> bool:
        return len(self.full_sequence) == 0


def _empty_r2() -> _R2Result:
    return _R2Result([], [], [], [], [], False)


def _plan_r2(states, team, is_retry, kfs_count) -> _R2Result:
    gp = _R2_RED if team == 'red' else _R2_BLUE

    boxes = [0] * _NUM_BLOCKS
    for i in range(len(states)):
        p = gp.get(i + 1)
        if p is not None:
            boxes[p[0] * _COLS + p[1]] = states[i]

    r2c0 = [i for i in range(_NUM_BLOCKS) if boxes[i] == 2]

    if not r2c0 and kfs_count > 0:
        return _empty_r2()

    target_count = max(0, min(kfs_count, len(r2c0)))
    entry_r2c0 = [c for c in r2c0 if _row_of(c) == _ROWS - 1]

    best: Optional[_Plan] = None
    best_entry0: Optional[int] = None
    best_pre_picks0: list = []

    if is_retry:
        for col in range(_COLS):
            eb = _idx(_ROWS - 1, col)
            if boxes[eb] == 3:
                continue
            sp, sc = 0, 0
            bit = r2c0.index(eb) if eb in r2c0 else -1
            if bit >= 0 and sc < target_count:
                sp |= (1 << bit)
                sc += 1
            res = _a_star(boxes, r2c0, eb, _DIR_UP, sp, sc, target_count)
            if res is not None and (best is None or _is_better(res, best)):
                best = res
                best_entry0 = eb
                best_pre_picks0 = []
    else:
        subsets = 1 << len(entry_r2c0)
        for mask in range(subsets):
            init_picked, init_collected = 0, 0
            pre_picks = []
            for i in range(len(entry_r2c0)):
                if mask & (1 << i):
                    bit = r2c0.index(entry_r2c0[i]) if entry_r2c0[i] in r2c0 else -1
                    if bit >= 0:
                        init_picked |= (1 << bit)
                        init_collected += 1
                        pre_picks.append(entry_r2c0[i])
            if init_collected >= target_count:
                continue

            for col in range(_COLS):
                eb = _idx(_ROWS - 1, col)
                if boxes[eb] == 3:
                    continue
                sp, sc = init_picked, init_collected
                bit = r2c0.index(eb) if eb in r2c0 else -1
                if bit >= 0 and (sp & (1 << bit)) == 0 and sc < target_count:
                    sp |= (1 << bit)
                    sc += 1
                res = _a_star(boxes, r2c0, eb, _DIR_UP, sp, sc, target_count)
                if res is not None and (best is None or _is_better(res, best)):
                    best = res
                    best_entry0 = eb
                    best_pre_picks0 = list(pre_picks)

    if best is None or best_entry0 is None:
        return _empty_r2()

    phys_path1 = [_block1(s[0], gp) for s in best.states]

    entry_block1 = _block1(best_entry0, gp)
    entry_is_kfs = (len(best.states) > 0
                    and best.states[0][1] > 0
                    and entry_block1 not in [_block1(c, gp) for c in best_pre_picks0]
                    and boxes[best_entry0] == 2)
    entry_kfs1 = [entry_block1] if entry_is_kfs else []

    chosen1 = []
    for i in range(1, len(best.states)):
        s = best.states[i]
        prev = best.states[i - 1]
        act = best.actions[i]
        if (act is not None
                and act.type in (_ACT_MOVE, _ACT_MOVE_PICKUP)
                and s[1] > prev[1]):
            chosen1.append(_block1(s[0], gp))

    arm_picks1 = []
    for i in range(1, len(best.actions)):
        a = best.actions[i]
        if a is not None and a.type == _ACT_PICKUP:
            arm_picks1.append(_block1(a.target_cell, gp))

    pre_entry1 = [_block1(c, gp) for c in best_pre_picks0]
    exit_block1 = _block1(best.states[-1][0], gp)

    return _R2Result(
        waypoints=[*entry_kfs1, *chosen1, exit_block1],
        full_sequence=phys_path1,
        chosen_target=chosen1,
        entry_kfs=entry_kfs1,
        side_targets=[*pre_entry1, *arm_picks1],
        using_side_arm=bool(arm_picks1) or bool(pre_entry1),
    )


# ── CAN path build (mirrors cal_path.buildCanPath) ───────────────────────────

def _calc_height_delta_enc(cur1: int, tgt1: int) -> int:
    hc = _BLOCK_HEIGHT_MM.get(cur1)
    ht = _BLOCK_HEIGHT_MM.get(tgt1)
    if hc is None or ht is None:
        return _HD_INVALID
    return {-400: _HD_NEG400, -200: _HD_NEG200, 0: _HD_ZERO,
            200: _HD_POS200, 400: _HD_POS400}.get(ht - hc, _HD_INVALID)


def _calc_height_delta_mm(cur1: int, tgt1: int) -> int:
    return _BLOCK_HEIGHT_MM.get(tgt1, 0) - _BLOCK_HEIGHT_MM.get(cur1, 0)


def _calc_pre_entry_height_enc(tgt1: int) -> int:
    return {200: _HD_POS200, 400: _HD_POS400, 600: _HD_POS600}.get(
        _BLOCK_HEIGHT_MM.get(tgt1, 0), _HD_ZERO)


def _step_dict(action_type, current_block, target_block, direction,
               collected_after, requires_r1_clear, auto_pickup,
               height_delta_enc, height_delta_mm) -> dict:
    return {
        'action_type':       action_type,
        'current_block':     current_block,
        'target_block':      target_block,
        'direction':         direction,
        'collected_after':   collected_after,
        'requires_r1_clear': requires_r1_clear,
        'auto_pickup':       auto_pickup,
        'height_delta_enc':  height_delta_enc,
        'height_delta_mm':   height_delta_mm,
    }


def _build_can_path(r2: _R2Result, states, team, is_retry) -> dict:
    gp = _R2_RED if team == 'red' else _R2_BLUE
    steps: list = []

    if r2.is_empty:
        return {
            'total_cost': 0, 'entry_block': 0, 'exit_block': 0,
            'boxes_to_collect': 3, 'grid_state': list(states),
            'pre_entry_count': 0, 'step_count': 0, 'steps': [],
        }

    phys_path = list(r2.full_sequence)
    chosen_targets = list(r2.chosen_target)
    side_targets = list(r2.side_targets)

    entry_block1 = phys_path[0]
    exit_block1 = phys_path[-1]
    entry_row = gp[entry_block1][0]
    entry_col = gp[entry_block1][1]

    if is_retry:
        pre_entry_picks = []
    else:
        pre_entry_picks = [s for s in side_targets
                           if gp.get(s) is not None
                           and gp[s][0] == entry_row
                           and s not in phys_path]

    collected_so_far = 0

    for pe in pre_entry_picks:
        pe_col = gp[pe][1]
        direction = _CAN_DIR_RIGHT if pe_col > entry_col else _CAN_DIR_LEFT
        collected_so_far += 1
        steps.append(_step_dict(
            _CAN_ACT_PRE_ENTRY, 0, pe, direction, collected_so_far,
            False, False, _calc_pre_entry_height_enc(pe), _BLOCK_HEIGHT_MM.get(pe, 0)))

    pre_entry_count = len(steps)
    entry_box_is_r2 = states[entry_block1 - 1] == 2
    entry_was_pre = entry_block1 in pre_entry_picks
    entry_auto_pickup = entry_box_is_r2 and not entry_was_pre
    if entry_auto_pickup:
        collected_so_far += 1

    steps.append(_step_dict(
        _CAN_ACT_MOVE, 0, entry_block1, _CAN_DIR_DOWN, collected_so_far,
        states[entry_block1 - 1] == 1, entry_auto_pickup,
        _calc_pre_entry_height_enc(entry_block1), _BLOCK_HEIGHT_MM.get(entry_block1, 0)))

    cur_block1 = entry_block1
    arm_picks = [s for s in side_targets if s not in pre_entry_picks]
    emitted = set()

    for s in [s for s in arm_picks if gp[s][0] == entry_row]:
        s_col = gp[s][1]
        direction = _CAN_DIR_RIGHT if s_col > entry_col else _CAN_DIR_LEFT
        collected_so_far += 1
        emitted.add(s)
        steps.append(_step_dict(
            _CAN_ACT_PICKUP, cur_block1, s, direction, collected_so_far,
            False, False, _calc_height_delta_enc(cur_block1, s),
            _calc_height_delta_mm(cur_block1, s)))

    for i in range(1, len(phys_path)):
        next1 = phys_path[i]
        next_row = gp[next1][0]
        next_col = gp[next1][1]
        prev_row = gp[phys_path[i - 1]][0]
        prev_col = gp[phys_path[i - 1]][1]

        if next_row < prev_row:
            move_dir = _CAN_DIR_UP
        elif next_row > prev_row:
            move_dir = _CAN_DIR_DOWN
        elif next_col > prev_col:
            move_dir = _CAN_DIR_RIGHT
        else:
            move_dir = _CAN_DIR_LEFT

        auto_pickup = next1 in chosen_targets
        if auto_pickup:
            collected_so_far += 1

        steps.append(_step_dict(
            _CAN_ACT_MOVE, cur_block1, next1, move_dir, collected_so_far,
            states[next1 - 1] == 1, auto_pickup,
            _calc_height_delta_enc(cur_block1, next1),
            _calc_height_delta_mm(cur_block1, next1)))
        cur_block1 = next1

        arm_here = [s for s in arm_picks
                    if gp[s][0] == next_row and s not in emitted]
        # Dart sorts left-of-pivot before right-of-pivot (stable).
        arm_here.sort(key=lambda s: 1 if gp[s][1] > next_col else 0)
        for s in arm_here:
            s_col = gp[s][1]
            direction = _CAN_DIR_RIGHT if s_col > next_col else _CAN_DIR_LEFT
            collected_so_far += 1
            emitted.add(s)
            steps.append(_step_dict(
                _CAN_ACT_PICKUP, cur_block1, s, direction, collected_so_far,
                False, False, _calc_height_delta_enc(cur_block1, s),
                _calc_height_delta_mm(cur_block1, s)))

    return {
        'total_cost': len(steps),
        'entry_block': entry_block1,
        'exit_block': exit_block1,
        'boxes_to_collect': 3,
        'grid_state': list(states),
        'pre_entry_count': pre_entry_count,
        'step_count': len(steps),
        'steps': steps,
    }


# ── Public entry point ───────────────────────────────────────────────────────

def plan_r2_path(states, team='red', kfs_count=3, already_picked=None) -> dict:
    """
    Plan R2's forest path and return an R2CanPath-shaped dict (identical layout
    to cal_path.dart's ``CalPath.buildCanPath(...).toJson()``).

    states         : list[12] of block classes (0=empty 1=R1 2=R2 3=fake).
    team           : 'red' | 'blue'.
    kfs_count      : how many R2 KFS to collect.
    already_picked : block numbers already collected (retry mode); None/[] = fresh.

    Returns {} fields exactly as the app produced them, so AggregatorNode's
    r2can_pack yields the same CAN frames.
    """
    states = list(states)
    if len(states) != _NUM_BLOCKS:
        raise ValueError(f'states must have {_NUM_BLOCKS} entries, got {len(states)}')

    picked = list(already_picked) if already_picked else []
    is_retry = len(picked) > 0

    left_to_pick = max(0, kfs_count - len(picked))

    effective = list(states)
    for p in picked:
        if 1 <= p <= 12:
            effective[p - 1] = 0

    r2 = _plan_r2(effective, team, is_retry, left_to_pick)
    return _build_can_path(r2, states, team, is_retry)
