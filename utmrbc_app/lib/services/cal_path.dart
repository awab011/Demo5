// RETIRED - offline/sim only.
//
// This planner used to run on-device and send computed CAN paths to the robot.
// As of the r2_brain integration, r2_brain is the SINGLE planner/controller.
// cal_path is kept ONLY to render the operator preview in mf.dart (grid
// overlays, route boxes, retry dialog). Nothing here is on the flight path to
// the robot. Do NOT wire buildCanPath()/R2CanPath back into the live send path
// — the app publishes raw grid intent on /r2/forest/manual_grid instead.
import 'package:flutter/foundation.dart';

const int _actMove = 0;
const int _actPickup = 1;
const int _actMovePickup = 2; // move onto KFS cell and pick up atomically

const int _dirDown = 0;
const int _dirRight = 1;
const int _dirUp = 2;
const int _dirLeft = 3;

const int _cMove2 = 2;
const int _cTurn902 = 2;
const int _cTurn1802 = 4;
const int _cBackPen2 = 6;
const int _cArm2 = 1;
const int _cR1Wait2 = 6;

const int _rows = 4, _cols = 3, _numBlocks = 12;

const Map<int, int> _blockHeightMm = {
  1: 400,
  2: 200,
  3: 400,
  4: 200,
  5: 400,
  6: 600,
  7: 400,
  8: 600,
  9: 400,
  10: 200,
  11: 400,
  12: 200,
};

class PathResult {
  final List<int> waypoints;
  final List<dynamic> fullSequence;
  final List<dynamic> chosenTarget;
  final List<int> entryKfs;
  final String robot;
  final List<int> sideTargets;
  final bool usingSideArm;

  const PathResult({
    required this.waypoints,
    required this.fullSequence,
    required this.chosenTarget,
    required this.robot,
    this.entryKfs = const [],
    this.sideTargets = const [],
    this.usingSideArm = false,
  });

  static PathResult empty(String robot) => PathResult(
    waypoints: [],
    fullSequence: [],
    chosenTarget: [],
    robot: robot,
    entryKfs: [],
    sideTargets: [],
    usingSideArm: false,
  );

  bool get isEmpty => fullSequence.isEmpty;

  List<int> get allKfs => [
    ...entryKfs,
    ...chosenTarget.cast<int>(),
    ...sideTargets,
  ];
}

class RobotResult {
  final PathResult r1;
  final PathResult r2;
  final List<int> priorityCells;
  final bool blockedKFS;
  final String team;
  final List<String> r2Directions;
  final bool isRetry;

  const RobotResult({
    required this.r1,
    required this.r2,
    required this.priorityCells,
    required this.blockedKFS,
    required this.team,
    required this.r2Directions,
    this.isRetry = false,
  });

  String sendSerialMes() {
    String r1Msg = '';
    if (r1.waypoints.isNotEmpty) {
      final targets = r1.waypoints.sublist(0, r1.waypoints.length - 1);
      final parts =
          targets.map((c) => priorityCells.contains(c) ? '*$c' : '$c').toList();
      parts.add('exit');
      r1Msg = parts.join(',');
    }
    return 'R1:$r1Msg|R2D:${r2Directions.join(',')}';
  }

  String r1Instruction() {
    if (r1.waypoints.isEmpty) return 'No R1 targets';
    final targets = r1.waypoints.sublist(0, r1.waypoints.length - 1);
    final exitId = r1.waypoints.last;
    final parts =
        targets
            .map((c) => priorityCells.contains(c) ? '⚡$c(FIRST)' : '$c')
            .toList();
    parts.add('exit($exitId)');
    return parts.join(' → ');
  }

  String r2Instruction() {
    if (r2Directions.isEmpty) return 'No R2 path';
    final allKfs = r2.allKfs;
    final pickLabel =
        allKfs.isEmpty ? 'none' : allKfs.map((c) => 'cell $c').join(', ');
    return 'KFS to pick: $pickLabel\n${r2Directions.join('  ')}';
  }
}

class _AState {
  final int block;
  final int collected;
  final int facing;
  final int picked;
  const _AState(this.block, this.collected, this.facing, this.picked);

  int get key =>
      (block & 0xF) |
      ((collected & 0x3) << 4) |
      ((facing & 0x3) << 6) |
      ((picked & 0xFF) << 8);
}

class _ANode {
  final int f, g;
  final _AState state;
  const _ANode(this.f, this.g, this.state);
}

class _Act {
  final int type;
  final int direction;
  final int targetCell;
  const _Act(this.type, this.direction, this.targetCell);
}

class _Plan {
  final int totalCost2;
  final List<_AState> states;
  final List<_Act?> actions;
  const _Plan(this.totalCost2, this.states, this.actions);
}

class CalPath {
  static const ringRed = [
    "corner-tl",
    "top-12",
    "top-11",
    "top-10",
    "corner-tr",
    "right-10",
    "right-7",
    "right-4",
    "right-1",
    "corner-br",
    "btm-1",
    "btm-2",
    "btm-3",
    "corner-bl",
    "left-3",
    "left-6",
    "left-9",
    "left-12",
  ];
  static const ringBlue = [
    "corner-tl",
    "top-10",
    "top-11",
    "top-12",
    "corner-tr",
    "right-12",
    "right-9",
    "right-6",
    "right-3",
    "corner-br",
    "btm-3",
    "btm-2",
    "btm-1",
    "corner-bl",
    "left-1",
    "left-4",
    "left-7",
    "left-10",
  ];
  static const Map<int, List<String>> r1Red = {
    12: ["top-12", "left-12"],
    11: ["top-11"],
    10: ["top-10", "right-10"],
    9: ["left-9"],
    7: ["right-7"],
    6: ["left-6"],
    4: ["right-4"],
    3: ["btm-3", "left-3"],
    2: ["btm-2"],
    1: ["right-1", "btm-1"],
  };
  static const Map<int, List<String>> r1Blue = {
    10: ["top-10", "left-10"],
    11: ["top-11"],
    12: ["top-12", "right-12"],
    9: ["right-9"],
    7: ["left-7"],
    6: ["right-6"],
    4: ["left-4"],
    3: ["right-3", "btm-3"],
    2: ["btm-2"],
    1: ["btm-1", "left-1"],
  };
  static const r1PerimeterCells = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12};

  static const Map<int, List<int>> _r2Red = {
    12: [0, 0],
    11: [0, 1],
    10: [0, 2],
    9: [1, 0],
    8: [1, 1],
    7: [1, 2],
    6: [2, 0],
    5: [2, 1],
    4: [2, 2],
    3: [3, 0],
    2: [3, 1],
    1: [3, 2],
  };
  static const Map<int, List<int>> _r2Blue = {
    10: [0, 0],
    11: [0, 1],
    12: [0, 2],
    7: [1, 0],
    8: [1, 1],
    9: [1, 2],
    4: [2, 0],
    5: [2, 1],
    6: [2, 2],
    1: [3, 0],
    2: [3, 1],
    3: [3, 2],
  };

  static int _rowOf(int i) => i ~/ _cols;
  static int _colOf(int i) => i % _cols;
  static int _idx(int r, int c) => r * _cols + c;

  static int _block1(int cell0, Map<int, List<int>> gp) {
    for (final e in gp.entries) {
      if (e.value[0] * _cols + e.value[1] == cell0) return e.key;
    }
    return cell0 + 1;
  }

  static int _step(int i, int dir) {
    int r = _rowOf(i), c = _colOf(i);
    if (dir == _dirDown) {
      r++;
    } else if (dir == _dirUp) {
      r--;
    } else if (dir == _dirRight) {
      c++;
    } else if (dir == _dirLeft) {
      c--;
    }
    if (r < 0 || r >= _rows || c < 0 || c >= _cols) return -1;
    return _idx(r, c);
  }

  static int _rotCost2(int cur, int tgt) {
    final d = (tgt - cur + 4) % 4;
    return d == 0
        ? 0
        : d == 2
        ? _cTurn1802
        : _cTurn902;
  }

  static int _moveCost2(int cf, int dir) =>
      _rotCost2(cf, dir) + _cMove2 + (dir == _dirDown ? _cBackPen2 : 0);

  static int _h2(int cell0, int collected, int targetCount) =>
      _rowOf(cell0) * 2 + (targetCount - collected) * 4;

  static String _hCmd(int a, int b) =>
      b > a
          ? 'C'
          : b < a
          ? 'D'
          : 'F';

  static RobotResult calculate(
    List<int> states,
    String team, {
    List<int>? selectedKfs,
    List<int>? alreadyPicked,
    int kfsCount = 3,
  }) {
    final isRetry = alreadyPicked != null && alreadyPicked.isNotEmpty;
    final picked = alreadyPicked ?? [];

    int leftTopick = kfsCount - picked.length;
    if (leftTopick < 0) leftTopick = 0;

    List<int> effectiveStates = List<int>.from(states);

    for (final p in picked) {
      if (p >= 1 && p <= 12) {
        effectiveStates[p - 1] = 0;
      }
    }
    if (selectedKfs != null && selectedKfs.isNotEmpty) {
      for (int i = 0; i < effectiveStates.length; i++) {
        int blockId = i + 1;
        if (effectiveStates[i] == 2) {
          if (!selectedKfs.contains(blockId) || picked.contains(blockId)) {
            effectiveStates[i] = 0;
          }
        }
      }
    }

    for (final p in picked) {
      if (p >= 1 && p <= 12) effectiveStates[p - 1] = 0;
    }

    debugPrint(
      '[RETRY=$isRetry] Need: $leftTopick more. Already pick: $picked',
    );

    final gp = team == 'red' ? _r2Red : _r2Blue;
    final r2result = _planR2(
      effectiveStates,
      team,
      isRetry: isRetry,
      kfsCount: leftTopick,
    );

    final r2Cells = r2result.fullSequence.toSet();
    final priorityCells = <int>[];
    for (int i = 0; i < states.length; i++) {
      if (states[i] == 1 && r2Cells.contains(i + 1)) {
        priorityCells.add(i + 1);
      }
    }

    final dirs = _buildDirs(r2result, gp, isRetry: isRetry);

    debugPrint('[R2 PHYS]  ${r2result.fullSequence.join(" → ")}');
    debugPrint('[R2 ENTRY] ${r2result.entryKfs}');
    debugPrint('[R2 CHOSE] ${r2result.chosenTarget}');
    debugPrint('[R2 SIDES] ${r2result.sideTargets}');
    debugPrint('[R2 DIRS]  ${dirs.join("  ")}');

    final result = RobotResult(
      r1: _r1(states, team, priorityCells),
      r2: r2result,
      priorityCells: priorityCells,
      blockedKFS: priorityCells.isNotEmpty,
      team: team,
      r2Directions: dirs,
      isRetry: isRetry,
    );
    debugPrint('[R1]  ${result.r1Instruction()}');
    debugPrint('[MSG] ${result.sendSerialMes()}');
    return result;
  }

  static PathResult _planR2(
    List<int> states,
    String team, {
    bool isRetry = false,
    int kfsCount = 3,
  }) {
    final gp = team == 'red' ? _r2Red : _r2Blue;

    final boxes = List<int>.filled(_numBlocks, 0);
    for (int i = 0; i < states.length; i++) {
      final p = gp[i + 1];
      if (p != null) {
        boxes[p[0] * _cols + p[1]] = states[i];
      }
    }

    final r2c0 = <int>[];
    for (int i = 0; i < _numBlocks; i++) {
      if (boxes[i] == 2) {
        r2c0.add(i);
      }
    }

    if (r2c0.isEmpty && kfsCount > 0) return PathResult.empty('R2');

    final targetCount = kfsCount.clamp(0, r2c0.length);
    final entryR2c0 = r2c0.where((c) => _rowOf(c) == _rows - 1).toList();

    _Plan? best;
    int? bestEntry0;
    List<int> bestPrePicks0 = [];

    if (isRetry) {
      for (int col = 0; col < _cols; col++) {
        final eb = _idx(_rows - 1, col);
        if (boxes[eb] == 3) continue;

        int sp = 0, sc = 0;
        final bit = r2c0.indexOf(eb);
        if (bit >= 0 && sc < targetCount) {
          sp |= (1 << bit);
          sc++;
        }

        final res = _aStar(boxes, r2c0, eb, _dirUp, sp, sc, targetCount);
        if (res != null && (best == null || _isBetter(res, best, r2c0))) {
          best = res;
          bestEntry0 = eb;
          bestPrePicks0 = [];
        }
      }
    } else {
      final subsets = 1 << entryR2c0.length;
      for (int mask = 0; mask < subsets; mask++) {
        int initPicked = 0, initCollected = 0;
        final prePicks = <int>[];
        for (int i = 0; i < entryR2c0.length; i++) {
          if (mask & (1 << i) != 0) {
            final bit = r2c0.indexOf(entryR2c0[i]);
            if (bit >= 0) {
              initPicked |= (1 << bit);
              initCollected++;
              prePicks.add(entryR2c0[i]);
            }
          }
        }
        if (initCollected >= targetCount) continue;

        for (int col = 0; col < _cols; col++) {
          final eb = _idx(_rows - 1, col);
          if (boxes[eb] == 3) continue;

          int sp = initPicked, sc = initCollected;
          final bit = r2c0.indexOf(eb);
          if (bit >= 0 && (sp & (1 << bit) == 0) && sc < targetCount) {
            sp |= (1 << bit);
            sc++;
          }

          final res = _aStar(boxes, r2c0, eb, _dirUp, sp, sc, targetCount);
          if (res != null && (best == null || _isBetter(res, best, r2c0))) {
            best = res;
            bestEntry0 = eb;
            bestPrePicks0 = List.from(prePicks);
          }
        }
      }
    }

    if (best == null || bestEntry0 == null) return PathResult.empty('R2');

    final physPath1 = best.states.map((s) => _block1(s.block, gp)).toList();

    final entryBlock1 = _block1(bestEntry0, gp);
    final entryIsKfs =
        best.states.isNotEmpty &&
        best.states[0].collected > 0 &&
        !bestPrePicks0.map((c) => _block1(c, gp)).contains(entryBlock1) &&
        boxes[bestEntry0] == 2;
    final entryKfs1 = entryIsKfs ? [entryBlock1] : <int>[];

    // Collect cells where the robot picked up a KFS — either by walking onto it
    // (_actMove auto-collect) or by the new combined move+pickup (_actMovePickup).
    final chosen1 = <int>[];
    for (int i = 1; i < best.states.length; i++) {
      final s = best.states[i];
      final prev = best.states[i - 1];
      final act = best.actions[i];
      if (act != null &&
          (act.type == _actMove || act.type == _actMovePickup) &&
          s.collected > prev.collected) {
        chosen1.add(_block1(s.block, gp));
      }
    }

    final armPicks1 = <int>[];
    for (int i = 1; i < best.actions.length; i++) {
      final a = best.actions[i];
      if (a != null && a.type == _actPickup) {
        armPicks1.add(_block1(a.targetCell, gp));
      }
    }

    final preEntry1 = bestPrePicks0.map((c) => _block1(c, gp)).toList();
    final exitBlock1 = _block1(best.states.last.block, gp);

    debugPrint('[A* COST2=${best.totalCost2} physSteps=${physPath1.length}]');
    debugPrint('[PRE-ENT]  $preEntry1  entryKfs=$entryKfs1');

    return PathResult(
      waypoints: [...entryKfs1, ...chosen1, exitBlock1],
      fullSequence: physPath1,
      chosenTarget: chosen1,
      entryKfs: entryKfs1,
      robot: 'R2',
      sideTargets: [...preEntry1, ...armPicks1],
      usingSideArm: armPicks1.isNotEmpty || preEntry1.isNotEmpty,
    );
  }

  static bool _isBetter(_Plan res, _Plan best, List<int> r2c0) {
    if (res.totalCost2 < best.totalCost2) return true;
    if (res.totalCost2 > best.totalCost2) return false;
    final resMoves =
        res.actions
            .where((a) => a?.type == _actMove || a?.type == _actMovePickup)
            .length;
    final bestMoves =
        best.actions
            .where((a) => a?.type == _actMove || a?.type == _actMovePickup)
            .length;
    return resMoves < bestMoves;
  }

  static _Plan? _aStar(
    List<int> boxes,
    List<int> r2c0,
    int entry0,
    int startFacing,
    int initPicked,
    int initCollected,
    int targetCount,
  ) {
    final gScore = <int, int>{};
    final parentK = <int, int>{};
    final parentA = <int, _Act?>{};

    final start = _AState(entry0, initCollected, startFacing, initPicked);
    final sk = start.key;
    gScore[sk] = 0;
    parentA[sk] = null;

    final heap = <_ANode>[
      _ANode(_h2(entry0, initCollected, targetCount), 0, start),
    ];

    void push(_ANode n) {
      heap.add(n);
      int i = heap.length - 1;
      while (i > 0) {
        final p = (i - 1) ~/ 2;
        if (heap[p].f <= heap[i].f) break;
        final t = heap[p];
        heap[p] = heap[i];
        heap[i] = t;
        i = p;
      }
    }

    _ANode pop() {
      final top = heap[0];
      final last = heap.removeLast();
      if (heap.isNotEmpty) {
        heap[0] = last;
        int i = 0;
        while (true) {
          int s = i, l = 2 * i + 1, r = 2 * i + 2;
          if (l < heap.length && heap[l].f < heap[s].f) s = l;
          if (r < heap.length && heap[r].f < heap[s].f) s = r;
          if (s == i) break;
          final t = heap[s];
          heap[s] = heap[i];
          heap[i] = t;
          i = s;
        }
      }
      return top;
    }

    while (heap.isNotEmpty) {
      final cur = pop();
      final gCur = gScore[cur.state.key];
      if (gCur == null || cur.g > gCur) continue;
      final cs = cur.state;

      if (cs.collected >= targetCount && _rowOf(cs.block) == 0) {
        final keys = <int>[];
        int c = cs.key;
        while (parentK.containsKey(c)) {
          keys.add(c);
          c = parentK[c]!;
        }
        keys.add(sk);
        final ord = keys.reversed.toList();
        final sts =
            ord
                .map(
                  (k) => _AState(
                    k & 0xF,
                    (k >> 4) & 0x3,
                    (k >> 6) & 0x3,
                    (k >> 8) & 0xFF,
                  ),
                )
                .toList();
        final acs = <_Act?>[null];
        for (int i = 1; i < ord.length; i++) {
          acs.add(parentA[ord[i]]);
        }
        return _Plan(cur.g, sts, acs);
      }

      // --- Side-arm pickup: pick up an adjacent KFS without moving ---
      if (cs.collected < targetCount) {
        for (int d = 0; d < 4; d++) {
          final adj = _step(cs.block, d);
          if (adj < 0) continue;
          final bit = r2c0.indexOf(adj);
          if (bit < 0 || cs.picked & (1 << bit) != 0) continue;
          const int pc = _cArm2;
          final ns = _AState(
            cs.block,
            cs.collected + 1,
            cs.facing,
            cs.picked | (1 << bit),
          );
          final nk = ns.key;
          final ng = cur.g + pc;
          if (!gScore.containsKey(nk) || ng < gScore[nk]!) {
            gScore[nk] = ng;
            parentK[nk] = cs.key;
            parentA[nk] = _Act(_actPickup, d, adj);
            push(
              _ANode(ng + _h2(cs.block, cs.collected + 1, targetCount), ng, ns),
            );
          }
        }
      }

      // --- Movement (with optional simultaneous KFS pickup) ---
      for (int d = 0; d < 4; d++) {
        final nxt = _step(cs.block, d);
        if (nxt < 0) continue;
        if (boxes[nxt] == 3) continue;

        final bit = r2c0.indexOf(nxt);

        // Still need KFS but this unpicked KFS cell is directly ahead and
        // we've already hit our target — skip it (don't walk past unneeded KFS).
        if (bit >= 0 &&
            (cs.picked & (1 << bit) == 0) &&
            cs.collected >= targetCount) {
          continue;
        }

        int mc = _moveCost2(cs.facing, d);
        if (boxes[nxt] == 1) mc += _cR1Wait2;

        // If the destination cell is an unpicked KFS and we still need one,
        // pick it up simultaneously (no extra arm cost).
        final bool isKfsAhead =
            bit >= 0 &&
            (cs.picked & (1 << bit) == 0) &&
            cs.collected < targetCount;

        final int nc = isKfsAhead ? cs.collected + 1 : cs.collected;
        final int np = isKfsAhead ? cs.picked | (1 << bit) : cs.picked;
        final int actType = isKfsAhead ? _actMovePickup : _actMove;

        final ns = _AState(nxt, nc, d, np);
        final nk = ns.key;
        final ng = cur.g + mc;
        if (!gScore.containsKey(nk) || ng < gScore[nk]!) {
          gScore[nk] = ng;
          parentK[nk] = cs.key;
          parentA[nk] = _Act(actType, d, nxt);
          push(_ANode(ng + _h2(nxt, nc, targetCount), ng, ns));
        }
      }
    }
    return null;
  }

  static List<String> _buildDirs(
    PathResult r2,
    Map<int, List<int>> gp, {
    bool isRetry = false,
  }) {
    final dirs = <String>[];
    if (r2.isEmpty) return dirs;

    final physPath = r2.fullSequence.cast<int>();
    final chosenTargets = r2.chosenTarget.cast<int>();
    final sideTargets = r2.sideTargets;

    final entryCell = physPath.first;
    final entryH = _blockHeightMm[entryCell] ?? 400;
    final entryRow = gp[entryCell]![0];
    final entryCol = gp[entryCell]![1];

    final preEntryPicks =
        isRetry
            ? <int>[]
            : sideTargets
                .where(
                  (s) =>
                      gp[s] != null &&
                      gp[s]![0] == entryRow &&
                      !physPath.contains(s),
                )
                .toList();
    final armPicks =
        sideTargets.where((s) => !preEntryPicks.contains(s)).toList();
    final emittedArms = <int>{};

    for (final pe in preEntryPicks) {
      final peH = _blockHeightMm[pe] ?? 400;
      dirs.add('PRE($pe)');
      dirs.add('K$peH');
    }

    dirs.add(isRetry ? 'RETRY_ENT($entryCell)' : 'ENT($entryCell)');
    dirs.add('C$entryH');

    _emitArmPicks(dirs, armPicks, emittedArms, entryRow, entryCol, gp);

    int prevH = entryH;
    int prevCol = entryCol;
    bool facingUp = true;

    for (int i = 1; i < physPath.length; i++) {
      final cur = physPath[i];
      final curH = _blockHeightMm[cur] ?? 400;
      final curRow = gp[cur]![0];
      final curCol = gp[cur]![1];

      if (armPicks.isEmpty) {
        final prevRow2 = gp[physPath[i - 1]]![0];
        final fwd = curRow < prevRow2;
        final side = curCol != prevCol;
        if (side && facingUp) {
          dirs.add(curCol < prevCol ? 'L' : 'R');
          facingUp = false;
        } else if (!side && !facingUp && fwd) {
          dirs.add(prevCol < entryCol ? 'R' : 'L');
          facingUp = true;
        }
      }

      if (chosenTargets.contains(cur)) {
        dirs.add('K${_hCmd(prevH, curH)}$curH');
      } else {
        dirs.add('${_hCmd(prevH, curH)}$curH');
      }

      // dirs.add('${_hCmd(prevH, curH)}$curH');
      prevH = curH;
      prevCol = curCol;

      _emitArmPicks(dirs, armPicks, emittedArms, curRow, curCol, gp);
    }

    dirs.add('exit(${physPath.last})');
    return dirs;
  }

  static void _emitArmPicks(
    List<String> dirs,
    List<int> armPicks,
    Set<int> emitted,
    int row,
    int pivotCol,
    Map<int, List<int>> gp,
  ) {
    final here =
        armPicks
            .where(
              (s) => gp[s] != null && gp[s]![0] == row && !emitted.contains(s),
            )
            .toList()
          ..sort((a, b) {
            final aR = gp[a]![1] > pivotCol;
            final bR = gp[b]![1] > pivotCol;
            if (aR == bR) return 0;
            return aR ? 1 : -1;
          });
    for (final s in here) {
      final sH = _blockHeightMm[s] ?? 400;
      final isRight = gp[s]![1] > pivotCol;
      dirs.add('${isRight ? "KR" : "KL"}$sH');
      emitted.add(s);
    }
  }

  static PathResult _r1(
    List<int> states,
    String team, [
    List<int> priorityCells = const [],
  ]) {
    final targets = [
      for (int i = 0; i < states.length; i++)
        if (states[i] == 1) i + 1,
    ];
    if (targets.isEmpty) return PathResult.empty('R1');

    final ring = team == 'red' ? ringRed : ringBlue;
    final st = team == 'red' ? r1Red : r1Blue;

    final exitBlocks =
        team == 'red'
            ? ["corner-tr", "top-11", "corner-tl"]
            : ["corner-tl", "top-11", "corner-tr"];
    final exitIds = [10, 11, 12];
    const starts = ["btm-1", "btm-2", "btm-3", "corner-bl", "corner-br"];

    final trn = <int, List<String>>{};
    for (final t in targets) {
      final nodes = (st[t] ?? []).where((n) => ring.contains(n)).toList();
      if (nodes.isEmpty) return PathResult.empty('R1');
      trn[t] = nodes;
    }

    final rl = ring.length;
    int gMin = 9999;
    List<String> bSeq = [];
    List<int> bOrd = [];
    int? bExit;

    for (final start in starts) {
      if (!ring.contains(start)) continue;
      final si = ring.indexOf(start);
      for (final dir in [1, -1]) {
        final vo = <int>[], vn = <String>[], vc = <int>{};
        for (final t in targets) {
          if (trn[t]!.contains(start)) {
            vc.add(t);
            vo.add(t);
            vn.add(start);
            break;
          }
        }
        for (int step = 1; step < rl; step++) {
          if (vc.length == targets.length) break;
          final idx = ((si + dir * step) % rl + rl) % rl;
          final node = ring[idx];
          for (final t in targets) {
            if (!vc.contains(t) && trn[t]!.contains(node)) {
              vc.add(t);
              vo.add(t);
              vn.add(node);
              break;
            }
          }
        }
        if (vc.length != targets.length) continue;

        if (priorityCells.isNotEmpty) {
          bool ok = true, sn = false;
          for (final cell in vo) {
            if (!priorityCells.contains(cell)) {
              sn = true;
            } else if (sn) {
              ok = false;
              break;
            }
          }
          if (!ok) continue;
        }

        final seq = <String>[start];
        int ci = si, cost = 0;
        for (final node in vn) {
          final ni = ring.indexOf(node);
          while (ci != ni) {
            ci = ((ci + dir) % rl + rl) % rl;
            seq.add(ring[ci]);
            cost++;
          }
        }

        int bec = 9999;
        String? ben;
        int? bei;
        for (int ex = 0; ex < exitBlocks.length; ex++) {
          final en = exitBlocks[ex];
          if (!ring.contains(en)) continue;
          final ei = ring.indexOf(en);
          final cw = (ei - ci + rl) % rl;
          final cc = (ci - ei + rl) % rl;
          final d = cw <= cc ? cw : cc;
          if (d < bec) {
            bec = d;
            ben = en;
            bei = exitIds[ex];
          }
        }
        if (ben == null) continue;

        final fs = List<String>.from(seq);
        if (ben != seq.last) {
          final ei = ring.indexOf(ben);
          final cw = (ei - ci + rl) % rl;
          final cc = (ci - ei + rl) % rl;
          final ed = cw <= cc ? 1 : -1;
          int cc2 = ci;
          while (cc2 != ei) {
            cc2 = ((cc2 + ed) % rl + rl) % rl;
            fs.add(ring[cc2]);
          }
        }

        final total = cost + bec;
        if (total < gMin) {
          gMin = total;
          bSeq = fs;
          bOrd = List.from(vo);
          bExit = bei;
        }
      }
    }

    if (bSeq.isEmpty || bExit == null) return PathResult.empty('R1');
    return PathResult(
      waypoints: [...bOrd, bExit],
      fullSequence: bSeq,
      chosenTarget: bOrd,
      robot: 'R1',
    );
  }

  static List<String>? shortestPath(
    String s,
    String e,
    List<String> ring,
    Map<String, int> n2c,
    List<int> states,
  ) {
    if (s == e) return [];
    final si = ring.indexOf(s), ei = ring.indexOf(e);
    if (si < 0 || ei < 0) return [];
    final cw =
        ei >= si
            ? ring.sublist(si + 1, ei + 1)
            : [...ring.sublist(si + 1), ...ring.sublist(0, ei + 1)];
    final rev = ring.reversed.toList();
    final ri = rev.indexOf(s), re = rev.indexOf(e);
    final ccw =
        re >= ri
            ? rev.sublist(ri + 1, re + 1)
            : [...rev.sublist(ri + 1), ...rev.sublist(0, re + 1)];
    bool blocked(List<String> p) => p.any((b) {
      final c = n2c[b];
      return c != null && states[c - 1] == 3;
    });
    final cwOk = !blocked(cw), acwOk = !blocked(ccw);
    if (!cwOk && !acwOk) return null;
    if (!cwOk) return ccw;
    if (!acwOk) return cw;
    return cw.length <= ccw.length ? cw : ccw;
  }

  static R2CanPath buildCanPath(RobotResult rr, List<int> states) {
    final gp = rr.team == 'red' ? _r2Red : _r2Blue;
    final r2 = rr.r2;
    final steps = <R2CanStep>[];

    if (r2.isEmpty) {
      return R2CanPath(
        totalCost: 0,
        entryBlock: 0,
        exitBlock: 0,
        gridState: List<int>.from(states),
        preEntryCount: 0,
        steps: [],
      );
    }

    final physPath = r2.fullSequence.cast<int>();
    final chosenTargets = r2.chosenTarget.cast<int>();
    final sideTargets = r2.sideTargets;

    final entryBlock1 = physPath.first;
    final exitBlock1 = physPath.last;
    final entryRow = gp[entryBlock1]![0];
    final entryCol = gp[entryBlock1]![1];

    final preEntryPicks =
        rr.isRetry
            ? <int>[]
            : sideTargets
                .where(
                  (s) =>
                      gp[s] != null &&
                      gp[s]![0] == entryRow &&
                      !physPath.contains(s),
                )
                .toList();

    int collectedSoFar = 0;

    for (final pe in preEntryPicks) {
      final peCol = gp[pe]![1];
      final dir = peCol > entryCol ? R2CanDir.right : R2CanDir.left;
      collectedSoFar++;
      steps.add(
        R2CanStep(
          actionType: R2CanAct.preEntry,
          currentBlock: 0,
          targetBlock: pe,
          direction: dir,
          collectedAfter: collectedSoFar,
          requiresR1Clear: false,
          autoPickup: false,
          heightDeltaEnc: R2CanStep.calcPreEntryHeightEnc(pe),
          heightDeltaMm: _blockHeightMm[pe] ?? 0,
        ),
      );
    }

    final preEntryCount = steps.length;
    final entryBoxIsR2 = states[entryBlock1 - 1] == 2;
    final entryWasPre = preEntryPicks.contains(entryBlock1);
    final entryAutoPickup = entryBoxIsR2 && !entryWasPre;
    if (entryAutoPickup) collectedSoFar++;

    steps.add(
      R2CanStep(
        actionType: R2CanAct.move,
        currentBlock: 0,
        targetBlock: entryBlock1,
        direction: R2CanDir.down,
        collectedAfter: collectedSoFar,
        requiresR1Clear: states[entryBlock1 - 1] == 1,
        autoPickup: entryAutoPickup,
        heightDeltaEnc: R2CanStep.calcPreEntryHeightEnc(entryBlock1),
        heightDeltaMm: _blockHeightMm[entryBlock1] ?? 0,
      ),
    );

    int curBlock1 = entryBlock1;
    final armPicks =
        sideTargets.where((s) => !preEntryPicks.contains(s)).toList();
    final emittedCan = <int>{};

    for (final s in armPicks.where((s) => gp[s]![0] == entryRow)) {
      final sCol = gp[s]![1];
      final dir = sCol > entryCol ? R2CanDir.right : R2CanDir.left;
      collectedSoFar++;
      emittedCan.add(s);
      steps.add(
        R2CanStep(
          actionType: R2CanAct.pickup,
          currentBlock: curBlock1,
          targetBlock: s,
          direction: dir,
          collectedAfter: collectedSoFar,
          requiresR1Clear: false,
          autoPickup: false,
          heightDeltaEnc: R2CanStep.calcHeightDeltaEnc(curBlock1, s),
          heightDeltaMm: R2CanStep.calcHeightDeltaMm(curBlock1, s),
        ),
      );
    }

    for (int i = 1; i < physPath.length; i++) {
      final next1 = physPath[i];
      final nextRow = gp[next1]![0];
      final nextCol = gp[next1]![1];
      final prevRow = gp[physPath[i - 1]]![0];
      final prevCol = gp[physPath[i - 1]]![1];

      final int moveDir;
      if (nextRow < prevRow) 
      {
        moveDir = R2CanDir.up;
      } 
      else if (nextRow > prevRow) 
      {
        moveDir = R2CanDir.down;
      } 
      else if (nextCol > prevCol) 
      {
        moveDir = R2CanDir.right;
      } 
      else 
      {
        moveDir = R2CanDir.left;
      }

      // autoPickup is true for both _actMovePickup (new combined action) and
      // the classic walk-onto-KFS behaviour — both are captured in chosenTargets.
      final autoPickup = chosenTargets.contains(next1);
      if (autoPickup) collectedSoFar++;

      steps.add(
        R2CanStep(
          actionType: R2CanAct.move,
          currentBlock: curBlock1,
          targetBlock: next1,
          direction: moveDir,
          collectedAfter: collectedSoFar,
          requiresR1Clear: states[next1 - 1] == 1,
          autoPickup: autoPickup,
          heightDeltaEnc: R2CanStep.calcHeightDeltaEnc(curBlock1, next1),
          heightDeltaMm: R2CanStep.calcHeightDeltaMm(curBlock1, next1),
        ),
      );
      curBlock1 = next1;

      final armHere =
          armPicks
              .where((s) => gp[s]![0] == nextRow && !emittedCan.contains(s))
              .toList()
            ..sort((a, b) 
            {
              final aR = gp[a]![1] > nextCol;
              final bR = gp[b]![1] > nextCol;
              if (aR == bR) return 0;
              return aR ? 1 : -1;
            });
      for (final s in armHere) 
      {
        final sCol = gp[s]![1];
        final dir = sCol > nextCol ? R2CanDir.right : R2CanDir.left;
        collectedSoFar++;
        emittedCan.add(s);
        steps.add(
          R2CanStep(
            actionType: R2CanAct.pickup,
            currentBlock: curBlock1,
            targetBlock: s,
            direction: dir,
            collectedAfter: collectedSoFar,
            requiresR1Clear: false,
            autoPickup: false,
            heightDeltaEnc: R2CanStep.calcHeightDeltaEnc(curBlock1, s),
            heightDeltaMm: R2CanStep.calcHeightDeltaMm(curBlock1, s),
          ),
        );
      }
    }

    return R2CanPath(
      totalCost: steps.length,
      entryBlock: entryBlock1,
      exitBlock: exitBlock1,
      gridState: List<int>.from(states),
      preEntryCount: preEntryCount,
      steps: steps,
    );
  }
}

class R2CanAct 
{
  static const int move = 0;
  static const int pickup = 1;
  static const int preEntry = 2;
}

class R2CanDir 
{
  static const int down = 0;
  static const int right = 1;
  static const int up = 2;
  static const int left = 3;
}

class R2CanHDelta 
{
  static const int neg400 = 0;
  static const int neg200 = 1;
  static const int zero = 2;
  static const int pos200 = 3;
  static const int pos400 = 4;
  static const int pos600 = 5;
  static const int invalid = 7;
}

class R2CanStep 
{
  final int actionType;
  final int currentBlock;
  final int targetBlock;
  final int direction;
  final int collectedAfter;
  final bool requiresR1Clear;
  final bool autoPickup;
  final int heightDeltaEnc;
  final int heightDeltaMm;

  const R2CanStep(
    {
    required this.actionType,
    required this.currentBlock,
    required this.targetBlock,
    required this.direction,
    required this.collectedAfter,
    required this.requiresR1Clear,
    required this.autoPickup,
    required this.heightDeltaEnc,
    required this.heightDeltaMm,
  });

  static const Map<int, int> _h = 
  {
    1: 400,
    2: 200,
    3: 400,
    4: 200,
    5: 400,
    6: 600,
    7: 400,
    8: 600,
    9: 400,
    10: 200,
    11: 400,
    12: 200,
  };

  static int calcHeightDeltaEnc(int cur1, int tgt1) 
  {
    final hc = _h[cur1], ht = _h[tgt1];
    if (hc == null || ht == null) return R2CanHDelta.invalid;
    switch (ht - hc) {
      case -400:
        return R2CanHDelta.neg400;
      case -200:
        return R2CanHDelta.neg200;
      case 0:
        return R2CanHDelta.zero;
      case 200:
        return R2CanHDelta.pos200;
      case 400:
        return R2CanHDelta.pos400;
      default:
        return R2CanHDelta.invalid;
    }
  }

  static int calcHeightDeltaMm(int cur1, int tgt1) =>
      (_h[tgt1] ?? 0) - (_h[cur1] ?? 0);

  static int calcPreEntryHeightEnc(int tgt1) 
  {
    switch (_h[tgt1] ?? 0) {
      case 200:
        return R2CanHDelta.pos200;
      case 400:
        return R2CanHDelta.pos400;
      case 600:
        return R2CanHDelta.pos600;
      default:
        return R2CanHDelta.zero;
    }
  }

  Map<String, dynamic> toJson() => 
  {
    'action_type': actionType,
    'current_block': currentBlock,
    'target_block': targetBlock,
    'direction': direction,
    'collected_after': collectedAfter,
    'requires_r1_clear': requiresR1Clear,
    'auto_pickup': autoPickup,
    'height_delta_enc': heightDeltaEnc,
    'height_delta_mm': heightDeltaMm,
  };

  @override
  String toString() {
    const a = ['MOVE', 'PICKUP', 'PRE_ENTRY', 'MOVE+PICKUP'];
    const d = ['DOWN', 'RIGHT', 'UP', 'LEFT'];
    return '[${a[actionType.clamp(0, a.length - 1)]}] $currentBlock→$targetBlock'
        ' dir=${d[direction]} col=$collectedAfter'
        ' r1=${requiresR1Clear ? 1 : 0} auto=${autoPickup ? 1 : 0}'
        ' Δ${heightDeltaMm}mm(enc=$heightDeltaEnc)';
  }
}

class R2CanPath 
{
  final int totalCost;
  final int entryBlock;
  final int exitBlock;
  final List<int> gridState;
  final int preEntryCount;
  final List<R2CanStep> steps;
  static const int maxSteps = 16;

  const R2CanPath({
    required this.totalCost,
    required this.entryBlock,
    required this.exitBlock,
    required this.gridState,
    required this.preEntryCount,
    required this.steps,
  });

  int get stepCount => steps.length;

  Map<String, dynamic> toJson() => {
    'total_cost': totalCost,
    'entry_block': entryBlock,
    'exit_block': exitBlock,
    'boxes_to_collect': 3,
    'grid_state': List<int>.from(gridState),
    'pre_entry_count': preEntryCount,
    'step_count': stepCount,
    'steps': steps.map((s) => s.toJson()).toList(),
  };

  @override
  String toString() {
    String out =
        'R2CanPath entry=$entryBlock exit=$exitBlock '
        'cost=$totalCost preEntry=$preEntryCount steps=$stepCount\n';
    for (int i = 0; i < steps.length; i++) {
      out += '  [$i] ${steps[i]}\n';
    }
    return out;
  }
}
