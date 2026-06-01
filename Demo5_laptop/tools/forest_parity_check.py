#!/usr/bin/env python3
"""
Forest-planner parity check: Python port (r2_brain/forest_planner.py) vs the
original Dart planner (utmrbc_app cal_path.dart).

Pipeline:
  1. Generate a deterministic corpus of grid inputs -> $PARITY_DIR/inputs.json
  2. Run the Dart dumper (flutter test) to produce $PARITY_DIR/dart_out.json
  3. Run the Python port over the same inputs and diff the R2CanPath dicts.

Usage:
  python3 tools/forest_parity_check.py                 # full run (needs flutter)
  python3 tools/forest_parity_check.py --python-only   # skip dart, just sanity-run port
  python3 tools/forest_parity_check.py --no-dart       # reuse existing dart_out.json

Exit code 0 = all cases match, 1 = mismatch / error.
"""

import argparse
import json
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)                                   # Demo5_laptop
PLANNER_DIR = os.path.join(REPO, 'src', 'r2_brain', 'r2_brain')
APP_DIR = os.path.normpath(os.path.join(REPO, '..', 'utmrbc_app'))
PARITY_DIR = os.environ.get('PARITY_DIR', '/tmp/forest_parity')

sys.path.insert(0, PLANNER_DIR)
import forest_planner  # noqa: E402


def _gen_corpus(seed=1234):
    """Deterministic mix of competition-valid and fully-random grids."""
    rng = random.Random(seed)
    bases = []

    # 30 "valid" grids: 2 R1, 4 R2, 1 FAKE at random positions.
    for _ in range(30):
        g = [0] * 12
        cells = rng.sample(range(12), 7)
        for c in cells[0:2]:
            g[c] = 1   # R1
        for c in cells[2:6]:
            g[c] = 2   # R2
        g[cells[6]] = 3  # fake
        bases.append(g)

    # 20 fully-random grids (weighted toward R2 so the planner has work to do).
    for _ in range(20):
        g = [rng.choices([0, 1, 2, 3], weights=[4, 2, 5, 1])[0] for _ in range(12)]
        bases.append(g)

    # A few degenerate grids (no R2 at all -> empty path on both sides).
    bases.append([0] * 12)
    bases.append([1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3])

    corpus = []
    for g in bases:
        for team in ('red', 'blue'):
            for kfs in (1, 2, 3):
                corpus.append({'states': g, 'team': team,
                               'kfs_count': kfs, 'already_picked': []})
        # one retry variant per grid: pretend the first R2 block was collected.
        r2_blocks = [i + 1 for i, v in enumerate(g) if v == 2]
        if r2_blocks:
            corpus.append({'states': g, 'team': 'red', 'kfs_count': 3,
                           'already_picked': [r2_blocks[0]]})
    return corpus


def _run_dart():
    os.makedirs(PARITY_DIR, exist_ok=True)
    env = dict(os.environ, PARITY_DIR=PARITY_DIR)
    print(f'[parity] running flutter test in {APP_DIR} ...')
    r = subprocess.run(
        ['flutter', 'test', 'test/forest_parity_dump_test.dart'],
        cwd=APP_DIR, env=env, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout)
        print(r.stderr, file=sys.stderr)
        raise SystemExit('[parity] flutter test failed')


def _diff(a, b, path=''):
    """Return first human-readable difference between two JSON values, or None."""
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a:
                return f'{path}.{k}: missing in python'
            if k not in b:
                return f'{path}.{k}: missing in dart'
            d = _diff(a[k], b[k], f'{path}.{k}')
            if d:
                return d
        return None
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return f'{path}: len python={len(a)} dart={len(b)}'
        for i, (x, y) in enumerate(zip(a, b)):
            d = _diff(x, y, f'{path}[{i}]')
            if d:
                return d
        return None
    if a != b:
        return f'{path}: python={a!r} dart={b!r}'
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--python-only', action='store_true',
                    help='only run the port (no dart, no diff)')
    ap.add_argument('--no-dart', action='store_true',
                    help='reuse existing dart_out.json')
    args = ap.parse_args()

    os.makedirs(PARITY_DIR, exist_ok=True)
    corpus = _gen_corpus()
    with open(os.path.join(PARITY_DIR, 'inputs.json'), 'w') as f:
        json.dump(corpus, f)
    print(f'[parity] corpus: {len(corpus)} cases -> {PARITY_DIR}/inputs.json')

    # Always run the python port (also catches crashes).
    py_out = []
    for c in corpus:
        py_out.append(forest_planner.plan_r2_path(
            c['states'], c['team'], c['kfs_count'], c['already_picked']))
    with open(os.path.join(PARITY_DIR, 'python_out.json'), 'w') as f:
        json.dump(py_out, f)
    print(f'[parity] python port ran {len(py_out)} cases with no exceptions')

    if args.python_only:
        return 0

    if not args.no_dart:
        _run_dart()

    dart_path = os.path.join(PARITY_DIR, 'dart_out.json')
    if not os.path.exists(dart_path):
        print('[parity] no dart_out.json — run without --python-only first',
              file=sys.stderr)
        return 1
    with open(dart_path) as f:
        dart_out = json.load(f)

    if len(dart_out) != len(py_out):
        print(f'[parity] FAIL: count python={len(py_out)} dart={len(dart_out)}')
        return 1

    mismatches = 0
    for i, (c, py, dt) in enumerate(zip(corpus, py_out, dart_out)):
        d = _diff(py, dt)
        if d:
            mismatches += 1
            if mismatches <= 10:
                print(f'[parity] MISMATCH case {i} '
                      f'(team={c["team"]} kfs={c["kfs_count"]} '
                      f'retry={bool(c["already_picked"])})')
                print(f'         states={c["states"]}')
                print(f'         {d}')

    if mismatches:
        print(f'[parity] FAIL: {mismatches}/{len(corpus)} cases differ')
        return 1
    print(f'[parity] PASS: all {len(corpus)} cases match cal_path.dart')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
