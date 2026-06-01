"""
Rich-based live dashboard for r2_brain — operator-friendly layout.

Design priorities:
  - Big, scannable rack with score-per-row visible.
  - Two banners across the middle: what R2 is doing, what R1 should do.
  - Side panels are restrained (small fonts, dim labels) so the eye lands
    on the rack and the banners first.
  - Consistent padding and rule separators give the layout room to breathe.

Run as a context manager. Caller updates by calling .render(snapshot).
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Optional

from rich.align import Align
from rich.box import HEAVY, ROUNDED, SIMPLE_HEAVY
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.padding import Padding
from rich.panel import Panel
from rich.progress_bar import ProgressBar
from rich.rule import Rule
from rich.table import Table
from rich.text import Text

from .state import BrainSnapshot, LINES, SLOT_POINTS


# ─────────────────────────────────────────────────────────────────────────────
#  Visual constants
# ─────────────────────────────────────────────────────────────────────────────

# Style by ownership only — the *robot* (R1 vs R2) is derived from the row
# (slots 1-3 = R1's bottom row; slots 4-9 = R2's middle/top rows).
_FAKE_STYLE  = 'bold yellow'
_OURS_STYLE  = 'bold bright_green'
_OPP_STYLE   = 'bold bright_red'
_EMPTY_STYLE = 'grey39'


def _slot_label_for(slot: int, owner: str) -> tuple:
    """Return (display label, fg style) for a slot+owner combo."""
    if owner == 'EMPTY' or owner is None:
        return ('·', _EMPTY_STYLE)
    if owner in ('Fake_KFS', 'FAKE_KFS'):
        return ('FAKE', _FAKE_STYLE)
    placer = 'R1' if slot in (1, 2, 3) else 'R2'
    if owner in ('TEAM_KFS', 'R2_KFS', 'R1_KFS'):
        return (placer, _OURS_STYLE)
    if owner == 'OPP_KFS':
        return (f'o{placer}', _OPP_STYLE)
    return ('?', 'grey50')

# Background tint for special line states
_BG_WIN_LINE   = 'on dark_green'
_BG_OPP_THREAT = 'on dark_red'

ROW_INFO = [
    ('TOP', 80, (7, 8, 9), 'requires lift'),
    ('MID', 40, (4, 5, 6), 'R2 places normally'),
    ('BOT', 30, (1, 2, 3), 'R1 only'),
]


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _owner(rack: dict, slot: int) -> str:
    if not rack:
        return 'EMPTY'
    info = rack.get(slot) or rack.get(str(slot))
    if info is None:
        return 'EMPTY'
    return info.get('kfs_type', 'UNKNOWN')


def _slot_in_lines(slot: int, lines: list) -> bool:
    return any(slot in line for line in lines)


def _mode_badge(mode: str) -> Text:
    if mode == 'attack':
        return Text(' ATTACK  ', style='bold black on bright_green')
    if mode == 'defense':
        return Text(' DEFENSE ', style='bold black on bright_red')
    return Text(f' {mode.upper()} ', style='bold white on grey39')


def _estop_badge(estop: bool) -> Text:
    if estop:
        return Text(' E-STOP ', style='bold white on red blink')
    return Text(' OK ', style='bold black on bright_green')


def _link_badge(h7) -> Text:
    if h7 is None:
        return Text(' NO LINK ', style='bold white on red')
    age_ms = (time.monotonic_ns() - int(h7.get('rx_ns', 0))) / 1e6
    if age_ms > 500:
        return Text(f' STALE {age_ms:.0f}ms ', style='bold white on red')
    if age_ms > 200:
        return Text(f' SLOW {age_ms:.0f}ms ', style='bold black on yellow')
    return Text(f' OK {age_ms:.0f}ms ', style='bold black on bright_green')


# ─────────────────────────────────────────────────────────────────────────────
#  Header
# ─────────────────────────────────────────────────────────────────────────────

def _header(snap: BrainSnapshot) -> Panel:
    left = Text.assemble(
        ('  R2 BRAIN  ', 'bold black on bright_white'),
        '   ',
        _mode_badge(snap.arena_mode),
        '   ',
        _estop_badge(snap.estop),
        '   ',
        _link_badge(snap.h7_state),
    )
    right = Text(
        f'tick {snap.tick_count}   {snap.tick_hz:5.1f} Hz   placements {snap.placements_total}',
        style='dim',
    )
    table = Table.grid(expand=True)
    table.add_column(justify='left', ratio=1)
    table.add_column(justify='right', ratio=1)
    table.add_row(left, right)
    return Panel(table, box=SIMPLE_HEAVY, border_style='bright_white', padding=(0, 1))


# ─────────────────────────────────────────────────────────────────────────────
#  Rack — the centerpiece
# ─────────────────────────────────────────────────────────────────────────────

def _rack_cell(snap: BrainSnapshot, slot: int) -> Text:
    owner = _owner(snap.rack, slot)
    label, style = _slot_label_for(slot, owner)

    bg_style = ''
    if _slot_in_lines(slot, snap.won_lines):
        bg_style = _BG_WIN_LINE
    elif _slot_in_lines(slot, snap.blocked_by_opp):
        bg_style = _BG_OPP_THREAT

    full_style = f'{style} {bg_style}'.strip()

    text = Text(justify='center', style=bg_style or '')
    text.append(f' {slot} ', style=f'dim {bg_style}'.strip())
    text.append('\n')
    text.append(f'{label:^5}', style=full_style)
    text.append('\n')
    return text


def _rack_panel(snap: BrainSnapshot) -> Panel:
    rack = Table(box=HEAVY, show_header=True, header_style='bold cyan',
                 padding=(0, 2), expand=False)
    rack.add_column('', justify='right', style='dim')
    rack.add_column('COL 1', justify='center', min_width=7)
    rack.add_column('COL 2', justify='center', min_width=7)
    rack.add_column('COL 3', justify='center', min_width=7)

    for label, pts, slots, _hint in ROW_INFO:
        row_label = Text.assemble(
            (f'{label}\n', 'bold white'),
            (f'{pts} pts', 'dim'),
        )
        rack.add_row(row_label, *(_rack_cell(snap, s) for s in slots))

    score = _score_panel(snap)
    body = Group(Align.center(rack), Padding('', (1, 0)), score)
    return Panel(body, title='[bold cyan]ARENA RACK[/]', border_style='cyan',
                 padding=(1, 2))


# ─────────────────────────────────────────────────────────────────────────────
#  Score gauges
# ─────────────────────────────────────────────────────────────────────────────

_MAX_SCORE = 360  # 3 in top row + center middle = arbitrary practical cap


def _score_bar(label: str, score: int, color: str) -> Table:
    bar = ProgressBar(total=_MAX_SCORE, completed=score,
                      complete_style=color, finished_style=color,
                      width=24)
    table = Table.grid(padding=(0, 1))
    table.add_column(width=4, style='dim')
    table.add_column(width=24)
    table.add_column(justify='right', width=8)
    table.add_row(Text(label, style=f'bold {color}'), bar,
                  Text(f'{score} pts', style=f'bold {color}'))
    return table


def _score_panel(snap: BrainSnapshot) -> Group:
    rows = [
        _score_bar('US',  snap.estimated_score,    'bright_green'),
        _score_bar('OPP', snap.opp_score_estimate, 'bright_red'),
    ]
    if snap.won_lines:
        rows.append(Padding(
            Align.center(Text(f'★ WIN — line(s) {snap.won_lines}', style='bold black on bright_green')),
            (1, 0)))
    elif snap.blocked_by_opp:
        rows.append(Padding(
            Align.center(Text(f'⚠ OPP THREAT — line(s) {snap.blocked_by_opp}', style='bold white on red')),
            (1, 0)))
    return Group(*rows)


# ─────────────────────────────────────────────────────────────────────────────
#  Banners — what R2 will do, what R1 should do
# ─────────────────────────────────────────────────────────────────────────────

def _banner(title: str, body_text: str, color: str) -> Panel:
    inner = Text(body_text or '—', style=f'bold {color}', justify='center')
    return Panel(
        Align.center(Padding(inner, (1, 2)), vertical='middle'),
        title=Text(title, style=f'bold {color}'),
        border_style=color, padding=(0, 1), box=HEAVY, height=7,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Side panels (slim)
# ─────────────────────────────────────────────────────────────────────────────

def _r2_state_panel(snap: BrainSnapshot) -> Panel:
    inv = Text()
    if not snap.inventory:
        inv.append('inventory empty', style='dim')
    else:
        for i, kfs in enumerate(snap.inventory):
            style = 'bold bright_green' if kfs == 'R2_KFS' else (
                'bold bright_cyan' if kfs == 'R1_KFS' else 'bold yellow')
            label = 'R2' if kfs == 'R2_KFS' else ('R1' if kfs == 'R1_KFS' else 'FK')
            if i: inv.append('  ')
            inv.append(f' {label} ', style=f'black on {style.split()[-1]}')

    pose_text = Text('—', style='dim')
    if snap.pose:
        pose_text = Text.assemble(
            ('x ', 'dim'), (f'{snap.pose.get("x", 0):+6.2f}m   ', 'white'),
            ('y ', 'dim'), (f'{snap.pose.get("y", 0):+6.2f}m', 'white'),
        )

    body = Group(
        Padding(Text('INVENTORY', style='dim bold'), (0, 0)),
        Padding(inv, (0, 0, 1, 0)),
        Rule(style='grey23'),
        Padding(Text('POSE', style='dim bold'), (1, 0, 0, 0)),
        Padding(pose_text, (0, 0)),
    )
    return Panel(body, title='[bold green]R2[/]', border_style='green',
                 padding=(1, 2))


def _r1_state_panel(snap: BrainSnapshot) -> Panel:
    if snap.r1_status is None:
        body = Text('(no telemetry yet)', style='dim')
    else:
        r1 = snap.r1_status
        rows = []
        if 'mode' in r1:
            rows.append(Text.assemble(('mode    ', 'dim'),
                                      (str(r1['mode']), 'bold cyan')))
        if 'x' in r1 or 'y' in r1:
            rows.append(Text.assemble(
                ('pos     ', 'dim'),
                (f'({r1.get("x", 0):+.2f}, {r1.get("y", 0):+.2f}) m', 'white'),
            ))
        if r1.get('holding'):
            rows.append(Text.assemble(('holding ', 'dim'),
                                      (str(r1['holding']), 'yellow')))
        if r1.get('defended_slot'):
            rows.append(Text.assemble(('defending ', 'dim'),
                                      (f'slot {r1["defended_slot"]}', 'bold red')))
        if r1.get('cleared_slot'):
            rows.append(Text.assemble(('cleared ', 'dim'),
                                      (f'slot {r1["cleared_slot"]}', 'bold green')))
        if r1.get('zone'):
            rows.append(Text.assemble(('zone    ', 'dim'),
                                      (str(r1['zone']), 'white')))
        body = Group(*(rows or [Text('(no fields)', style='dim')]))
    return Panel(body, title='[bold cyan]R1[/]', border_style='cyan',
                 padding=(1, 2))


def _last_cmd_panel(snap: BrainSnapshot) -> Panel:
    if snap.last_command is None:
        body = Text('(no commands yet)', style='dim')
    else:
        cmd_name = snap.last_command.get('cmd', '?')
        rest = ', '.join(f'{k}={v}' for k, v in snap.last_command.items()
                         if k not in ('cmd', 'seq'))
        body = Group(
            Text(cmd_name, style='bold yellow'),
            Padding(Text(rest or '(no args)', style='white'), (0, 0)),
            Padding(Text(f'{snap.last_command_age_s:.1f}s ago',
                         style='dim'), (1, 0, 0, 0)),
        )
    return Panel(body, title='[bold yellow]LAST CMD →[/]', border_style='yellow',
                 padding=(1, 2))


# ─────────────────────────────────────────────────────────────────────────────
#  BT tree panel — show the running path highlighted
# ─────────────────────────────────────────────────────────────────────────────

def _bt_panel(snap: BrainSnapshot) -> Panel:
    if snap.tree_render:
        body = Text(snap.tree_render)
    else:
        body = Text('(tree not yet rendered)', style='dim')

    if snap.running_path:
        path_text = Text()
        for i, name in enumerate(snap.running_path):
            if i:
                path_text.append('  ›  ', style='grey50')
            path_text.append(name, style='bold bright_white' if i < len(snap.running_path) - 1 else 'bold bright_yellow')
        header = Group(Text('running:', style='dim'), path_text, Rule(style='grey23'))
    else:
        header = Group(Text('running: (none)', style='dim'), Rule(style='grey23'))

    return Panel(Group(header, body),
                 title='[bold blue]BEHAVIOUR TREE[/]',
                 border_style='blue', padding=(1, 2))


# ─────────────────────────────────────────────────────────────────────────────
#  Log
# ─────────────────────────────────────────────────────────────────────────────

def _log_panel(snap: BrainSnapshot) -> Panel:
    if not snap.log:
        body = Text('(no events yet)', style='dim')
    else:
        rows = []
        for line in snap.log:
            if 'cmd →' in line:
                style = 'yellow'
                icon = '◆'
            elif 'BT →' in line:
                style = 'cyan'
                icon = '▸'
            elif 'event @' in line:
                style = 'magenta'
                icon = '•'
            elif 'sim →' in line:
                style = 'green'
                icon = '✓'
            elif 'estop' in line.lower() or 'STOP' in line:
                style = 'bold red'
                icon = '✗'
            else:
                style = 'white'
                icon = ' '
            rows.append(Text.assemble((f' {icon} ', style), (line, style)))
        body = Group(*rows)
    return Panel(body, title='[dim]EVENT LOG[/]', border_style='grey23',
                 padding=(0, 1))


# ─────────────────────────────────────────────────────────────────────────────
#  Layout assembly
# ─────────────────────────────────────────────────────────────────────────────

def make_layout() -> Layout:
    root = Layout(name='root')
    root.split_column(
        Layout(name='header', size=3),
        Layout(name='body',   ratio=1),
        Layout(name='banner', size=8),
        Layout(name='log',    size=10),
    )
    root['body'].split_row(
        Layout(name='left',  ratio=4),
        Layout(name='mid',   ratio=4),
        Layout(name='right', ratio=3),
    )
    root['right'].split_column(
        Layout(name='r2',    ratio=1),
        Layout(name='r1',    ratio=1),
        Layout(name='cmd',   ratio=1),
    )
    root['banner'].split_row(
        Layout(name='b_r2'),
        Layout(name='b_r1'),
    )
    return root


def render_snapshot(layout: Layout, snap: BrainSnapshot):
    layout['header'].update(_header(snap))
    layout['left'].update(_rack_panel(snap))
    layout['mid'].update(_bt_panel(snap))
    layout['r2'].update(_r2_state_panel(snap))
    layout['r1'].update(_r1_state_panel(snap))
    layout['cmd'].update(_last_cmd_panel(snap))
    layout['b_r2'].update(_banner('R2 NEXT MOVE',
                                  snap.r2_next_action,
                                  'bright_green'))
    layout['b_r1'].update(_banner('R1 OPERATOR — DO THIS',
                                  snap.r1_advice,
                                  'bright_cyan'))
    layout['log'].update(_log_panel(snap))


# ─────────────────────────────────────────────────────────────────────────────
#  Public Dashboard
# ─────────────────────────────────────────────────────────────────────────────

class Dashboard:

    def __init__(self, refresh_per_second: int = 8):
        self._console = Console()
        self._layout  = make_layout()
        self._live    = Live(self._layout, console=self._console,
                             refresh_per_second=refresh_per_second,
                             screen=True)

    def __enter__(self):
        self._live.__enter__()
        return self

    def __exit__(self, *exc):
        self._live.__exit__(*exc)

    def render(self, snap: BrainSnapshot):
        render_snapshot(self._layout, snap)


@contextmanager
def open_dashboard(refresh_per_second: int = 8):
    with Dashboard(refresh_per_second=refresh_per_second) as dash:
        yield dash
