"""
Browser-based dashboard for r2_brain.

Stdlib-only HTTP server (no Flask / no extra deps). Serves:
  - GET /                — single-page dashboard (HTML/CSS/JS)
  - GET /snapshot.json   — latest BrainSnapshot as JSON
  - GET /healthz         — liveness probe

The page polls /snapshot.json every 200 ms and re-renders. Designed so
the R1 operator can pin it on a tablet / second monitor and read R2's
intent at a glance.

Usage (from any owner of a snapshot):

    web = WebDashboard(port=8080)
    web.start()                # runs in a background thread
    ...
    web.update(snapshot)       # call every BT tick
    ...
    web.stop()
"""

from __future__ import annotations

import json
import socket
import threading
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .arena import compute_next_move
from .state import BrainSnapshot


# ─────────────────────────────────────────────────────────────────────────────
#  HTML page
# ─────────────────────────────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>R2 Brain — Arena Dashboard</title>
<style>
  :root {
    --bg:        #0d1117;
    --bg-2:      #161b22;
    --border:    #30363d;
    --muted:     #7d8590;
    --text:      #e6edf3;
    --green:     #3fb950;
    --green-dim: #033a16;
    --red:       #f85149;
    --red-dim:   #421515;
    --blue:      #58a6ff;
    --blue-dim:  #052e3a;
    --orange:    #f0883e;
    --yellow:    #d29922;
    --purple:    #d2a8ff;
  }
  * { box-sizing: border-box; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto,
                 "Helvetica Neue", Arial, sans-serif;
    background: var(--bg); color: var(--text); margin: 0; padding: 24px;
    min-height: 100vh;
  }

  /* Header */
  .header {
    display: flex; justify-content: space-between; align-items: center;
    padding: 14px 22px; background: var(--bg-2);
    border: 1px solid var(--border); border-radius: 10px;
    margin-bottom: 18px;
  }
  .header .title  { font-size: 18px; font-weight: 700; letter-spacing: 1px; }
  .header .meta   { color: var(--muted); font-size: 13px; }
  .badges { display: inline-flex; gap: 10px; margin-left: 16px; }
  .badge  { padding: 5px 12px; border-radius: 20px;
            font-size: 11px; font-weight: 700; letter-spacing: 1px; }
  .badge.ok       { background: var(--green); color: black; }
  .badge.bad      { background: var(--red); color: white; }
  .badge.attack   { background: var(--green); color: black; }
  .badge.defense  { background: var(--red); color: white; }
  .badge.warn     { background: var(--yellow); color: black; }

  /* Banners */
  .banners {
    display: grid; grid-template-columns: 1fr 1fr; gap: 14px;
    margin-bottom: 18px;
  }
  .banner {
    padding: 28px 24px; border-radius: 10px; text-align: center;
    border-width: 2px; border-style: solid;
  }
  .banner-title { font-size: 11px; letter-spacing: 2.5px;
                  opacity: 0.65; margin-bottom: 10px; font-weight: 700; }
  .banner-body  { font-size: 22px; font-weight: 700; line-height: 1.35; }
  .banner.r2 { background: var(--green-dim); border-color: var(--green); color: var(--green); }
  .banner.r1 { background: var(--blue-dim);  border-color: var(--blue);  color: var(--blue); }

  /* Body grid */
  .grid {
    display: grid; grid-template-columns: 1.4fr 1.4fr 1fr;
    gap: 16px; margin-bottom: 16px;
  }
  .panel {
    background: var(--bg-2); border: 1px solid var(--border);
    border-radius: 10px; padding: 20px;
  }
  .panel h3 {
    margin: 0 0 16px 0; color: var(--muted);
    font-size: 11px; text-transform: uppercase; letter-spacing: 2px;
    font-weight: 700;
  }

  /* Rack */
  .rack {
    display: grid; grid-template-columns: 80px repeat(3, 1fr); gap: 8px;
    margin-bottom: 22px;
  }
  .rack-row-head {
    display: flex; flex-direction: column; justify-content: center;
    color: var(--muted); padding: 8px;
  }
  .rack-row-head b { color: var(--text); font-size: 14px; }
  .rack-row-head span { font-size: 11px; opacity: 0.6; }
  .rack-cell {
    background: #21262d; border-radius: 8px; padding: 18px 10px;
    text-align: center; min-height: 88px;
    display: flex; flex-direction: column; justify-content: space-between;
    border: 1px solid transparent; transition: all 200ms;
  }
  .rack-cell .num   { font-size: 11px; color: var(--muted); }
  .rack-cell .owner { font-size: 28px; font-weight: 800; }
  .rack-cell.ours   { background: var(--green-dim); border-color: var(--green); color: var(--green); }
  .rack-cell.opp    { background: var(--red-dim);   border-color: var(--red);   color: var(--red); }
  .rack-cell.win    { background: var(--green); color: black;
                      box-shadow: 0 0 24px rgba(63,185,80,0.55); border-color: var(--green); }
  .rack-cell.threat { background: var(--red);   color: white;
                      box-shadow: 0 0 24px rgba(248,81,73,0.55); border-color: var(--red); }

  /* Score bars */
  .score { display: grid; grid-template-columns: 50px 1fr 80px;
           gap: 12px; align-items: center; margin: 6px 0; }
  .score .lbl  { font-weight: 700; font-size: 13px; }
  .score .pts  { font-weight: 700; text-align: right; font-size: 13px; }
  .bar { height: 10px; background: #30363d; border-radius: 5px; overflow: hidden; }
  .bar > div { height: 100%; transition: width 250ms ease; }

  .verdict {
    margin-top: 14px; padding: 12px; border-radius: 8px;
    font-weight: 700; text-align: center; font-size: 13px;
  }
  .verdict.win    { background: var(--green); color: black; }
  .verdict.threat { background: var(--red);   color: white; }

  /* Tree */
  .breadcrumb { font-size: 12px; color: var(--muted);
                margin-bottom: 14px; padding-bottom: 14px;
                border-bottom: 1px solid var(--border); line-height: 1.6; }
  .breadcrumb b { color: var(--orange); }
  .tree { font-family: "SF Mono", Monaco, Consolas, "Courier New", monospace;
          font-size: 12px; line-height: 1.5; white-space: pre;
          color: #c9d1d9; max-height: 360px; overflow-y: auto; }

  /* KV */
  .kv { display: grid; grid-template-columns: 90px 1fr; row-gap: 6px; column-gap: 8px; }
  .kv .k { color: var(--muted); font-size: 13px; }
  .kv .v { font-size: 13px; }

  /* Pills */
  .pill { display: inline-block; padding: 4px 12px; border-radius: 14px;
          font-weight: 700; font-size: 12px; margin-right: 6px;
          background: var(--green); color: black; }
  .pill.r1   { background: var(--blue); color: white; }
  .pill.fake { background: var(--orange); color: black; }

  /* Last cmd */
  .cmd-name { font-size: 18px; font-weight: 700; color: var(--orange); }
  .cmd-args { margin-top: 4px; color: var(--text); font-size: 13px; }
  .cmd-age  { margin-top: 8px; color: var(--muted); font-size: 12px; }

  /* Stack right column */
  .stack > .panel { margin-bottom: 16px; }
  .stack > .panel:last-child { margin-bottom: 0; }

  /* Log */
  .log { font-family: "SF Mono", Monaco, Consolas, monospace;
         font-size: 12px; max-height: 220px; overflow-y: auto; }
  .log-line { padding: 3px 6px; border-radius: 3px; line-height: 1.5; }
  .log-line.cmd   { color: var(--orange); }
  .log-line.bt    { color: var(--blue); }
  .log-line.sim   { color: var(--green); }
  .log-line.event { color: var(--purple); }
  .log-line.stop  { color: var(--red); font-weight: 700; }

  .stale {
    position: fixed; top: 0; left: 0; right: 0; padding: 8px;
    background: var(--red); color: white; text-align: center;
    font-weight: 700; transform: translateY(-100%); transition: transform 200ms;
  }
  .stale.show { transform: translateY(0); }
</style>
</head>
<body>

<div id="staleBanner" class="stale">⚠ DASHBOARD STALE — backend not responding</div>

<div class="header">
  <div>
    <span class="title">R2 BRAIN</span>
    <span class="badges">
      <span id="modeBadge"  class="badge">—</span>
      <span id="estopBadge" class="badge">—</span>
      <span id="linkBadge"  class="badge">—</span>
    </span>
  </div>
  <div class="meta" id="meta">tick — 0.0 Hz</div>
</div>

<div class="banners">
  <div class="banner r2">
    <div class="banner-title">R2 NEXT MOVE</div>
    <div class="banner-body" id="r2Action">—</div>
  </div>
  <div class="banner r1">
    <div class="banner-title">R1 OPERATOR — DO THIS</div>
    <div class="banner-body" id="r1Advice">—</div>
  </div>
</div>

<div class="grid">
  <div class="panel">
    <h3>Arena Rack</h3>
    <div class="rack" id="rack"></div>
    <div id="scores"></div>
  </div>
  <div class="panel">
    <h3>Behaviour Tree</h3>
    <div class="breadcrumb" id="breadcrumb">—</div>
    <pre class="tree" id="tree"></pre>
  </div>
  <div class="stack">
    <div class="panel">
      <h3>R2</h3>
      <div id="r2State">—</div>
    </div>
    <div class="panel">
      <h3>R1</h3>
      <div id="r1State">—</div>
    </div>
    <div class="panel">
      <h3>Last Cmd →</h3>
      <div id="lastCmd">—</div>
    </div>
  </div>
</div>

<div class="panel">
  <h3>Event Log</h3>
  <div class="log" id="log"></div>
</div>

<script>
const ROW_INFO = [
  {label:'TOP', pts:80, slots:[7,8,9]},
  {label:'MID', pts:40, slots:[4,5,6]},
  {label:'BOT', pts:30, slots:[1,2,3]},
];

// Per the rules: bottom row is always R1's, middle + top are R2's.
// The fusion node only tags TEAM_KFS / OPP_KFS, so we derive the placing
// robot from the slot row.
function slotLabel(slot, owner) {
  if (!owner || owner === 'EMPTY')                 return '·';
  if (owner === 'Fake_KFS' || owner === 'FAKE_KFS') return 'FAKE';
  const placer = (slot >= 1 && slot <= 3) ? 'R1' : 'R2';
  if (owner === 'OPP_KFS')                         return 'o' + placer;
  if (owner === 'TEAM_KFS' || owner === 'R1_KFS' || owner === 'R2_KFS')
    return placer;
  return '?';
}

function esc(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;'); }

function rackOwner(rack, slot){
  if (!rack) return 'EMPTY';
  const info = rack[slot] || rack[String(slot)];
  if (!info) return 'EMPTY';
  return info.kfs_type || 'UNKNOWN';
}

function renderRack(snap){
  const el = document.getElementById('rack');
  const winSet = new Set();
  (snap.won_lines || []).forEach(l => l.forEach(s => winSet.add(s)));
  const threatSet = new Set();
  (snap.blocked_by_opp || []).forEach(l => l.forEach(s => threatSet.add(s)));
  const html = [];
  for (const row of ROW_INFO){
    html.push(`<div class="rack-row-head"><b>${row.label}</b><span>${row.pts} pts</span></div>`);
    for (const s of row.slots){
      const owner = rackOwner(snap.rack, s);
      let cls = '';
      if (winSet.has(s)) cls = 'win';
      else if (threatSet.has(s)) cls = 'threat';
      else if (owner === 'TEAM_KFS' || owner === 'R2_KFS' || owner === 'R1_KFS') cls = 'ours';
      else if (owner === 'OPP_KFS') cls = 'opp';
      html.push(`<div class="rack-cell ${cls}"><div class="num">${s}</div><div class="owner">${slotLabel(s, owner)}</div></div>`);
    }
  }
  el.innerHTML = html.join('');
}

function bar(label, score, color, max){
  const pct = Math.min(100, (score / max) * 100);
  return `<div class="score">
    <span class="lbl" style="color:${color}">${label}</span>
    <div class="bar"><div style="width:${pct}%;background:${color}"></div></div>
    <span class="pts" style="color:${color}">${score} pts</span>
  </div>`;
}

function renderScores(snap){
  const max = 360;
  let h  = bar('US',  snap.estimated_score,    'var(--green)', max);
      h += bar('OPP', snap.opp_score_estimate, 'var(--red)',   max);
  if (snap.won_lines && snap.won_lines.length)
    h += `<div class="verdict win">★ INSTANT WIN — ${JSON.stringify(snap.won_lines)}</div>`;
  else if (snap.blocked_by_opp && snap.blocked_by_opp.length)
    h += `<div class="verdict threat">⚠ OPP THREAT — line(s) ${JSON.stringify(snap.blocked_by_opp)}</div>`;
  document.getElementById('scores').innerHTML = h;
}

function renderHeader(snap){
  const m = document.getElementById('modeBadge');
  m.className = 'badge ' + (snap.arena_mode === 'attack' ? 'attack' : (snap.arena_mode === 'defense' ? 'defense' : 'warn'));
  m.textContent = (snap.arena_mode || '').toUpperCase() || '—';

  const e = document.getElementById('estopBadge');
  e.className = 'badge ' + (snap.estop ? 'bad' : 'ok');
  e.textContent = snap.estop ? 'E-STOP' : 'OK';

  const l = document.getElementById('linkBadge');
  if (!snap.h7_state) { l.className = 'badge bad'; l.textContent = 'NO H7 LINK'; }
  else { l.className = 'badge ok'; l.textContent = 'H7 LINK OK'; }

  document.getElementById('meta').textContent =
    `tick ${snap.tick_count}    ${(snap.tick_hz||0).toFixed(1)} Hz    placements ${snap.placements_total}`;
}

function renderBanners(snap){
  document.getElementById('r2Action').textContent = snap.r2_next_action || '—';
  document.getElementById('r1Advice').textContent = snap.r1_advice    || '—';
}

function renderTree(snap){
  const path = snap.running_path || [];
  const bc = document.getElementById('breadcrumb');
  if (!path.length) bc.innerHTML = '<i>not running</i>';
  else {
    bc.innerHTML = path.slice(0,-1).map(p => esc(p)).join(' › ')
      + (path.length>1?' › ':'') + '<b>' + esc(path[path.length-1]) + '</b>';
  }
  document.getElementById('tree').textContent = snap.tree_render || '';
}

function renderR2(snap){
  let h = '';
  const inv = snap.inventory || [];
  if (!inv.length) h += '<div style="color:var(--muted);font-style:italic">inventory empty</div>';
  else {
    h += '<div>' + inv.map(k => {
      const cls = k === 'R1_KFS' ? 'r1' : (k === 'Fake_KFS' || k === 'FAKE' ? 'fake' : '');
      const lbl = k === 'R2_KFS' ? 'R2' : (k === 'R1_KFS' ? 'R1' : 'FK');
      return `<span class="pill ${cls}">${lbl}</span>`;
    }).join('') + '</div>';
  }
  if (snap.pose) {
    h += `<div style="margin-top:14px" class="kv">
      <span class="k">x</span><span class="v">${snap.pose.x.toFixed(2)} m</span>
      <span class="k">y</span><span class="v">${snap.pose.y.toFixed(2)} m</span>
    </div>`;
  }
  document.getElementById('r2State').innerHTML = h;
}

function renderR1(snap){
  if (!snap.r1_status) {
    document.getElementById('r1State').innerHTML =
      '<div style="color:var(--muted);font-style:italic">no telemetry</div>';
    return;
  }
  const r = snap.r1_status;
  const rows = [];
  if (r.mode)          rows.push(['mode',     `<b style="color:var(--blue)">${esc(r.mode)}</b>`]);
  if (r.x !== undefined) rows.push(['pos',    `(${r.x.toFixed(2)}, ${r.y.toFixed(2)}) m`]);
  if (r.holding)       rows.push(['holding',  `<span style="color:var(--orange)">${esc(r.holding)}</span>`]);
  if (r.defended_slot) rows.push(['defending',`<b style="color:var(--red)">slot ${r.defended_slot}</b>`]);
  if (r.cleared_slot)  rows.push(['cleared',  `<b style="color:var(--green)">slot ${r.cleared_slot}</b>`]);
  if (r.zone)          rows.push(['zone',     esc(r.zone)]);
  document.getElementById('r1State').innerHTML =
    rows.length ? '<div class="kv">' + rows.map(([k,v]) =>
      `<span class="k">${k}</span><span class="v">${v}</span>`).join('') + '</div>'
    : '<div style="color:var(--muted)">(no fields)</div>';
}

function renderCmd(snap){
  if (!snap.last_command) {
    document.getElementById('lastCmd').innerHTML =
      '<div style="color:var(--muted);font-style:italic">no commands yet</div>';
    return;
  }
  const c = snap.last_command;
  const args = Object.keys(c).filter(k => k !== 'cmd' && k !== 'seq')
                .map(k => `${k}=${c[k]}`).join(', ');
  document.getElementById('lastCmd').innerHTML =
    `<div class="cmd-name">${esc(c.cmd)}</div>` +
    `<div class="cmd-args">${esc(args) || '<i>no args</i>'}</div>` +
    `<div class="cmd-age">${(snap.last_command_age_s || 0).toFixed(1)}s ago</div>`;
}

function renderLog(snap){
  const el = document.getElementById('log');
  el.innerHTML = (snap.log || []).map(line => {
    let cls = '';
    if (line.includes('cmd →'))      cls = 'cmd';
    else if (line.includes('BT →'))  cls = 'bt';
    else if (line.includes('sim →')) cls = 'sim';
    else if (line.includes('event @')) cls = 'event';
    else if (line.includes('STOP') || line.toLowerCase().includes('estop')) cls = 'stop';
    return `<div class="log-line ${cls}">${esc(line)}</div>`;
  }).join('');
}

let lastOk = Date.now();
async function poll(){
  try {
    const r = await fetch('/snapshot.json', { cache: 'no-store' });
    const snap = await r.json();
    renderHeader(snap); renderBanners(snap);
    renderRack(snap);   renderScores(snap);
    renderTree(snap);   renderR2(snap);
    renderR1(snap);     renderCmd(snap);
    renderLog(snap);
    lastOk = Date.now();
    document.getElementById('staleBanner').classList.remove('show');
  } catch (e) {
    if (Date.now() - lastOk > 1500) {
      document.getElementById('staleBanner').classList.add('show');
    }
  }
  setTimeout(poll, 200);
}
poll();
</script>
</body>
</html>
"""


_TESTER_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>R2 Strategy Tester</title>
<style>
  :root {
    --bg:#0d1117; --bg2:#161b22; --border:#30363d; --muted:#7d8590;
    --text:#e6edf3; --green:#3fb950; --green-dim:#033a16;
    --red:#f85149; --red-dim:#421515; --blue:#58a6ff; --orange:#f0883e;
    --yellow:#d29922;
  }
  * { box-sizing:border-box; }
  body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;
         background:var(--bg); color:var(--text); margin:0; padding:24px; }

  h1 { font-size:20px; font-weight:700; letter-spacing:1px; margin:0 0 6px; }
  .sub { color:var(--muted); font-size:13px; margin-bottom:24px; }

  .layout { display:grid; grid-template-columns:auto 1fr; gap:28px; align-items:start; }

  /* rack */
  .rack-wrap { background:var(--bg2); border:1px solid var(--border);
               border-radius:12px; padding:24px; }
  .rack { display:grid; grid-template-columns:64px repeat(3,1fr); gap:8px; }
  .row-head { display:flex; flex-direction:column; justify-content:center;
              color:var(--muted); font-size:12px; padding:4px 8px; }
  .row-head b { color:var(--text); font-size:15px; }
  .cell {
    width:90px; height:90px; border-radius:10px; border:2px solid var(--border);
    display:flex; flex-direction:column; align-items:center; justify-content:center;
    cursor:pointer; background:#21262d; user-select:none;
    transition:all 150ms; position:relative;
  }
  .cell:hover { border-color:#58a6ff; transform:scale(1.04); }
  .cell .snum { font-size:11px; color:var(--muted); position:absolute; top:6px; left:8px; }
  .cell .owner { font-size:22px; font-weight:800; }
  .cell.team  { background:var(--green-dim); border-color:var(--green); color:var(--green); }
  .cell.opp   { background:var(--red-dim);   border-color:var(--red);   color:var(--red); }
  .cell.target { box-shadow:0 0 0 3px var(--yellow), 0 0 18px rgba(210,153,34,.5);
                 border-color:var(--yellow); }

  /* legend */
  .legend { display:flex; gap:14px; margin-top:14px; font-size:12px; color:var(--muted); }
  .legend span { display:inline-flex; align-items:center; gap:5px; }
  .dot { width:10px; height:10px; border-radius:50%; }

  /* controls */
  .controls { background:var(--bg2); border:1px solid var(--border);
              border-radius:12px; padding:20px; margin-bottom:16px; }
  .controls h3 { margin:0 0 14px; font-size:11px; text-transform:uppercase;
                 letter-spacing:2px; color:var(--muted); font-weight:700; }
  .row { display:flex; align-items:center; gap:12px; margin-bottom:12px; }
  .row label { font-size:13px; color:var(--muted); min-width:80px; }
  .btn-group { display:flex; gap:6px; }
  .btn { padding:6px 16px; border-radius:6px; border:1px solid var(--border);
         background:#21262d; color:var(--text); font-size:13px; cursor:pointer; }
  .btn.active { background:var(--blue); border-color:var(--blue); color:#000; font-weight:700; }
  .inv-input { width:60px; padding:6px 10px; border-radius:6px;
               border:1px solid var(--border); background:#21262d;
               color:var(--text); font-size:15px; font-weight:700; text-align:center; }

  /* move banner */
  .move-panel { border-radius:12px; padding:28px 24px; text-align:center;
                border:2px solid; margin-bottom:16px; }
  .move-panel .priority { font-size:11px; letter-spacing:2.5px; opacity:.65;
                          margin-bottom:8px; font-weight:700; }
  .move-panel .move-text { font-size:24px; font-weight:800; line-height:1.3; }
  .move-panel.win     { background:var(--green-dim); border-color:var(--green); color:var(--green); }
  .move-panel.block   { background:var(--red-dim);   border-color:var(--red);   color:var(--red); }
  .move-panel.middle  { background:#0d2137;           border-color:var(--blue);  color:var(--blue); }
  .move-panel.default { background:#1c1a0a;           border-color:var(--orange);color:var(--orange); }

  /* score */
  .scores { background:var(--bg2); border:1px solid var(--border);
            border-radius:12px; padding:20px; }
  .scores h3 { margin:0 0 14px; font-size:11px; text-transform:uppercase;
               letter-spacing:2px; color:var(--muted); font-weight:700; }
  .score-row { display:grid; grid-template-columns:40px 1fr 70px;
               gap:10px; align-items:center; margin:6px 0; }
  .bar { height:10px; background:#30363d; border-radius:5px; overflow:hidden; }
  .bar > div { height:100%; transition:width 200ms; }
  .lines { margin-top:12px; font-size:13px; }

  .reset-btn { padding:7px 18px; border-radius:6px; border:1px solid var(--border);
               background:#21262d; color:var(--muted); font-size:13px; cursor:pointer; }
  .reset-btn:hover { color:var(--text); border-color:var(--text); }
  .nav { margin-bottom:20px; }
  .nav a { color:var(--blue); font-size:13px; text-decoration:none; }
  .nav a:hover { text-decoration:underline; }

  /* Action buttons inside the move panel */
  .move-actions { display:flex; gap:10px; justify-content:center; margin-top:20px; }
  .action-btn {
    padding:10px 24px; border-radius:8px; border:2px solid; font-size:14px;
    font-weight:700; cursor:pointer; background:transparent; transition:all 150ms;
    letter-spacing:0.5px;
  }
  .action-btn.execute { border-color:var(--green); color:var(--green); }
  .action-btn.execute:hover:not(:disabled) { background:var(--green); color:#000; }
  .action-btn.execute:disabled { border-color:var(--muted); color:var(--muted);
                                  cursor:not-allowed; opacity:0.5; }
  .action-btn.undo { border-color:var(--muted); color:var(--muted); }
  .action-btn.undo:hover:not(:disabled) { color:var(--text); border-color:var(--text); }
  .action-btn.undo:disabled { opacity:0.3; cursor:not-allowed; }

  /* Move history */
  .history-panel { background:var(--bg2); border:1px solid var(--border);
                   border-radius:12px; padding:20px; margin-top:16px; }
  .history-panel h3 { margin:0 0 14px; font-size:11px; text-transform:uppercase;
                      letter-spacing:2px; color:var(--muted); font-weight:700; }
  .history { font-size:13px; max-height:260px; overflow-y:auto; }
  .history-empty { color:var(--muted); font-style:italic; }
  .history-item {
    display:flex; align-items:center; gap:10px; padding:8px 4px;
    border-bottom:1px solid var(--border);
  }
  .history-item:last-child { border-bottom:none; }
  .history-item .step { color:var(--muted); font-size:11px; min-width:30px; }
  .history-item .icon {
    width:24px; height:24px; border-radius:50%; display:inline-flex;
    align-items:center; justify-content:center; font-size:10px; font-weight:800;
  }
  .history-item .icon.team { background:var(--green); color:#000; }
  .history-item .icon.opp  { background:var(--red);   color:#fff; }
  .history-item .desc { flex:1; color:var(--muted); font-size:12px; }
  .history-item .priority-tag {
    background:#0d2137; color:var(--blue); padding:2px 8px;
    border-radius:4px; font-size:10px; font-weight:700;
    letter-spacing:0.5px; text-transform:uppercase;
  }
  .history-item .slot-num {
    background:#21262d; color:var(--text); padding:3px 10px; border-radius:4px;
    font-weight:700; font-family:"SF Mono",Monaco,monospace; font-size:12px;
  }
</style>
</head>
<body>
<div class="nav"><a href="/">← Live Dashboard</a></div>
<h1>R2 Strategy Tester</h1>
<p class="sub">Click any slot to cycle it EMPTY → TEAM → OPP → EMPTY. Highlighted border = R2's chosen target.</p>

<div class="layout">
  <!-- left: rack -->
  <div class="rack-wrap">
    <div class="rack" id="rack"></div>
    <div class="legend">
      <span><span class="dot" style="background:var(--green)"></span>TEAM (ours)</span>
      <span><span class="dot" style="background:var(--red)"></span>OPP</span>
      <span><span class="dot" style="background:var(--yellow)"></span>R2 target</span>
    </div>
    <div style="margin-top:16px; text-align:right;">
      <button class="reset-btn" onclick="resetRack()">Reset rack</button>
    </div>
  </div>

  <!-- right: controls + move -->
  <div>
    <div class="controls">
      <h3>Scenario</h3>
      <div class="row">
        <label>Mode</label>
        <div class="btn-group">
          <button class="btn" id="btn-attack"  onclick="setMode('attack')" >Attack</button>
          <button class="btn" id="btn-defense" onclick="setMode('defense')">Defense</button>
        </div>
      </div>
      <div class="row">
        <label>Inventory</label>
        <input class="inv-input" type="number" id="inv" min="0" max="9" value="3"
               oninput="update()">
        <span style="font-size:13px;color:var(--muted)">KFS</span>
      </div>
    </div>

    <div class="move-panel default" id="movePanel">
      <div class="priority" id="movePriority">PRIORITY</div>
      <div class="move-text" id="moveText">—</div>
      <div class="move-actions">
        <button class="action-btn execute" id="executeBtn" onclick="executeMove()" disabled>
          ✓ Execute Move
        </button>
        <button class="action-btn undo" id="undoBtn" onclick="undo()" disabled>
          ↶ Undo
        </button>
      </div>
    </div>

    <div class="scores">
      <h3>Score estimate</h3>
      <div class="score-row">
        <span style="color:var(--green);font-weight:700">US</span>
        <div class="bar"><div id="barUs" style="background:var(--green);width:0%"></div></div>
        <span id="ptsUs" style="color:var(--green);font-weight:700;text-align:right">0 pts</span>
      </div>
      <div class="score-row">
        <span style="color:var(--red);font-weight:700">OPP</span>
        <div class="bar"><div id="barOpp" style="background:var(--red);width:0%"></div></div>
        <span id="ptsOpp" style="color:var(--red);font-weight:700;text-align:right">0 pts</span>
      </div>
      <div class="lines" id="lines"></div>
    </div>

    <div class="history-panel">
      <h3>Move history</h3>
      <div class="history" id="history">
        <div class="history-empty">No moves yet — click "Execute Move" to start.</div>
      </div>
    </div>
  </div>
</div>

<script>
const ROWS = [
  {label:'TOP', pts:80, slots:[7,8,9]},
  {label:'MID', pts:40, slots:[4,5,6]},
  {label:'BOT', pts:30, slots:[1,2,3]},
];
const SLOT_PTS = {1:30,2:30,3:30,4:40,5:40,6:40,7:80,8:80,9:80};
const LINES = [[1,4,7],[2,5,8],[3,6,9],[1,5,9],[3,5,7]];
const CYCLE = ['EMPTY','TEAM_KFS','OPP_KFS'];

let rack = {};      // slot -> 'EMPTY'|'TEAM_KFS'|'OPP_KFS'
let mode = 'attack';
let targetSlot = null;
let lastSuggestion = null;
let moveHistory = [];   // [{rack, inv, mode, move}, ...]

for(let s=1;s<=9;s++) rack[s]='EMPTY';

function esc(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;'); }

function setMode(m){
  mode=m;
  document.getElementById('btn-attack').className  = 'btn'+(m==='attack'?' active':'');
  document.getElementById('btn-defense').className = 'btn'+(m==='defense'?' active':'');
  update();
}

function resetRack(){
  for(let s=1;s<=9;s++) rack[s]='EMPTY';
  moveHistory = [];
  renderHistory();
  update();
}

function cycleSlot(s){
  const cur = rack[s]||'EMPTY';
  const idx = (CYCLE.indexOf(cur)+1)%CYCLE.length;
  rack[s] = CYCLE[idx];
  update();
}

function executeMove(){
  if(!lastSuggestion || !lastSuggestion.slot) return;
  const invInp = document.getElementById('inv');
  const inv = parseInt(invInp.value) || 0;
  if(inv <= 0) return;

  // Save snapshot for undo
  moveHistory.push({
    rack: {...rack},
    inv: inv,
    mode: mode,
    move: lastSuggestion,
  });

  // Apply the suggested move
  rack[lastSuggestion.slot] = 'TEAM_KFS';
  invInp.value = inv - 1;

  renderHistory();
  update();
}

function undo(){
  if(!moveHistory.length) return;
  const last = moveHistory.pop();
  rack = last.rack;
  document.getElementById('inv').value = last.inv;
  if(last.mode !== mode) setMode(last.mode);
  renderHistory();
  update();
}

function renderHistory(){
  const el = document.getElementById('history');
  if(!moveHistory.length){
    el.innerHTML = '<div class="history-empty">No moves yet — click "Execute Move" to start.</div>';
    document.getElementById('undoBtn').disabled = true;
    return;
  }
  document.getElementById('undoBtn').disabled = false;
  el.innerHTML = moveHistory.map((h,i) => {
    const slot = h.move.slot;
    const lbl  = slot <= 3 ? 'R1' : 'R2';
    const pri  = (h.move.priority || '').toUpperCase().replace('_',' ');
    return `<div class="history-item">
      <span class="step">#${i+1}</span>
      <span class="icon team">${lbl}</span>
      <span class="priority-tag">${esc(pri)}</span>
      <span class="desc">→ slot</span>
      <span class="slot-num">${slot}</span>
    </div>`;
  }).join('');
  // auto-scroll to bottom
  el.scrollTop = el.scrollHeight;
}

function renderRack(){
  const el=document.getElementById('rack');
  const rows=[];
  for(const row of ROWS){
    rows.push(`<div class="row-head"><b>${row.label}</b><span>${row.pts}pt</span></div>`);
    for(const s of row.slots){
      const st=rack[s]||'EMPTY';
      let cls='cell';
      let label='·';
      if(st==='TEAM_KFS'){cls+=' team'; label=s<=3?'R1':'R2';}
      else if(st==='OPP_KFS'){cls+=' opp'; label='OPP';}
      if(s===targetSlot) cls+=' target';
      rows.push(`<div class="${cls}" onclick="cycleSlot(${s})">
        <span class="snum">${s}</span>
        <span class="owner">${label}</span>
      </div>`);
    }
  }
  el.innerHTML=rows.join('');
}

function score(owner){
  return Object.entries(rack).reduce((acc,[s,v])=>
    acc+(v===owner?SLOT_PTS[+s]||0:0),0);
}

function renderScores(){
  const MAX=360;
  const us=score('TEAM_KFS'), opp=score('OPP_KFS');
  document.getElementById('barUs').style.width=Math.min(100,us/MAX*100)+'%';
  document.getElementById('barOpp').style.width=Math.min(100,opp/MAX*100)+'%';
  document.getElementById('ptsUs').textContent=us+' pts';
  document.getElementById('ptsOpp').textContent=opp+' pts';

  const wins=LINES.filter(l=>l.every(s=>rack[s]==='TEAM_KFS'));
  const threats=LINES.filter(l=>{
    const o=l.filter(s=>rack[s]==='OPP_KFS');
    const e=l.filter(s=>rack[s]==='EMPTY');
    return o.length===2&&e.length===1;
  });
  let h='';
  if(wins.length) h+=`<div style="color:var(--green);font-weight:700">★ WIN — line(s) ${JSON.stringify(wins)}</div>`;
  if(threats.length) h+=`<div style="color:var(--red);font-weight:700;margin-top:6px">⚠ OPP THREAT — ${JSON.stringify(threats)}</div>`;
  document.getElementById('lines').innerHTML=h;
}

async function update(){
  renderRack();
  renderScores();
  const inv=parseInt(document.getElementById('inv').value)||0;
  try{
    const res=await fetch('/tester/compute',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({rack,inventory:inv,mode}),
    });
    const d=await res.json();
    lastSuggestion=d;
    targetSlot=d.slot||null;
    const panel=document.getElementById('movePanel');
    const panelClass={win:'win',block:'block',middle:'middle'};
    panel.className='move-panel '+(panelClass[d.priority]||'default');
    document.getElementById('movePriority').textContent=
      (d.priority||'').toUpperCase().replace('_',' ');
    document.getElementById('moveText').textContent=d.move||'—';
    // Enable Execute only when there's a reachable slot AND inventory remaining
    document.getElementById('executeBtn').disabled = !d.slot || inv <= 0;
  }catch(e){
    document.getElementById('moveText').textContent='(server error)';
    document.getElementById('executeBtn').disabled = true;
  }
  renderRack();
}

setMode('attack');
update();
</script>
</body>
</html>
"""


# ─────────────────────────────────────────────────────────────────────────────
#  Server
# ─────────────────────────────────────────────────────────────────────────────

def _make_handler(dashboard):
    class _Handler(BaseHTTPRequestHandler):
        def do_GET(self_inner):
            try:
                if self_inner.path in ('/', '/index.html'):
                    self_inner._send(200, 'text/html; charset=utf-8', _HTML)
                elif self_inner.path in ('/tester', '/tester/'):
                    self_inner._send(200, 'text/html; charset=utf-8', _TESTER_HTML)
                elif self_inner.path == '/snapshot.json':
                    with dashboard._lock:
                        snap = dashboard._snapshot
                    body = json.dumps(asdict(snap), default=str)
                    self_inner._send(200, 'application/json', body)
                elif self_inner.path == '/healthz':
                    self_inner._send(200, 'text/plain', 'ok')
                else:
                    self_inner._send(404, 'text/plain', 'not found')
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self_inner):
            try:
                if self_inner.path == '/tester/compute':
                    length = int(self_inner.headers.get('Content-Length', 0))
                    body = json.loads(self_inner.rfile.read(length))
                    raw_rack = body.get('rack', {})
                    rack = {int(k): (None if v == 'EMPTY' else {'kfs_type': v})
                            for k, v in raw_rack.items()}
                    result = compute_next_move(
                        rack,
                        int(body.get('inventory', 0)),
                        str(body.get('mode', 'attack')),
                    )
                    self_inner._send(200, 'application/json', json.dumps(result))
                else:
                    self_inner._send(404, 'text/plain', 'not found')
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _send(self_inner, code, ct, body):
            data = body.encode() if isinstance(body, str) else body
            self_inner.send_response(code)
            self_inner.send_header('Content-Type', ct)
            self_inner.send_header('Content-Length', str(len(data)))
            self_inner.send_header('Cache-Control', 'no-store')
            self_inner.end_headers()
            self_inner.wfile.write(data)

        def log_message(self_inner, fmt, *args):
            pass  # silence access logs

    return _Handler


class WebDashboard:
    """Background HTTP server that serves the latest BrainSnapshot."""

    def __init__(self, port: int = 8080, bind: str = '0.0.0.0'):
        self._port = port
        self._bind = bind
        self._snapshot = BrainSnapshot()
        self._lock = threading.Lock()
        self._server = None
        self._thread = None

    def update(self, snap: BrainSnapshot):
        with self._lock:
            self._snapshot = snap

    def start(self):
        handler = _make_handler(self)
        try:
            self._server = ThreadingHTTPServer((self._bind, self._port), handler)
        except OSError as e:
            raise RuntimeError(f'Could not bind {self._bind}:{self._port} — {e}') from e
        self._thread = threading.Thread(
            target=self._server.serve_forever, name='r2_brain_web', daemon=True
        )
        self._thread.start()

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    @property
    def url(self) -> str:
        host = self._bind if self._bind not in ('0.0.0.0', '') else _local_ip()
        return f'http://{host}:{self._port}/'


def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'
