#!/usr/bin/env python3
"""
grid_mapper_visualizer.py — NIDAR AirMouse: Real-Time Web Grid Dashboard
=========================================================================
Subscribes to /survivor_grid_locations and /drone_pose via ROS 2,
then serves a live web dashboard at http://localhost:8080

Open in any browser on Windows — updates every second automatically.

Run inside nidar-dev container:
    docker run --rm -v "${PWD}:/workspace" -p 8080:8080 nidar-dev bash -c \
      "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
       python3 /workspace/tools/mock_publishers/grid_mapper_visualizer.py"

Then open: http://localhost:8080
"""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)
from geometry_msgs.msg import PoseStamped
from nidar_msgs.msg import SurvivorGridArray


# =============================================================================
# Shared state (written by ROS thread, read by HTTP thread)
# =============================================================================
_state_lock = threading.Lock()
_state = {
    "survivors": [],          # list of {id, grid_box, world_x, world_y, conf}
    "drone_x": None,          # drone world X in metres
    "drone_y": None,          # drone world Y in metres
    "drone_grid": None,       # drone current grid box string
    "grid_origin_x": 0.0,
    "grid_origin_y": 0.0,
    "last_update": 0.0,
}

ARENA_COLS = 15
ARENA_ROWS = 15


def world_to_grid(world_x, world_y, origin_x=0.0, origin_y=0.0):
    ax = world_x - origin_x
    ay = world_y - origin_y
    if ax < 0 or ay < 0:
        return None
    col = int(ax)
    row = int(ay) + 1
    if col >= ARENA_COLS or row > ARENA_ROWS:
        return None
    return chr(65 + col) + str(row)


# =============================================================================
# ROS 2 Node
# =============================================================================

class GridVisualizerNode(Node):
    def __init__(self):
        super().__init__("grid_visualizer_web")

        RELIABLE_TL = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        )
        BEST_EFFORT = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10,
        )

        self.create_subscription(
            SurvivorGridArray, "/survivor_grid_locations",
            self._survivors_cb, RELIABLE_TL,
        )
        self.create_subscription(
            PoseStamped, "/drone_pose",
            self._pose_cb, BEST_EFFORT,
        )
        self.get_logger().info("Grid visualizer node started — serving http://0.0.0.0:8080")

    def _survivors_cb(self, msg):
        survivors = []
        for loc in msg.locations:
            survivors.append({
                "id": loc.survivor_id,
                "grid_box": loc.grid_box,
                "world_x": round(loc.world_x, 2),
                "world_y": round(loc.world_y, 2),
                "confidence": round(loc.confidence, 2),
            })
        with _state_lock:
            _state["survivors"] = survivors
            _state["last_update"] = time.time()

    def _pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        grid = world_to_grid(x, y,
                             _state["grid_origin_x"],
                             _state["grid_origin_y"])
        with _state_lock:
            _state["drone_x"] = round(x, 2)
            _state["drone_y"] = round(y, 2)
            _state["drone_grid"] = grid


# =============================================================================
# HTTP Server — serves the dashboard + API
# =============================================================================

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>NIDAR AirMouse — Grid Mapper Live</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700&display=swap');
  :root {
    --bg: #0d1117; --panel: #161b22; --border: #30363d;
    --accent: #58a6ff; --green: #3fb950; --orange: #f78166;
    --yellow: #e3b341; --text: #c9d1d9; --muted: #8b949e;
    --grid-free: #1c2128; --grid-line: #2d3748;
    --grid-survivor: #f78166; --grid-drone: #58a6ff;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg); color: var(--text);
    font-family: 'Inter', system-ui, sans-serif;
    min-height: 100vh; display: flex; flex-direction: column;
  }
  header {
    background: var(--panel); border-bottom: 1px solid var(--border);
    padding: 14px 24px; display: flex; align-items: center; gap: 16px;
  }
  header h1 { font-size: 1.1rem; font-weight: 600; color: var(--accent); }
  .badge {
    font-size: 0.7rem; padding: 3px 10px; border-radius: 20px;
    font-weight: 600; letter-spacing: 0.05em;
  }
  .badge-live { background: #1a3a1a; color: var(--green); border: 1px solid var(--green); }
  .badge-count { background: #1a2a3a; color: var(--accent); border: 1px solid var(--accent); }
  .pulse { display:inline-block; width:8px; height:8px; border-radius:50%;
    background:var(--green); margin-right:5px;
    animation: pulse 1.5s ease-in-out infinite; }
  @keyframes pulse { 0%,100%{opacity:1;transform:scale(1)} 50%{opacity:.4;transform:scale(.8)} }

  main { flex: 1; display: flex; gap: 20px; padding: 20px; flex-wrap: wrap; }

  .panel {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 16px;
  }
  .panel h2 { font-size: 0.8rem; font-weight: 600; color: var(--muted);
    text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 14px; }

  /* ── Grid ── */
  .grid-wrap { flex: 0 0 auto; }
  #arena {
    display: grid;
    grid-template-columns: 28px repeat(15, 38px);
    grid-template-rows: repeat(15, 38px) 28px;
    gap: 2px;
  }
  .row-label, .col-label {
    display: flex; align-items: center; justify-content: center;
    font-size: 0.65rem; color: var(--muted); font-weight: 600;
  }
  .cell {
    background: var(--grid-free); border-radius: 4px;
    position: relative; cursor: default;
    transition: transform 0.15s, background 0.3s;
    display: flex; align-items: center; justify-content: center;
    font-size: 0.55rem; font-weight: 700;
  }
  .cell:hover { transform: scale(1.15); z-index: 10; }
  .cell.has-survivor {
    background: var(--grid-survivor);
    box-shadow: 0 0 12px rgba(247,129,102,0.6);
    animation: glow 2s ease-in-out infinite;
  }
  @keyframes glow {
    0%,100%{ box-shadow: 0 0 8px rgba(247,129,102,0.5); }
    50%{ box-shadow: 0 0 18px rgba(247,129,102,0.9); }
  }
  .cell.has-drone {
    background: var(--grid-drone);
    box-shadow: 0 0 10px rgba(88,166,255,0.6);
  }
  .cell.has-drone.has-survivor { background: var(--yellow); }
  .survivor-icon { font-size: 1rem; }
  .drone-icon { font-size: 0.9rem; }

  /* ── Side panels ── */
  .side { flex: 1; min-width: 220px; display: flex; flex-direction: column; gap: 16px; }

  .drone-card {
    background: #0d1f35; border: 1px solid #1e3a5a; border-radius: 8px; padding: 14px;
  }
  .drone-card .label { font-size: 0.7rem; color: var(--muted); margin-bottom: 4px; }
  .drone-card .value { font-size: 1.4rem; font-weight: 700; color: var(--accent); font-family: monospace; }
  .drone-card .sub { font-size: 0.75rem; color: var(--muted); margin-top: 3px; }

  .survivor-list { display: flex; flex-direction: column; gap: 8px; }
  .survivor-card {
    background: #1a1a1a; border: 1px solid #2a1a1a;
    border-radius: 8px; padding: 12px;
    display: flex; align-items: center; gap: 12px;
    transition: border-color 0.3s;
  }
  .survivor-card:hover { border-color: var(--grid-survivor); }
  .s-id {
    width: 32px; height: 32px; border-radius: 50%;
    background: var(--grid-survivor); color: white;
    display: flex; align-items: center; justify-content: center;
    font-weight: 700; font-size: 0.85rem; flex-shrink: 0;
  }
  .s-info { flex: 1; }
  .s-grid { font-size: 1.1rem; font-weight: 700; color: var(--orange); }
  .s-pos { font-size: 0.7rem; color: var(--muted); margin-top: 2px; }
  .s-conf {
    font-size: 0.7rem; padding: 2px 8px; border-radius: 20px;
    background: #1a2f1a; color: var(--green); font-weight: 600;
  }

  .empty-state {
    color: var(--muted); font-size: 0.85rem; text-align: center;
    padding: 30px 20px; border: 1px dashed var(--border); border-radius: 8px;
  }
  .empty-state .icon { font-size: 2rem; margin-bottom: 8px; }

  footer {
    text-align: center; padding: 10px; font-size: 0.7rem; color: var(--muted);
    border-top: 1px solid var(--border);
  }
  #last-update { color: var(--accent); }
</style>
</head>
<body>

<header>
  <h1>🚁 NIDAR AirMouse — Grid Mapper</h1>
  <span class="badge badge-live"><span class="pulse"></span>LIVE</span>
  <span class="badge badge-count" id="survivor-count-badge">0 / 6 Survivors</span>
  <span style="margin-left:auto;font-size:0.75rem;color:var(--muted)">
    Updated: <span id="last-update">—</span>
  </span>
</header>

<main>
  <!-- 15×15 Arena Grid -->
  <div class="panel grid-wrap">
    <h2>🗺 Arena Grid (15 × 15 m)</h2>
    <div id="arena"></div>
  </div>

  <!-- Side info -->
  <div class="side">
    <div class="panel">
      <h2>🚁 Drone Position</h2>
      <div class="drone-card">
        <div class="label">Current Grid Box</div>
        <div class="value" id="drone-grid">—</div>
        <div class="sub" id="drone-coords">Waiting for pose…</div>
      </div>
    </div>

    <div class="panel" style="flex:1">
      <h2>🔴 Confirmed Survivors</h2>
      <div class="survivor-list" id="survivor-list">
        <div class="empty-state">
          <div class="icon">🔍</div>
          No survivors detected yet.<br>
          Inject a detection to see it here.
        </div>
      </div>
    </div>

    <div class="panel">
      <h2>📡 Legend</h2>
      <div style="display:flex;flex-direction:column;gap:8px;font-size:0.8rem">
        <div style="display:flex;align-items:center;gap:10px">
          <div style="width:20px;height:20px;background:#f78166;border-radius:4px"></div>
          Survivor location
        </div>
        <div style="display:flex;align-items:center;gap:10px">
          <div style="width:20px;height:20px;background:#58a6ff;border-radius:4px"></div>
          Drone position
        </div>
        <div style="display:flex;align-items:center;gap:10px">
          <div style="width:20px;height:20px;background:#e3b341;border-radius:4px"></div>
          Drone + Survivor
        </div>
        <div style="display:flex;align-items:center;gap:10px">
          <div style="width:20px;height:20px;background:#1c2128;border-radius:4px;border:1px solid #30363d"></div>
          Free space
        </div>
      </div>
    </div>
  </div>
</main>

<footer>
  NIDAR AirMouse — grid_mapper_node | ROS 2 Humble | Polling /api/state every 1s
</footer>

<script>
const COLS = 15, ROWS = 15;
const COL_LABELS = 'ABCDEFGHIJKLMNO'.split('');

// Build the grid DOM once
function buildGrid() {
  const arena = document.getElementById('arena');
  arena.innerHTML = '';

  // Top row: empty corner + col labels A-O
  arena.appendChild(Object.assign(document.createElement('div'), {className:'col-label'}));
  COL_LABELS.forEach(c => {
    const el = document.createElement('div');
    el.className = 'col-label';
    el.textContent = c;
    arena.appendChild(el);
  });

  // Rows 15→1 (top to bottom visual = high Y → low Y)
  for (let row = ROWS; row >= 1; row--) {
    // Row label
    const rl = document.createElement('div');
    rl.className = 'row-label';
    rl.textContent = row;
    arena.appendChild(rl);

    // Cells
    for (let col = 0; col < COLS; col++) {
      const cell = document.createElement('div');
      cell.className = 'cell';
      cell.id = `cell-${COL_LABELS[col]}${row}`;
      cell.title = `${COL_LABELS[col]}${row}`;
      arena.appendChild(cell);
    }
  }
}

function resetGrid() {
  document.querySelectorAll('.cell').forEach(c => {
    c.className = 'cell';
    c.innerHTML = '';
  });
}

function updateGrid(state) {
  resetGrid();

  // Mark drone
  if (state.drone_grid) {
    const dc = document.getElementById(`cell-${state.drone_grid}`);
    if (dc) { dc.classList.add('has-drone'); dc.innerHTML = '<span class="drone-icon">🚁</span>'; }
  }

  // Mark survivors
  state.survivors.forEach(s => {
    const sc = document.getElementById(`cell-${s.grid_box}`);
    if (sc) {
      const isDrone = sc.classList.contains('has-drone');
      sc.classList.add('has-survivor');
      sc.innerHTML = isDrone
        ? '<span class="survivor-icon">🚁🔴</span>'
        : '<span class="survivor-icon">🔴</span>';
    }
  });
}

function updateDroneInfo(state) {
  const gridEl = document.getElementById('drone-grid');
  const coordEl = document.getElementById('drone-coords');
  gridEl.textContent = state.drone_grid || '—';
  if (state.drone_x !== null) {
    coordEl.textContent = `World: (${state.drone_x}m, ${state.drone_y}m)`;
  }
}

function updateSurvivorList(state) {
  const list = document.getElementById('survivor-list');
  const badge = document.getElementById('survivor-count-badge');
  badge.textContent = `${state.survivors.length} / 6 Survivors`;

  if (state.survivors.length === 0) {
    list.innerHTML = `<div class="empty-state"><div class="icon">🔍</div>No survivors detected yet.<br>Inject a detection to see it here.</div>`;
    return;
  }

  list.innerHTML = state.survivors.map(s => `
    <div class="survivor-card">
      <div class="s-id">${s.id}</div>
      <div class="s-info">
        <div class="s-grid">${s.grid_box}</div>
        <div class="s-pos">World: (${s.world_x}m, ${s.world_y}m)</div>
      </div>
      <div class="s-conf">${Math.round(s.confidence * 100)}%</div>
    </div>
  `).join('');
}

async function poll() {
  try {
    const res = await fetch('/api/state');
    const state = await res.json();
    updateGrid(state);
    updateDroneInfo(state);
    updateSurvivorList(state);
    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
  } catch (e) {
    document.getElementById('last-update').textContent = 'Connection lost…';
  }
}

buildGrid();
poll();
setInterval(poll, 1000);  // refresh every second
</script>
</body>
</html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # silence HTTP access logs

    def do_GET(self):
        if self.path == "/api/state":
            with _state_lock:
                data = json.dumps(_state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        else:
            # Serve dashboard HTML for any other path
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML.encode())


def run_http_server():
    server = HTTPServer(("0.0.0.0", 8080), DashboardHandler)
    print("[Dashboard] Serving at http://localhost:8080  — open in your browser")
    server.serve_forever()


# =============================================================================
# Main
# =============================================================================

def main():
    # Start HTTP server in a daemon thread
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    # Run ROS 2 node in the main thread
    rclpy.init()
    node = GridVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Dashboard] Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
