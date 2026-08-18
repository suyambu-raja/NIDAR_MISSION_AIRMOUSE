#!/usr/bin/env python3
"""
slam_visualizer.py — NIDAR AirMouse: Standalone SLAM Visualizer
================================================================
Simulates what RViz2 shows when slam_node is running.
No ROS 2 install needed — pure Python + matplotlib.

Usage options (choose based on your OS):

  # [RECOMMENDED for WSL / Windows] — saves animated GIF, open in browser:
  python3 tools/mock_publishers/slam_visualizer.py --save-gif

  # [Linux with display] — live animated window:
  python3 tools/mock_publishers/slam_visualizer.py

  # [Any system] — saves single PNG frame:
  python3 tools/mock_publishers/slam_visualizer.py --save-frame

  # Speed multiplier (works with any mode):
  python3 tools/mock_publishers/slam_visualizer.py --save-gif --speed 2.0
"""

import sys
import math
import argparse
import time
import numpy as np
import matplotlib
# Backend is selected after argument parsing — do NOT set it here
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch
from matplotlib.gridspec import GridSpec


# =============================================================================
# Arena / map constants — must match mock_slam.py
# =============================================================================
ARENA_SIZE_M      = 15.0     # metres
MAP_RESOLUTION    = 0.05     # m/cell
MAP_CELLS         = int(ARENA_SIZE_M / MAP_RESOLUTION)   # 300

FIGURE8_AMP_X     = 5.0      # metres
FIGURE8_AMP_Y     = 3.0      # metres
FIGURE8_PERIOD    = 30.0     # seconds per loop

MAP_RATE_HZ       = 2.0      # /map publish rate
POSE_RATE_HZ      = 10.0     # /drone_pose publish rate
ANIM_INTERVAL_MS  = int(1000 / POSE_RATE_HZ)   # animation frame interval

# GIF export: 6 seconds of animation at 10 fps = 60 frames
GIF_DURATION_SEC  = 6
GIF_FPS           = 10
GIF_FRAMES        = GIF_DURATION_SEC * GIF_FPS


# =============================================================================
# Build the static occupancy grid (matches _build_empty_map in mock_slam.py)
# =============================================================================
def build_grid() -> np.ndarray:
    """
    Returns a (MAP_CELLS × MAP_CELLS) numpy array:
      0   = FREE  (white in RViz2)
      100 = OCCUPIED / wall  (black in RViz2)
    """
    grid = np.zeros((MAP_CELLS, MAP_CELLS), dtype=np.int8)
    # Border walls — 1-cell thick perimeter
    grid[0, :]  = 100
    grid[-1, :] = 100
    grid[:, 0]  = 100
    grid[:, -1] = 100

    # Optional: add a few interior walls to make it look like a real maze
    # Horizontal wall segment — mimics a corridor divider
    mid = MAP_CELLS // 2
    grid[mid - 20 : mid + 20, mid] = 100         # vertical wall at centre
    grid[mid, 30 : mid - 10]       = 100         # horizontal wall left
    grid[mid, mid + 10 : -30]      = 100         # horizontal wall right
    # A room outline in top-left quadrant
    room_r, room_c = 60, 60
    grid[room_r : room_r + 40, room_c]            = 100
    grid[room_r : room_r + 40, room_c + 40]       = 100
    grid[room_r, room_c : room_c + 40]            = 100
    grid[room_r + 40, room_c : room_c + 41]       = 100
    # Doorway gap in room
    grid[room_r + 15 : room_r + 25, room_c]       = 0

    return grid


# =============================================================================
# Trajectory computation
# =============================================================================
def compute_pose(t: float, speed: float = 1.0):
    """
    Returns (x_m, y_m, yaw_rad) at time t [seconds].
    Matches the figure-8 in mock_slam.py exactly.
    """
    period = FIGURE8_PERIOD / speed
    omega  = (2.0 * math.pi) / period
    x   = FIGURE8_AMP_X * math.sin(omega * t)
    y   = FIGURE8_AMP_Y * math.sin(2.0 * omega * t) / 2.0
    dx  = FIGURE8_AMP_X * omega * math.cos(omega * t)
    dy  = FIGURE8_AMP_Y * omega * math.cos(2.0 * omega * t)
    yaw = math.atan2(dy, dx)
    return x, y, yaw


def metres_to_cell(m_x: float, m_y: float):
    """Convert world metres → grid cell indices (col, row)."""
    col = int((m_x + ARENA_SIZE_M / 2) / MAP_RESOLUTION)
    row = int((m_y + ARENA_SIZE_M / 2) / MAP_RESOLUTION)
    col = max(1, min(MAP_CELLS - 2, col))
    row = max(1, min(MAP_CELLS - 2, row))
    return col, row


# =============================================================================
# Visualizer
# =============================================================================
class SlamVisualizer:
    def __init__(self, speed: float = 1.0):
        self.speed      = speed
        self.grid       = build_grid()
        self.start_time = time.monotonic()

        # Trail history — stores (col, row) pairs
        self.trail_cols: list[float] = []
        self.trail_rows: list[float] = []
        self.max_trail  = 500   # keep last N poses in trail

        # Telemetry history for the speed graph
        self.time_history: list[float]  = []
        self.x_history:    list[float]  = []
        self.y_history:    list[float]  = []

        self._build_figure()

    # ------------------------------------------------------------------ #
    # Figure layout                                                        #
    # ------------------------------------------------------------------ #
    def _build_figure(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(14, 7), facecolor="#0d1117")
        # set_window_title only works with interactive backends — skip for Agg
        try:
            self.fig.canvas.manager.set_window_title(
                "NIDAR AirMouse — SLAM Visualizer (mock_slam)")
        except AttributeError:
            pass

        gs = GridSpec(3, 2, figure=self.fig,
                      left=0.06, right=0.97, top=0.93, bottom=0.08,
                      wspace=0.35, hspace=0.55)

        # ── Left: Map panel (occupies all 3 rows, column 0) ──
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self._setup_map_panel()

        # ── Right top: X/Y position over time ──
        self.ax_xy = self.fig.add_subplot(gs[0, 1])
        self._setup_xy_panel()

        # ── Right middle: Yaw (heading) over time ──
        self.ax_yaw = self.fig.add_subplot(gs[1, 1])
        self._setup_yaw_panel()

        # ── Right bottom: Live telemetry text ──
        self.ax_telem = self.fig.add_subplot(gs[2, 1])
        self._setup_telem_panel()

        # Title
        self.fig.text(0.5, 0.97,
                      "⚠  MOCK SLAM VISUALIZER  —  replace with real slam_node for production",
                      ha="center", va="top", fontsize=9,
                      color="#f97316", fontweight="bold")

    def _setup_map_panel(self):
        ax = self.ax_map
        ax.set_facecolor("#111827")
        ax.set_title("/map  (nav_msgs/OccupancyGrid)  300×300 cells @ 5cm/cell",
                     fontsize=9, color="#94a3b8", pad=6)
        ax.set_xlabel("X  [metres]", fontsize=8, color="#64748b")
        ax.set_ylabel("Y  [metres]", fontsize=8, color="#64748b")
        ax.tick_params(colors="#64748b", labelsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor("#1e293b")

        # Display the occupancy grid
        # Flip vertically because matplotlib row 0 is top, ROS row 0 is bottom
        display_grid = np.flipud(self.grid)
        extent = [-ARENA_SIZE_M / 2, ARENA_SIZE_M / 2,
                  -ARENA_SIZE_M / 2, ARENA_SIZE_M / 2]
        self._map_im = ax.imshow(display_grid, cmap="gray_r",
                                  vmin=0, vmax=100,
                                  extent=extent, origin="upper",
                                  interpolation="nearest", alpha=0.9)

        # Grid lines at 1m intervals
        for v in np.arange(-7, 8, 1):
            ax.axvline(v, color="#1e293b", linewidth=0.4, alpha=0.6)
            ax.axhline(v, color="#1e293b", linewidth=0.4, alpha=0.6)

        ax.set_xlim(-ARENA_SIZE_M / 2 - 0.2, ARENA_SIZE_M / 2 + 0.2)
        ax.set_ylim(-ARENA_SIZE_M / 2 - 0.2, ARENA_SIZE_M / 2 + 0.2)
        ax.set_aspect("equal")

        # Drone trail line
        self._trail_line, = ax.plot([], [], "-", color="#22d3ee",
                                     linewidth=1.2, alpha=0.7,
                                     label="drone trail")

        # Drone arrow (heading indicator)
        self._drone_arrow = ax.annotate(
            "", xy=(0.1, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color="#f97316",
                            lw=2.5, mutation_scale=18)
        )

        # Drone body dot
        self._drone_dot, = ax.plot([0], [0], "o",
                                    color="#f97316", markersize=10,
                                    zorder=5, label="drone")

        # Topic label
        ax.text(0.02, 0.02, "/drone_pose → (x, y, yaw)",
                transform=ax.transAxes, fontsize=7,
                color="#f97316", alpha=0.8)

        # Legend
        ax.legend(loc="upper right", fontsize=7,
                  facecolor="#1e293b", edgecolor="#334155",
                  labelcolor="white")

        # Frame rate watermark
        self._fps_text = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                                  fontsize=7, color="#64748b", va="top")

    def _setup_xy_panel(self):
        ax = self.ax_xy
        ax.set_facecolor("#111827")
        ax.set_title("/drone_pose  position", fontsize=8, color="#94a3b8")
        ax.set_ylabel("metres", fontsize=7, color="#64748b")
        ax.tick_params(colors="#64748b", labelsize=6)
        for spine in ax.spines.values(): spine.set_edgecolor("#1e293b")
        ax.set_ylim(-8, 8)
        self._x_line, = ax.plot([], [], "-", color="#3b82f6", linewidth=1.4, label="x")
        self._y_line, = ax.plot([], [], "-", color="#10b981", linewidth=1.4, label="y")
        ax.legend(loc="upper right", fontsize=6, facecolor="#1e293b",
                  edgecolor="#334155", labelcolor="white")
        ax.axhline(0, color="#334155", linewidth=0.5)

    def _setup_yaw_panel(self):
        ax = self.ax_yaw
        ax.set_facecolor("#111827")
        ax.set_title("/drone_pose  yaw (heading)", fontsize=8, color="#94a3b8")
        ax.set_ylabel("degrees", fontsize=7, color="#64748b")
        ax.set_xlabel("time [s]", fontsize=7, color="#64748b")
        ax.tick_params(colors="#64748b", labelsize=6)
        for spine in ax.spines.values(): spine.set_edgecolor("#1e293b")
        ax.set_ylim(-200, 200)
        self._yaw_line, = ax.plot([], [], "-", color="#a855f7", linewidth=1.4)
        ax.axhline(0, color="#334155", linewidth=0.5)

    def _setup_telem_panel(self):
        ax = self.ax_telem
        ax.set_facecolor("#111827")
        ax.axis("off")
        ax.set_title("Live telemetry", fontsize=8, color="#94a3b8")
        for spine in ax.spines.values(): spine.set_edgecolor("#1e293b")

        fields = ["topic_map", "topic_pose", "x", "y", "yaw",
                  "speed", "map_rate", "pose_rate", "elapsed"]
        self._telem_texts = {}
        row_h = 0.13
        for i, key in enumerate(fields):
            y_pos = 0.88 - i * row_h
            self._telem_texts[key] = ax.text(
                0.05, y_pos, "", transform=ax.transAxes,
                fontsize=8, color="#e2e8f0",
                fontfamily="monospace", va="top"
            )

    # ------------------------------------------------------------------ #
    # Animation update                                                     #
    # ------------------------------------------------------------------ #
    def update(self, frame: int):
        elapsed = time.monotonic() - self.start_time
        x, y, yaw = compute_pose(elapsed, self.speed)

        col, row = metres_to_cell(x, y)

        # Update trail
        self.trail_cols.append(x)
        self.trail_rows.append(y)
        if len(self.trail_cols) > self.max_trail:
            self.trail_cols.pop(0)
            self.trail_rows.pop(0)

        # Telemetry history (keep last 10s)
        self.time_history.append(elapsed)
        self.x_history.append(x)
        self.y_history.append(y)
        window = 12.0  # seconds to show
        while self.time_history and self.time_history[0] < elapsed - window:
            self.time_history.pop(0)
            self.x_history.pop(0)
            self.y_history.pop(0)

        # ── Map panel ──
        self._trail_line.set_data(self.trail_cols, self.trail_rows)
        self._drone_dot.set_data([x], [y])

        # Arrow tip: 0.5m ahead of drone in heading direction
        tip_x = x + 0.6 * math.cos(yaw)
        tip_y = y + 0.6 * math.sin(yaw)
        self._drone_arrow.set_position((x, y))
        self._drone_arrow.xy = (tip_x, tip_y)

        self._fps_text.set_text(f"t = {elapsed:.1f}s  |  frame #{frame}")

        # ── XY panel ──
        if self.time_history:
            t0 = self.time_history[0]
            self.ax_xy.set_xlim(t0, t0 + window)
            self.ax_yaw.set_xlim(t0, t0 + window)

        self._x_line.set_data(self.time_history, self.x_history)
        self._y_line.set_data(self.time_history, self.y_history)

        # ── Yaw panel ──
        yaw_hist = [math.degrees(compute_pose(t, self.speed)[2])
                     for t in self.time_history]
        self._yaw_line.set_data(self.time_history, yaw_hist)

        # ── Telemetry text ──
        speed_est = math.sqrt(
            (FIGURE8_AMP_X * (2 * math.pi / (FIGURE8_PERIOD / self.speed))) ** 2 +
            (FIGURE8_AMP_Y * (2 * math.pi / (FIGURE8_PERIOD / self.speed))) ** 2
        ) / 2
        yaw_deg = math.degrees(yaw)

        self._telem_texts["topic_map"].set_text(
            f"📡 /map          ~{MAP_RATE_HZ:.0f} Hz | 300×300 | 5cm/cell")
        self._telem_texts["topic_pose"].set_text(
            f"📡 /drone_pose   ~{POSE_RATE_HZ:.0f} Hz | frame: map")
        self._telem_texts["x"].set_text(     f"   x      : {x:+7.3f} m")
        self._telem_texts["y"].set_text(     f"   y      : {y:+7.3f} m")
        self._telem_texts["yaw"].set_text(   f"   yaw    : {yaw_deg:+7.1f} °")
        self._telem_texts["speed"].set_text( f"   speed  : {speed_est:.2f} m/s  (×{self.speed:.1f})")
        self._telem_texts["map_rate"].set_text(
            f"   /map   : {MAP_RATE_HZ:.1f} Hz  (TRANSIENT_LOCAL)")
        self._telem_texts["pose_rate"].set_text(
            f"   /pose  : {POSE_RATE_HZ:.1f} Hz  (BEST_EFFORT)")
        self._telem_texts["elapsed"].set_text(
            f"   uptime : {elapsed:.1f} s")

        return (self._trail_line, self._drone_dot,
                self._x_line, self._y_line, self._yaw_line)

    # ------------------------------------------------------------------ #
    # Run                                                                  #
    # ------------------------------------------------------------------ #
    def run(self, mode: str = "live"):
        if mode == "frame":
            # ── Single PNG frame ──────────────────────────────────────── #
            self.start_time = time.monotonic() - 8.0   # t=8s into trajectory
            self.update(0)
            out = "tools/mock_publishers/slam_visualizer_frame.png"
            self.fig.savefig(out, dpi=140, facecolor=self.fig.get_facecolor())
            print(f"[slam_visualizer] Frame saved → {out}")
            print(f"[slam_visualizer] Open in Windows: explorer.exe '{out}'")

        elif mode == "gif":
            # ── Animated GIF — works on any system, open in browser ──── #
            print(f"[slam_visualizer] Rendering {GIF_FRAMES} frames for GIF ...")
            anim = animation.FuncAnimation(
                self.fig, self.update,
                frames=GIF_FRAMES,
                interval=int(1000 / GIF_FPS),
                blit=False,
            )
            out = "tools/mock_publishers/slam_visualizer.gif"
            writer = animation.PillowWriter(fps=GIF_FPS)
            anim.save(out, writer=writer,
                      savefig_kwargs={"facecolor": self.fig.get_facecolor()})
            print(f"[slam_visualizer] GIF saved  → {out}")
            print(f"[slam_visualizer] Open in Windows: explorer.exe '{out}'")
            # Auto-open in Windows when running from WSL
            import subprocess
            subprocess.run(["explorer.exe", out.replace("/", "\\")],
                           capture_output=True)

        else:
            # ── Live interactive window ────────────────────────────────── #
            self.anim = animation.FuncAnimation(
                self.fig, self.update,
                interval=ANIM_INTERVAL_MS,
                blit=False,
                cache_frame_data=False,
            )
            plt.show()


# =============================================================================
# Entry point
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="NIDAR AirMouse — Standalone SLAM Visualizer (no ROS 2 needed)"
    )
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Simulation speed multiplier (default: 1.0)")
    parser.add_argument("--save-frame", action="store_true",
                        help="Save a single PNG frame (works on all systems)")
    parser.add_argument("--save-gif", action="store_true",
                        help="Save an animated GIF — RECOMMENDED for WSL/Windows")
    args = parser.parse_args()

    # ── Select matplotlib backend before any figure is created ──────── #
    if args.save_frame or args.save_gif:
        # Non-interactive: Agg renders to file without needing a display
        matplotlib.use("Agg")
    else:
        # Interactive: try TkAgg first (needs python3-tk), fall back to Agg
        try:
            matplotlib.use("TkAgg")
        except Exception:
            print("[slam_visualizer] No display backend — switching to --save-gif mode")
            matplotlib.use("Agg")
            args.save_gif = True

    # Determine run mode
    if args.save_gif:
        mode = "gif"
    elif args.save_frame:
        mode = "frame"
    else:
        mode = "live"

    print("=" * 60)
    print("  NIDAR AirMouse — SLAM Visualizer")
    print("  MOCK DATA — no real hardware")
    print(f"  Mode : {mode.upper()}")
    print(f"  Speed: {args.speed}x  |  Map: {MAP_CELLS}x{MAP_CELLS} @ {MAP_RESOLUTION}m/cell")
    if mode == "gif":
        print(f"  GIF  : {GIF_DURATION_SEC}s @ {GIF_FPS} fps = {GIF_FRAMES} frames")
    print("=" * 60)

    vis = SlamVisualizer(speed=args.speed)
    vis.run(mode=mode)


if __name__ == "__main__":
    main()
