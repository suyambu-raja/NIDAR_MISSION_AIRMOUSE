/**
 * Flight Telemetry, Kinematics, Autonomy Pipeline, and Sensor Health Controller for NIDAR M2 GCS.
 */
class GCSTelemetryView {
  constructor() {
    this.canvas = document.getElementById("attitude-canvas");
    this.ctx = this.canvas ? this.canvas.getContext("2d") : null;
    this.roll = 0.0;
    this.pitch = 0.0;
  }

  updateTelemetry(telem) {
    const setElem = (id, txt) => {
      const el = document.getElementById(id);
      if (el) el.textContent = txt;
    };

    setElem("val-altitude", (telem.altitude || 0.0).toFixed(2));
    setElem("val-speed", (telem.velocity || 0.0).toFixed(2));
    setElem("val-heading", String(Math.round(telem.heading || 0)).padStart(3, "0"));
    setElem(
      "val-roll-pitch",
      `${(telem.roll || 0) >= 0 ? "+" : ""}${(telem.roll || 0).toFixed(1)} / ${(telem.pitch || 0) >= 0 ? "+" : ""}${(telem.pitch || 0).toFixed(1)}`
    );
    setElem("val-pose", `(${(telem.x || 0).toFixed(2)}, ${(telem.y || 0).toFixed(2)})`);
    setElem("val-grid", telem.grid_cell || "A1");

    // Battery Gauge
    const battFill = document.getElementById("battery-bar-fill");
    const battText = document.getElementById("battery-text");
    if (battFill && battText) {
      const pct = Math.max(0, Math.min(100, Math.round(telem.battery_pct || 100)));
      const v = telem.battery_v || 16.6;
      battFill.style.width = `${pct}%`;
      battText.textContent = `${pct}% (${v.toFixed(1)}V)`;

      if (pct < 20) {
        battFill.style.backgroundColor = "var(--accent-red)";
      } else if (pct < 40) {
        battFill.style.backgroundColor = "var(--accent-yellow)";
      } else {
        battFill.style.backgroundColor = "var(--accent-green)";
      }
    }

    // Artificial Horizon Attitude Indicator
    this.roll = telem.roll || 0.0;
    this.pitch = telem.pitch || 0.0;
    this.drawAttitude();
  }

  updateAutonomyStatus(status) {
    const setElem = (id, txt, cls) => {
      const el = document.getElementById(id);
      if (el) {
        el.textContent = txt;
        if (cls) el.className = cls;
      }
    };

    if (status.state) setElem("auto-state", status.state, "am-val active");
    if (status.frontiers !== undefined) setElem("auto-frontiers", String(status.frontiers));
    if (status.corridor_type) setElem("auto-corridor", status.corridor_type.toUpperCase(), "am-val highlight");
    if (status.obstacle_distance !== undefined) setElem("auto-obstacle", `${status.obstacle_distance.toFixed(2)} m`);
    if (status.progress !== undefined) setElem("auto-coverage", `${Math.round(status.progress * 100)}%`);
  }

  updateFailsafeStatus(fs) {
    const badge = document.getElementById("fs-state-badge");
    const reasonEl = document.getElementById("fs-reason");
    const actionEl = document.getElementById("fs-action");
    const sysBadge = document.getElementById("system-safety-badge");

    const active = Boolean(fs.active || (fs.state && fs.state !== "SAFE" && fs.state !== "STANDBY"));

    if (badge) {
      badge.textContent = fs.state || (active ? "FAILSAFE ACTIVE" : "SAFE / READY");
      badge.className = active ? "fs-badge fs-active" : "fs-badge fs-safe";
    }

    if (reasonEl) reasonEl.textContent = fs.reason || "NONE";
    if (actionEl) actionEl.textContent = fs.action || (active ? "EMERGENCY HOLD" : "STANDBY");

    if (sysBadge) {
      if (active) {
        sysBadge.textContent = "⚠ FAILSAFE ACTIVE";
        sysBadge.className = "badge badge-disconnected";
      } else {
        sysBadge.textContent = "SYSTEM OK";
        sysBadge.className = "badge badge-connected";
      }
    }
  }

  updateSensorHealth(component, state, detail) {
    const card = document.getElementById(`health-${component}`);
    if (card) {
      const statusSpan = card.querySelector(".h-status");
      if (statusSpan) {
        statusSpan.textContent = detail ? `${state} (${detail})` : state;
        statusSpan.className = `h-status ${state.toLowerCase()}`;
      }
    }
  }

  drawAttitude() {
    if (!this.ctx || !this.canvas) return;
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const radius = Math.min(cx, cy) - 4;

    ctx.clearRect(0, 0, W, H);

    ctx.save();
    // Circular clip
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.clip();

    // Rotate for Roll
    ctx.translate(cx, cy);
    ctx.rotate((-this.roll * Math.PI) / 180.0);

    // Pitch shift (1.2 px per degree)
    const pitchOffset = Math.max(-radius, Math.min(radius, this.pitch * 1.2));

    // Sky
    ctx.fillStyle = "#0284c7";
    ctx.fillRect(-radius * 2, -radius * 2, radius * 4, radius * 2 + pitchOffset);

    // Ground
    ctx.fillStyle = "#78350f";
    ctx.fillRect(-radius * 2, pitchOffset, radius * 4, radius * 2);

    // Horizon Line
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-radius, pitchOffset);
    ctx.lineTo(radius, pitchOffset);
    ctx.stroke();

    // Pitch Ladder Lines (+10, -10 deg)
    ctx.strokeStyle = "rgba(255, 255, 255, 0.7)";
    ctx.lineWidth = 1;
    for (const deg of [-10, 10]) {
      const py = pitchOffset - deg * 1.2;
      ctx.beginPath();
      ctx.moveTo(-12, py);
      ctx.lineTo(12, py);
      ctx.stroke();
    }

    ctx.restore();

    // Outer Bezel Ring
    ctx.strokeStyle = "#212c3d";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();
  }
}

window.GCSTelemetryView = GCSTelemetryView;
