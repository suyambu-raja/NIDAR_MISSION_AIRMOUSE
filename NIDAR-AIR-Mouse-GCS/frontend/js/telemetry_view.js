/**
 * Telemetry, Power, Artificial Horizon, and Sensor Matrix View Controller for NIDAR AirMouse GCS.
 */
class GCSTelemetryView {
  constructor() {
    this.canvas = document.getElementById("attitude-canvas");
    this.ctx = this.canvas ? this.canvas.getContext("2d") : null;
    this.roll = 0.0;
    this.pitch = 0.0;
  }

  updateTelemetry(telem) {
    // 1. Text Metrics
    const setElem = (id, txt) => {
      const el = document.getElementById(id);
      if (el) el.textContent = txt;
    };

    setElem("val-altitude", telem.altitude.toFixed(2));
    setElem("val-speed", telem.velocity.toFixed(2));
    setElem("val-heading", String(Math.round(telem.heading)).padStart(3, "0"));
    setElem("val-roll-pitch", `${telem.roll >= 0 ? "+" : ""}${telem.roll.toFixed(1)} / ${telem.pitch >= 0 ? "+" : ""}${telem.pitch.toFixed(1)}`);
    setElem("val-pose", `(${telem.x.toFixed(2)}, ${telem.y.toFixed(2)})`);
    setElem("val-grid", telem.grid_cell || "A1");

    // Battery
    const battFill = document.getElementById("battery-bar-fill");
    const battText = document.getElementById("battery-text");
    if (battFill && battText) {
      const pct = Math.max(0, Math.min(100, Math.round(telem.battery_pct)));
      battFill.style.width = `${pct}%`;
      battText.textContent = `${pct}% (${telem.battery_v.toFixed(1)}V)`;

      if (pct < 20) {
        battFill.style.backgroundColor = "var(--accent-red)";
      } else if (pct < 40) {
        battFill.style.backgroundColor = "var(--accent-yellow)";
      } else {
        battFill.style.backgroundColor = "var(--accent-green)";
      }
    }

    // 2. Artificial Horizon Attitude Indicator
    this.roll = telem.roll;
    this.pitch = telem.pitch;
    this.drawAttitude();
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

    // Pitch shift (1.5 px per degree)
    const pitchOffset = Math.max(-radius, Math.min(radius, this.pitch * 1.5));

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
      const py = pitchOffset - deg * 1.5;
      ctx.beginPath();
      ctx.moveTo(-15, py);
      ctx.lineTo(15, py);
      ctx.stroke();
    }

    ctx.restore();

    // Outer Bezel Ring
    ctx.strokeStyle = "#334155";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();

    // Fixed Aircraft Wings Symbol (Yellow)
    ctx.strokeStyle = "#ffea00";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(cx - 20, cy);
    ctx.lineTo(cx - 6, cy);
    ctx.moveTo(cx + 6, cy);
    ctx.lineTo(cx + 20, cy);
    ctx.stroke();

    ctx.fillStyle = "#ffea00";
    ctx.beginPath();
    ctx.arc(cx, cy, 2, 0, Math.PI * 2);
    ctx.fill();
  }
}

window.GCSTelemetryView = GCSTelemetryView;
