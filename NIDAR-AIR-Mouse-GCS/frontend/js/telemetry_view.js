/**
 * Authentic Mission Planner 1.3.83 Primary Flight Display (PFD) HUD & Telemetry Controller.
 * Replicates the exact aviation artificial horizon, pitch ladder, compass tape,
 * airspeed/altimeter tapes, and the 6 color-coded Quick Tab telemetry cards.
 */
class GCSTelemetryView {
  constructor() {
    this.canvas = document.getElementById("attitude-canvas");
    this.ctx = this.canvas ? this.canvas.getContext("2d") : null;

    // Kinematics State
    this.roll = 0.0;
    this.pitch = 0.0;
    this.heading = 0.0;
    this.altitude = 0.0;
    this.velocity = 0.0;
    this.climbRate = 0.0;
    this.armed = false;
    this.flightMode = "DISARMED";
    this.batteryV = 16.6;
    this.batteryPct = 100.0;
    this.elapsedTime = "00:00:00";

    this._initCanvas();
    this.drawAttitude();
  }

  _initCanvas() {
    if (!this.canvas) return;
    const parent = this.canvas.parentElement;
    if (parent) {
      this.canvas.width = parent.clientWidth || 370;
      this.canvas.height = parent.clientHeight || 260;
    }
    window.addEventListener("resize", () => {
      if (this.canvas && parent) {
        this.canvas.width = parent.clientWidth || 370;
        this.canvas.height = parent.clientHeight || 260;
        this.drawAttitude();
      }
    });
  }

  updateTelemetry(telem) {
    this.altitude = Number(telem.altitude || 0.0);
    this.velocity = Number(telem.velocity || 0.0);
    this.heading = Number(telem.heading || 0.0);
    this.roll = Number(telem.roll || 0.0);
    this.pitch = Number(telem.pitch || 0.0);
    this.climbRate = Number(telem.vertical_speed || telem.climb_rate || 0.0);
    this.armed = Boolean(telem.armed);
    this.flightMode = telem.flight_mode || (this.armed ? "GUIDED_NOGPS" : "DISARMED");
    this.batteryV = Number(telem.battery_v || 16.6);
    this.batteryPct = Number(telem.battery_pct || 100.0);

    // Update Quick Tab 6 Digital Readouts (Exact Mission Planner Palette)
    const setElem = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.textContent = val;
    };

    setElem("val-altitude", this.altitude.toFixed(2));
    setElem("val-speed", this.velocity.toFixed(2));
    setElem("val-dist-wp", (telem.dist_to_wp !== undefined ? Number(telem.dist_to_wp) : 0.0).toFixed(2));
    setElem("val-heading", this.heading.toFixed(2));
    setElem("val-vert-speed", this.climbRate.toFixed(2));
    setElem("val-dist-mav", (telem.dist_to_mav !== undefined ? Number(telem.dist_to_mav) : 0.0).toFixed(2));

    // Footer readouts
    setElem("val-pose", `(${(telem.x || 0).toFixed(2)}, ${(telem.y || 0).toFixed(2)})`);
    setElem("val-grid", telem.grid_cell || "A1");
    setElem("battery-text", `${Math.round(this.batteryPct)}% (${this.batteryV.toFixed(1)}V)`);

    // Flight mode badges
    const modeBadge = document.getElementById("flight-mode-badge");
    if (modeBadge) {
      modeBadge.textContent = this.armed ? (telem.flight_mode || "ARMED") : "DISARMED";
      modeBadge.style.color = this.armed ? "#2ecc71" : "#e74c3c";
    }

    // Battery bar in Gauges Tab
    const battFill = document.getElementById("battery-bar-fill");
    const battReadout = document.getElementById("battery-dial-readout");
    if (battFill) battFill.style.width = `${Math.min(100, Math.max(0, this.batteryPct))}%`;
    if (battReadout) battReadout.textContent = `${this.batteryV.toFixed(1)}V`;

    // Compass needle in Gauges Tab
    const compassNeedle = document.getElementById("compass-needle");
    const headingReadout = document.getElementById("gauge-heading-text");
    if (compassNeedle) compassNeedle.style.transform = `rotate(${-this.heading}deg)`;
    if (headingReadout) headingReadout.textContent = `${String(Math.round(this.heading)).padStart(3, "0")}°`;

    // Render Mission Planner PFD HUD
    this.drawAttitude();
  }

  setClock(clockStr) {
    this.elapsedTime = clockStr;
    const el = document.getElementById("pfd-overlay-clock");
    if (el) el.textContent = clockStr;
  }

  updateFailsafe(fs) {
    // Handled in app.js
  }

  updateAutonomy(auto) {
    // Handled in app.js
  }

  /**
   * Renders the authentic Mission Planner Primary Flight Display (PFD) HUD.
   */
  drawAttitude() {
    if (!this.ctx || !this.canvas) return;
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;
    const cx = W / 2;
    const cy = H / 2;

    ctx.clearRect(0, 0, W, H);

    // 1. SKY & GROUND RECTANGLE TRANSFORMED BY ROLL & PITCH
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, W, H);
    ctx.clip();

    ctx.translate(cx, cy);
    ctx.rotate((-this.roll * Math.PI) / 180.0);

    // Pitch pixels (approx 2.4 px per degree of pitch)
    const pitchOffset = Math.max(-H * 1.5, Math.min(H * 1.5, this.pitch * 2.4));

    // Sky: Realistic Aviator Blue
    const skyGrad = ctx.createLinearGradient(0, -H * 2, 0, pitchOffset);
    skyGrad.addColorStop(0, "#1266a8");
    skyGrad.addColorStop(1, "#2189d9");
    ctx.fillStyle = skyGrad;
    ctx.fillRect(-W * 2, -H * 2, W * 4, H * 2 + pitchOffset);

    // Ground: Realistic Earth Olive/Brown
    const gndGrad = ctx.createLinearGradient(0, pitchOffset, 0, H * 2);
    gndGrad.addColorStop(0, "#5b8422");
    gndGrad.addColorStop(1, "#446417");
    ctx.fillStyle = gndGrad;
    ctx.fillRect(-W * 2, pitchOffset, W * 4, H * 2);

    // Horizon Line (Crisp White)
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(-W * 2, pitchOffset);
    ctx.lineTo(W * 2, pitchOffset);
    ctx.stroke();

    // 2. PITCH LADDER RUNGS (-20, -10, 0, 10, 20)
    ctx.lineWidth = 1.8;
    ctx.font = "bold 11px Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    for (let deg = -30; deg <= 30; deg += 5) {
      if (deg === 0) continue;
      const yPos = pitchOffset - deg * 2.4;
      if (yPos < -H || yPos > H) continue;

      const isMajor = deg % 10 === 0;
      const rungWidth = isMajor ? 54 : 26;

      ctx.strokeStyle = "rgba(255, 255, 255, 0.85)";
      ctx.beginPath();

      if (deg > 0) {
        // Positive pitch: solid line with down-tick at ends
        ctx.moveTo(-rungWidth / 2, yPos);
        ctx.lineTo(rungWidth / 2, yPos);
        ctx.moveTo(-rungWidth / 2, yPos);
        ctx.lineTo(-rungWidth / 2, yPos + 4);
        ctx.moveTo(rungWidth / 2, yPos);
        ctx.lineTo(rungWidth / 2, yPos + 4);
      } else {
        // Negative pitch: dashed lines
        ctx.setLineDash([4, 4]);
        ctx.moveTo(-rungWidth / 2, yPos);
        ctx.lineTo(rungWidth / 2, yPos);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(-rungWidth / 2, yPos);
        ctx.lineTo(-rungWidth / 2, yPos - 4);
        ctx.moveTo(rungWidth / 2, yPos);
        ctx.lineTo(rungWidth / 2, yPos - 4);
      }
      ctx.stroke();

      // Degree labels beside major rungs
      if (isMajor) {
        ctx.fillStyle = "#ffffff";
        ctx.fillText(String(Math.abs(deg)), -rungWidth / 2 - 14, yPos);
        ctx.fillText(String(Math.abs(deg)), rungWidth / 2 + 14, yPos);
      }
    }

    ctx.restore();

    // 3. ROLL ANGLE INDICATOR ARC AT TOP
    ctx.save();
    ctx.translate(cx, cy);
    const arcRadius = Math.min(cx, cy) - 25;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.75)";
    ctx.lineWidth = 1.5;

    // Arc
    ctx.beginPath();
    ctx.arc(0, 0, arcRadius, (-140 * Math.PI) / 180, (-40 * Math.PI) / 180);
    ctx.stroke();

    // Roll tick marks at -60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60 deg
    const rollTicks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
    for (const ang of rollTicks) {
      const rad = ((ang - 90) * Math.PI) / 180;
      const isMajor = ang % 30 === 0 || ang === 0;
      const len = isMajor ? 8 : 4;
      const x1 = Math.cos(rad) * arcRadius;
      const y1 = Math.sin(rad) * arcRadius;
      const x2 = Math.cos(rad) * (arcRadius + len);
      const y2 = Math.sin(rad) * (arcRadius + len);

      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }

    // Roll Pointer Triangle (rotates with aircraft roll)
    ctx.rotate((-this.roll * Math.PI) / 180.0);
    ctx.fillStyle = "#f39c12";
    ctx.beginPath();
    ctx.moveTo(0, -arcRadius + 2);
    ctx.lineTo(-6, -arcRadius + 12);
    ctx.lineTo(6, -arcRadius + 12);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // 4. CENTER AIRCRAFT WATERLINE CROSSHAIR (ORANGE / RED)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = "#e74c3c";
    ctx.fillStyle = "#e74c3c";
    ctx.lineWidth = 3.0;

    // Left wing
    ctx.beginPath();
    ctx.moveTo(-45, 0);
    ctx.lineTo(-14, 0);
    ctx.lineTo(-14, 8);
    ctx.stroke();

    // Right wing
    ctx.beginPath();
    ctx.moveTo(45, 0);
    ctx.lineTo(14, 0);
    ctx.lineTo(14, 8);
    ctx.stroke();

    // Center dot / pip
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    // 5. TOP COMPASS HEADING TAPE
    this._drawHeadingTape(ctx, W);

    // 6. LEFT AIRSPEED TAPE & RIGHT ALTITUDE TAPE
    this._drawAirspeedTape(ctx, H);
    this._drawAltitudeTape(ctx, W, H);

    // 7. CENTER STATUS TEXT: "DISARMED" (RED) / "ARMED" (GREEN)
    ctx.save();
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.font = "900 24px 'Segoe UI', Arial, sans-serif";
    ctx.shadowColor = "#000000";
    ctx.shadowBlur = 6;
    ctx.shadowOffsetX = 0;
    ctx.shadowOffsetY = 2;

    if (!this.armed) {
      ctx.fillStyle = "#ff2222";
      ctx.fillText("DISARMED", cx, cy - 25);
    } else {
      ctx.fillStyle = "#00ff00";
      ctx.fillText("ARMED", cx, cy - 25);
    }
    ctx.restore();

    // 8. HUD FLIGHT OVERLAY TEXTS (EXACT MISSION PLANNER HUD FOOTER)
    ctx.save();
    ctx.font = "bold 10.5px Consolas, monospace";
    ctx.shadowColor = "#000000";
    ctx.shadowBlur = 4;

    // Bottom Left
    ctx.textAlign = "left";
    ctx.fillStyle = "#ffffff";
    ctx.fillText("AS 0.0m/s", 8, H - 38);
    ctx.fillText(`GS ${this.velocity.toFixed(1)}m/s`, 8, H - 24);

    // Bottom Center (Flight Status Line 1)
    ctx.textAlign = "center";
    if (!this.armed) {
      ctx.fillStyle = "#ff3333";
      ctx.fillText("Not Ready to Arm", cx, H - 34);
    } else {
      ctx.fillStyle = "#00ff66";
      ctx.fillText(`ARMED - ${this.flightMode}`, cx, H - 34);
    }

    // Bottom Center (Flight Status Line 2: Battery & Sensors)
    ctx.fillStyle = "#ffea00";
    const batStr = `Bat ${this.batteryV.toFixed(1)}V 0.0A ${Math.round(this.batteryPct)}%`;
    ctx.fillText(`${batStr}  EKF Vibe  GPS: No GPS`, cx, H - 18);

    // Bottom Right
    ctx.textAlign = "right";
    ctx.fillStyle = "#ffffff";
    ctx.fillText("Unknown", W - 8, H - 38);
    ctx.fillText("0m>0", W - 8, H - 24);

    ctx.restore();
  }

  _drawHeadingTape(ctx, W) {
    ctx.save();
    const tapeY = 0;
    const tapeH = 22;

    // Background translucent bar
    ctx.fillStyle = "rgba(0, 0, 0, 0.55)";
    ctx.fillRect(0, tapeY, W, tapeH);

    ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, tapeH);
    ctx.lineTo(W, tapeH);
    ctx.stroke();

    // Center Index Arrow (Red)
    ctx.fillStyle = "#e74c3c";
    ctx.beginPath();
    ctx.moveTo(W / 2 - 5, 0);
    ctx.lineTo(W / 2 + 5, 0);
    ctx.lineTo(W / 2, 8);
    ctx.closePath();
    ctx.fill();

    // Heading labels (every 15 deg)
    ctx.font = "bold 9.5px Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    const pxPerDeg = 2.8;
    const centerHdg = (this.heading % 360 + 360) % 360;

    for (let deg = -180; deg <= 540; deg += 5) {
      const normDeg = (deg % 360 + 360) % 360;
      const diff = deg - centerHdg;
      const x = W / 2 + diff * pxPerDeg;

      if (x < -20 || x > W + 20) continue;

      const is15 = normDeg % 15 === 0;
      const tickH = is15 ? 6 : 3;

      ctx.strokeStyle = "#ffffff";
      ctx.beginPath();
      ctx.moveTo(x, tapeH - tickH);
      ctx.lineTo(x, tapeH);
      ctx.stroke();

      if (is15) {
        let label = String(normDeg).padStart(3, "0");
        if (normDeg === 0) label = "N";
        else if (normDeg === 45) label = "NE";
        else if (normDeg === 90) label = "E";
        else if (normDeg === 135) label = "SE";
        else if (normDeg === 180) label = "S";
        else if (normDeg === 225) label = "SW";
        else if (normDeg === 270) label = "W";
        else if (normDeg === 315) label = "NW";

        ctx.fillStyle = normDeg % 90 === 0 ? "#f1c40f" : "#ffffff";
        ctx.fillText(label, x, 10);
      }
    }

    ctx.restore();
  }

  _drawAirspeedTape(ctx, H) {
    ctx.save();
    const tapeW = 34;
    const startY = 30;
    const endY = H - 46;
    const tapeH = endY - startY;
    const cy = startY + tapeH / 2;

    ctx.fillStyle = "rgba(0, 0, 0, 0.4)";
    ctx.fillRect(0, startY, tapeW, tapeH);

    ctx.strokeStyle = "rgba(255, 255, 255, 0.3)";
    ctx.strokeRect(0, startY, tapeW, tapeH);

    // Ticks: 10, 5, 0, -5, -10
    ctx.font = "bold 9px Consolas, monospace";
    ctx.fillStyle = "#ffffff";
    ctx.textAlign = "left";

    const marks = [10, 5, 0, -5, -10];
    for (const m of marks) {
      const y = cy - (m - this.velocity) * 4.5;
      if (y >= startY && y <= endY) {
        ctx.strokeStyle = "rgba(255, 255, 255, 0.6)";
        ctx.beginPath();
        ctx.moveTo(tapeW - 8, y);
        ctx.lineTo(tapeW, y);
        ctx.stroke();

        ctx.fillText(String(m), 4, y + 3);
      }
    }

    // Current Speed Readout Box
    ctx.fillStyle = "#000000";
    ctx.strokeStyle = "#39e75f";
    ctx.lineWidth = 1.5;
    ctx.fillRect(1, cy - 9, tapeW + 8, 18);
    ctx.strokeRect(1, cy - 9, tapeW + 8, 18);

    ctx.fillStyle = "#39e75f";
    ctx.font = "bold 10px Consolas, monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${this.velocity.toFixed(0)}m/s`, tapeW / 2 + 4, cy + 4);

    ctx.restore();
  }

  _drawAltitudeTape(ctx, W, H) {
    ctx.save();
    const tapeW = 34;
    const startX = W - tapeW;
    const startY = 30;
    const endY = H - 46;
    const tapeH = endY - startY;
    const cy = startY + tapeH / 2;

    ctx.fillStyle = "rgba(0, 0, 0, 0.4)";
    ctx.fillRect(startX, startY, tapeW, tapeH);

    ctx.strokeStyle = "rgba(255, 255, 255, 0.3)";
    ctx.strokeRect(startX, startY, tapeW, tapeH);

    // Ticks: 10, 5, 0, -5, -10
    ctx.font = "bold 9px Consolas, monospace";
    ctx.fillStyle = "#ffffff";
    ctx.textAlign = "right";

    const marks = [10, 5, 0, -5, -10];
    for (const m of marks) {
      const y = cy - (m - this.altitude) * 4.5;
      if (y >= startY && y <= endY) {
        ctx.strokeStyle = "rgba(255, 255, 255, 0.6)";
        ctx.beginPath();
        ctx.moveTo(startX, y);
        ctx.lineTo(startX + 8, y);
        ctx.stroke();

        ctx.fillText(String(m), W - 4, y + 3);
      }
    }

    // Current Altitude Readout Box
    ctx.fillStyle = "#000000";
    ctx.strokeStyle = "#00e5ff";
    ctx.lineWidth = 1.5;
    ctx.fillRect(startX - 8, cy - 9, tapeW + 7, 18);
    ctx.strokeRect(startX - 8, cy - 9, tapeW + 7, 18);

    ctx.fillStyle = "#00e5ff";
    ctx.font = "bold 10px Consolas, monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${this.altitude.toFixed(0)}m`, startX + tapeW / 2 - 4, cy + 4);

    ctx.restore();
  }
}

window.GCSTelemetryView = GCSTelemetryView;
