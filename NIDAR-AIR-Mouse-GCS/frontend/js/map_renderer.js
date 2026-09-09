/**
 * Interactive 2D Dynamic SLAM & Search Grid Map Canvas Renderer for NIDAR AirMouse GCS.
 * Supports pan, zoom, follow drone, search grid (A1-H8), occupancy grid walls, trail, and survivor pins.
 */
class GCSMapRenderer {
  constructor(canvasId, readoutId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.readout = document.getElementById(readoutId);

    // Map Dimensions (Arena 20m x 20m)
    this.arenaWidth = 20.0;
    this.arenaHeight = 20.0;
    this.cellSize = 2.5;
    this.numCols = 8;
    this.numRows = 8;
    this.rowLetters = "ABCDEFGH";

    // View Transformation
    this.scale = 25.0; // pixels per meter
    this.offsetX = 60.0;
    this.offsetY = 60.0;

    // State
    this.droneX = 1.25;
    this.droneY = 1.25;
    this.droneYaw = 0.0;
    this.trail = [];
    this.survivors = new Map();
    this.occupiedCells = [];
    this.freeCells = [];
    this.slamMode = "SIMULATION";

    // Toggles
    this.followDrone = false;
    this.showGrid = true;
    this.showTrail = true;

    // Mouse Interaction
    this.isDragging = false;
    this.lastMouseX = 0;
    this.lastMouseY = 0;

    this._initEvents();
    this._resizeCanvas();
    this.fitToView();
    this._startRenderLoop();
  }

  _initEvents() {
    window.addEventListener("resize", () => {
      this._resizeCanvas();
      this.fitToView();
    });

    const parent = this.canvas.parentElement;
    if (parent && window.ResizeObserver) {
      const ro = new ResizeObserver(() => {
        this._resizeCanvas();
      });
      ro.observe(parent);
    }

    this.canvas.addEventListener("mousedown", (e) => {
      this.isDragging = true;
      this.lastMouseX = e.clientX;
      this.lastMouseY = e.clientY;
    });

    window.addEventListener("mousemove", (e) => {
      // Coordinate hover readout
      const rect = this.canvas.getBoundingClientRect();
      if (
        e.clientX >= rect.left && e.clientX <= rect.right &&
        e.clientY >= rect.top && e.clientY <= rect.bottom
      ) {
        const px = e.clientX - rect.left;
        const py = e.clientY - rect.top;
        const [wx, wy] = this.screenToWorld(px, py);
        const cell = this.metricToGrid(wx, wy);
        if (this.readout) {
          this.readout.textContent = `X: ${wx.toFixed(2)}m | Y: ${wy.toFixed(2)}m | Grid: ${cell}`;
        }
      }

      if (this.isDragging) {
        const dx = e.clientX - this.lastMouseX;
        const dy = e.clientY - this.lastMouseY;
        this.offsetX += dx;
        this.offsetY += dy;
        this.lastMouseX = e.clientX;
        this.lastMouseY = e.clientY;
        this.followDrone = false;
        const chk = document.getElementById("chk-follow-drone");
        if (chk) chk.checked = false;
      }
    });

    window.addEventListener("mouseup", () => {
      this.isDragging = false;
    });

    this.canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      const rect = this.canvas.getBoundingClientRect();
      const px = e.clientX - rect.left;
      const py = e.clientY - rect.top;
      const [wx, wy] = this.screenToWorld(px, py);

      const factor = e.deltaY < 0 ? 1.15 : 0.87;
      this.scale = Math.max(8.0, Math.min(120.0, this.scale * factor));

      this.offsetX = px - wx * this.scale;
      this.offsetY = py + wy * this.scale;
    }, { passive: false });
  }

  _resizeCanvas() {
    const parent = this.canvas.parentElement;
    if (!parent) return;
    this.canvas.width = parent.clientWidth;
    this.canvas.height = parent.clientHeight;
  }

  fitToView() {
    const pad = 35;
    const w = this.canvas.width - pad * 2;
    const h = this.canvas.height - pad * 2;
    if (w <= 0 || h <= 0) return;

    this.scale = Math.min(w / this.arenaWidth, h / this.arenaHeight);
    const arenaPxW = this.arenaWidth * this.scale;
    const arenaPxH = this.arenaHeight * this.scale;
    this.offsetX = (this.canvas.width - arenaPxW) / 2;
    this.offsetY = (this.canvas.height + arenaPxH) / 2;
  }

  zoom(factor) {
    const cx = this.canvas.width / 2;
    const cy = this.canvas.height / 2;
    const [wx, wy] = this.screenToWorld(cx, cy);
    this.scale = Math.max(8.0, Math.min(120.0, this.scale * factor));
    this.offsetX = cx - wx * this.scale;
    this.offsetY = cy + wy * this.scale;
  }

  worldToScreen(x_m, y_m) {
    const px = this.offsetX + x_m * this.scale;
    const py = this.offsetY - y_m * this.scale; // Invert Y
    return [px, py];
  }

  screenToWorld(px, py) {
    const x_m = (px - this.offsetX) / this.scale;
    const y_m = (this.offsetY - py) / this.scale;
    return [x_m, y_m];
  }

  metricToGrid(x_m, y_m) {
    if (x_m < 0 || x_m > this.arenaWidth || y_m < 0 || y_m > this.arenaHeight) {
      return "--";
    }
    const colIdx = Math.max(0, Math.min(this.numCols - 1, Math.floor(x_m / this.cellSize)));
    const rowIdx = Math.max(0, Math.min(this.numRows - 1, Math.floor(y_m / this.cellSize)));
    const colChar = this.rowLetters[colIdx] || "A";
    return `${colChar}${rowIdx + 1}`;
  }

  updateDronePose(x, y, yaw) {
    this.droneX = x;
    this.droneY = y;
    this.droneYaw = yaw;

    if (this.trail.length === 0 || Math.hypot(this.trail[this.trail.length - 1][0] - x, this.trail[this.trail.length - 1][1] - y) > 0.15) {
      this.trail.push([x, y]);
      if (this.trail.length > 800) this.trail.shift();
    }

    if (this.followDrone) {
      const cx = this.canvas.width / 2;
      const cy = this.canvas.height / 2;
      this.offsetX = cx - x * this.scale;
      this.offsetY = cy + y * this.scale;
    }
  }

  updateMapData(mapData) {
    if (mapData.occupied_cells) {
      this.occupiedCells = mapData.occupied_cells;
    }
    if (mapData.free_cells) {
      this.freeCells = mapData.free_cells;
    }
    if (mapData.slam_mode) {
      this.slamMode = mapData.slam_mode;
    } else if (mapData.is_simulation !== undefined) {
      this.slamMode = mapData.is_simulation ? "SIMULATION" : "REALTIME";
    }
  }

  setSlamMode(mode) {
    this.slamMode = (mode || "SIMULATION").toUpperCase();
  }

  addSurvivor(survivor) {
    this.survivors.set(survivor.id, survivor);
  }

  focusOnSurvivor(x, y) {
    this.followDrone = false;
    const chk = document.getElementById("chk-follow-drone");
    if (chk) chk.checked = false;
    const cx = this.canvas.width / 2;
    const cy = this.canvas.height / 2;
    this.offsetX = cx - x * this.scale;
    this.offsetY = cy + y * this.scale;
  }

  reset() {
    this.trail = [];
    this.survivors.clear();
    this.occupiedCells = [];
    this.freeCells = [];
    this.fitToView();
  }

  _startRenderLoop() {
    const render = () => {
      this._renderFrame();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  _renderFrame() {
    const ctx = this.ctx;
    const W = this.canvas.width;
    const H = this.canvas.height;

    // 1. Clear background
    ctx.fillStyle = "#080a0e";
    ctx.fillRect(0, 0, W, H);

    // 2. Render Unexplored Arena Base
    const [p1x, p1y] = this.worldToScreen(0, this.arenaHeight);
    const [p2x, p2y] = this.worldToScreen(this.arenaWidth, 0);
    ctx.fillStyle = "#11151e";
    ctx.fillRect(p1x, p1y, p2x - p1x, p2y - p1y);

    // 3. Render Explored Free Cells
    if (this.freeCells.length > 0) {
      ctx.fillStyle = "#1e2736";
      const cellPx = Math.max(2, this.scale * 0.2);
      for (let i = 0; i < this.freeCells.length; i++) {
        const [fx, fy] = this.freeCells[i];
        const [spx, spy] = this.worldToScreen(fx, fy);
        ctx.fillRect(spx - cellPx/2, spy - cellPx/2, cellPx, cellPx);
      }
    }

    // 4. Render Search Grid Overlay (A1 - H8)
    if (this.showGrid) {
      this._drawSearchGrid(ctx);
    }

    // 5. Render Explored Wall / Obstacle Cells
    if (this.occupiedCells.length > 0) {
      ctx.fillStyle = "#00e5ff";
      const wallPx = Math.max(3, this.scale * 0.12);
      for (let i = 0; i < this.occupiedCells.length; i++) {
        const [wx, wy] = this.occupiedCells[i];
        const [spx, spy] = this.worldToScreen(wx, wy);
        ctx.fillRect(spx - wallPx/2, spy - wallPx/2, wallPx, wallPx);
      }
    }

    // 6. Render Flight Breadcrumbs Trail
    if (this.showTrail && this.trail.length > 1) {
      ctx.strokeStyle = "rgba(0, 229, 255, 0.75)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      const [startPx, startPy] = this.worldToScreen(this.trail[0][0], this.trail[0][1]);
      ctx.moveTo(startPx, startPy);
      for (let i = 1; i < this.trail.length; i++) {
        const [tpx, tpy] = this.worldToScreen(this.trail[i][0], this.trail[i][1]);
        ctx.lineTo(tpx, tpy);
      }
      ctx.stroke();
    }

    // 7. Render Survivor Markers (S01 - S06)
    this.survivors.forEach((surv) => {
      this._drawSurvivorPin(ctx, surv);
    });

    // 8. Render Drone Marker & Heading Cone
    this._drawDrone(ctx);

    // 9. Render SLAM Mode Canvas Watermark
    ctx.save();
    ctx.font = "bold 9px 'Segoe UI', Inter, sans-serif";
    if (this.slamMode === "REALTIME") {
      ctx.fillStyle = "rgba(6, 78, 59, 0.85)";
      ctx.strokeStyle = "#059669";
      ctx.lineWidth = 1;
      ctx.fillRect(8, 8, 195, 20);
      ctx.strokeRect(8, 8, 195, 20);
      ctx.fillStyle = "#34d399";
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      ctx.fillText("📡 REAL-TIME SLAM (LIVE LIDAR)", 14, 18);
    } else {
      ctx.fillStyle = "rgba(12, 45, 72, 0.85)";
      ctx.strokeStyle = "#0284c7";
      ctx.lineWidth = 1;
      ctx.fillRect(8, 8, 205, 20);
      ctx.strokeRect(8, 8, 205, 20);
      ctx.fillStyle = "#38bdf8";
      ctx.textAlign = "left";
      ctx.textBaseline = "middle";
      ctx.fillText("🎮 SIMULATION SLAM (RAYCASTING)", 14, 18);
    }
    ctx.restore();
  }

  _drawSearchGrid(ctx) {
    ctx.strokeStyle = "rgba(45, 55, 72, 0.7)";
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);

    for (let c = 1; c < this.numCols; c++) {
      const xm = c * this.cellSize;
      const [sx1, sy1] = this.worldToScreen(xm, 0);
      const [sx2, sy2] = this.worldToScreen(xm, this.arenaHeight);
      ctx.beginPath();
      ctx.moveTo(sx1, sy1);
      ctx.lineTo(sx2, sy2);
      ctx.stroke();
    }

    for (let r = 1; r < this.numRows; r++) {
      const ym = r * this.cellSize;
      const [sx1, sy1] = this.worldToScreen(0, ym);
      const [sx2, sy2] = this.worldToScreen(this.arenaWidth, ym);
      ctx.beginPath();
      ctx.moveTo(sx1, sy1);
      ctx.lineTo(sx2, sy2);
      ctx.stroke();
    }
    ctx.setLineDash([]);

    // Cell Labels (e.g. B3)
    ctx.font = "9px Consolas, monospace";
    ctx.fillStyle = "rgba(100, 116, 139, 0.4)";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    for (let r = 0; r < this.numRows; r++) {
      for (let c = 0; c < this.numCols; c++) {
        const cxm = (c + 0.5) * this.cellSize;
        const cym = (r + 0.5) * this.cellSize;
        const [scx, scy] = this.worldToScreen(cxm, cym);
        ctx.fillText(`${this.rowLetters[c]}${r + 1}`, scx, scy);
      }
    }

    // Boundary Rect
    const [p1x, p1y] = this.worldToScreen(0, this.arenaHeight);
    const [p2x, p2y] = this.worldToScreen(this.arenaWidth, 0);
    ctx.strokeStyle = "#00e5ff";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(p1x, p1y, p2x - p1x, p2y - p1y);
  }

  _drawSurvivorPin(ctx, surv) {
    const [sx, sy] = this.worldToScreen(surv.local_x, surv.local_y);

    // Glowing Pulse
    ctx.fillStyle = "rgba(0, 230, 118, 0.25)";
    ctx.beginPath();
    ctx.arc(sx, sy, 14, 0, Math.PI * 2);
    ctx.fill();

    // Center Pin
    ctx.fillStyle = "#00e676";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(sx, sy, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Label Tag
    const tagText = `${surv.id} (${Math.round(surv.confidence * 100)}%) [${surv.grid_cell}]`;
    ctx.font = "bold 9px Segoe UI, sans-serif";
    ctx.fillStyle = "#00e676";
    ctx.fillRect(sx + 10, sy - 14, ctx.measureText(tagText).width + 8, 16);

    ctx.fillStyle = "#000000";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText(tagText, sx + 14, sy - 6);
  }

  _drawDrone(ctx) {
    const [dx, dy] = this.worldToScreen(this.droneX, this.droneY);
    const yawRad = (this.droneYaw * Math.PI) / 180.0;

    // 1. Heading Sensor Cone (60 deg forward spread)
    const coneLen = 28;
    const spread = (30 * Math.PI) / 180.0;

    ctx.fillStyle = "rgba(255, 234, 0, 0.2)";
    ctx.beginPath();
    ctx.moveTo(dx, dy);
    ctx.lineTo(dx + Math.cos(yawRad - spread) * coneLen, dy - Math.sin(yawRad - spread) * coneLen);
    ctx.lineTo(dx + Math.cos(yawRad + spread) * coneLen, dy - Math.sin(yawRad + spread) * coneLen);
    ctx.closePath();
    ctx.fill();

    // 2. Drone Chassis
    ctx.fillStyle = "#ffea00";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(dx, dy, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // 3. Heading Vector Line
    ctx.strokeStyle = "#ff1744";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(dx, dy);
    ctx.lineTo(dx + Math.cos(yawRad) * 16, dy - Math.sin(yawRad) * 16);
    ctx.stroke();
  }
}

window.GCSMapRenderer = GCSMapRenderer;
