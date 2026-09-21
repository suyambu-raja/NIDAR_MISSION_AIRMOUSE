/**
 * Interactive 2D Dynamic SLAM & Search Grid Map Canvas Renderer for NIDAR M2 Rescue Drone.
 * Supports pan, zoom, follow drone, search grid (A1-H8), occupancy grid walls,
 * planned A* path, failsafe path, exploration frontiers, corridor boundaries, trail, and survivor pins.
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

    // Drone State
    this.droneX = 1.25;
    this.droneY = 1.25;
    this.droneYaw = 0.0;
    this.trail = [];

    // Navigation & Map Overlays
    this.survivors = new Map();
    this.occupiedCells = [];
    this.freeCells = [];
    this.plannedPath = [];
    this.failsafePath = [];
    this.frontiers = [];
    this.corridors = [];
    this.slamMode = "REALTIME";

    // Layer Toggles
    this.followDrone = false;
    this.showGrid = true;
    this.showTrail = true;
    this.showPath = true;
    this.showFrontiers = true;
    this.showCorridors = true;

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
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const [worldX, worldY] = this.screenToWorld(mouseX, mouseY);
      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
      this.scale = Math.max(8.0, Math.min(120.0, this.scale * zoomFactor));

      this.offsetX = mouseX - worldX * this.scale;
      this.offsetY = mouseY + worldY * this.scale;
    }, { passive: false });

    // Wire Layer Checkboxes
    const bindToggle = (id, prop) => {
      const el = document.getElementById(id);
      if (el) {
        el.addEventListener("change", (e) => {
          this[prop] = e.target.checked;
        });
      }
    };
    bindToggle("chk-show-grid", "showGrid");
    bindToggle("chk-show-trail", "showTrail");
    bindToggle("chk-show-path", "showPath");
    bindToggle("chk-show-frontiers", "showFrontiers");
    bindToggle("chk-show-corridors", "showCorridors");

    const chkFollow = document.getElementById("chk-follow-drone");
    if (chkFollow) {
      chkFollow.addEventListener("change", (e) => {
        this.followDrone = e.target.checked;
      });
    }

    const btnFit = document.getElementById("btn-map-fit");
    if (btnFit) btnFit.addEventListener("click", () => this.fitToView());

    const btnZoomIn = document.getElementById("btn-map-zoom-in");
    if (btnZoomIn) btnZoomIn.addEventListener("click", () => this.zoom(1.2));

    const btnZoomOut = document.getElementById("btn-map-zoom-out");
    if (btnZoomOut) btnZoomOut.addEventListener("click", () => this.zoom(0.8));
  }

  _resizeCanvas() {
    if (!this.canvas) return;
    const parent = this.canvas.parentElement;
    if (parent) {
      this.canvas.width = parent.clientWidth;
      this.canvas.height = parent.clientHeight;
    }
  }

  zoom(factor) {
    const cx = this.canvas.width / 2;
    const cy = this.canvas.height / 2;
    const [wx, wy] = this.screenToWorld(cx, cy);
    this.scale = Math.max(8.0, Math.min(120.0, this.scale * factor));
    this.offsetX = cx - wx * this.scale;
    this.offsetY = cy + wy * this.scale;
  }

  fitToView() {
    if (!this.canvas) return;
    const margin = 30;
    const availW = this.canvas.width - margin * 2;
    const availH = this.canvas.height - margin * 2;
    const scaleX = availW / this.arenaWidth;
    const scaleY = availH / this.arenaHeight;
    this.scale = Math.min(scaleX, scaleY);
    this.offsetX = margin + (availW - this.arenaWidth * this.scale) / 2;
    this.offsetY = margin + this.arenaHeight * this.scale + (availH - this.arenaHeight * this.scale) / 2;
  }

  worldToScreen(worldX, worldY) {
    const screenX = this.offsetX + worldX * this.scale;
    const screenY = this.offsetY - worldY * this.scale;
    return [screenX, screenY];
  }

  screenToWorld(screenX, screenY) {
    const worldX = (screenX - this.offsetX) / this.scale;
    const worldY = (this.offsetY - screenY) / this.scale;
    return [worldX, worldY];
  }

  metricToGrid(x, y) {
    if (x < 0 || x >= this.arenaWidth || y < 0 || y >= this.arenaHeight) {
      return "--";
    }
    const colIdx = Math.min(this.numCols - 1, Math.floor(x / this.cellSize));
    const rowIdx = Math.min(this.numRows - 1, Math.floor(y / this.cellSize));
    return `${this.rowLetters[colIdx]}${rowIdx + 1}`;
  }

  updateDronePose(x, y, heading) {
    this.droneX = x;
    this.droneY = y;
    this.droneYaw = heading;

    // Append to flight trail
    if (
      this.trail.length === 0 ||
      Math.hypot(x - this.trail[this.trail.length - 1][0], y - this.trail[this.trail.length - 1][1]) > 0.15
    ) {
      this.trail.push([x, y]);
      if (this.trail.length > 500) this.trail.shift();
    }

    if (this.followDrone) {
      const cx = this.canvas.width / 2;
      const cy = this.canvas.height / 2;
      this.offsetX = cx - this.droneX * this.scale;
      this.offsetY = cy + this.droneY * this.scale;
    }
  }

  updateMapData(mapData) {
    if (mapData.occupied_cells) {
      this.occupiedCells = mapData.occupied_cells;
    }
    if (mapData.free_cells) {
      this.freeCells = mapData.free_cells;
    }
    if (mapData.path_overlay && Array.isArray(mapData.path_overlay)) {
      this.plannedPath = mapData.path_overlay;
    }
    if (mapData.frontiers && Array.isArray(mapData.frontiers)) {
      this.frontiers = mapData.frontiers;
    }
    if (mapData.slam_mode) {
      this.slamMode = mapData.slam_mode;
    }
  }

  setPlannedPath(path) {
    this.plannedPath = path || [];
  }

  setFailsafePath(path) {
    this.failsafePath = path || [];
  }

  setFrontiers(frontiers) {
    this.frontiers = frontiers || [];
  }

  setCorridors(corridors) {
    this.corridors = corridors || [];
  }

  setSlamMode(mode) {
    this.slamMode = (mode || "REALTIME").toUpperCase();
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
    this.plannedPath = [];
    this.failsafePath = [];
    this.frontiers = [];
    this.corridors = [];
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
    ctx.fillStyle = "#050810";
    ctx.fillRect(0, 0, W, H);

    // 2. Render Unexplored Arena Base
    const [p1x, p1y] = this.worldToScreen(0, this.arenaHeight);
    const [p2x, p2y] = this.worldToScreen(this.arenaWidth, 0);
    ctx.fillStyle = "#0a0e1a";
    ctx.fillRect(p1x, p1y, p2x - p1x, p2y - p1y);

    // 3. Render Explored Free Space
    if (this.freeCells.length > 0) {
      ctx.fillStyle = "#162032";
      const cellPx = Math.max(2, this.scale * 0.18);
      for (let i = 0; i < this.freeCells.length; i++) {
        const [fx, fy] = this.freeCells[i];
        const [spx, spy] = this.worldToScreen(fx, fy);
        ctx.fillRect(spx - cellPx / 2, spy - cellPx / 2, cellPx, cellPx);
      }
    }

    // 4. Render Corridor & Room Region Boundaries (from corridor_classifier)
    if (this.showCorridors && this.corridors.length > 0) {
      this._drawCorridorRegions(ctx);
    }

    // 5. Render Search Grid Overlay (A1 - H8)
    if (this.showGrid) {
      this._drawSearchGrid(ctx);
    }

    // 6. Render Explored Obstacles & Walls (0.05m cells)
    if (this.occupiedCells.length > 0) {
      ctx.fillStyle = "#38bdf8";
      const wallPx = Math.max(3, this.scale * 0.12);
      for (let i = 0; i < this.occupiedCells.length; i++) {
        const [wx, wy] = this.occupiedCells[i];
        const [spx, spy] = this.worldToScreen(wx, wy);
        ctx.fillRect(spx - wallPx / 2, spy - wallPx / 2, wallPx, wallPx);
      }
    }

    // 7. Render Flight Breadcrumbs Trail
    if (this.showTrail && this.trail.length > 1) {
      ctx.strokeStyle = "rgba(0, 229, 255, 0.4)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      const [startPx, startPy] = this.worldToScreen(this.trail[0][0], this.trail[0][1]);
      ctx.moveTo(startPx, startPy);
      for (let i = 1; i < this.trail.length; i++) {
        const [tpx, tpy] = this.worldToScreen(this.trail[i][0], this.trail[i][1]);
        ctx.lineTo(tpx, tpy);
      }
      ctx.stroke();
    }

    // 8. Render Planned A* Path (cyan line + waypoints)
    if (this.showPath && this.plannedPath.length > 1) {
      this._drawPlannedPath(ctx);
    }

    // 9. Render Failsafe Path (amber line)
    if (this.showPath && this.failsafePath.length > 1) {
      this._drawFailsafePath(ctx);
    }

    // 10. Render Exploration Frontiers (green points)
    if (this.showFrontiers && this.frontiers.length > 0) {
      this._drawFrontiers(ctx);
    }

    // 11. Render Survivor Markers (S01, S02, ...)
    this.survivors.forEach((surv) => {
      this._drawSurvivorPin(ctx, surv);
    });

    // 12. Render Drone Marker & Heading Cone
    this._drawDrone(ctx);
  }

  _drawPlannedPath(ctx) {
    ctx.save();
    ctx.strokeStyle = "#00e5ff";
    ctx.lineWidth = 2.5;
    ctx.shadowColor = "rgba(0, 229, 255, 0.8)";
    ctx.shadowBlur = 8;
    ctx.beginPath();

    const [firstX, firstY] = this.worldToScreen(this.plannedPath[0][0], this.plannedPath[0][1]);
    ctx.moveTo(firstX, firstY);
    for (let i = 1; i < this.plannedPath.length; i++) {
      const [px, py] = this.worldToScreen(this.plannedPath[i][0], this.plannedPath[i][1]);
      ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Draw waypoints
    ctx.fillStyle = "#ffffff";
    for (let i = 0; i < this.plannedPath.length; i++) {
      const [px, py] = this.worldToScreen(this.plannedPath[i][0], this.plannedPath[i][1]);
      ctx.beginPath();
      ctx.arc(px, py, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  _drawFailsafePath(ctx) {
    ctx.save();
    ctx.strokeStyle = "#f97316";
    ctx.lineWidth = 2.0;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();

    const [firstX, firstY] = this.worldToScreen(this.failsafePath[0][0], this.failsafePath[0][1]);
    ctx.moveTo(firstX, firstY);
    for (let i = 1; i < this.failsafePath.length; i++) {
      const [px, py] = this.worldToScreen(this.failsafePath[i][0], this.failsafePath[i][1]);
      ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.restore();
  }

  _drawFrontiers(ctx) {
    ctx.save();
    ctx.fillStyle = "#4ade80";
    for (let i = 0; i < this.frontiers.length; i++) {
      const [fx, fy] = this.frontiers[i];
      const [px, py] = this.worldToScreen(fx, fy);
      ctx.beginPath();
      ctx.arc(px, py, 4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  _drawCorridorRegions(ctx) {
    ctx.save();
    ctx.font = "bold 9px 'Segoe UI', sans-serif";
    for (const reg of this.corridors) {
      if (!reg.bbox) continue;
      const [minX, minY, maxX, maxY] = reg.bbox;
      const [p1x, p1y] = this.worldToScreen(minX, maxY);
      const [p2x, p2y] = this.worldToScreen(maxX, minY);

      const type = (reg.type || "corridor").toLowerCase();
      if (type === "room") {
        ctx.strokeStyle = "rgba(168, 85, 247, 0.5)";
        ctx.fillStyle = "rgba(168, 85, 247, 0.08)";
      } else if (type === "corridor") {
        ctx.strokeStyle = "rgba(56, 189, 248, 0.5)";
        ctx.fillStyle = "rgba(56, 189, 248, 0.08)";
      } else {
        ctx.strokeStyle = "rgba(250, 204, 21, 0.5)";
        ctx.fillStyle = "rgba(250, 204, 21, 0.08)";
      }

      ctx.setLineDash([3, 3]);
      ctx.fillRect(p1x, p1y, p2x - p1x, p2y - p1y);
      ctx.strokeRect(p1x, p1y, p2x - p1x, p2y - p1y);

      // Label
      ctx.fillStyle = "#e2e8f0";
      ctx.fillText(`${reg.type || "Region"} ${reg.id || ""}`, p1x + 4, p1y + 12);
    }
    ctx.restore();
  }

  _drawSearchGrid(ctx) {
    ctx.save();
    ctx.strokeStyle = "rgba(45, 55, 72, 0.5)";
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);

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
    ctx.fillStyle = "rgba(100, 116, 139, 0.35)";
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
    ctx.restore();
  }

  _drawSurvivorPin(ctx, surv) {
    const x = typeof surv.local_x === "number" ? surv.local_x : (surv.x || 0.0);
    const y = typeof surv.local_y === "number" ? surv.local_y : (surv.y || 0.0);
    const [sx, sy] = this.worldToScreen(x, y);

    const status = (surv.status || "CONFIRMED").toUpperCase();
    let color = "#22c55e";
    if (status === "TRACKING") color = "#facc15";
    else if (status === "DETECTED") color = "#38bdf8";
    else if (status === "LOST") color = "#f87171";
    else if (status === "RESCUED") color = "#10b981";

    // Outer Glow
    ctx.fillStyle = color === "#22c55e" ? "rgba(34, 197, 94, 0.25)" : "rgba(250, 204, 21, 0.25)";
    ctx.beginPath();
    ctx.arc(sx, sy, 14, 0, Math.PI * 2);
    ctx.fill();

    // Center Pin
    ctx.fillStyle = color;
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(sx, sy, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Label Tag
    const confPct = Math.round((surv.confidence || 0.9) * 100);
    const tagText = `${surv.id || "S"} (${confPct}%) [${surv.grid_cell || "--"}]`;
    ctx.font = "bold 9px Segoe UI, sans-serif";
    const textWidth = ctx.measureText(tagText).width;
    ctx.fillStyle = color;
    ctx.fillRect(sx + 10, sy - 14, textWidth + 8, 16);

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

    ctx.fillStyle = "rgba(0, 229, 255, 0.2)";
    ctx.beginPath();
    ctx.moveTo(dx, dy);
    ctx.lineTo(dx + Math.cos(yawRad - spread) * coneLen, dy - Math.sin(yawRad - spread) * coneLen);
    ctx.lineTo(dx + Math.cos(yawRad + spread) * coneLen, dy - Math.sin(yawRad + spread) * coneLen);
    ctx.closePath();
    ctx.fill();

    // 2. Drone Body
    ctx.fillStyle = "#00e5ff";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(dx, dy, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // 3. Heading Pointer
    ctx.strokeStyle = "#ff1744";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(dx, dy);
    ctx.lineTo(dx + Math.cos(yawRad) * 16, dy - Math.sin(yawRad) * 16);
    ctx.stroke();
  }
}

window.GCSMapRenderer = GCSMapRenderer;
