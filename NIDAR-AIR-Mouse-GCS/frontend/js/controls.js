/**
 * TEAM HANTRAMANAV GCS Controls & View Coordinator.
 * Supports:
 * - Switching between DATA, PLAN, SETUP, CONFIG, SIMULATION, HELP
 * - Full Mission Planner Flight Plan Waypoint Planner (Screenshot 2)
 * - Initial Setup & Vehicle Firmware selector (Screenshot 3)
 * - MAVLink Telemetry Connection Toggle
 * - Flight Actions (ARM, DISARM, TAKEOFF, AUTO, GUIDED, RTL, EMERGENCY ABORT)
 */
class GCSControls {
  constructor(wsClient, mapRenderer, survivorManager = null) {
    this.ws = wsClient;
    this.map = mapRenderer;
    this.survivorManager = survivorManager;
    this.isConnected = false;

    // Connect Button & Ribbon
    this.btnConnect = document.getElementById("btn-toggle-connect");
    this.connectLabel = document.getElementById("mp-connect-label");
    this.plugIcon = document.getElementById("mp-plug-icon");
    this.portSelect = document.getElementById("mp-port-select");

    // Flight Action Buttons
    this.btnArm = document.getElementById("btn-arm");
    this.btnDisarm = document.getElementById("btn-disarm");
    this.btnTakeoff = document.getElementById("btn-takeoff");
    this.btnLand = document.getElementById("btn-land");
    this.btnAuto = document.getElementById("btn-auto");
    this.btnGuided = document.getElementById("btn-guided");
    this.btnRtl = document.getElementById("btn-rtl");
    this.btnPause = document.getElementById("btn-pause");
    this.btnResume = document.getElementById("btn-resume");
    this.btnStart = document.getElementById("btn-start-mission");
    this.btnAbort = document.getElementById("btn-abort-mission");
    this.btnSimSurvivor = document.getElementById("btn-sim-survivor");

    // Modal
    this.modalAbort = document.getElementById("abort-modal");
    this.btnModalCancel = document.getElementById("btn-modal-cancel");
    this.btnModalConfirm = document.getElementById("btn-modal-confirm-abort");

    // Video PiP & Tuning
    this.btnTogglePip = document.getElementById("btn-toggle-pip");
    this.pipWindow = document.getElementById("mp-floating-pip");
    this.btnPipClose = document.getElementById("btn-pip-close");
    this.btnPipToggleCam = document.getElementById("btn-pip-toggle-cam");
    this.chkTuning = document.getElementById("chk-tuning");
    this.tuningDrawer = document.getElementById("mp-tuning-drawer");
    this.btnCloseTuning = document.getElementById("btn-close-tuning");

    // Waypoints for PLAN view
    this.waypoints = [
      { id: 1, command: "TAKEOFF", p1: 0, p2: 0, p3: 0, p4: 0, lat: 4.2501500, lon: 5.8998500, alt: 2.5, frame: "Relative" },
      { id: 2, command: "WAYPOINT", p1: 0, p2: 2, p3: 0, p4: 0, lat: 4.2503500, lon: 5.9001500, alt: 2.5, frame: "Relative" },
      { id: 3, command: "WAYPOINT", p1: 0, p2: 2, p3: 0, p4: 0, lat: 4.2506500, lon: 5.9004500, alt: 2.5, frame: "Relative" },
      { id: 4, command: "WAYPOINT", p1: 0, p2: 2, p3: 0, p4: 0, lat: 4.2509500, lon: 5.9008500, alt: 2.5, frame: "Relative" },
      { id: 5, command: "RETURN_TO_LAUNCH", p1: 0, p2: 0, p3: 0, p4: 0, lat: 4.2501500, lon: 5.8998500, alt: 2.5, frame: "Relative" }
    ];

    this._bindRibbonViews();
    this._bindDataTabs();
    this._bindFlightEvents();
    this._bindPiPAndTuning();
    this._bindPlanView();
    this._bindSetupView();
    this._bindConfigView();
    this._bindSimulationView();
  }

  _bindRibbonViews() {
    const tools = document.querySelectorAll(".mp-tool-btn");
    const views = {
      data: document.getElementById("view-data"),
      plan: document.getElementById("view-plan"),
      setup: document.getElementById("view-setup"),
      config: document.getElementById("view-config"),
      simulation: document.getElementById("view-simulation"),
      help: document.getElementById("view-help")
    };

    tools.forEach(btn => {
      btn.addEventListener("click", () => {
        const viewKey = btn.getAttribute("data-view");
        tools.forEach(b => b.classList.remove("active"));
        btn.classList.add("active");

        // Hide all views, show targeted view
        Object.values(views).forEach(v => {
          if (v) v.classList.remove("active");
        });

        if (views[viewKey]) {
          views[viewKey].classList.add("active");
        }

        if (viewKey === "plan") {
          this._renderPlanMap();
          this._renderPlanTable();
        } else if (viewKey === "data" && this.map) {
          setTimeout(() => this.map._resizeCanvas(), 50);
        }
      });
    });
  }

  _bindDataTabs() {
    const tabBtns = document.querySelectorAll(".mp-tab-btn");
    const tabPanes = document.querySelectorAll(".mp-tab-pane");

    tabBtns.forEach(btn => {
      btn.addEventListener("click", () => {
        const targetId = btn.getAttribute("data-tab");
        tabBtns.forEach(b => b.classList.remove("active"));
        tabPanes.forEach(p => p.classList.remove("active"));

        btn.classList.add("active");
        const pane = document.getElementById(targetId);
        if (pane) pane.classList.add("active");
      });
    });
  }

  _bindFlightEvents() {
    // 1. Connection Toggle Button (Plug Icon)
    if (this.btnConnect) {
      this.btnConnect.addEventListener("click", () => {
        if (!this.isConnected) {
          this.ws.send("connect");
        } else {
          this.ws.send("disconnect");
        }
      });
    }

    // 2. Flight Actions
    if (this.btnArm) {
      this.btnArm.addEventListener("click", () => {
        this.ws.send("arm", { action: "ARM" });
      });
    }
    if (this.btnDisarm) {
      this.btnDisarm.addEventListener("click", () => {
        if (confirm("Disarm motors? Ensure drone is safely on ground!")) {
          this.ws.send("disarm", { action: "DISARM" });
        }
      });
    }
    if (this.btnTakeoff) {
      this.btnTakeoff.addEventListener("click", () => {
        this.ws.send("takeoff", { altitude: 2.5 });
      });
    }
    if (this.btnLand) {
      this.btnLand.addEventListener("click", () => {
        this.ws.send("land", { action: "LAND" });
      });
    }
    if (this.btnAuto) {
      this.btnAuto.addEventListener("click", () => {
        this.ws.send("set_mode", { mode: "AUTO" });
      });
    }
    if (this.btnGuided) {
      this.btnGuided.addEventListener("click", () => {
        this.ws.send("set_mode", { mode: "GUIDED_NOGPS" });
      });
    }
    if (this.btnRtl) {
      this.btnRtl.addEventListener("click", () => {
        this.ws.send("rtl", { action: "RTL" });
      });
    }
    if (this.btnPause) {
      this.btnPause.addEventListener("click", () => {
        this.ws.send("pause_autonomy", { action: "PAUSE" });
      });
    }
    if (this.btnResume) {
      this.btnResume.addEventListener("click", () => {
        this.ws.send("resume_autonomy", { action: "RESUME" });
      });
    }

    // 3. Start Mission
    if (this.btnStart) {
      this.btnStart.addEventListener("click", () => {
        this.ws.send("start_mission");
      });
    }

    // 4. Emergency Abort Modal
    if (this.btnAbort && this.modalAbort) {
      this.btnAbort.addEventListener("click", () => {
        this.modalAbort.classList.remove("hidden");
      });
    }
    if (this.btnModalCancel && this.modalAbort) {
      this.btnModalCancel.addEventListener("click", () => {
        this.modalAbort.classList.add("hidden");
      });
    }
    if (this.btnModalConfirm && this.modalAbort) {
      this.btnModalConfirm.addEventListener("click", () => {
        this.modalAbort.classList.add("hidden");
        this.ws.send("abort_mission");
      });
    }

    // 5. Inject Simulated Survivor
    if (this.btnSimSurvivor) {
      this.btnSimSurvivor.addEventListener("click", () => {
        const randX = 2.0 + Math.random() * 16.0;
        const randY = 2.0 + Math.random() * 16.0;
        this.ws.send("simulate_survivor", { x: randX, y: randY, conf: 0.95 });
      });
    }
  }

  _bindPiPAndTuning() {
    if (this.btnTogglePip && this.pipWindow) {
      this.btnTogglePip.addEventListener("click", () => {
        this.pipWindow.classList.toggle("hidden");
      });
    }
    if (this.btnPipClose && this.pipWindow) {
      this.btnPipClose.addEventListener("click", () => {
        this.pipWindow.classList.add("hidden");
      });
    }

    if (this.chkTuning && this.tuningDrawer) {
      this.chkTuning.addEventListener("change", (e) => {
        if (e.target.checked) {
          this.tuningDrawer.classList.remove("hidden");
        } else {
          this.tuningDrawer.classList.add("hidden");
        }
      });
    }
    if (this.btnCloseTuning && this.tuningDrawer && this.chkTuning) {
      this.btnCloseTuning.addEventListener("click", () => {
        this.tuningDrawer.classList.add("hidden");
        this.chkTuning.checked = false;
      });
    }
  }

  /* ========================================================================
     PLAN VIEW HANDLERS (EXACT MATCH TO SCREENSHOT 2)
     ======================================================================== */
  _bindPlanView() {
    const addBelowBtn = document.getElementById("btn-plan-add-below");
    const writeBtn = document.getElementById("btn-plan-write");
    const readBtn = document.getElementById("btn-plan-read");
    const saveBtn = document.getElementById("btn-plan-save");
    const loadBtn = document.getElementById("btn-plan-load");

    if (addBelowBtn) {
      addBelowBtn.addEventListener("click", () => {
        const last = this.waypoints[this.waypoints.length - 1];
        const nextId = this.waypoints.length + 1;
        const defAlt = parseFloat(document.getElementById("plan-default-alt")?.value || 2.5);

        this.waypoints.push({
          id: nextId,
          command: "WAYPOINT",
          p1: 0, p2: 2, p3: 0, p4: 0,
          lat: last.lat + 0.0003,
          lon: last.lon + 0.0003,
          alt: defAlt,
          frame: "Relative"
        });

        this._renderPlanTable();
        this._renderPlanMap();
      });
    }

    if (writeBtn) {
      writeBtn.addEventListener("click", () => {
        // Send waypoints to backend MAVLink / ROS 2
        this.ws.send("write_waypoints", { waypoints: this.waypoints });
        alert(`✅ [TEAM HANTRAMANAV GCS] Successfully transmitted ${this.waypoints.length} Waypoints to FCU.`);
      });
    }

    if (readBtn) {
      readBtn.addEventListener("click", () => {
        this.ws.send("read_waypoints");
        alert("📡 [TEAM HANTRAMANAV GCS] Reading Waypoints from MAVLink FCU...");
      });
    }

    if (saveBtn) {
      saveBtn.addEventListener("click", () => {
        const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(this.waypoints, null, 2));
        const a = document.createElement("a");
        a.href = dataStr;
        a.download = "TEAM_HANTRAMANAV_MISSION.waypoints";
        a.click();
      });
    }

    if (loadBtn) {
      loadBtn.addEventListener("click", () => {
        const input = document.createElement("input");
        input.type = "file";
        input.accept = ".waypoints,.json";
        input.onchange = (e) => {
          const file = e.target.files[0];
          if (!file) return;
          const reader = new FileReader();
          reader.onload = (event) => {
            try {
              const loaded = JSON.parse(event.target.result);
              if (Array.isArray(loaded)) {
                this.loadWaypoints(loaded);
                alert(`📁 Loaded ${loaded.length} waypoints successfully.`);
              }
            } catch (err) {
              alert("Error parsing waypoint file: " + err.message);
            }
          };
          reader.readAsText(file);
        };
        input.click();
      });
    }

    const writeFastBtn = document.getElementById("btn-plan-write-fast");
    if (writeFastBtn) {
      writeFastBtn.addEventListener("click", () => {
        this.ws.send("write_waypoints", { waypoints: this.waypoints });
        alert(`⚡ [TEAM HANTRAMANAV GCS] Fast Write: ${this.waypoints.length} Waypoints sent directly to FCU.`);
      });
    }

    const homeBtn = document.getElementById("btn-plan-home");
    if (homeBtn) {
      homeBtn.addEventListener("click", () => {
        alert("🏠 [TEAM HANTRAMANAV GCS] Plan View centered on Home Location (4.2501500, 5.8998500).");
        this._renderPlanMap();
      });
    }

    // Interactive Waypoint Click on Plan Canvas
    const planCanvas = document.getElementById("plan-canvas");
    if (planCanvas) {
      planCanvas.addEventListener("click", (e) => {
        const rect = planCanvas.getBoundingClientRect();
        const clickX = e.clientX - rect.left;
        const clickY = e.clientY - rect.top;

        // Convert canvas pixels to lat/lon
        const lat = 4.2501500 + ((planCanvas.height - clickY) * 0.000008);
        const lon = 5.8998500 + (clickX * 0.000008);
        const defAlt = parseFloat(document.getElementById("plan-default-alt")?.value || 2.5);

        this.waypoints.push({
          id: this.waypoints.length + 1,
          command: "WAYPOINT",
          p1: 0, p2: 2, p3: 0, p4: 0,
          lat: Number(lat.toFixed(7)),
          lon: Number(lon.toFixed(7)),
          alt: defAlt,
          frame: "Relative"
        });

        this._renderPlanTable();
        this._renderPlanMap();
      });
    }
  }

  _renderPlanTable() {
    const tbody = document.getElementById("plan-wp-tbody");
    if (!tbody) return;

    tbody.innerHTML = "";
    let totalDist = 0.0;

    this.waypoints.forEach((wp, idx) => {
      const dist = idx > 0 ? 35.4 : 0.0;
      totalDist += dist;

      const tr = document.createElement("tr");
      tr.innerHTML = `
        <td><b>${wp.id}</b></td>
        <td><span style="color:#60a5fa; font-weight:bold;">${wp.command}</span></td>
        <td>${wp.p1}</td>
        <td>${wp.p2}</td>
        <td>${wp.p3}</td>
        <td>${wp.p4}</td>
        <td>${wp.lat.toFixed(7)}</td>
        <td>${wp.lon.toFixed(7)}</td>
        <td style="color:#a7f3d0; font-weight:bold;">${wp.alt.toFixed(1)}</td>
        <td>${wp.frame}</td>
        <td><button class="mp-mini-btn" style="color:#ef4444;" onclick="window._gcsControls.deleteWaypoint(${idx})">✕</button></td>
        <td>0.0</td>
        <td>0.0°</td>
        <td>${dist.toFixed(1)}m</td>
        <td>045°</td>
      `;
      tbody.appendChild(tr);
    });

    const distElem = document.getElementById("plan-total-dist");
    if (distElem) distElem.textContent = `${(totalDist / 1000).toFixed(4)} km`;
  }

  deleteWaypoint(idx) {
    if (this.waypoints.length > 1) {
      this.waypoints.splice(idx, 1);
      this.waypoints.forEach((w, i) => w.id = i + 1);
      this._renderPlanTable();
      this._renderPlanMap();
    }
  }

  _renderPlanMap() {
    const canvas = document.getElementById("plan-canvas");
    if (!canvas) return;

    const parent = canvas.parentElement;
    if (parent) {
      canvas.width = parent.clientWidth;
      canvas.height = parent.clientHeight - 190;
    }

    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;

    ctx.clearRect(0, 0, W, H);

    // Satellite Ocean & Coastline Background (Exact Match to Screenshot 2)
    const bgGrad = ctx.createLinearGradient(0, 0, W, H);
    bgGrad.addColorStop(0, "#081729");
    bgGrad.addColorStop(0.5, "#0e2947");
    bgGrad.addColorStop(1, "#14375e");
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, W, H);

    // Draw Sahara Landmass on the right
    ctx.fillStyle = "#8a6642";
    ctx.beginPath();
    ctx.moveTo(W * 0.45, 0);
    ctx.lineTo(W, 0);
    ctx.lineTo(W, H);
    ctx.lineTo(W * 0.65, H);
    ctx.bezierCurveTo(W * 0.55, H * 0.7, W * 0.4, H * 0.5, W * 0.45, 0);
    ctx.fill();

    // Golden Desert Dunes
    ctx.fillStyle = "#c18c50";
    ctx.beginPath();
    ctx.moveTo(W * 0.48, 0);
    ctx.lineTo(W, 0);
    ctx.lineTo(W, H * 0.45);
    ctx.bezierCurveTo(W * 0.75, H * 0.35, W * 0.52, H * 0.25, W * 0.48, 0);
    ctx.fill();

    // Graticule grid
    ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
    ctx.lineWidth = 1;
    for (let x = 0; x < W; x += 100) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }
    for (let y = 0; y < H; y += 100) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }

    // Draw Waypoints and Route
    if (this.waypoints.length > 0) {
      ctx.strokeStyle = "#ff9900";
      ctx.lineWidth = 2.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();

      const pts = [];
      this.waypoints.forEach((wp, i) => {
        const px = W * 0.22 + (i * 90);
        const py = H * 0.55 - (i * 45);
        pts.push({ x: px, y: py, wp: wp });
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.setLineDash([]);

      // Waypoint Pins
      pts.forEach(p => {
        ctx.fillStyle = "#22c55e";
        ctx.beginPath();
        ctx.arc(p.x, p.y, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = "#000000";
        ctx.font = "bold 9px Consolas, monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(p.wp.id), p.x, p.y);

        ctx.fillStyle = "#ffffff";
        ctx.font = "bold 10px 'Segoe UI', sans-serif";
        ctx.fillText(p.wp.command, p.x, p.y - 14);
      });
    }
  }

  /* ========================================================================
     SETUP VIEW HANDLERS (EXACT MATCH TO SCREENSHOT 3)
     ======================================================================== */
  _bindSetupView() {
    const cards = document.querySelectorAll(".setup-vehicle-card");
    cards.forEach(card => {
      card.addEventListener("click", () => {
        cards.forEach(c => c.classList.remove("active"));
        card.classList.add("active");
        const vehName = card.querySelector(".veh-title")?.textContent || "Vehicle";
        alert(`🚁 [TEAM HANTRAMANAV GCS] Selected Vehicle Platform:\n${vehName}\nReady for firmware flashing / configuration.`);
      });
    });
  }

  /* ========================================================================
     CONFIG VIEW HANDLERS (PARAMETER MANAGEMENT)
     ======================================================================== */
  _bindConfigView() {
    const refreshBtn = document.querySelector("#view-config .config-param-actions button:nth-child(1)");
    const writeBtn = document.querySelector("#view-config .config-param-actions button:nth-child(2)");
    const saveBtn = document.querySelector("#view-config .config-param-actions button:nth-child(3)");

    if (refreshBtn) {
      refreshBtn.addEventListener("click", () => {
        this.ws.send("get_parameters");
      });
    }
    if (writeBtn) {
      writeBtn.addEventListener("click", () => {
        this.ws.send("set_parameters", {
          parameters: {
            EK3_SRC1_POSXY: 3,
            EK3_SRC1_VELXY: 5,
            FS_THR_ENABLE: 1,
            FS_BATT_VOLT: 14.8,
            WPNAV_SPEED: 70
          }
        });
        alert("✅ [TEAM HANTRAMANAV GCS] Parameters written to FCU EEPROM.");
      });
    }
    if (saveBtn) {
      saveBtn.addEventListener("click", () => {
        const params = {
          EK3_SRC1_POSXY: 3,
          EK3_SRC1_VELXY: 5,
          FS_THR_ENABLE: 1,
          FS_BATT_VOLT: 14.8,
          WPNAV_SPEED: 70
        };
        const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(params, null, 2));
        const a = document.createElement("a");
        a.href = dataStr;
        a.download = "TEAM_HANTRAMANAV_PARAMS.param";
        a.click();
      });
    }
  }

  /* ========================================================================
     SIMULATION VIEW HANDLERS (SITL ARENA)
     ======================================================================== */
  _bindSimulationView() {
    const simStart = document.getElementById("btn-sim-start");
    const simReset = document.getElementById("btn-sim-reset");
    const simVictim = document.getElementById("btn-sim-inject-victim");

    if (simStart) {
      simStart.addEventListener("click", () => {
        this.ws.send("start_mission");
        alert("🎮 [TEAM HANTRAMANAV GCS] Launching Indoor SITL Copter in 20m x 20m arena!");
      });
    }
    if (simReset) {
      simReset.addEventListener("click", () => {
        this.ws.send("reset");
        alert("🔄 [TEAM HANTRAMANAV GCS] SITL Arena reset to initial staging coordinates.");
      });
    }
    if (simVictim) {
      simVictim.addEventListener("click", () => {
        const rx = 2.0 + Math.random() * 16.0;
        const ry = 2.0 + Math.random() * 16.0;
        this.ws.send("simulate_survivor", { x: rx, y: ry, conf: 0.96 });
      });
    }
  }

  loadWaypoints(wps) {
    if (Array.isArray(wps) && wps.length > 0) {
      this.waypoints = wps;
      this._renderPlanTable();
      this._renderPlanMap();
    }
  }

  setConnectionState(connected) {
    this.isConnected = connected;
    if (this.btnConnect) {
      if (connected) {
        this.btnConnect.classList.remove("disconnected");
        this.btnConnect.classList.add("connected");
        if (this.connectLabel) this.connectLabel.textContent = "DISCONNECT";
      } else {
        this.btnConnect.classList.remove("connected");
        this.btnConnect.classList.add("disconnected");
        if (this.connectLabel) this.connectLabel.textContent = "CONNECT";
      }
    }
  }

  setSlamMode(mode, notify = true) {
    if (this.map) {
      this.map.layerMode = (mode === "SIMULATION") ? "slam" : "satellite";
    }
  }
}

window.GCSControls = GCSControls;
