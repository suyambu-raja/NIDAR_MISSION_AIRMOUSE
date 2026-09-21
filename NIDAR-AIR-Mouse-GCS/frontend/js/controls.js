/**
 * Mission & Dashboard Controls Manager for NIDAR AirMouse GCS.
 */
class GCSControls {
  constructor(wsClient, mapRenderer, survivorManager = null) {
    this.ws = wsClient;
    this.map = mapRenderer;
    this.survivorManager = survivorManager;
    this.isConnected = false;

    this.btnConnect = document.getElementById("btn-toggle-connect");
    this.btnStart = document.getElementById("btn-start-mission");
    this.btnAbort = document.getElementById("btn-abort-mission");
    this.btnReset = document.getElementById("btn-reset-mission");
    this.btnSimSurvivor = document.getElementById("btn-sim-survivor");

    // Flight controls
    this.btnArm = document.getElementById("btn-arm");
    this.btnDisarm = document.getElementById("btn-disarm");
    this.btnTakeoff = document.getElementById("btn-takeoff");
    this.btnLand = document.getElementById("btn-land");
    this.btnRtl = document.getElementById("btn-rtl");
    this.btnPause = document.getElementById("btn-pause");
    this.btnResume = document.getElementById("btn-resume");

    // Modal elements
    this.modalAbort = document.getElementById("abort-modal");
    this.btnModalCancel = document.getElementById("btn-modal-cancel");
    this.btnModalConfirm = document.getElementById("btn-modal-confirm-abort");

    // Map control buttons
    this.btnMapFit = document.getElementById("btn-map-fit");
    this.btnZoomIn = document.getElementById("btn-map-zoom-in");
    this.btnZoomOut = document.getElementById("btn-map-zoom-out");
    this.chkFollow = document.getElementById("chk-follow-drone");
    this.chkGrid = document.getElementById("chk-show-grid");
    this.chkTrail = document.getElementById("chk-show-trail");

    // SLAM mode select dropdown & status tag
    this.selectSlamMode = document.getElementById("slam-mode-select");
    this.tagSlamModeStatus = document.getElementById("slam-mode-status-tag");

    this._bindEvents();
  }

  _bindEvents() {
    // 1. Connection Toggle
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
        if (confirm("Disarm motors? Ensure drone is on ground!")) {
          this.ws.send("disarm", { action: "DISARM" });
        }
      });
    }
    if (this.btnTakeoff) {
      this.btnTakeoff.addEventListener("click", () => {
        this.ws.send("takeoff", { altitude: 1.5 });
      });
    }
    if (this.btnLand) {
      this.btnLand.addEventListener("click", () => {
        this.ws.send("land", { action: "LAND" });
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

    // 4. Emergency Abort & Modal
    if (this.btnAbort) {
      this.btnAbort.addEventListener("click", () => {
        if (this.modalAbort) this.modalAbort.classList.remove("hidden");
      });
    }
    if (this.btnModalCancel) {
      this.btnModalCancel.addEventListener("click", () => {
        if (this.modalAbort) this.modalAbort.classList.add("hidden");
      });
    }
    if (this.btnModalConfirm) {
      this.btnModalConfirm.addEventListener("click", () => {
        if (this.modalAbort) this.modalAbort.classList.add("hidden");
        this.ws.send("abort_mission", { reason: "Operator Emergency Abort Modal Confirmation" });
      });
    }

    // 5. Reset Mission
    if (this.btnReset) {
      this.btnReset.addEventListener("click", () => {
        if (confirm("Reset mission and map back to initial staging?")) {
          this.ws.send("reset");
          this.map.reset();
          if (this.survivorManager) {
            this.survivorManager.reset();
          }
        }
      });
    }

    // 5. Inject Simulated Survivor
    if (this.btnSimSurvivor) {
      this.btnSimSurvivor.addEventListener("click", () => {
        this.ws.send("simulate_survivor", { source: "MANUAL_OPERATOR" });
      });
    }

    // 6. Map Controls
    if (this.btnMapFit) {
      this.btnMapFit.addEventListener("click", () => this.map.fitToView());
    }
    if (this.btnZoomIn) {
      this.btnZoomIn.addEventListener("click", () => this.map.zoom(1.2));
    }
    if (this.btnZoomOut) {
      this.btnZoomOut.addEventListener("click", () => this.map.zoom(0.8));
    }
    if (this.chkFollow) {
      this.chkFollow.addEventListener("change", (e) => {
        this.map.followDrone = e.target.checked;
      });
    }
    if (this.chkGrid) {
      this.chkGrid.addEventListener("change", (e) => {
        this.map.showGrid = e.target.checked;
      });
    }
    if (this.chkTrail) {
      this.chkTrail.addEventListener("change", (e) => {
        this.map.showTrail = e.target.checked;
      });
    }

    // 7. SLAM Mode Dropdown Selection (Options Selector)
    if (this.selectSlamMode) {
      this.selectSlamMode.addEventListener("change", (e) => {
        this.setSlamMode(e.target.value, true);
      });
    }
  }

  setSlamMode(mode, sendWs = false) {
    const isSim = (mode || "SIMULATION").toUpperCase() === "SIMULATION";
    const selectedMode = isSim ? "SIMULATION" : "REALTIME";
    if (this.selectSlamMode && this.selectSlamMode.value !== selectedMode) {
      this.selectSlamMode.value = selectedMode;
    }
    if (this.tagSlamModeStatus) {
      this.tagSlamModeStatus.textContent = isSim ? "🎮 SIMULATION ACTIVE" : "📡 REAL-TIME ACTIVE";
      this.tagSlamModeStatus.className = isSim ? "mode-status-tag tag-sim" : "mode-status-tag tag-realtime";
    }
    if (this.map) this.map.setSlamMode(selectedMode);
    if (sendWs && this.ws) {
      this.ws.send("set_slam_mode", { mode: selectedMode });
    }
  }

  setConnectionState(connected) {
    this.isConnected = connected;
    if (this.btnConnect) {
      this.btnConnect.textContent = connected ? "DISCONNECT" : "CONNECT";
      this.btnConnect.style.backgroundColor = connected ? "var(--accent-red)" : "var(--accent-blue)";
    }
  }
}

window.GCSControls = GCSControls;
