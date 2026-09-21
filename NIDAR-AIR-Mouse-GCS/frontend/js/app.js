/**
 * Master Application Bootstrapper for NIDAR AirMouse GCS.
 * Coordinates WebSocket messaging and updates all UI modules to match the design screenshot.
 */
document.addEventListener("DOMContentLoaded", () => {
  console.log("[GCS] Initializing NIDAR AirMouse Dashboard...");

  // 1. Initialize Subsystem Controllers
  const wsClient = new GCSWebSocketClient();
  const mapRenderer = new GCSMapRenderer("map-canvas", "map-coord-readout");
  const telemetryView = new GCSTelemetryView();
  const cameraView = new GCSCameraView();
  const survivorManager = new GCSSurvivorManager(mapRenderer);
  const radioView = new GCSRadioView();
  const eventLog = new GCSEventLog();
  const controls = new GCSControls(wsClient, mapRenderer, survivorManager);

  // UI Badges & Elements
  const wsBadge = document.getElementById("ws-badge");
  const radioBadge = document.getElementById("radio-badge");
  const missionBadge = document.getElementById("mission-badge");
  const flightModeBadge = document.getElementById("flight-mode-badge");
  const missionTimer = document.getElementById("mission-timer");
  const compassNeedle = document.getElementById("compass-needle");

  // 2. Wire WebSocket Message Subscriptions

  // Watchdog timer for connection loss
  let lastTelemetryTimestamp = Date.now();
  const connLossBanner = document.getElementById("connection-loss-banner");
  const connLossTime = document.getElementById("connection-loss-time");
  const failsafeBanner = document.getElementById("failsafe-alert-banner");
  const failsafeReason = document.getElementById("failsafe-reason-text");
  const failsafeAction = document.getElementById("failsafe-action-text");

  setInterval(() => {
    const elapsed = Math.floor((Date.now() - lastTelemetryTimestamp) / 1000);
    if (elapsed >= 3) {
      if (connLossBanner) connLossBanner.classList.remove("hidden");
      if (connLossTime) connLossTime.textContent = elapsed;
    } else {
      if (connLossBanner) connLossBanner.classList.add("hidden");
    }
  }, 1000);

  // WebSocket Connection Lifecycle
  wsClient.on("_status", (status) => {
    if (wsBadge) {
      if (status.connected) {
        wsBadge.textContent = "CONNECTED";
        wsBadge.className = "badge badge-connected";
        eventLog.appendEvent({ level: "SUCCESS", category: "WS", message: "WebSocket bridge established" });
      } else {
        wsBadge.textContent = "DISCONNECTED";
        wsBadge.className = "badge badge-disconnected";
        eventLog.appendEvent({ level: "ERROR", category: "WS", message: "WebSocket connection closed - reconnecting..." });
      }
    }
  });

  // Telemetry Packets (10 Hz)
  wsClient.on("telemetry", (telem) => {
    lastTelemetryTimestamp = Date.now();
    mapRenderer.updateDronePose(telem.x, telem.y, telem.heading);
    telemetryView.updateTelemetry(telem);
    cameraView.updateHudTelemetry(telem.velocity, 0.0);

    if (flightModeBadge) {
      const mode = (telem.flight_mode || "").toUpperCase();
      flightModeBadge.textContent = mode || (telem.armed ? "ARMED" : "DISARMED");
      if (mode.includes("EMERGENCY") || mode.includes("ABORT") || mode.includes("FAILSAFE")) {
        flightModeBadge.className = "badge badge-disconnected";
      } else if (telem.armed) {
        flightModeBadge.className = "badge badge-connected";
      } else {
        flightModeBadge.className = "badge badge-disarmed";
      }
    }

    if (compassNeedle) {
      compassNeedle.style.transform = `rotate(${-telem.heading}deg)`;
    }
  });

  // 2D Dynamic SLAM Map Updates (5 Hz)
  wsClient.on("map_update", (mapData) => {
    mapRenderer.updateMapData(mapData);
    if (mapData.slam_mode) {
      controls.setSlamMode(mapData.slam_mode, false);
    }
  });

  // Planned Path & Frontiers
  wsClient.on("path_update", (pathData) => {
    if (pathData && pathData.path) {
      mapRenderer.updatePlannedPath(pathData.path);
    }
    if (pathData && pathData.frontiers) {
      mapRenderer.updateFrontiers(pathData.frontiers);
    }
  });

  // Failsafe Path Updates
  wsClient.on("failsafe_path_update", (data) => {
    if (data && data.path) {
      mapRenderer.updateFailsafePath(data.path);
    }
  });

  // Corridor Topology Regions (/map_regions)
  wsClient.on("map_regions", (data) => {
    if (data && data.regions) {
      mapRenderer.updateCorridors(data.regions);
      if (data.active_type) {
        telemetryView.updateAutonomy({ corridor_type: data.active_type });
      }
    }
  });

  // Failsafe Status Updates (/failsafe/status)
  wsClient.on("failsafe_status", (fs) => {
    telemetryView.updateFailsafe(fs);
    if (failsafeBanner) {
      if (fs.active && fs.state !== "NORMAL") {
        failsafeBanner.classList.remove("hidden");
        if (failsafeReason) failsafeReason.textContent = fs.reason || "Autonomous safety boundary triggered";
        if (failsafeAction) failsafeAction.textContent = fs.action || "HOLD / RETURN TO SAFE ZONE";
      } else {
        failsafeBanner.classList.add("hidden");
      }
    }
  });

  // FCU State Updates (/mavros_bridge/state)
  wsClient.on("fcu_state", (fcu) => {
    const fcuBadge = document.getElementById("health-fcu");
    const mavBadge = document.getElementById("health-mavros");
    if (fcuBadge) {
      fcuBadge.textContent = fcu.connected ? "ONLINE" : "OFFLINE";
      fcuBadge.className = fcu.connected ? "health-badge ok" : "health-badge error";
    }
    if (mavBadge) {
      mavBadge.textContent = fcu.mavros_healthy ? "ONLINE" : "OFFLINE";
      mavBadge.className = fcu.mavros_healthy ? "health-badge ok" : "health-badge error";
    }
  });

  // Autonomous Survivor Detections (High Priority)
  wsClient.on("survivor_detected", (surv) => {
    survivorManager.addOrUpdateSurvivor(surv);
    mapRenderer.addSurvivor(surv);
  });
  wsClient.on("survivor_update", (surv) => {
    survivorManager.addOrUpdateSurvivor(surv);
    mapRenderer.addSurvivor(surv);
  });

  // Dual Video Frames (12 FPS)
  wsClient.on("camera_frame", (framePayload) => {
    cameraView.updateFrame(framePayload);
  });

  // RF Radio Link Diagnostics (1 Hz)
  wsClient.on("radio_status", (rf) => {
    radioView.updateRadio(rf);
    controls.setConnectionState(rf.connected);

    if (radioBadge) {
      if (rf.connected) {
        radioBadge.textContent = `RF: ${rf.signal_quality || "GOOD"} (${rf.rssi_dbm ? rf.rssi_dbm.toFixed(1) : "-64.6"} dBm)`;
        radioBadge.className = "badge badge-rf-good";
      } else {
        radioBadge.textContent = "NO SIGNAL";
        radioBadge.className = "badge badge-disconnected";
      }
    }
  });

  // Mission State & Clock (1 Hz)
  wsClient.on("mission_status", (status) => {
    if (status.slam_mode) {
      controls.setSlamMode(status.slam_mode, false);
    }

    if (missionBadge) {
      missionBadge.textContent = status.state;
      const stateClasses = {
        IDLE: "badge badge-idle",
        SYSTEM_CHECK: "badge badge-active",
        READY: "badge badge-ready",
        TAKEOFF: "badge badge-active",
        ENTERING: "badge badge-active",
        EXPLORING: "badge badge-active",
        SURVIVOR_DETECTED: "badge badge-warning",
        EXITING: "badge badge-active",
        MISSION_COMPLETE: "badge badge-ready",
        ABORTED: "badge badge-disconnected",
      };
      missionBadge.className = stateClasses[status.state] || "badge badge-idle";
    }

    if (missionTimer && status.elapsed_time) {
      missionTimer.textContent = status.elapsed_time;
      const hudClock = document.querySelector(".hud-left-clock");
      if (hudClock) hudClock.textContent = status.elapsed_time;
    }
  });

  // System Events Log
  wsClient.on("event_log", (evt) => {
    eventLog.appendEvent(evt);
  });

  // 3. Initiate Connection
  wsClient.connect();
  console.log("[GCS] Dashboard successfully mounted and running.");
});
