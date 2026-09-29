/**
 * Master Application Bootstrapper for Mission Planner 1.3.83 GCS.
 * Coordinates WebSocket telemetry, Primary Flight Display (PFD) HUD,
 * satellite map, dual cameras, survivor tracking, and controls.
 */
document.addEventListener("DOMContentLoaded", () => {
  console.log("[MissionPlanner] Initializing Mission Planner 1.3.83 Dashboard...");

  // 1. Initialize Subsystem Controllers
  const wsClient = new GCSWebSocketClient();
  const mapRenderer = new GCSMapRenderer("map-canvas", "map-coord-readout");
  const telemetryView = new GCSTelemetryView();
  const cameraView = new GCSCameraView();
  const survivorManager = new GCSSurvivorManager(mapRenderer);
  const radioView = new GCSRadioView();
  const eventLog = new GCSEventLog("event-terminal", "btn-clear-log");
  const controls = new GCSControls(wsClient, mapRenderer, survivorManager);
  window._gcsControls = controls;

  // UI Badges & Elements
  const missionBadge = document.getElementById("mission-badge");
  const radioBadge = document.getElementById("radio-badge");
  const flightModeBadge = document.getElementById("flight-mode-badge");

  // Telemetry Packets (10 Hz)
  wsClient.on("telemetry", (telem) => {
    mapRenderer.updateDronePose(telem.x, telem.y, telem.heading);
    telemetryView.updateTelemetry(telem);
    cameraView.updateHudTelemetry(telem.velocity, telem.altitude);
  });

  // 2D Dynamic SLAM & Search Grid Map Updates (5 Hz)
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

  // Waypoints Readback from Drone
  wsClient.on("waypoints", (wpData) => {
    if (wpData && wpData.waypoints && window._gcsControls) {
      window._gcsControls.loadWaypoints(wpData.waypoints);
    }
  });

  // Failsafe Path Updates
  wsClient.on("failsafe_path_update", (data) => {
    if (data && data.path) {
      mapRenderer.updateFailsafePath(data.path);
    }
  });

  // Corridor Topology Regions
  wsClient.on("map_regions", (data) => {
    if (data && data.regions) {
      mapRenderer.updateCorridors(data.regions);
    }
  });

  // Failsafe Status Updates
  wsClient.on("failsafe_status", (fs) => {
    telemetryView.updateFailsafe(fs);
  });

  // Autonomous Survivor Detections
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
        radioBadge.textContent = `GOOD (${rf.rssi_dbm ? rf.rssi_dbm.toFixed(1) : "-64.6"} dBm)`;
        radioBadge.style.color = "#2ecc71";
      } else {
        radioBadge.textContent = "NO SIGNAL";
        radioBadge.style.color = "#e74c3c";
      }
    }
  });

  // Mission State & Elapsed Clock (1 Hz)
  wsClient.on("mission_status", (status) => {
    if (status.slam_mode) {
      controls.setSlamMode(status.slam_mode, false);
    }

    if (missionBadge && status.state) {
      missionBadge.textContent = status.state;
      missionBadge.style.color = status.state === "READY" || status.state === "MISSION_COMPLETE" ? "#2ecc71" : "#f1c40f";
    }

    if (status.elapsed_time) {
      telemetryView.setClock(status.elapsed_time);
    }
  });

  // System Events Log
  wsClient.on("event_log", (evt) => {
    eventLog.appendEvent(evt);
  });

  // WebSocket Connection Lifecycle
  wsClient.on("_status", (status) => {
    controls.setConnectionState(status.connected);
    if (status.connected) {
      eventLog.appendEvent({ level: "SUCCESS", category: "LINK", message: "MAVLink & GCS WebSocket link established" });
    } else {
      eventLog.appendEvent({ level: "WARNING", category: "LINK", message: "GCS link disconnected - attempting reconnect..." });
    }
  });

  // Initiate WebSocket Connection
  wsClient.connect();
  console.log("[MissionPlanner] Dashboard successfully mounted.");
});
