/**
 * Dual Camera (RGB & Thermal) View Controller for NIDAR AirMouse GCS.
 * Dedicated RGB primary pane in main split and FLIR Thermal in sidebar.
 */
class GCSCameraView {
  constructor() {
    this.rgbImg = document.getElementById("img-rgb-stream");
    this.thermalImg = document.getElementById("img-thermal-stream");
    this.rgbOverlay = document.getElementById("rgb-overlay-text");
    this.thermalOverlay = document.getElementById("thermal-overlay-text");
    this.rgbBadge = document.getElementById("rgb-status-badge");
    this.thermalBadge = document.getElementById("thermal-status-badge");
  }

  updateFrame(payload) {
    const camId = payload.camera_id;
    const b64 = payload.frame_base64;
    const src = `data:image/jpeg;base64,${b64}`;

    if (camId === "rgb") {
      if (this.rgbImg) this.rgbImg.src = src;
      if (this.rgbOverlay) this.rgbOverlay.style.display = "none";
      if (this.rgbBadge) {
        this.rgbBadge.textContent = payload.status === "LIVE" ? "LIVE RGB" : "SIMULATED RGB";
        this.rgbBadge.className = payload.status === "LIVE" ? "badge-mini badge-live" : "badge-mini badge-sim";
      }
    } else if (camId === "thermal") {
      if (this.thermalImg) this.thermalImg.src = src;
      if (this.thermalOverlay) this.thermalOverlay.style.display = "none";
      if (this.thermalBadge) {
        this.thermalBadge.textContent = payload.status === "LIVE" ? "LIVE THERMAL" : "SIMULATED THERMAL";
        this.thermalBadge.className = payload.status === "LIVE" ? "badge-mini badge-live" : "badge-mini badge-sim";
      }
    }
  }

  updateHudTelemetry(hSpeed, vSpeed) {
    const hsElem = document.getElementById("hud-hs");
    const vsElem = document.getElementById("hud-vs");
    if (hsElem) hsElem.textContent = (hSpeed || 0.0).toFixed(1);
    if (vsElem) vsElem.textContent = (vSpeed || 0.0).toFixed(1);
  }

  reset() {
    if (this.rgbImg) this.rgbImg.src = "";
    if (this.thermalImg) this.thermalImg.src = "";
    if (this.rgbOverlay) this.rgbOverlay.style.display = "block";
    if (this.thermalOverlay) this.thermalOverlay.style.display = "block";
  }
}

window.GCSCameraView = GCSCameraView;
