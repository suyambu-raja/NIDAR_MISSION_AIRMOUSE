/**
 * Dual Camera (RGB & Thermal) View Controller for Mission Planner GCS.
 * Feeds the Payload Tab video viewports and the floating Map PiP window.
 */
class GCSCameraView {
  constructor() {
    this.rgbImg = document.getElementById("img-rgb-stream");
    this.thermalImg = document.getElementById("img-thermal-stream");
    this.pipImg = document.getElementById("pip-video-img");
    this.pipTitle = document.getElementById("pip-title");
    this.btnPipToggle = document.getElementById("btn-pip-toggle-cam");

    this.rgbBadge = document.getElementById("rgb-status-badge");
    this.thermalBadge = document.getElementById("thermal-status-badge");

    this.activePipCam = "rgb"; // "rgb" or "thermal"

    if (this.btnPipToggle) {
      this.btnPipToggle.addEventListener("click", () => {
        this.activePipCam = this.activePipCam === "rgb" ? "thermal" : "rgb";
        if (this.pipTitle) {
          this.pipTitle.textContent = this.activePipCam === "rgb" ? "FPV: RGB (OAK-D)" : "FPV: FLIR Thermal";
        }
      });
    }
  }

  updateFrame(payload) {
    const camId = payload.camera_id;
    const b64 = payload.frame_base64;
    const src = `data:image/jpeg;base64,${b64}`;

    if (camId === "rgb") {
      if (this.rgbImg) this.rgbImg.src = src;
      if (this.rgbBadge) {
        this.rgbBadge.textContent = payload.status === "LIVE" ? "LIVE" : "SIM";
      }
      if (this.pipImg && this.activePipCam === "rgb") {
        this.pipImg.src = src;
      }
    } else if (camId === "thermal") {
      if (this.thermalImg) this.thermalImg.src = src;
      if (this.thermalBadge) {
        this.thermalBadge.textContent = payload.status === "LIVE" ? "LIVE" : "SIM";
      }
      if (this.pipImg && this.activePipCam === "thermal") {
        this.pipImg.src = src;
      }
    }
  }

  updateHudTelemetry(velocity, altitude) {
    // HUD overlays
  }
}

window.GCSCameraView = GCSCameraView;
