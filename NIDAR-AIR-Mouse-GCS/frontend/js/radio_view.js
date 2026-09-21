/**
 * RF / Radio Link Status View Controller for NIDAR AirMouse GCS.
 */
class GCSRadioView {
  constructor() {
    this.badge = document.getElementById("radio-badge");
    this.statusText = document.getElementById("rf-status-text");
    this.rssiText = document.getElementById("rf-rssi-text");
    this.lossText = document.getElementById("rf-loss-text");
    this.latencyText = document.getElementById("rf-latency-text");
  }

  updateRadio(rf) {
    if (this.badge) {
      if (!rf.connected) {
        this.badge.textContent = "LINK LOST";
        this.badge.className = "badge badge-critical";
      } else if (rf.signal_quality === "GOOD") {
        this.badge.textContent = `GOOD (${rf.rssi_dbm} dBm)`;
        this.badge.className = "badge badge-nominal";
      } else if (rf.signal_quality === "FAIR") {
        this.badge.textContent = `FAIR (${rf.rssi_dbm} dBm)`;
        this.badge.className = "badge badge-warning";
      } else {
        this.badge.textContent = `POOR (${rf.rssi_dbm} dBm)`;
        this.badge.className = "badge badge-critical";
      }
    }

    if (this.statusText) {
      this.statusText.textContent = rf.connected ? "CONNECTED" : "LINK LOST";
      this.statusText.style.color = rf.connected ? "var(--status-nominal)" : "var(--status-critical)";
    }

    if (this.rssiText) {
      this.rssiText.textContent = `${rf.rssi_dbm} dBm`;
      if (rf.rssi_dbm >= -70) {
        this.rssiText.style.color = "var(--status-nominal)";
      } else if (rf.rssi_dbm >= -85) {
        this.rssiText.style.color = "var(--status-warning)";
      } else {
        this.rssiText.style.color = "var(--status-critical)";
      }
    }

    if (this.lossText) {
      this.lossText.textContent = `${rf.packet_loss_pct}%`;
      if (rf.packet_loss_pct <= 1.0) {
        this.lossText.style.color = "var(--status-nominal)";
      } else if (rf.packet_loss_pct <= 4.0) {
        this.lossText.style.color = "var(--status-warning)";
      } else {
        this.lossText.style.color = "var(--status-critical)";
      }
    }

    if (this.latencyText) {
      this.latencyText.textContent = `${rf.latency_ms}ms`;
      if (rf.latency_ms <= 40) {
        this.latencyText.style.color = "var(--status-nominal)";
      } else if (rf.latency_ms <= 100) {
        this.latencyText.style.color = "var(--status-warning)";
      } else {
        this.latencyText.style.color = "var(--status-critical)";
      }
    }
  }
}

window.GCSRadioView = GCSRadioView;
