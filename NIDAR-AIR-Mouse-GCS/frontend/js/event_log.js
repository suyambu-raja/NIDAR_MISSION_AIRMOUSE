/**
 * System Event Log Terminal Controller for NIDAR AirMouse GCS.
 */
class GCSEventLog {
  constructor(terminalId = "event-terminal", clearBtnId = "btn-clear-log") {
    this.terminal = document.getElementById(terminalId);
    this.clearBtn = document.getElementById(clearBtnId);
    this.maxEntries = 300;

    if (this.clearBtn) {
      this.clearBtn.addEventListener("click", () => this.clear());
    }
  }

  appendEvent(evt) {
    if (!this.terminal) return;

    const row = document.createElement("div");
    row.className = "log-entry";

    const levelClass = `log-level-${evt.level || "INFO"}`;
    row.innerHTML = `
      <span class="log-time">[${evt.timestamp || "--:--:--"}]</span>
      <span class="${levelClass}">[${evt.level || "INFO"}]</span>
      <span class="log-category">[${evt.category || "SYSTEM"}]</span>
      <span class="log-msg">${evt.message || ""}</span>
    `;

    this.terminal.appendChild(row);

    // Limit buffer
    while (this.terminal.childElementCount > this.maxEntries) {
      this.terminal.removeChild(this.terminal.firstElementChild);
    }

    // Auto-scroll
    this.terminal.scrollTop = this.terminal.scrollHeight;
  }

  clear() {
    if (this.terminal) {
      this.terminal.innerHTML = "";
    }
  }
}

window.GCSEventLog = GCSEventLog;
