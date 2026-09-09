/**
 * Autonomous Survivor Localisation & Alert Controller for NIDAR AirMouse GCS.
 * Dynamically tracks and displays detected survivors with coordinates, grid cells, and real-time count.
 */
class GCSSurvivorManager {
  constructor(mapRenderer) {
    this.mapRenderer = mapRenderer;
    this.container = document.getElementById("survivor-cards-container");
    this.summaryBadge = document.getElementById("survivor-summary-count");
    this.toast = document.getElementById("survivor-toast");
    this.toastDetails = document.getElementById("toast-details");
    this.toastDismiss = document.getElementById("btn-toast-dismiss");
    this.toastTimer = null;

    this.survivors = new Map();

    if (this.toastDismiss) {
      this.toastDismiss.addEventListener("click", () => this.hideToast());
    }

    this._renderEmptyState();
  }

  _renderEmptyState() {
    if (!this.container) return;
    this.container.innerHTML = `
      <div class="survivor-empty-placeholder">No survivors detected yet. Searching arena corridors...</div>
    `;
    if (this.summaryBadge) {
      this.summaryBadge.textContent = "NO. OF SURVIVORS DETECTED: 0";
    }
  }

  addOrUpdateSurvivor(surv) {
    const isNew = !this.survivors.has(surv.id);
    this.survivors.set(surv.id, surv);

    if (!this.container) return;

    // Remove empty placeholder if present
    const placeholder = this.container.querySelector(".survivor-empty-placeholder");
    if (placeholder) {
      placeholder.remove();
    }

    // Find or create Card dynamically
    let card = document.getElementById(`card-${surv.id}`);
    if (!card) {
      card = document.createElement("div");
      card.id = `card-${surv.id}`;
      card.className = "survivor-card confirmed";
      this.container.appendChild(card);
    }

    const confPct = Math.round((surv.confidence || 0.9) * 100);
    const sourceLabel = surv.source || "AUTONOMOUS_VISION";
    const xStr = (typeof surv.local_x === "number" ? surv.local_x : 0).toFixed(1);
    const yStr = (typeof surv.local_y === "number" ? surv.local_y : 0).toFixed(1);

    card.innerHTML = `
      <span class="surv-id">${surv.id}</span>
      <span class="surv-info">Grid: <b>${surv.grid_cell || "--"}</b> | (${xStr}m, ${yStr}m) | ${sourceLabel}</span>
      <span class="surv-conf">${confPct}%</span>
    `;

    card.onclick = () => {
      if (this.mapRenderer && typeof this.mapRenderer.focusOnSurvivor === "function") {
        this.mapRenderer.focusOnSurvivor(surv.local_x, surv.local_y);
      }
    };

    // Update Summary count
    const count = this.survivors.size;
    if (this.summaryBadge) {
      this.summaryBadge.textContent = `NO. OF SURVIVORS DETECTED: ${count}`;
    }

    // Trigger non-blocking pop-up notification if newly detected
    if (isNew) {
      this.showToast(surv);
    }
  }

  showToast(surv) {
    if (!this.toast || !this.toastDetails) return;
    const xStr = (typeof surv.local_x === "number" ? surv.local_x : 0).toFixed(1);
    const yStr = (typeof surv.local_y === "number" ? surv.local_y : 0).toFixed(1);
    const confPct = Math.round((surv.confidence || 0.9) * 100);

    this.toastDetails.textContent = `ID: ${surv.id} | Grid: ${surv.grid_cell} | Local: (${xStr}m, ${yStr}m) | Conf: ${confPct}% | Src: ${surv.source || "AUTONOMOUS_VISION"}`;
    this.toast.classList.remove("hidden");

    if (this.toastTimer) clearTimeout(this.toastTimer);
    this.toastTimer = setTimeout(() => {
      this.hideToast();
    }, 5000);
  }

  hideToast() {
    if (this.toast) {
      this.toast.classList.add("hidden");
    }
  }

  reset() {
    this.survivors.clear();
    this._renderEmptyState();
    this.hideToast();
  }
}

window.GCSSurvivorManager = GCSSurvivorManager;
