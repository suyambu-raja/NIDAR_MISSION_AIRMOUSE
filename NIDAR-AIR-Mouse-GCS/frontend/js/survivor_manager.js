/**
 * Autonomous Survivor Localisation & Rescue Registry Controller for NIDAR M2 GCS.
 * Manages detailed survivor cards with explicit states (CONFIRMED, TRACKING, DETECTED, LOST, RESCUED),
 * confidence scores, 3D positions, detection modality (RGB + Thermal), track state, and distance.
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
    this.droneX = 0.0;
    this.droneY = 0.0;

    if (this.toastDismiss) {
      this.toastDismiss.addEventListener("click", () => this.hideToast());
    }

    this._renderEmptyState();
  }

  setDronePose(x, y) {
    this.droneX = x;
    this.droneY = y;
    this._updateAllDistances();
  }

  _renderEmptyState() {
    if (!this.container) return;
    this.container.innerHTML = `
      <div class="survivor-empty-placeholder">No survivors confirmed yet. Searching arena corridors...</div>
    `;
    if (this.summaryBadge) {
      this.summaryBadge.textContent = "SURVIVORS CONFIRMED: 0";
    }
  }

  addOrUpdateSurvivor(surv) {
    const isNew = !this.survivors.has(surv.id);
    const existing = this.survivors.get(surv.id) || {};

    const nowStr = new Date().toTimeString().split(" ")[0].slice(0, 5);
    const x = typeof surv.local_x === "number" ? surv.local_x : (surv.x || 0.0);
    const y = typeof surv.local_y === "number" ? surv.local_y : (surv.y || 0.0);
    const z = typeof surv.z === "number" ? surv.z : 0.0;
    const dist = Math.hypot(x - this.droneX, y - this.droneY).toFixed(1);

    const merged = {
      id: surv.id,
      local_x: x,
      local_y: y,
      z: z,
      grid_cell: surv.grid_cell || existing.grid_cell || "--",
      confidence: typeof surv.confidence === "number" ? surv.confidence : (existing.confidence || 0.9),
      source: surv.source || surv.detection_source || existing.source || "RGB + THERMAL",
      status: (surv.status || existing.status || "CONFIRMED").toUpperCase(),
      track: surv.track || existing.track || "ACTIVE",
      first_seen: existing.first_seen || nowStr,
      last_seen: nowStr,
      distance: dist,
    };

    this.survivors.set(surv.id, merged);

    if (!this.container) return;

    // Remove empty placeholder
    const placeholder = this.container.querySelector(".survivor-empty-placeholder");
    if (placeholder) {
      placeholder.remove();
    }

    // Find or create Card dynamically
    let card = document.getElementById(`card-${merged.id}`);
    if (!card) {
      card = document.createElement("div");
      card.id = `card-${merged.id}`;
      card.className = "survivor-card";
      this.container.appendChild(card);
    }

    const confPct = Math.round(merged.confidence * 100);

    card.innerHTML = `
      <div class="surv-card-top">
        <div class="surv-id-box">
          <span class="surv-id">${merged.id}</span>
          <span class="surv-status-badge surv-status-${merged.status}">${merged.status}</span>
        </div>
        <div class="surv-conf-box">
          <div class="surv-conf-bar-bg">
            <div class="surv-conf-bar-fill" style="width: ${confPct}%;"></div>
          </div>
          <span>${confPct}%</span>
        </div>
      </div>

      <div class="surv-card-mid">
        <span>Pos: <b>(${merged.local_x.toFixed(1)}, ${merged.local_y.toFixed(1)}, ${merged.z.toFixed(1)})</b></span>
        <span>Grid: <b>${merged.grid_cell}</b></span>
        <span>Dist: <b>${merged.distance}m</b></span>
      </div>

      <div class="surv-card-bottom">
        <span>Modality: <b>${merged.source}</b></span>
        <span>Track: <b>${merged.track}</b></span>
        <span>Seen: <b>${merged.first_seen} - ${merged.last_seen}</b></span>
      </div>
    `;

    card.onclick = () => {
      if (this.mapRenderer && typeof this.mapRenderer.focusOnSurvivor === "function") {
        this.mapRenderer.focusOnSurvivor(merged.local_x, merged.local_y);
      }
    };

    // Update Summary count
    let confirmedCount = 0;
    this.survivors.forEach((s) => {
      if (s.status === "CONFIRMED") confirmedCount++;
    });

    if (this.summaryBadge) {
      this.summaryBadge.textContent = `SURVIVORS CONFIRMED: ${confirmedCount} / TOTAL: ${this.survivors.size}`;
    }

    if (isNew) {
      this.showToast(merged);
    }
  }

  _updateAllDistances() {
    this.survivors.forEach((surv) => {
      const dist = Math.hypot(surv.local_x - this.droneX, surv.local_y - this.droneY).toFixed(1);
      surv.distance = dist;
      const card = document.getElementById(`card-${surv.id}`);
      if (card) {
        const distEl = card.querySelector(".surv-card-mid span:last-child");
        if (distEl) distEl.innerHTML = `Dist: <b>${dist}m</b>`;
      }
    });
  }

  showToast(surv) {
    if (!this.toast || !this.toastDetails) return;
    const confPct = Math.round(surv.confidence * 100);

    this.toastDetails.textContent = `${surv.id} [${surv.status}] at Grid ${surv.grid_cell} (${surv.local_x.toFixed(1)}m, ${surv.local_y.toFixed(1)}m) | Conf: ${confPct}% | ${surv.source}`;
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
}

window.GCSSurvivorManager = GCSSurvivorManager;
