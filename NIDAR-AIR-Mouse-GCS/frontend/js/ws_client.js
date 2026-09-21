/**
 * Robust WebSocket Client for NIDAR AirMouse GCS.
 * Implements auto-reconnect, exponential backoff, message dispatching, and heartbeat.
 */
class GCSWebSocketClient {
  constructor(url = null) {
    const loc = window.location;
    const protocol = loc.protocol === "https:" ? "wss:" : "ws:";
    this.url = url || `${protocol}//${loc.host}/ws`;
    this.ws = null;
    this.listeners = new Map();
    this.reconnectAttempts = 0;
    this.maxReconnectDelay = 5000;
    this.reconnectTimer = null;
    this.isConnected = false;
  }

  on(messageType, callback) {
    if (!this.listeners.has(messageType)) {
      this.listeners.set(messageType, []);
    }
    this.listeners.get(messageType).push(callback);
  }

  connect() {
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    try {
      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        this.isConnected = true;
        this.reconnectAttempts = 0;
        console.log("[WS] Connected to GCS Backend");
        this._emit("_status", { connected: true });
      };

      this.ws.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          const type = payload.type || "_raw";
          this._emit(type, payload);
        } catch (e) {
          console.warn("[WS] Failed to parse message:", e);
        }
      };

      this.ws.onclose = () => {
        this.isConnected = false;
        this._emit("_status", { connected: false });
        this._scheduleReconnect();
      };

      this.ws.onerror = (err) => {
        console.warn("[WS] Error:", err);
      };
    } catch (e) {
      console.error("[WS] Connection initiation error:", e);
      this._scheduleReconnect();
    }
  }

  _scheduleReconnect() {
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    const delay = Math.min(1000 * Math.pow(1.5, this.reconnectAttempts), this.maxReconnectDelay);
    this.reconnectAttempts++;
    this.reconnectTimer = setTimeout(() => {
      this.connect();
    }, delay);
  }

  send(action, params = {}) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action, params }));
    } else {
      console.warn("[WS] Cannot send message - socket not open");
    }
  }

  _emit(type, data) {
    if (this.listeners.has(type)) {
      for (const cb of this.listeners.get(type)) {
        try {
          cb(data);
        } catch (err) {
          console.error(`[WS] Error in handler for ${type}:`, err);
        }
      }
    }
  }
}

window.GCSWebSocketClient = GCSWebSocketClient;
