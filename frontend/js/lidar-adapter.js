// lidar-adapter.js
// Generic LidarAdapter that connects to a WebSocket source, detects sensor type,
// parses raw data using registered parsers, and emits FusionFrames to the UI.
// NOTE: This adapter is only activated when window.useLidarAdapter === true.

// Registry of parsers – each returns an ArrayBuffer or Uint8Array.
const parsers = {
  // TR70: raw packet is already in FusionFrames binary format – pass through.
  TR70: async (raw) => raw,

  // Placeholder: naive float32 xyz triples, returned as-is.
  PLACEHOLDER: async (raw) => raw
};

export class LidarAdapter {
  constructor(options = {}) {
    this.wsUrl = options.wsUrl || 'ws://localhost:8000/lidar';
    this.parser = null;
    this.onFrameCallback = null;
    this._connect();
  }

  _connect() {
    this.socket = new WebSocket(this.wsUrl);
    this.socket.binaryType = 'arraybuffer';
    this.socket.onopen = () => console.log('LidarAdapter: WebSocket connected');
    this.socket.onmessage = async (event) => {
      const raw = event.data;
      if (!this.parser) {
        const type = this._detectSensorType(raw);
        console.log('LidarAdapter: detected sensor', type);
        this.parser = parsers[type] || parsers.PLACEHOLDER;
      }
      const fusionBuf = await this.parser(raw);
      if (this.onFrameCallback) this.onFrameCallback(fusionBuf);
    };
    this.socket.onclose = () => console.warn('LidarAdapter: WebSocket closed');
    this.socket.onerror = (e) => console.error('LidarAdapter error', e);
  }

  // Very simple heuristic based on packet size.
  _detectSensorType(raw) {
    const len = raw.byteLength;
    if (len === 64 || len === 76) return 'TR70';
    return 'PLACEHOLDER';
  }

  // Register a consumer of FusionFrames buffers.
  onFrame(cb) {
    this.onFrameCallback = cb;
  }
}

// Helper used by the UI to bootstrap the adapter.
export function initLidarAdapter({ onFrame }) {
  const adapter = new LidarAdapter();
  adapter.onFrame(onFrame);
  // expose globally for debugging
  window.lidarAdapter = adapter;
}
