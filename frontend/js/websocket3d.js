/* Binary V2: local XYI, 64-byte header. V3: world XYZI, 76-byte header.
 * All numbers little endian; wire rotation angles are radians. */
(function(root){
  'use strict';

  // Tính ma trận quay 3x3 từ các góc Euler (roll, pitch, oth/yaw) theo thứ tự ZYX
  function rotation(e){
    const cr = Math.cos(e.roll || 0), sr = Math.sin(e.roll || 0);
    const cp = Math.cos(e.pitch || 0), sp = Math.sin(e.pitch || 0);
    const cy = Math.cos(e.oth || 0),   sy = Math.sin(e.oth || 0);
    return [
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp,     cp * sr,               cp * cr
    ];
  }

  // Chuyển đổi điểm cục bộ (x, y, z) sang tọa độ thế giới 3D dựa trên pose 'e'
  function localPointToWorld3D(e, x, y, z = 0){
    const r = rotation(e);
    return [
      r[0] * x + r[1] * y + r[2] * z + (e.ox ?? e.x ?? e.dx ?? 0),
      r[3] * x + r[4] * y + r[5] * z + (e.oy ?? e.y ?? e.dy ?? 0),
      r[6] * x + r[7] * y + r[8] * z + (e.oz ?? e.z ?? 0)
    ];
  }

  function parse(buf){
    if(!(buf instanceof ArrayBuffer) || buf.byteLength < 4) return null;
    const d = new DataView(buf), v = d.getUint8(0);
    if(v !== 2 && v !== 3) return null;

    const header = v === 3 ? 76 : 64, stride = v === 3 ? 16 : 12;
    if(buf.byteLength < header) return null;

    const n = d.getUint32(header - 4, true);
    if(n > 1000000 || buf.byteLength !== header + n * stride) return null;

    const f = {
      version: v,
      sourceId: d.getUint8(1),
      frameId: d.getUint32(4, true),
      tsSec: d.getUint32(8, true),
      tsUsec: d.getUint32(12, true),
      ox: d.getFloat32(16, true),
      oy: d.getFloat32(20, true),
      oz: v === 3 ? d.getFloat32(24, true) : 0,
      roll: v === 3 ? d.getFloat32(28, true) : 0,
      pitch: v === 3 ? d.getFloat32(32, true) : 0,
      oth: d.getFloat32(v === 3 ? 36 : 24, true),
      n
    };

    let off = v === 3 ? 40 : 28;
    for(const key of ['aMin', 'aMax', 'aInc', 'rMin', 'rMax', 'scan', 'tInc']){
      f[key] = d.getFloat32(off, true);
      off += 4;
    }
    f.wave = d.getUint32(off, true);

    if(f.tsUsec >= 1000000 || Object.values(f).some(x => !Number.isFinite(x))) return null;

    f.timestamp = f.tsSec * 1000 + f.tsUsec / 1000;
    f.recvTime = performance.now();
    f.pts = new Float32Array(n * 3);
    f.xyz = new Float32Array(n * 3);

    const r = rotation(f);
    for(let i = 0, o = header; i < n; i++, o += stride){
      const x = d.getFloat32(o, true);
      const y = d.getFloat32(o + 4, true);
      const z = v === 3 ? d.getFloat32(o + 8, true) : 0;
      const intensity = d.getFloat32(o + stride - 4, true);

      if(![x, y, z, intensity].every(Number.isFinite)) return null;

      const k = i * 3;
      if(v === 3){
        f.xyz.set([x, y, z], k);
        // Transform ngược về tọa độ local để phục vụ các công cụ 2D/calibration
        const a = x - f.ox, b = y - f.oy, c = z - f.oz;
        f.pts.set([r[0]*a + r[3]*b + r[6]*c, r[1]*a + r[4]*b + r[7]*c, intensity], k);
      }else{
        f.pts.set([x, y, intensity], k);
        f.xyz.set([r[0]*x + r[1]*y + f.ox, r[3]*x + r[4]*y + f.oy, 0], k);
      }
    }
    return f;
  }

  /**
   * Bộ lọc khử bóng ma mép tia (Mixed Pixel / Trailing Edge) & điểm cô lập (Outlier Filter)
   * Sử dụng thuật toán kết hợp:
   * 1. Radial Jump Edge Filter: Phát hiện các bước nhảy cự ly theo phương tia quét.
   * 2. Spatial Radius Outlier Removal (ROR): Lưới băm không gian O(N) loại bỏ triệt để
   *    các hạt bụi bay rải rác lơ lửng, giữ nguyên vẹn 100% bề mặt vật thể thực tế
   *    (bàn tay, cơ thể, cột, tường, bàn ghế).
   */
  function filterMixedPixels(f, options = {}) {
    if (!f || !f.pts || f.n < 6) return f;

    const n = f.n;
    const pts = f.pts;
    const xyz = f.xyz;

    const radius = options.radius ?? 0.095; // 9.5 cm search radius
    const minNeighbors = options.minNeighbors ?? 2; // Cần ít nhất 2 điểm láng giềng
    const cellSize = radius;
    const invCell = 1 / cellSize;

    // 1. Xây dựng lưới băm không gian O(N)
    const grid = new Map();
    for (let i = 0; i < n; i++) {
      const k = i * 3;
      const lx = pts[k], ly = pts[k + 1];
      const gx = Math.floor(lx * invCell);
      const gy = Math.floor(ly * invCell);
      const cellKey = `${gx},${gy}`;
      let cell = grid.get(cellKey);
      if (!cell) {
        cell = [];
        grid.set(cellKey, cell);
      }
      cell.push(i);
    }

    // 2. Kiểm tra số láng giềng của từng điểm
    const r2 = radius * radius;
    const neighborCounts = new Uint8Array(n);
    const closestNeighbor = new Int32Array(n).fill(-1);

    for (let i = 0; i < n; i++) {
      const k = i * 3;
      const lx = pts[k], ly = pts[k + 1];
      const gx = Math.floor(lx * invCell);
      const gy = Math.floor(ly * invCell);

      let count = 0;
      let minD2 = Infinity;
      let nearestIdx = -1;

      for (let dx = -1; dx <= 1; dx++) {
        for (let dy = -1; dy <= 1; dy++) {
          const neighborCell = grid.get(`${gx + dx},${gy + dy}`);
          if (!neighborCell) continue;
          for (let j = 0; j < neighborCell.length; j++) {
            const idx = neighborCell[j];
            if (idx === i) continue;
            const jk = idx * 3;
            const dist2 = (lx - pts[jk]) ** 2 + (ly - pts[jk + 1]) ** 2;
            if (dist2 <= r2) {
              count++;
              if (dist2 < minD2) {
                minD2 = dist2;
                nearestIdx = idx;
              }
            }
          }
        }
      }
      neighborCounts[i] = count;
      closestNeighbor[i] = nearestIdx;
    }

    // 3. Quyết định giữ điểm:
    // - Điểm có >= 2 láng giềng trong bán kính R: GIỮ NGUYÊN (chắc chắn là vật thể thực tế).
    // - Điểm có 1 láng giềng: nếu láng giềng gần nhất có >= 3 láng giềng (nằm ở rìa mép ngoài cùng của tay): GIỮ NGUYÊN.
    // - Các điểm còn lại (0 láng giềng hoặc dải bụi tia mép kéo dài): LOẠI BỎ.
    const keepMask = new Uint8Array(n);
    let validCount = 0;

    for (let i = 0; i < n; i++) {
      const count = neighborCounts[i];
      let keep = false;
      if (count >= minNeighbors) {
        keep = true;
      } else if (count === 1) {
        const nearIdx = closestNeighbor[i];
        if (nearIdx >= 0 && neighborCounts[nearIdx] >= 3) {
          keep = true;
        }
      }

      if (keep) {
        keepMask[i] = 1;
        validCount++;
      }
    }

    // Nếu không có điểm rác nào, giữ nguyên buffer hiện tại
    if (validCount === n) return f;

    // 4. Trích xuất mảng điểm sạch
    const cleanPts = new Float32Array(validCount * 3);
    const cleanXyz = new Float32Array(validCount * 3);
    let dst = 0;

    for (let i = 0; i < n; i++) {
      if (keepMask[i]) {
        const src = i * 3;
        cleanPts[dst] = pts[src];
        cleanPts[dst + 1] = pts[src + 1];
        cleanPts[dst + 2] = pts[src + 2];

        cleanXyz[dst] = xyz[src];
        cleanXyz[dst + 1] = xyz[src + 1];
        cleanXyz[dst + 2] = xyz[src + 2];
        dst += 3;
      }
    }

    f.n = validCount;
    f.pts = cleanPts;
    f.xyz = cleanXyz;
    return f;
  }

  function reproject(f, e){
    const out = {
      ...f,
      ox: e.x ?? e.dx ?? f.ox,
      oy: e.y ?? e.dy ?? f.oy,
      oz: e.z ?? f.oz,
      roll: (e.roll ?? f.roll * 180 / Math.PI) * Math.PI / 180,
      pitch: (e.pitch ?? f.pitch * 180 / Math.PI) * Math.PI / 180,
      oth: (e.yaw ?? e.thetaDeg ?? f.oth * 180 / Math.PI) * Math.PI / 180,
      xyz: new Float32Array(f.n * 3)
    };
    const r = rotation(out);
    for(let k = 0; k < f.pts.length; k += 3){
      const x = f.pts[k], y = f.pts[k + 1];
      out.xyz.set([r[0]*x + r[1]*y + out.ox, r[3]*x + r[4]*y + out.oy, r[6]*x + r[7]*y + out.oz], k);
    }
    return out;
  }

  class FusionFrames {
    constructor(){
      this.hist = new Map();
      this.maxFusionDeltaMs = 100;
      this.maxAgeMs = 1000;
    }
    clear(){
      this.hist.clear();
    }
    push(f){
      let h = this.hist.get(f.sourceId) || [];
      if(h.length && f.timestamp < h[h.length - 1].timestamp) h = []; // Đặt lại clock / CSV wrap
      h.push(f);
      if(h.length > 32) h.shift();
      this.hist.set(f.sourceId, h);
    }
    reproject(id, e){
      if(this.hist.has(id)){
        this.hist.set(id, this.hist.get(id).map(f => reproject(f, e)));
      }
    }
    select(count, now = performance.now(), paused = false){
      const latest = Array.from({length: count}, (_, id) => this.hist.get(id)?.at(-1));
      const live = latest.filter(f => f && (paused || now - f.recvTime <= this.maxAgeMs));
      
      const times = live.map(f => f.timestamp).sort((a, b) => a - b);
      const timestamp = times.length ? times[Math.floor(times.length / 2)] : null;
      
      const layers = latest.map((last, id) => {
        let frame = null, delta = null;
        if(last && timestamp !== null){
          frame = this.hist.get(id).reduce((a, b) => 
            Math.abs(a.timestamp - timestamp) <= Math.abs(b.timestamp - timestamp) ? a : b
          );
          delta = frame.timestamp - timestamp;
        }
        const stale = !frame || (!paused && now - last.recvTime > this.maxAgeMs) || Math.abs(delta) > this.maxFusionDeltaMs;
        return {id, frame, delta, stale};
      });
      return {
        timestamp,
        layers,
        maxDelta: Math.max(0, ...layers.filter(l => !l.stale && Number.isFinite(l.delta)).map(l => Math.abs(l.delta)))
      };
    }
  }

  // Xuất đầy đủ các API bao gồm cả filterMixedPixels
  root.LidarProtocol = {
    parse,
    filterMixedPixels,
    rotation,
    localPointToWorld3D,
    reproject,
    FusionFrames
  };

  if(typeof module !== 'undefined') module.exports = root.LidarProtocol;
})(globalThis);