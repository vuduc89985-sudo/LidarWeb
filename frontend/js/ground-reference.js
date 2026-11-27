/**
 * ground-reference.js
 * Quản lý tham chiếu đường bao đối tượng chiếu xuống đất (Ground Reference Manager)
 * 
 * Thiết kế tối ưu hiệu năng:
 * - KHÔNG chạy thuật toán nặng O(N^2) trong vòng lặp render() 60 FPS.
 * - Tích lũy đa khung thời gian O(1) buffer nhẹ.
 * - Tính toán đường bao ôm sát (Convex/Concave Hull, Z=0) TỨC THÌ THEO YÊU CẦU (On-Demand) khi chụp.
 * - Hỗ trợ cả ev.targets (vật thể di chuyển thời gian thực từ C++), fusedObjects, và điểm cụm.
 * - Xem lại 2D overlay và 3D Prism Extrusion.
 * - Quản lý danh sách lịch sử, lưu trữ LocalStorage, xuất/nạp JSON.
 * - Chuyển đổi trực tiếp thành Vùng loại trừ (Exclusion Zone) mà không làm loạn hệ thống cảnh báo backend.
 */

(function(root) {
  'use strict';

  // ================= THUẬT TOÁN HÌNH HỌC =================

  /**
   * Tính Convex Hull 2D bằng thuật toán Monotone Chain (Andrew's Algorithm)
   * O(N log N), cực nhanh và an toàn tuyệt đối.
   */
  function convexHull(points) {
    if (!points || points.length < 3) return points ? points.slice() : [];
    const pts = points.slice().sort((a, b) => a[0] === b[0] ? a[1] - b[1] : a[0] - b[0]);
    const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);

    const lower = [];
    for (const p of pts) {
      while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
        lower.pop();
      }
      lower.push(p);
    }

    const upper = [];
    for (let i = pts.length - 1; i >= 0; i--) {
      const p = pts[i];
      while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
        upper.pop();
      }
      upper.push(p);
    }

    lower.pop();
    upper.pop();
    return lower.concat(upper);
  }

  /**
   * Tính Concave Hull (Alpha Shape 2D) xấp xỉ ôm sát hình học thực tế
   * Tối ưu hóa: Downsample lưới điểm trước để O(N) rất nhỏ, chạy on-demand < 1ms
   */
  function concaveHull(points, alpha = 0.45) {
    if (!points || points.length < 4) return convexHull(points);

    // Grid downsample để giới hạn số điểm xử lý tối đa ~100 điểm
    const grid = new Map();
    const cellSize = 0.05; // 5cm
    for (const p of points) {
      const gx = Math.round(p[0] / cellSize);
      const gy = Math.round(p[1] / cellSize);
      const key = `${gx},${gy}`;
      if (!grid.has(key)) grid.set(key, [p[0], p[1]]);
    }
    const cleanPts = Array.from(grid.values());
    if (cleanPts.length < 4) return convexHull(points);

    // Nếu điểm còn quá nhiều, fallback về convexHull cho an toàn và tốc độ
    if (cleanPts.length > 250) {
      return convexHull(cleanPts);
    }

    const k = Math.min(Math.max(5, Math.floor(cleanPts.length / 4)), 16);
    let startIdx = 0;
    let minY = cleanPts[0][1];
    for (let i = 1; i < cleanPts.length; i++) {
      if (cleanPts[i][1] < minY) {
        minY = cleanPts[i][1];
        startIdx = i;
      }
    }

    const hull = [cleanPts[startIdx]];
    const used = new Set([startIdx]);
    let currentIdx = startIdx;
    let prevAngle = 0;
    const maxSteps = cleanPts.length * 2;
    let step = 0;

    while (step++ < maxSteps) {
      const cur = cleanPts[currentIdx];
      const neighbors = [];
      for (let i = 0; i < cleanPts.length; i++) {
        if (i === currentIdx) continue;
        const d2 = (cleanPts[i][0] - cur[0]) ** 2 + (cleanPts[i][1] - cur[1]) ** 2;
        if (d2 <= (alpha * 1.8) ** 2) {
          neighbors.push({ idx: i, d2, pt: cleanPts[i] });
        }
      }

      if (neighbors.length === 0) break;
      neighbors.sort((a, b) => a.d2 - b.d2);
      const candidates = neighbors.slice(0, k);

      let bestIdx = -1;
      let minAngleDiff = Infinity;

      for (const cand of candidates) {
        const dx = cand.pt[0] - cur[0];
        const dy = cand.pt[1] - cur[1];
        let angle = Math.atan2(dy, dx);
        let diff = angle - prevAngle;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        if (diff < 0) diff += 2 * Math.PI;

        if (cand.idx === startIdx && hull.length >= 3) {
          bestIdx = startIdx;
          break;
        }

        if (!used.has(cand.idx) && diff < minAngleDiff) {
          minAngleDiff = diff;
          bestIdx = cand.idx;
        }
      }

      if (bestIdx === -1 || bestIdx === startIdx) break;

      hull.push(cleanPts[bestIdx]);
      used.add(bestIdx);
      prevAngle = Math.atan2(cleanPts[bestIdx][1] - cur[1], cleanPts[bestIdx][0] - cur[0]);
      currentIdx = bestIdx;
    }

    if (hull.length < 3) return convexHull(points);
    return hull;
  }

  /**
   * Tính diện tích đa giác 2D (Shoelace formula) tính bằng m²
   */
  function polygonArea(contour) {
    if (!contour || contour.length < 3) return 0;
    let area = 0;
    for (let i = 0, j = contour.length - 1; i < contour.length; j = i++) {
      area += (contour[j][0] * contour[i][1]) - (contour[i][0] * contour[j][1]);
    }
    return Math.abs(area / 2);
  }

  /**
   * Tính chu vi đa giác 2D tính bằng mét
   */
  function polygonPerimeter(contour) {
    if (!contour || contour.length < 2) return 0;
    let perim = 0;
    for (let i = 0, j = contour.length - 1; i < contour.length; j = i++) {
      perim += Math.hypot(contour[i][0] - contour[j][0], contour[i][1] - contour[j][1]);
    }
    return perim;
  }

  /**
   * Tam giác hóa đa giác 2D cho nắp khối 3D Prism
   * Tự động nhận diện và xử lý cả thứ tự điểm chiều kim đồng hồ (CW) lẫn ngược chiều (CCW)
   */
  function triangulatePolygon(polygon) {
    if (!polygon || polygon.length < 3) return [];
    const n = polygon.length;
    if (n === 3) return [0, 1, 2];

    // Tính diện tích có dấu để nhận biết hướng CCW hay CW
    let signedArea = 0;
    for (let i = 0, j = n - 1; i < n; j = i++) {
      signedArea += (polygon[j][0] * polygon[i][1]) - (polygon[i][0] * polygon[j][1]);
    }

    // Nếu là CW, đảo ngược lại để thành CCW
    const poly = signedArea < 0 ? polygon.slice().reverse() : polygon.slice();
    const mapOrig = signedArea < 0 ? Array.from({ length: n }, (_, i) => n - 1 - i) : Array.from({ length: n }, (_, i) => i);

    const indices = Array.from({ length: n }, (_, i) => i);
    const triangles = [];

    const isEar = (u, v, w, curIndices) => {
      const ax = poly[u][0], ay = poly[u][1];
      const bx = poly[v][0], by = poly[v][1];
      const cx = poly[w][0], cy = poly[w][1];

      if ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax) <= 1e-7) return false;

      for (const idx of curIndices) {
        if (idx === u || idx === v || idx === w) continue;
        const px = poly[idx][0], py = poly[idx][1];
        const b0 = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
        const b1 = (cx - bx) * (py - by) - (cy - by) * (px - bx);
        const b2 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx);
        if (b0 >= 0 && b1 >= 0 && b2 >= 0) return false;
      }
      return true;
    };

    let count = 2 * n;
    while (indices.length > 2 && count-- > 0) {
      let earFound = false;
      for (let i = 0; i < indices.length; i++) {
        const u = indices[(i + indices.length - 1) % indices.length];
        const v = indices[i];
        const w = indices[(i + 1) % indices.length];

        if (isEar(u, v, w, indices)) {
          triangles.push(mapOrig[u], mapOrig[v], mapOrig[w]);
          indices.splice(i, 1);
          earFound = true;
          break;
        }
      }
      if (!earFound) break;
    }

    // Fallback quạt tam giác nếu còn sót đỉnh
    if (indices.length >= 3) {
      for (let i = 1; i < indices.length - 1; i++) {
        triangles.push(mapOrig[indices[0]], mapOrig[indices[i]], mapOrig[indices[i + 1]]);
      }
    }

    return triangles;
  }

  /**
   * Tính Oriented Bounding Box (OBB) - Hộp bao định hướng thực tế
   * Tính chính xác Chiều rộng (Width), Chiều dài (Length) và Góc xoay (Angle)
   * loại bỏ sai số phóng đại sqrt(2) của AABB thông thường.
   */
  function computeOBB(contour) {
    if (!contour || contour.length < 3) return { width: 0.5, length: 0.5, angleDeg: 0 };
    const n = contour.length;
    let cx = 0, cy = 0;
    for (const [x, y] of contour) { cx += x; cy += y; }
    cx /= n; cy /= n;

    let cxx = 0, cyy = 0, cxy = 0;
    for (const [x, y] of contour) {
      const dx = x - cx, dy = y - cy;
      cxx += dx * dx;
      cyy += dy * dy;
      cxy += dx * dy;
    }
    const theta = 0.5 * Math.atan2(2 * cxy, cxx - cyy);
    const cosT = Math.cos(theta), sinT = Math.sin(theta);

    let minU = Infinity, maxU = -Infinity, minV = Infinity, maxV = -Infinity;
    for (const [x, y] of contour) {
      const dx = x - cx, dy = y - cy;
      const u = dx * cosT + dy * sinT;
      const v = -dx * sinT + dy * cosT;
      if (u < minU) minU = u; if (u > maxU) maxU = u;
      if (v < minV) minV = v; if (v > maxV) maxV = v;
    }

    const spanU = maxU - minU;
    const spanV = maxV - minV;
    const length = Math.max(spanU, spanV);
    const width = Math.min(spanU, spanV);

    return {
      width: +width.toFixed(2),
      length: +length.toFixed(2),
      angleDeg: +(theta * 180 / Math.PI).toFixed(1)
    };
  }

  /**
   * Trích xuất các điểm thuộc về cơ thể của mục tiêu, tách rời triệt để khỏi tường tĩnh.
   * Thuật toán kết hợp:
   * 1. Bán kính lọc cứng: R = max(w/2, h/2, 0.45) + 0.15m quanh tâm (t.x, t.y).
   * 2. Tìm điểm mầm (seed point) gần tâm động học nhất.
   * 3. Cụm liên thông Euclidean BFS (epsilon = 0.28m): Nối thông ngực và lưng người (khe dày 18-25cm).
   * 4. Bộ lọc tuyến tính (Wall Line Filter): Nhận diện và loại trừ điểm tạo thành đường thẳng áp tường.
   */
  function extractTargetBodyPoints(t, frames) {
    const tCenterX = t.x, tCenterY = t.y;
    // Bán kính & Hộp cắt giới hạn động:
    // Đảm bảo bán kính tìm kiếm tối thiểu 0.38m (đường kính 76cm) để bao trọn cả 2 mặt trước (ngực)
    // và sau (lưng) của cơ thể khi 2 LiDAR quét đối diện nhau.
    const targetW = t.w || 0.35;
    const targetH = t.h || 0.35;
    const halfW = Math.max(targetW / 2, 0.20);
    const halfH = Math.max(targetH / 2, 0.20);
    const margin = 0.12; // Đệm an toàn 12cm xung quanh mục tiêu
    const maxRx = Math.max(halfW + margin, 0.38);
    const maxRy = Math.max(halfH + margin, 0.38);
    const maxR = Math.max(maxRx, maxRy);
    const maxR2 = maxR * maxR;

    const rawPts = [];
    let zMin = Number.isFinite(t.z1) ? t.z1 : 0;
    let zMax = Number.isFinite(t.z2) ? t.z2 : zMin + 1.6;

    for (const f of frames) {
      if (!f || !f.xyz || !f.n) continue;
      for (let i = 0; i < f.n; i++) {
        const px = f.xyz[i * 3];
        const py = f.xyz[i * 3 + 1];
        const pz = f.xyz[i * 3 + 2];
        const dx = px - tCenterX;
        const dy = py - tCenterY;
        // Lấy điểm trong phạm vi hộp động của mục tiêu
        if (Math.abs(dx) <= maxRx && Math.abs(dy) <= maxRy) {
          const d2 = dx * dx + dy * dy;
          if (d2 <= maxR2) {
            rawPts.push({ x: px, y: py, z: pz, d2 });
            if (pz < zMin) zMin = pz;
            if (pz > zMax) zMax = pz;
          }
        }
      }
    }

    if (rawPts.length < 3) {
      return { points: rawPts.map(p => [p.x, p.y]), zMin, zMax };
    }

    // Điểm mầm gần tâm động học nhất
    let bestIdx = 0;
    let minD2 = rawPts[0].d2;
    for (let i = 1; i < rawPts.length; i++) {
      if (rawPts[i].d2 < minD2) {
        minD2 = rawPts[i].d2;
        bestIdx = i;
      }
    }

    // Ngưỡng BFS thích ứng:
    // eps = 0.26m: Nối liền ngực và lưng xuyên qua khoảng rỗng bên trong cơ thể (18-25cm),
    // đồng thời không nhảy sang người khác hoặc bờ tường ở xa.
    const eps = 0.26;
    const eps2 = eps * eps;
    const visited = new Uint8Array(rawPts.length);
    const queue = [bestIdx];
    visited[bestIdx] = 1;
    const cluster = [];

    while (queue.length > 0) {
      const curIdx = queue.shift();
      const cur = rawPts[curIdx];
      cluster.push(cur);

      for (let i = 0; i < rawPts.length; i++) {
        if (!visited[i]) {
          const dist2 = (rawPts[i].x - cur.x) ** 2 + (rawPts[i].y - cur.y) ** 2;
          if (dist2 <= eps2) {
            visited[i] = 1;
            queue.push(i);
          }
        }
      }
    }

    // Tách lọc tường (Wall Filter)
    let finalPts = cluster;
    if (cluster.length >= 8) {
      const farPts = cluster.filter(p => p.d2 > (maxR * 0.5) ** 2);
      if (farPts.length >= 5) {
        let sx = 0, sy = 0;
        for (const p of farPts) { sx += p.x; sy += p.y; }
        sx /= farPts.length; sy /= farPts.length;
        let sxx = 0, syy = 0, sxy = 0;
        for (const p of farPts) {
          const dx = p.x - sx, dy = p.y - sy;
          sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
        }
        const wallAngle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
        const nx = -Math.sin(wallAngle), ny = Math.cos(wallAngle);
        let maxDev = 0;
        for (const p of farPts) {
          const dev = Math.abs((p.x - sx) * nx + (p.y - sy) * ny);
          if (dev > maxDev) maxDev = dev;
        }
        if (maxDev < 0.025) {
          finalPts = cluster.filter(p => {
            const dev = Math.abs((p.x - sx) * nx + (p.y - sy) * ny);
            const distToCenter = Math.hypot(p.x - tCenterX, p.y - tCenterY);
            return !(dev < 0.03 && distToCenter > maxR * 0.50);
          });
        }
      }
    }

    const clean2D = (finalPts.length >= 3 ? finalPts : cluster).map(p => [p.x, p.y]);
    return { points: clean2D, zMin, zMax };
  }

  /**
   * Đóng khối toàn diện (Solid Enclosing):
   * Khi vật thể chỉ được quét từ 1 phía (cung góc < 240 độ), hàm này tự động kéo giãn
   * mặt sau theo chiều sâu thực tế D dọc theo hướng nhìn tia quét, tạo thành khối hộp 3D
   * kín vững chắc, thể tích chuẩn xác, không bị xẹp lép dạng mỏng dẹt.
   */
  function encloseSolidContour(points, target, sensorOrigin = null) {
    const targetW = target?.w || 0.25;
    const targetH = target?.h || 0.20;

    if (!points || points.length < 3) {
      const hw = Math.max(targetW / 2, 0.05);
      const hh = Math.max(targetH / 2, 0.05);
      const tx = target?.x ?? 0, ty = target?.y ?? 0;
      return [
        [tx - hw, ty - hh],
        [tx + hw, ty - hh],
        [tx + hw, ty + hh],
        [tx - hw, ty + hh]
      ];
    }

    let cx = target?.x, cy = target?.y;
    if (!Number.isFinite(cx) || !Number.isFinite(cy)) {
      cx = 0; cy = 0;
      for (const [x, y] of points) { cx += x; cy += y; }
      cx /= points.length; cy /= points.length;
    }

    const angles = points.map(p => Math.atan2(p[1] - cy, p[0] - cx)).sort((a, b) => a - b);
    let maxGap = 0;
    for (let i = 0; i < angles.length; i++) {
      const nextAngle = (i === angles.length - 1) ? (angles[0] + 2 * Math.PI) : angles[i + 1];
      const gap = nextAngle - angles[i];
      if (gap > maxGap) maxGap = gap;
    }

    // Nếu điểm đã bao quanh >= 200 độ (maxGap <= 160 độ ~ 2.79 rad)
    // Tức là cả 2 LiDAR quét từ 2 phía đối diện (trước + sau) hoặc đã có độ dày
    if (maxGap <= (2 * Math.PI * 160 / 360)) {
      return concaveHull(points, 0.35);
    }

    // Nếu chỉ là một cung phía trước (1 LiDAR đơn lẻ nhìn từ 1 hướng):
    // Tự động tìm nguồn phát sensorOrigin gần nhất nếu chưa truyền vào
    if (!sensorOrigin && window.extrinsics && window.extrinsics.length) {
      let bestDist2 = Infinity;
      for (const e of window.extrinsics) {
        if (!e) continue;
        const ex = +(e.x ?? e.dx ?? 0), ey = +(e.y ?? e.dy ?? 0);
        const d2 = (cx - ex) ** 2 + (cy - ey) ** 2;
        if (d2 < bestDist2) {
          bestDist2 = d2;
          sensorOrigin = { x: ex, y: ey };
        }
      }
    }

    let viewDirX = 0, viewDirY = 1;
    if (sensorOrigin && Number.isFinite(sensorOrigin.x) && Number.isFinite(sensorOrigin.y)) {
      viewDirX = cx - sensorOrigin.x;
      viewDirY = cy - sensorOrigin.y;
    } else {
      const midAngle = (angles[0] + angles[angles.length - 1]) / 2;
      viewDirX = Math.cos(midAngle);
      viewDirY = Math.sin(midAngle);
    }
    const len = Math.hypot(viewDirX, viewDirY) || 1;
    viewDirX /= len; viewDirY /= len;

    // Chiều sâu đóng khối D: Tỷ lệ theo kích thước thực tế của đối tượng
    // Vật nhỏ (tay): chỉ dày 6-12cm; Người cả thân: dày 22-28cm
    const estDepth = targetW >= 0.28 ? Math.max(0.22, Math.min(targetH > 0.08 ? targetH : 0.25, 0.32)) : Math.min(targetH > 0.05 ? targetH : 0.12, targetW * 0.55);
    const targetDepth = Math.max(0.06, Math.min(estDepth, 0.30));

    const extrudedPts = [];
    for (const p of points) {
      extrudedPts.push([p[0], p[1]]);
      extrudedPts.push([p[0] + viewDirX * targetDepth, p[1] + viewDirY * targetDepth]);
    }

    return convexHull(extrudedPts);
  }

  // ================= 1.B: BỘ TÍCH LŨY ĐA KHUNG (TEMPORAL ACCUMULATOR) =================

  class TemporalAccumulator {
    constructor(maxFrames = 20, maxHistoryMs = 2000) {
      this.maxFrames = maxFrames;
      this.maxHistoryMs = maxHistoryMs;
      // Map: objectKey -> [ { timestamp, points: [[dx, dy], ...], bbox: {x,y,w,h,z1,z2} } ]
      this.history = new Map();
    }

    /**
     * Nạp nhanh O(1) buffer điểm tương đối trong hệ tọa độ mục tiêu (dx, dy)
     * Khi mục tiêu di chuyển, các điểm tích lũy gom quanh tâm (0,0) tạo hình dạng 360 độ hoàn chỉnh.
     */
    bufferTarget(targetKey, relativePoints, bbox, now = performance.now()) {
      if (!this.history.has(targetKey)) {
        this.history.set(targetKey, []);
      }
      const queue = this.history.get(targetKey);
      queue.push({ timestamp: now, points: relativePoints || [], bbox });

      while (queue.length > this.maxFrames || (queue.length > 1 && (now - queue[0].timestamp) > this.maxHistoryMs)) {
        queue.shift();
      }
    }

    /**
     * Dọn dẹp các target đã mất dấu
     */
    cleanStale(now = performance.now()) {
      for (const [key, queue] of this.history) {
        if (!queue.length || (now - queue[queue.length - 1].timestamp > 1500)) {
          this.history.delete(key);
        }
      }
    }

    clear() {
      this.history.clear();
    }
  }

  // ================= QUẢN LÝ THAM CHIẾU (GROUND REFERENCE MANAGER) =================

  class GroundReferenceManager {
    constructor() {
      this.accumulator = new TemporalAccumulator(20, 2000);
      this.references = [];
      this.nextId = 1;
      this.palette = ['#e67e22', '#2ecc71', '#9b59b6', '#1abc9c', '#f1c40f', '#e74c3c', '#3498db'];
      this.onReferencesChanged = null;

      // Cấu hình Tự động chụp (Auto-Capture)
      // Mặc định: 'dwell' (Chỉ chụp khi dừng lại >= 2s để tránh rác sàn nhà khi đi lại)
      this.autoCapture = {
        enabled: false,           // Mặc định tắt (giữ nguyên chụp thủ công), bật khi cần
        mode: 'dwell',            // 'dwell' (khi dừng lại), 'alarm' (khi có báo động), 'exit' (khi thoát vùng), 'all' (thông minh)
        dwellTimeSec: 2.0,        // Thời gian đứng yên tối thiểu để chụp (giây)
        cooldownMs: 8000,         // Thời gian chờ tránh chụp liên tục cùng 1 target
        maxItems: 20              // Giới hạn số lượng tham chiếu tự động tối đa (tự cuốn chiếu)
      };
      this.targetDwell = new Map(); // targetId -> { firstSeen, firstStillTime, lastCaptureTime, capturedModes, dwellRefId, dwellStartX, dwellStartY }
      this.liveProfiles = new Map(); // targetId -> { id, centroid, contour, obb, zMin, zMax, height, pointCount, firstSeen, maxSpeed }

      this.loadFromStorage();
    }

    /**
     * Cập nhật tracking thời gian thực mỗi khung hình (Phương án 1 + Mốc 1)
     * - Quản lý Hồ sơ Động theo thời gian thực (Live Profile) bám sát người
     * - Dọn dẹp tự động khi người thoát vùng quét
     * - Chạy siêu nhẹ O(N điểm cục bộ) < 0.5ms trong render()
     */
    updateTracking(ev, frames = [], now = performance.now()) {
      try {
        const activeEv = ev || window._ev;
        if (!activeEv || !activeEv.targets || !activeEv.targets.length) {
          if (this.autoCapture?.enabled) this.cleanStaleDwell(now, activeEv);
          this.liveProfiles.clear();
          return;
        }
        if (!frames || !frames.length) return;

        const activeIds = new Set();
        for (const t of activeEv.targets) {
          if (t.age < 2) continue;
          activeIds.add(t.id);
          const res = extractTargetBodyPoints(t, frames);
          if (res.points && res.points.length >= 3) {
            // Lưu tọa độ tương đối (dx, dy) so với tâm động học hiện tại
            const relPts = res.points.map(p => [+(p[0] - t.x).toFixed(3), +(p[1] - t.y).toFixed(3)]);
            this.accumulator.bufferTarget(`TARGET_${t.id}`, relPts, t, now);

            // Cập nhật Hồ sơ Động theo thời gian thực (Live Dynamic Profile) khi đã xác thực >= 4 frames
            if (t.age >= 4) {
              let sensorOrigin = null;
              if (window.extrinsics && window.extrinsics[0]) {
                const e = window.extrinsics[0];
                sensorOrigin = { x: +(e.x ?? e.dx ?? 0), y: +(e.y ?? e.dy ?? 0) };
              }
              const contour = encloseSolidContour(res.points, t, sensorOrigin);
              if (contour.length >= 3) {
                const obb = computeOBB(contour);
                const speed = Math.hypot(t.vx || 0, t.vy || 0);
                let prof = this.liveProfiles.get(t.id);
                if (!prof) {
                  prof = {
                    id: t.id,
                    firstSeen: now,
                    lastSeen: now,
                    maxSpeed: speed,
                    centroid: { x: +t.x.toFixed(3), y: +t.y.toFixed(3) },
                    contour,
                    obb,
                    zMin: res.zMin,
                    zMax: res.zMax,
                    pointCount: res.points.length
                  };
                } else {
                  prof.lastSeen = now;
                  prof.centroid = { x: +t.x.toFixed(3), y: +t.y.toFixed(3) };
                  prof.contour = contour;
                  prof.obb = obb;
                  prof.zMin = res.zMin;
                  prof.zMax = res.zMax;
                  prof.pointCount = res.points.length;
                  if (speed > prof.maxSpeed) prof.maxSpeed = speed;
                }
                this.liveProfiles.set(t.id, prof);
              }
            }
          }
        }
        this.accumulator.cleanStale(now);

        // Dọn dẹp các liveProfiles khi đối tượng đã biến mất hoặc thoát vùng
        for (const [id, prof] of this.liveProfiles) {
          if (!activeIds.has(id)) {
            const durationSec = +((now - prof.firstSeen) / 1000).toFixed(1);
            if (durationSec >= 0.6) {
              const isQuick = durationSec <= 4.0;
              const tag = isQuick ? 'LƯỚT QUA NHANH' : 'KẾT THÚC THEO VẾT';
              if (typeof addLog === 'function') {
                addLog(`[${tag}] Mục tiêu #${id}: ${prof.obb.width}x${prof.obb.length}m | Vmax: ${prof.maxSpeed.toFixed(1)}m/s | Hiện diện: ${durationSec}s`);
              }
            }
            this.liveProfiles.delete(id);
          }
        }

        // Kiểm tra và thực hiện Tự động chụp nếu được bật
        if (this.autoCapture && this.autoCapture.enabled) {
          this._checkAutoCapture(activeEv, frames, now);
        }
      } catch (err) {
        console.warn('[GroundReferenceManager] updateTracking error:', err);
      }
    }

    /**
     * Logic Tự động chụp Thông minh (Phương án 1 + Mốc 1):
     * - Khi đối tượng đang đi lại: KHÔNG sinh tem rải rác trên sàn nhà
     * - Khi đối tượng DỪNG LẠI (Dwell >= 2s): Chốt duy nhất 1 mốc Dwell tại vị trí đứng
     * - Nếu đứng yên tiếp tục: Cập nhật thời gian dừng trên cùng 1 thẻ, không tạo thẻ trùng
     */
    _checkAutoCapture(ev, frames, now) {
      if (!ev || !ev.targets || !ev.targets.length) return;
      const cfg = this.autoCapture;
      const dwellSecThreshold = cfg.dwellTimeSec || 2.0;

      for (const t of ev.targets) {
        // Với Báo động: chỉ cần đối tượng đã xác thực (age >= 3) là chụp ngay
        // Với Dừng lại (dwell): yêu cầu age >= 5 để ổn định
        const minAge = (cfg.mode === 'alarm' || cfg.mode === 'all') ? 3 : 5;
        if (t.age < minAge) continue;
        const speed = Math.hypot(t.vx || 0, t.vy || 0);
        if (t.realMotion !== undefined && !t.realMotion) continue;

        let state = this.targetDwell.get(t.id);
        if (!state) {
          state = { 
            firstSeen: now, 
            firstStillTime: null, 
            lastCaptureTime: 0, 
            capturedModes: new Set(),
            dwellRefId: null,
            alarmRefId: null,
            bestPointCount: t.n || 0,
            bestArea: 0,
            dwellStartX: t.x,
            dwellStartY: t.y
          };
          this.targetDwell.set(t.id, state);
        }

        // 1. CHẾ ĐỘ MỐC 1: DỪNG LẠI / ĐỨNG YÊN (DWELL >= 2.0s) [Khuyên dùng cho nhà kho & hàng rào]
        if (cfg.mode === 'dwell' || cfg.mode === 'all') {
          if (speed < 0.12) {
            if (!state.firstStillTime) {
              state.firstStillTime = now;
              state.dwellStartX = t.x;
              state.dwellStartY = t.y;
            }
            const currentStillSec = (now - state.firstStillTime) / 1000;

            if (currentStillSec >= dwellSecThreshold) {
              // Tìm xem đã có thẻ Dwell nào cho đợt dừng này chưa
              let existingDwell = state.dwellRefId ? this.references.find(r => r.id === state.dwellRefId) : null;
              if (!existingDwell) {
                // Kiểm tra theo khoảng cách (trong vòng 35cm quanh điểm bắt đầu dừng)
                existingDwell = this.references.find(r => r.isAuto && r.isDwell && Math.hypot(r.centroid.x - t.x, r.centroid.y - t.y) <= 0.35);
              }

              if (existingDwell) {
                // ĐÃ CÓ: CHỈ CẬP NHẬT THỜI GIAN DỪNG - TUYỆT ĐỐI KHÔNG TẠO THẺ MỚI
                const roundedSec = Math.round(currentStillSec);
                existingDwell.dwellDurationSec = roundedSec;
                existingDwell.name = `Mục tiêu #${t.id} [DỪNG LẠI ${roundedSec}s] (${existingDwell.obb.width}x${existingDwell.obb.length}m)`;
                if (now - (existingDwell._lastUiUpdate || 0) > 1000) {
                  existingDwell._lastUiUpdate = now;
                  if (this.onReferencesChanged) this.onReferencesChanged(this.references);
                }
              } else if (!state.capturedModes.has('dwell_done_for_stop')) {
                // CHƯA CÓ: Chụp DUY NHẤT 1 bản tại điểm dừng này
                state.capturedModes.add('dwell_done_for_stop');
                this._pruneAutoReferences();
                const captured = this.captureSnapshot(t.id, ev, frames, { 
                  isAuto: true, 
                  isDwell: true,
                  reason: `Dừng lại ${Math.round(currentStillSec)}s` 
                });
                if (captured && captured.length) {
                  const cap = captured[0];
                  cap.isDwell = true;
                  cap.dwellDurationSec = Math.round(currentStillSec);
                  state.dwellRefId = cap.id;
                  if (typeof addLog === 'function') {
                    addLog(`[DỪNG LẠI] 🛑 Mục tiêu #${t.id} dừng lại tại (${t.x.toFixed(1)}, ${t.y.toFixed(1)}): ${cap.obb.width}x${cap.obb.length}m`);
                  }
                }
              }
            }
          } else {
            // Khi di chuyển trở lại (vận tốc > 0.15 m/s) và đã rời xa điểm dừng cũ > 35cm
            if (state.firstStillTime) {
              const movedDist = Math.hypot(t.x - state.dwellStartX, t.y - state.dwellStartY);
              if (movedDist > 0.35) {
                state.capturedModes.delete('dwell_done_for_stop');
                state.dwellRefId = null;
                state.firstStillTime = null;
              }
            }
          }
        }

        // 2. CHẾ ĐỘ BÁO ĐỘNG ZONE (Chụp ngay khi có xâm nhập và báo động Zone)
        if (cfg.mode === 'alarm' || cfg.mode === 'all') {
          const isExcl = typeof window.isTargetInExclusionZone === 'function' ? window.isTargetInExclusionZone(t) : false;
          // Nhận diện báo động từ window.alarmOn, ev.confirmed, hoặc t.age >= 3
          const isInAlarm = (!!window.alarmOn || (ev && ev.confirmed > 0)) && !isExcl;
          if (isInAlarm) {
            let existingAlarmRef = state.alarmRefId ? this.references.find(r => r.id === state.alarmRefId) : null;
            if (!existingAlarmRef) {
              existingAlarmRef = this.references.find(r => r.isAuto && r.targetId === t.id);
            }

            // Chụp ngay bản đầu tiên khi có báo động (chỉ ghi nhận khi chụp thành công vượt qua bộ lọc)
            if (!existingAlarmRef && !state.capturedModes.has('alarm')) {
              this._pruneAutoReferences();
              const captured = this.captureSnapshot(t.id, ev, frames, { isAuto: true, reason: 'Báo động' });
              if (captured && captured.length) {
                state.capturedModes.add('alarm');
                state.alarmRefId = captured[0].id;
                state.lastCaptureTime = now;
                state.bestPointCount = captured[0].pointCount || t.n || 0;
                state.bestArea = captured[0].area || 0;
                if (typeof addLog === 'function') {
                  addLog(`[BÁO ĐỘNG] 🚨 Tự động chụp ${captured[0].id}: ${captured[0].name}`);
                }
              }
            } else if (existingAlarmRef && (now - state.lastCaptureTime >= 1000)) {
              // Tự động nâng cấp snapshot nếu mây điểm đối tượng rõ hơn (>= 30%)
              const curPts = t.n || 0;
              const prevPts = state.bestPointCount || existingAlarmRef.pointCount || 5;
              if (curPts >= prevPts * 1.30) {
                state.lastCaptureTime = now;
                const captured = this.captureSnapshot(t.id, ev, frames, { 
                  isAuto: true, 
                  reason: 'Báo động hoàn thiện', 
                  upgradeRefId: existingAlarmRef.id 
                });
                if (captured && captured.length) {
                  state.bestPointCount = Math.max(state.bestPointCount, captured[0].pointCount || curPts);
                }
              }
            }
          }
        }
      }

      this.cleanStaleDwell(now, ev);
    }

    cleanStaleDwell(now, ev) {
      const activeIds = new Set((ev?.targets || []).map(t => t.id));
      for (const [id, state] of this.targetDwell) {
        if (!activeIds.has(id) && (now - (state.lastCaptureTime || state.firstSeen) > 15000)) {
          this.targetDwell.delete(id);
        }
      }
    }

    _pruneAutoReferences() {
      const maxAuto = (this.autoCapture && this.autoCapture.maxItems) || 20;
      const autoRefs = this.references.filter(r => r.isAuto);
      if (autoRefs.length >= maxAuto) {
        const excess = autoRefs.length - maxAuto + 1;
        let removed = 0;
        this.references = this.references.filter(r => {
          if (r.isAuto && removed < excess) {
            removed++;
            return false;
          }
          return true;
        });
      }
    }

    setAutoCaptureConfig(cfg) {
      if (!cfg || typeof cfg !== 'object') return;
      Object.assign(this.autoCapture, cfg);
      this.saveToStorage();
      if (this.onReferencesChanged) this.onReferencesChanged(this.references);
    }

    /**
     * 1.B + 1.C: Chụp snapshot đường bao đối tượng đang chuyển động hoặc toàn bộ
     * Đóng khối toàn diện, tách tường, tính kích thước chuẩn xác OBB.
     */
    captureSnapshot(targetIdOrKey = null, ev = null, frames = [], meta = {}) {
      const activeEv = ev || window._ev || {};
      const targets = activeEv.targets || [];
      const capturedList = [];
      const isAuto = !!meta?.isAuto;
      const autoReason = meta?.reason || '';

      // Lấy danh sách frame thực tế đang có
      const currentFrames = (frames && frames.length) ? frames : [];
      if (!currentFrames.length && typeof getFrame === 'function') {
        const count = window.lidarCount || 1;
        for (let i = 0; i < count; i++) {
          const f = getFrame(i);
          if (f) currentFrames.push(f);
        }
      }

      // Xác định mục tiêu cần chụp
      let targetsToCapture = [];
      if (targets && targets.length) {
        if (targetIdOrKey !== null) {
          targetsToCapture = targets.filter(t => t.id === targetIdOrKey || `TARGET_${t.id}` === targetIdOrKey);
        } else {
          targetsToCapture = targets.filter(t => t.age >= 2);
          if (!targetsToCapture.length) targetsToCapture = targets;
        }
      }

      // 1. Trường hợp có mục tiêu chuyển động trong targets
      for (const t of targetsToCapture) {
        const tKey = `TARGET_${t.id}`;
        const bodyData = extractTargetBodyPoints(t, currentFrames);
        const currentBodyPts = bodyData.points;
        const zMin = bodyData.zMin;
        const zMax = bodyData.zMax;

        // Lưu frame hiện tại vào bộ tích lũy tương đối
        if (currentBodyPts.length >= 3) {
          const relPts = currentBodyPts.map(p => [+(p[0] - t.x).toFixed(3), +(p[1] - t.y).toFixed(3)]);
          this.accumulator.bufferTarget(tKey, relPts, t);
        }

        // Gom toàn bộ điểm tích lũy qua các khung gần nhất và chiếu về vị trí hiện tại
        const histQueue = this.accumulator.history.get(tKey) || [];
        const allAccumPts = [];
        for (const h of histQueue) {
          for (const rp of h.points) {
            allAccumPts.push([t.x + rp[0], t.y + rp[1]]);
          }
        }

        // Ưu tiên mây điểm tức thời của khung hình hiện tại (currentBodyPts) để phản ánh
        // chính xác kích thước thật của vật thể, tránh vệt quét kéo dài theo chuyển động (motion smear)
        const ptsToUse = currentBodyPts.length >= 4 ? currentBodyPts : (allAccumPts.length >= 4 ? allAccumPts : currentBodyPts);

        // Xác định cảm biến chính quan sát
        let sensorOrigin = null;
        if (window.extrinsics && window.extrinsics[0]) {
          const e = window.extrinsics[0];
          sensorOrigin = { x: +(e.x ?? e.dx ?? 0), y: +(e.y ?? e.dy ?? 0) };
        }

        // Đóng khối toàn diện
        const contour = encloseSolidContour(ptsToUse, t, sensorOrigin);

        if (contour.length >= 3) {
          const candidateObb = computeOBB(contour);
          const candidateArea = polygonArea(contour);

          // BỘ LỌC CHẤT LƯỢNG TỐI THIỂU CHO CHỤP TỰ ĐỘNG:
          // Loại bỏ mẩu viền / sliver cực mỏng (ví dụ 0.01x0.26m) khi người mới thò ngón chân qua vạch zone.
          // Chờ khung hình tiếp theo khi thân người vào đủ sâu (ít nhất 0.14m dày, 0.22m rộng, 0.035m² diện tích).
          if (isAuto) {
            const minDim = Math.min(candidateObb.width, candidateObb.length);
            const maxDim = Math.max(candidateObb.width, candidateObb.length);
            if (minDim < 0.14 || maxDim < 0.22 || candidateArea < 0.035) {
              continue;
            }
          }

          const upgradeId = meta?.upgradeRefId;
          let ref = null;
          if (upgradeId) {
            ref = this.references.find(r => r.id === upgradeId);
          }
          if (!ref && isAuto) {
            // Tìm thẻ tự động đã tồn tại của mục tiêu này
            ref = this.references.find(r => r.isAuto && r.targetId === t.id);
            if (!ref) {
              // Hoặc thẻ tự động vừa tạo trong 15s gần nhất ở cự ly <= 0.40m
              const recentThresh = Date.now() - 15000;
              ref = this.references.find(r => r.isAuto && r.timestamp >= recentThresh && Math.hypot(r.centroid.x - t.x, r.centroid.y - t.y) <= 0.40);
              if (ref) {
                ref.targetId = t.id;
              }
            }
          }

          if (ref) {
            // Nâng cấp tại chỗ (In-place Upgrade) bản chụp cũ - ĐẢM BẢO 1 NGƯỜI CHỈ CÓ 1 THẺ DUY NHẤT
            const obb = candidateObb;
            const area = candidateArea;
            const perimeter = polygonPerimeter(contour);
            let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
            for (const [x, y] of contour) {
              if (x < minX) minX = x; if (x > maxX) maxX = x;
              if (y < minY) minY = y; if (y > maxY) maxY = y;
            }

            // Chỉ cập nhật hình học nếu bản chụp mới có độ rõ nét / số điểm tương đương hoặc cao hơn
            const shouldUpdateGeometry = !ref.pointCount || ptsToUse.length >= ref.pointCount * 0.85 || area >= (ref.area || 0) * 0.85;
            if (shouldUpdateGeometry) {
              ref.contour = contour.map(p => [+p[0].toFixed(3), +p[1].toFixed(3)]);
              ref.zMin = +zMin.toFixed(2);
              ref.zMax = +zMax.toFixed(2);
              ref.height = +(zMax - zMin).toFixed(2);
              ref.obb = obb;
              ref.bbox = {
                minX: +minX.toFixed(3), maxX: +maxX.toFixed(3),
                minY: +minY.toFixed(3), maxY: +maxY.toFixed(3),
                width: obb.width, length: obb.length
              };
              ref.pointCount = ptsToUse.length;
              ref.area = +area.toFixed(2);
              ref.perimeter = +perimeter.toFixed(2);
            }
            ref.centroid = { x: +t.x.toFixed(3), y: +t.y.toFixed(3) };
            ref.timestamp = Date.now();
            ref.targetId = t.id;
            if (meta?.isDwell) {
              ref.isDwell = true;
              if (meta.dwellDurationSec) ref.dwellDurationSec = meta.dwellDurationSec;
            }
            const activeReason = autoReason || ref.autoReason || 'Tự động';
            ref.name = `Mục tiêu #${t.id} [${activeReason}] (${ref.obb.width}x${ref.obb.length}m)`;
            capturedList.push(ref);
          } else {
            const namePrefix = isAuto ? `Mục tiêu #${t.id} [${autoReason}]` : `Mục tiêu #${t.id}`;
            ref = this._buildReferenceObject(namePrefix, contour, t.x, t.y, zMin, zMax, ptsToUse.length, { isAuto, reason: autoReason });
            ref.targetId = t.id;
            if (meta?.isDwell) {
              ref.isDwell = true;
              if (meta.dwellDurationSec) ref.dwellDurationSec = meta.dwellDurationSec;
            }
            this.references.push(ref);
            capturedList.push(ref);
          }
        }
      }

      // 2. Trường hợp không có targets nhưng có fusedObjects (chỉ áp dụng khi chụp thủ công)
      if (!isAuto && !capturedList.length && window.fusedObjects && window.fusedObjects.length) {
        for (const obj of window.fusedObjects) {
          const pts = (obj.points || []).map(p => [p.x, p.y]);
          const contour = pts.length >= 4 ? encloseSolidContour(pts, { x: obj.center.x, y: obj.center.y, w: obj.width, h: obj.length }) : convexHull(pts);
          if (contour.length >= 3) {
            const ref = this._buildReferenceObject(`Vật thể ROI Z${obj.roiId}`, contour, obj.center.x, obj.center.y, 0, 1.5, pts.length, { isAuto: false });
            this.references.push(ref);
            capturedList.push(ref);
          }
        }
      }

      if (capturedList.length) {
        this.saveToStorage();
        if (this.onReferencesChanged) this.onReferencesChanged(this.references);
      }

      return capturedList;
    }

    /**
     * Chụp đường bao tại một tọa độ chỉ định (khi Double-Click chuột trên bản đồ)
     */
    captureAtCoordinate(wx, wy, radius = 1.0, frames = []) {
      const currentFrames = (frames && frames.length) ? frames : [];
      if (!currentFrames.length && typeof getFrame === 'function') {
        const count = window.lidarCount || 1;
        for (let i = 0; i < count; i++) {
          const f = getFrame(i);
          if (f) currentFrames.push(f);
        }
      }

      const ptsNear = [];
      let zMin = Infinity, zMax = -Infinity;
      for (const f of currentFrames) {
        if (!f || !f.xyz || !f.n) continue;
        for (let i = 0; i < f.n; i++) {
          const px = f.xyz[i * 3];
          const py = f.xyz[i * 3 + 1];
          const pz = f.xyz[i * 3 + 2];
          if (Math.hypot(px - wx, py - wy) <= radius) {
            ptsNear.push([px, py]);
            if (pz < zMin) zMin = pz;
            if (pz > zMax) zMax = pz;
          }
        }
      }

      let contour = [];
      if (ptsNear.length >= 4) {
        contour = encloseSolidContour(ptsNear, { x: wx, y: wy, w: 0.5, h: 0.3 });
      } else {
        const hr = 0.25;
        contour = [
          [wx - hr, wy - hr],
          [wx + hr, wy - hr],
          [wx + hr, wy + hr],
          [wx - hr, wy + hr]
        ];
      }

      if (contour.length >= 3) {
        const ref = this._buildReferenceObject(`Vị trí (${wx.toFixed(1)}, ${wy.toFixed(1)})`, contour, wx, wy, 
          Number.isFinite(zMin) ? zMin : 0, Number.isFinite(zMax) ? zMax : 1.5, ptsNear.length);
        this.references.push(ref);
        this.saveToStorage();
        if (this.onReferencesChanged) this.onReferencesChanged(this.references);
        return [ref];
      }
      return [];
    }

    /**
     * Chụp trực tiếp từ đối tượng Target hoặc Snapshot lịch sử (dùng trong Camera AI & Mini Inspector)
     */
    captureFromTargetData(targetData, customName = null) {
      if (!targetData) return null;
      const t = targetData;
      const w = t.w || 0.40;
      const d = t.h || 0.30;
      const z1 = Number.isFinite(t.z1) ? t.z1 : 0.0;
      const z2 = Number.isFinite(t.z2) ? t.z2 : (z1 + 1.65);
      const cx = t.x ?? 0;
      const cy = t.y ?? 0;

      // Lấy mây điểm nếu có
      let pts = [];
      if (t.fused && t.fused.allPts && t.fused.allPts.length >= 3) {
        pts = t.fused.allPts.map(p => [p.x, p.y]);
      } else if (t.points && t.points.length >= 3) {
        pts = t.points.map(p => Array.isArray(p) ? [p[0], p[1]] : [p.x, p.y]);
      }

      let contour = [];
      if (pts.length >= 3) {
        contour = encloseSolidContour(pts, t);
      } else {
        const hw = w / 2, hd = d / 2;
        contour = [
          [cx - hw, cy - hd],
          [cx + hw, cy - hd],
          [cx + hw, cy + hd],
          [cx - hw, cy + hd]
        ];
      }

      const defaultName = customName || `Mục tiêu #${t.id} [Camera AI]`;
      const ref = this._buildReferenceObject(defaultName, contour, cx, cy, z1, z2, pts.length || t.n || 0, { isAuto: false, reason: 'Camera AI Capture' });
      ref.targetId = t.id;
      ref.fromCameraAI = true;
      this.references.push(ref);
      this.saveToStorage();
      if (this.onReferencesChanged) this.onReferencesChanged(this.references);
      return ref;
    }

    _buildReferenceObject(namePrefix, contour, cx, cy, zMin, zMax, ptCount, meta = {}) {
      const now = new Date();
      const timeStr = now.toLocaleTimeString('vi-VN', { hour12: false });
      const isAuto = !!meta?.isAuto;
      const refId = `${isAuto ? 'AUTO' : 'REF'}-${String(this.nextId++).padStart(3, '0')}`;
      const color = isAuto ? '#8e44ad' : this.palette[(this.references.length) % this.palette.length];

      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (const [x, y] of contour) {
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
      }

      const area = polygonArea(contour);
      const perimeter = polygonPerimeter(contour);
      const obb = computeOBB(contour);
      const height = Number.isFinite(zMax - zMin) && (zMax - zMin) > 0.05 ? (zMax - zMin) : 1.5;

      return {
        id: refId,
        isAuto,
        autoReason: meta?.reason || '',
        name: `${namePrefix} (${obb.width}x${obb.length}m)`,
        createdAt: timeStr,
        timestamp: Date.now(),
        color,
        visible2D: true,
        visible3D: true,
        contour: contour.map(p => [+p[0].toFixed(3), +p[1].toFixed(3)]),
        centroid: { x: +cx.toFixed(3), y: +cy.toFixed(3) },
        obb: {
          width: obb.width,
          length: obb.length,
          angleDeg: obb.angleDeg
        },
        bbox: {
          minX: +minX.toFixed(3),
          maxX: +maxX.toFixed(3),
          minY: +minY.toFixed(3),
          maxY: +maxY.toFixed(3),
          width: obb.width,
          length: obb.length
        },
        zMin: +zMin.toFixed(2),
        zMax: +(zMin + height).toFixed(2),
        height: +height.toFixed(2),
        area: +area.toFixed(2),
        perimeter: +perimeter.toFixed(2),
        pointCount: ptCount
      };
    }

    /**
     * Bật / tắt hiển thị 2D
     */
    toggle2D(refId, visible = null) {
      const ref = this.references.find(r => r.id === refId);
      if (ref) {
        ref.visible2D = visible !== null ? visible : !ref.visible2D;
        this.saveToStorage();
        if (this.onReferencesChanged) this.onReferencesChanged(this.references);
      }
    }

    /**
     * Bật / tắt hiển thị 3D
     */
    toggle3D(refId, visible = null) {
      const ref = this.references.find(r => r.id === refId);
      if (ref) {
        ref.visible3D = visible !== null ? visible : !ref.visible3D;
        this.saveToStorage();
        if (this.onReferencesChanged) this.onReferencesChanged(this.references);
      }
    }

    /**
     * Xóa 1 đối tượng tham chiếu
     */
    remove(refId) {
      const idx = this.references.findIndex(r => r.id === refId);
      if (idx !== -1) {
        this.references.splice(idx, 1);
        this.saveToStorage();
        if (this.onReferencesChanged) this.onReferencesChanged(this.references);
      }
    }

    /**
     * Xóa toàn bộ lịch sử tham chiếu
     */
    clearAll() {
      this.references = [];
      this.saveToStorage();
      if (this.onReferencesChanged) this.onReferencesChanged(this.references);
    }

    /**
     * Xuất danh sách tham chiếu thành file JSON
     */
    exportJSON() {
      const data = {
        exportedAt: new Date().toISOString(),
        version: '1.0',
        count: this.references.length,
        references: this.references
      };
      const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `lidar_references_${new Date().toISOString().slice(0, 10)}.json`;
      a.click();
      URL.revokeObjectURL(url);
    }

    /**
     * Nạp dữ liệu tham chiếu từ file JSON
     */
    importJSON(jsonText) {
      try {
        const data = json.parse(jsonText);
        const list = Array.isArray(data) ? data : (data.references || []);
        if (!Array.isArray(list)) return false;

        for (const item of list) {
          if (item && item.contour && item.contour.length >= 3) {
            item.id = item.id || `REF-${String(this.nextId++).padStart(3, '0')}`;
            item.visible2D = item.visible2D ?? true;
            item.visible3D = item.visible3D ?? true;
            item.color = item.color || this.palette[this.references.length % this.palette.length];
            this.references.push(item);
          }
        }
        this.saveToStorage();
        if (this.onReferencesChanged) this.onReferencesChanged(this.references);
        return true;
      } catch (err) {
        console.error('Lỗi nạp file tham chiếu JSON:', err);
        return false;
      }
    }

    saveToStorage() {
      try {
        localStorage.setItem('lidar_ground_references', JSON.stringify(this.references));
        localStorage.setItem('lidar_ground_ref_autocfg', JSON.stringify(this.autoCapture));
      } catch (e) {}
    }

    loadFromStorage() {
      try {
        const raw = localStorage.getItem('lidar_ground_references');
        if (raw) {
          const list = JSON.parse(raw);
          if (Array.isArray(list)) {
            this.references = list;
            let maxId = 0;
            for (const r of list) {
              const m = r.id && r.id.match(/(?:REF|AUTO)-(\d+)/);
              if (m) maxId = Math.max(maxId, parseInt(m[1], 10));
            }
            this.nextId = maxId + 1;
          }
        }
        const autoCfgRaw = localStorage.getItem('lidar_ground_ref_autocfg');
        if (autoCfgRaw) {
          const cfg = JSON.parse(autoCfgRaw);
          if (cfg && typeof cfg === 'object') {
            Object.assign(this.autoCapture, cfg);
          }
        }
      } catch (e) {}
    }

    /**
     * 2.A: Vẽ lớp phủ tham chiếu 2D trên Canvas
     * Cực nhẹ: chỉ duyệt các reference đã lưu và vẽ viền nét đứt màu sắc nổi bật
     */
    draw2D(ctx, p, v, w2s) {
      ctx.save();

      // 1. VẼ CÁC HỒ SƠ ĐỘNG THEO THỜI GIAN THỰC (Live Dynamic Profiles - Phương án 1)
      // Chạy bám sát người khi đang di chuyển, tuyệt đối không rải tem chết trên sàn
      if (this.liveProfiles && this.liveProfiles.size > 0) {
        for (const [id, prof] of this.liveProfiles) {
          if (!prof.contour || prof.contour.length < 3) continue;
          const col = '#00e5ff'; // Cyan neon nổi bật

          // Vùng mờ bao quanh đối tượng theo thời gian thực
          ctx.fillStyle = 'rgba(0, 229, 255, 0.12)';
          ctx.strokeStyle = col;
          ctx.lineWidth = 1.8;
          ctx.setLineDash([4, 2]);

          ctx.beginPath();
          prof.contour.forEach((pt, idx) => {
            const [sx, sy] = w2s(v, p, pt[0], pt[1]);
            if (idx === 0) ctx.moveTo(sx, sy);
            else ctx.lineTo(sx, sy);
          });
          ctx.closePath();
          ctx.fill();
          ctx.stroke();

          // Nhãn kích thước thời gian thực chạy theo đầu người
          if (prof.centroid) {
            const [cx, cy] = w2s(v, p, prof.centroid.x, prof.centroid.y);
            ctx.setLineDash([]);
            ctx.font = 'bold 11px Segoe UI, sans-serif';
            const labelText = `⚡ #${id} | ${prof.obb.width}x${prof.obb.length}m`;
            const tw = ctx.measureText(labelText).width;

            ctx.fillStyle = 'rgba(0, 15, 25, 0.82)';
            ctx.fillRect(cx - tw / 2 - 4, cy - 22, tw + 8, 16);
            ctx.fillStyle = '#00e5ff';
            ctx.fillText(labelText, cx - tw / 2, cy - 10);
          }
        }
      }

      // 2. VẼ CÁC THAM CHIẾU ĐÃ LƯU (Dwell snapshots & Thủ công)
      const visibleRefs = (this.references || []).filter(r => r.visible2D && r.contour && r.contour.length >= 3);
      if (!visibleRefs.length) {
        ctx.restore();
        return;
      }

      for (const ref of visibleRefs) {
        const col = ref.color || (ref.isAuto ? '#8e44ad' : '#e67e22');

        // Vẽ vùng mờ bên trong
        ctx.fillStyle = col + '22';
        ctx.strokeStyle = col;
        ctx.lineWidth = 2.0;
        ctx.setLineDash([6, 3]);

        ctx.beginPath();
        ref.contour.forEach((pt, idx) => {
          const [sx, sy] = w2s(v, p, pt[0], pt[1]);
          if (idx === 0) ctx.moveTo(sx, sy);
          else ctx.lineTo(sx, sy);
        });
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        // Vẽ các đỉnh chốt (chỉ vẽ khi số đỉnh <= 36 để duy trì 60 FPS mượt mà)
        if (ref.contour.length <= 36) {
          ctx.setLineDash([]);
          ctx.fillStyle = col;
          ref.contour.forEach(pt => {
            const [sx, sy] = w2s(v, p, pt[0], pt[1]);
            ctx.fillRect(sx - 2.5, sy - 2.5, 5, 5);
          });
        }

        // Nhãn thông tin tham chiếu
        if (ref.centroid) {
          const [cx, cy] = w2s(v, p, ref.centroid.x, ref.centroid.y);
          ctx.font = 'bold 11px Segoe UI, sans-serif';
          const icon = ref.isAuto ? '⚡' : '📸';
          const labelText = `${icon} ${ref.id} | ${ref.bbox?.width || 0}x${ref.bbox?.length || 0}m (${ref.area || 0}m²)`;
          const tw = ctx.measureText(labelText).width;

          ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
          ctx.fillRect(cx - tw / 2 - 4, cy - 20, tw + 8, 16);
          ctx.fillStyle = '#fff';
          ctx.fillText(labelText, cx - tw / 2, cy - 8);
        }
      }
      ctx.restore();
    }
  }

  // Export module
  root.GroundReferenceManager = GroundReferenceManager;
  root.GroundReferenceUtils = {
    convexHull,
    concaveHull,
    polygonArea,
    polygonPerimeter,
    triangulatePolygon,
    computeOBB
  };

  if (typeof module !== 'undefined' && module.exports) {
    module.exports = { GroundReferenceManager, GroundReferenceUtils: root.GroundReferenceUtils };
  }
})(globalThis);
