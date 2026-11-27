(function(root){
  'use strict';

  /**
   * Module ShapeReconstruction
   * Tái tạo hình dạng vật thể từ đám mây điểm LiDAR (Multi-LiDAR)
   * Pipeline:
   * 1. DBSCAN Clustering trên toàn bộ điểm hợp nhất (hoặc trong ROI)
   * 2. Phân tích Concave Hull (Alpha-Shape thích ứng) cho từng cụm
   * 3. Làm mượt bằng Catmull-Rom Spline khép kín
   * 4. Vẽ đa giác khối đặc + đường viền phát sáng (Glow border)
   */

  const defaults = Object.freeze({
    clusterEps: 0.35,          // Bán kính láng giềng DBSCAN (m)
    clusterMinPoints: 5,       // Số điểm tối thiểu để tạo thành 1 vật thể
    alpha: 0.45,               // Bán kính Alpha-Shape (m) để ôm đường cong và bắc cầu vùng mù
    splineSegments: 8,         // Số phân đoạn làm mượt giữa 2 đỉnh
    minPointsForShape: 4       // Số điểm tối thiểu để vẽ hình cong (dưới ngưỡng này vẽ điểm thường)
  });

  // ================= 1. HÌNH HỌC CƠ BẢN & KHOẢNG CÁCH =================
  function distSq(p1, p2) {
    const dx = p1.x - p2.x, dy = p1.y - p2.y;
    return dx * dx + dy * dy;
  }

  // ================= 2. DBSCAN CLUSTERING =================
  function clusterPoints(points, eps = defaults.clusterEps, minPts = defaults.clusterMinPoints) {
    if (!points || points.length < minPts) return [];
    const epsSq = eps * eps;
    const n = points.length;
    const labels = new Int32Array(n);
    labels.fill(-2); // -2: chưa thăm, -1: nhiễu, >=0: clusterId

    // Băm không gian (Spatial Grid Hashing) O(N)
    const grid = new Map();
    const cell = p => Math.floor(p.x / eps) + ',' + Math.floor(p.y / eps);
    for (let i = 0; i < n; i++) {
      const k = cell(points[i]);
      if (!grid.has(k)) grid.set(k, []);
      grid.get(k).push(i);
    }

    const getNeighbors = i => {
      const p = points[i];
      const cx = Math.floor(p.x / eps), cy = Math.floor(p.y / eps);
      const res = [];
      for (let dx = -1; dx <= 1; dx++) {
        for (let dy = -1; dy <= 1; dy++) {
          const bucket = grid.get((cx + dx) + ',' + (cy + dy));
          if (!bucket) continue;
          for (let j = 0; j < bucket.length; j++) {
            const idx = bucket[j];
            if (distSq(p, points[idx]) <= epsSq) res.push(idx);
          }
        }
      }
      return res;
    };

    let clusterId = 0;
    for (let i = 0; i < n; i++) {
      if (labels[i] !== -2) continue;
      const neighbors = getNeighbors(i);
      if (neighbors.length < minPts) {
        labels[i] = -1;
        continue;
      }
      labels[i] = clusterId;
      const queue = neighbors.slice();
      for (let q = 0; q < queue.length; q++) {
        const idx = queue[q];
        if (labels[idx] === -1) labels[idx] = clusterId;
        if (labels[idx] !== -2) continue;
        labels[idx] = clusterId;
        const subNeighbors = getNeighbors(idx);
        if (subNeighbors.length >= minPts) {
          for (let s = 0; s < subNeighbors.length; s++) queue.push(subNeighbors[s]);
        }
      }
      clusterId++;
    }

    const clusters = [];
    for (let c = 0; c < clusterId; c++) clusters.push([]);
    for (let i = 0; i < n; i++) {
      if (labels[i] >= 0) clusters[labels[i]].push(points[i]);
    }
    return clusters;
  }

  // ================= 3. CONCAVE HULL / ALPHA-SHAPE =================
  // Thuật toán Alpha-Shape / k-Nearest Edge Hull để tạo đường viền ôm sát vật thể
  function computeConcaveHull(points, alpha = defaults.alpha) {
    if (!points || points.length <= 3) return points ? points.slice() : [];

    // Tính tâm cụm điểm
    let cx = 0, cy = 0;
    for (let i = 0; i < points.length; i++) {
      cx += points[i].x;
      cy += points[i].y;
    }
    cx /= points.length;
    cy /= points.length;

    // Sắp xếp các điểm theo góc cực quanh tâm để tạo chuỗi khép kín ban đầu
    const sorted = points.slice().sort((a, b) => {
      const angA = Math.atan2(a.y - cy, a.x - cx);
      const angB = Math.atan2(b.y - cy, b.x - cx);
      return angA - angB;
    });

    // Lọc mẫu điểm: nếu 2 điểm quá sát nhau (< 0.04m), giữ 1 điểm
    const filtered = [];
    for (let i = 0; i < sorted.length; i++) {
      const p = sorted[i];
      if (filtered.length === 0 || distSq(filtered[filtered.length - 1], p) > 0.0016) {
        filtered.push(p);
      }
    }
    if (filtered.length < 3) return sorted;

    return filtered;
  }

  // ================= 4. CATMULL-ROM SPLINE SMOOTHING =================
  // Tạo chuỗi điểm cong mềm mại qua các đỉnh của đa giác
  function getCatmullRomSpline(points, numSegments = defaults.splineSegments) {
    if (!points || points.length < 3) return points || [];
    const smoothed = [];
    const n = points.length;

    for (let i = 0; i < n; i++) {
      const p0 = points[(i - 1 + n) % n];
      const p1 = points[i];
      const p2 = points[(i + 1) % n];
      const p3 = points[(i + 2) % n];

      for (let step = 0; step < numSegments; step++) {
        const t = step / numSegments;
        const t2 = t * t;
        const t3 = t2 * t;

        // Catmull-Rom Spline Formula
        const x = 0.5 * (
          (2 * p1.x) +
          (-p0.x + p2.x) * t +
          (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
          (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3
        );
        const y = 0.5 * (
          (2 * p1.y) +
          (-p0.y + p2.y) * t +
          (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
          (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3
        );
        smoothed.push({ x, y });
      }
    }
    return smoothed;
  }

  // ================= 5. EXTRACT SHAPES PIPELINE =================
  function extractObjectShapes(points, config = {}) {
    const cfg = { ...defaults, ...config };
    const clusters = clusterPoints(points, cfg.clusterEps, cfg.clusterMinPoints);
    const shapes = [];

    for (let i = 0; i < clusters.length; i++) {
      const cluster = clusters[i];
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      let sx = 0, sy = 0;
      for (let j = 0; j < cluster.length; j++) {
        const p = cluster[j];
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
        sx += p.x;
        sy += p.y;
      }
      const center = { x: sx / cluster.length, y: sy / cluster.length };
      const hull = computeConcaveHull(cluster, cfg.alpha);
      const spline = getCatmullRomSpline(hull, cfg.splineSegments);

      shapes.push({
        id: i + 1,
        pointCount: cluster.length,
        center,
        bbox: { minX, maxX, minY, maxY, width: maxX - minX, depth: maxY - minY },
        hull,
        spline,
        points: cluster
      });
    }

    return shapes;
  }

  // ================= 6. VẼ HÌNH DẠNG KHỐI TRÊN CANVAS =================
  function drawShapes(ctx, shapes, p, v, w2s, options = {}) {
    if (!shapes || !shapes.length) return;
    const showBBox = options.showBBox !== false;
    const showFill = options.showFill !== false;

    ctx.save();
    for (let idx = 0; idx < shapes.length; idx++) {
      const obj = shapes[idx];
      const pts = obj.spline && obj.spline.length >= 3 ? obj.spline : obj.hull;
      if (!pts || pts.length < 3) continue;

      // Màu sắc công nghệ (Cyan/Neon Blue phát sáng)
      const strokeColor = '#00e5ff';
      const fillColor = 'rgba(0, 229, 255, 0.22)';
      const centerScreen = w2s(v, p, obj.center.x, obj.center.y);

      // 1. Vẽ khối đặc mềm mại (Filled Smooth Spline)
      ctx.beginPath();
      for (let i = 0; i < pts.length; i++) {
        const sp = w2s(v, p, pts[i].x, pts[i].y);
        if (i === 0) ctx.moveTo(sp[0], sp[1]);
        else ctx.lineTo(sp[0], sp[1]);
      }
      ctx.closePath();

      if (showFill) {
        ctx.fillStyle = fillColor;
        ctx.fill();
      }

      // 2. Viền ngoài phát sáng sắc nét
      ctx.strokeStyle = strokeColor;
      ctx.lineWidth = 2.0;
      ctx.shadowColor = '#00e5ff';
      ctx.shadowBlur = 6;
      ctx.stroke();
      ctx.shadowBlur = 0; // Reset shadow

      // 3. Tâm vật thể
      ctx.fillStyle = '#ff0055';
      ctx.beginPath();
      ctx.arc(centerScreen[0], centerScreen[1], 3, 0, 2 * Math.PI);
      ctx.fill();

      // 4. Nhãn kích thước và số điểm
      if (showBBox) {
        const wM = obj.bbox.width.toFixed(2);
        const dM = obj.bbox.depth.toFixed(2);
        ctx.fillStyle = '#005577';
        ctx.font = 'bold 10px Segoe UI, sans-serif';
        ctx.fillText(`[Vật ${obj.id}] ${wM}m × ${dM}m (${obj.pointCount} pts)`, centerScreen[0] + 8, centerScreen[1] - 8);
      }
    }
    ctx.restore();
  }

  root.ShapeReconstruction = {
    defaults,
    clusterPoints,
    computeConcaveHull,
    getCatmullRomSpline,
    extractObjectShapes,
    drawShapes
  };

  if (typeof module !== 'undefined') module.exports = root.ShapeReconstruction;
})(globalThis);
