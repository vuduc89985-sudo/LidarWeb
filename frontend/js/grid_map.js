/**
 * Grid Map Engine (Occupancy Grid Map - OGM)
 * Thuật toán Log-Odds Ray-Tracing (Bresenham) 2D chuẩn ROS / Robotics
 * Tự động tổng hợp dữ liệu mây điểm từ tất cả LiDAR sau khi Fusion / Auto Align
 */
(function(root) {
  'use strict';

  class OccupancyGridMap {
    constructor(options = {}) {
      this.resolution = options.resolution || 0.05; // 0.05m = 5cm / cell (Chuẩn ROS)
      this.minX = options.minX || -15.0;
      this.maxX = options.maxX || 15.0;
      this.minY = options.minY || -15.0;
      this.maxY = options.maxY || 15.0;

      // Tham số xác suất Log-Odds (Moravec & Elfes, Thrun)
      this.lFree = -0.40;   // Giảm log-odds dọc theo tia quét (Free space)
      this.lOcc  =  1.20;   // Tăng log-odds tại điểm phản xạ cuối (Occupied / Wall)
      this.lMin  = -4.00;   // Giới hạn dưới xác suất (~0.018)
      this.lMax  =  4.00;   // Giới hạn trên xác suất (~0.982)
      this.threshOcc  =  0.80; // Ngưỡng coi là vật cản/tường
      this.threshFree = -0.80; // Ngưỡng coi là vùng trống đi lại

      this.isMapping = false;
      this.frameCount = 0;
      this.showMap = true;
      this.dirty = true;

      // Offscreen canvas để cache hình vẽ bản đồ
      this.canvas = document.createElement('canvas');
      this.ctx = this.canvas.getContext('2d');

      this.initGrid();
    }

    initGrid(minX, maxX, minY, maxY, resolution) {
      if (minX !== undefined) this.minX = minX;
      if (maxX !== undefined) this.maxX = maxX;
      if (minY !== undefined) this.minY = minY;
      if (maxY !== undefined) this.maxY = maxY;
      if (resolution !== undefined) this.resolution = resolution;

      this.widthCells = Math.max(10, Math.ceil((this.maxX - this.minX) / this.resolution));
      this.heightCells = Math.max(10, Math.ceil((this.maxY - this.minY) / this.resolution));
      this.logOdds = new Float32Array(this.widthCells * this.heightCells); // Mặc định 0.0 = Unknown

      this.canvas.width = this.widthCells;
      this.canvas.height = this.heightCells;
      this.dirty = true;
      this.frameCount = 0;
    }

    clear() {
      if (this.logOdds) this.logOdds.fill(0.0);
      this.frameCount = 0;
      this.dirty = true;
      this.updateCanvas();
    }

    // Tự động tính toán vùng bao quanh (Bounding Box) tối ưu từ mây điểm hiện tại
    autoFitBounds(frames, extrinsics, margin = 2.0) {
      let bMinX = 1e9, bMaxX = -1e9, bMinY = 1e9, bMaxY = -1e9;
      let hasPts = false;

      // Lấy vị trí các LiDAR
      if (extrinsics) {
        for (const e of extrinsics) {
          if (!e) continue;
          const ex = +(e.dx ?? e.x ?? 0), ey = +(e.dy ?? e.y ?? 0);
          bMinX = Math.min(bMinX, ex); bMaxX = Math.max(bMaxX, ex);
          bMinY = Math.min(bMinY, ey); bMaxY = Math.max(bMaxY, ey);
          hasPts = true;
        }
      }

      // Lấy các điểm quét
      if (frames) {
        for (const f of frames) {
          if (!f || !f.xyz) continue;
          for (let i = 0; i < f.xyz.length; i += 3) {
            const x = f.xyz[i], y = f.xyz[i + 1];
            if (Math.hypot(x, y) < 0.15) continue;
            bMinX = Math.min(bMinX, x); bMaxX = Math.max(bMaxX, x);
            bMinY = Math.min(bMinY, y); bMaxY = Math.max(bMaxY, y);
            hasPts = true;
          }
        }
      }

      if (!hasPts) return;

      // Làm tròn ra số nguyên mét kèm lề
      const newMinX = Math.floor(bMinX - margin);
      const newMaxX = Math.ceil(bMaxX + margin);
      const newMinY = Math.floor(bMinY - margin);
      const newMaxY = Math.ceil(bMaxY + margin);

      this.initGrid(newMinX, newMaxX, newMinY, newMaxY, this.resolution);
    }

    // Chuyển tọa độ thế giới (m) -> ô lưới (cell)
    worldToGrid(wx, wy) {
      const gx = Math.floor((wx - this.minX) / this.resolution);
      const gy = Math.floor((wy - this.minY) / this.resolution);
      return [gx, gy];
    }

    // Chuyển ô lưới -> tọa độ thế giới (m) tâm ô
    gridToWorld(gx, gy) {
      const wx = this.minX + (gx + 0.5) * this.resolution;
      const wy = this.minY + (gy + 0.5) * this.resolution;
      return [wx, wy];
    }

    // Thuật toán kẻ tia Bresenham 2D cập nhật Log-Odds
    raytrace(gx0, gy0, gx1, gy1) {
      const w = this.widthCells, h = this.heightCells;
      const grid = this.logOdds;
      const lFree = this.lFree, lOcc = this.lOcc;
      const lMin = this.lMin, lMax = this.lMax;

      let dx = Math.abs(gx1 - gx0);
      let dy = Math.abs(gy1 - gy0);
      let sx = (gx0 < gx1) ? 1 : -1;
      let sy = (gy0 < gy1) ? 1 : -1;
      let err = dx - dy;

      let curX = gx0;
      let curY = gy0;

      while (true) {
        if (curX === gx1 && curY === gy1) {
          // Điểm phản xạ vật cản (Occupied)
          if (curX >= 0 && curX < w && curY >= 0 && curY < h) {
            const idx = curY * w + curX;
            let val = grid[idx] + lOcc;
            if (val > lMax) val = lMax;
            grid[idx] = val;
          }
          break;
        }

        // Vùng không gian trống giữa tia (Free)
        if (curX >= 0 && curX < w && curY >= 0 && curY < h) {
          const idx = curY * w + curX;
          let val = grid[idx] + lFree;
          if (val < lMin) val = lMin;
          grid[idx] = val;
        }

        let e2 = 2 * err;
        if (e2 > -dy) {
          err -= dy;
          curX += sx;
        }
        if (e2 < dx) {
          err += dx;
          curY += sy;
        }
      }
    }

    // Cập nhật bản đồ từ một tập các frame quét hiện tại
    updateFromFrames(frames, stepRay = 1) {
      if (!frames || !frames.length) return;
      let pointsProcessed = 0;

      for (const f of frames) {
        if (!f || !f.xyz || !f.xyz.length) continue;
        const ox = f.ox || 0, oy = f.oy || 0;
        const [gx0, gy0] = this.worldToGrid(ox, oy);
        const pts = f.xyz;
        const n = pts.length / 3;

        for (let i = 0; i < n; i += stepRay) {
          const px = pts[i * 3], py = pts[i * 3 + 1];
          const distSq = (px - ox) * (px - ox) + (py - oy) * (py - oy);
          if (distSq < 0.04 || distSq > 3600) continue; // Bỏ qua điểm quá gần (<20cm) hoặc quá xa (>60m)

          const [gx1, gy1] = this.worldToGrid(px, py);
          this.raytrace(gx0, gy0, gx1, gy1);
          pointsProcessed++;
        }
      }

      if (pointsProcessed > 0) {
        this.frameCount++;
        this.dirty = true;
      }
    }

    // Kết xuất lưới xác suất ra Offscreen Canvas
    updateCanvas() {
      if (!this.dirty) return;
      const w = this.widthCells, h = this.heightCells;
      if (this.canvas.width !== w || this.canvas.height !== h) {
        this.canvas.width = w;
        this.canvas.height = h;
      }

      const imgData = this.ctx.createImageData(w, h);
      const data = imgData.data;
      const grid = this.logOdds;
      const tOcc = this.threshOcc, tFree = this.threshFree;

      // Chú ý hệ trục:
      // Trong Canvas, dòng 0 (y=0) là đỉnh trên cùng, dòng h-1 là đáy dưới cùng.
      // Trong hệ tọa độ thế giới (World Y), gy=0 là đáy dưới cùng (minY), gy=h-1 là đỉnh trên cùng (maxY).
      // Do đó: canvas pixel y = (h - 1 - gy).
      for (let gy = 0; gy < h; gy++) {
        const py = (h - 1 - gy);
        const rowOffset = py * w * 4;
        const gridRowOffset = gy * w;

        for (let gx = 0; gx < w; gx++) {
          const pixIdx = rowOffset + gx * 4;
          const val = grid[gridRowOffset + gx];

          if (val > tOcc) {
            // Tường / Vật cản cố định: Đen đậm sắc nét
            data[pixIdx] = 20;
            data[pixIdx + 1] = 20;
            data[pixIdx + 2] = 20;
            data[pixIdx + 3] = 240;
          } else if (val < tFree) {
            // Vùng trống an toàn (Free space): Trắng sáng nhẹ
            data[pixIdx] = 250;
            data[pixIdx + 1] = 252;
            data[pixIdx + 2] = 255;
            data[pixIdx + 3] = 200; // Bán trong suốt để thấy lưới nền nếu muốn
          } else {
            // Vùng chưa quét (Unknown): Trong suốt hoặc xám mờ
            data[pixIdx] = 220;
            data[pixIdx + 1] = 225;
            data[pixIdx + 2] = 230;
            data[pixIdx + 3] = 50; // Mờ nhẹ
          }
        }
      }

      this.ctx.putImageData(imgData, 0, 0);
      this.dirty = false;
    }

    // Vẽ lớp bản đồ nền lên Canvas chính của Viewer (đồng bộ hoàn hảo với Pan/Zoom)
    draw(ctx, p, v, w2s) {
      if (!this.showMap || this.frameCount === 0) return;
      this.updateCanvas();

      // Điểm góc trên-trái của bản đồ trong hệ tọa độ thế giới: (minX, maxY)
      const [sx, sy] = w2s(v, p, this.minX, this.maxY);
      const sw = (this.maxX - this.minX) * v.zoom;
      const sh = (this.maxY - this.minY) * v.zoom;

      ctx.save();
      ctx.imageSmoothingEnabled = false; // Giữ nét pixel ô vuông không bị nhòe
      ctx.drawImage(this.canvas, sx, sy, sw, sh);

      // Vẽ khung viền bao quanh bản đồ lưới
      ctx.strokeStyle = 'rgba(40, 167, 69, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(sx, sy, sw, sh);

      // Nhãn kích thước và độ phân giải góc dưới bản đồ
      ctx.fillStyle = '#28a745';
      ctx.font = '10px Segoe UI';
      ctx.fillText(
        `Map: ${(this.maxX - this.minX).toFixed(1)}m × ${(this.maxY - this.minY).toFixed(1)}m (Res: ${(this.resolution * 100).toFixed(0)}cm | ${this.widthCells}×${this.heightCells} px)`,
        sx + 4, sy + sh - 6
      );
      ctx.restore();
    }

    // ================= XUẤT BẢN ĐỒ CHUẨN ROS (YAML + PNG/PGM) =================

    generateRosYaml(imageName = 'map.png') {
      return (
`image: ${imageName}
resolution: ${this.resolution.toFixed(6)}
origin: [${this.minX.toFixed(6)}, ${this.minY.toFixed(6)}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
`
      );
    }

    // Tạo Canvas ảnh chuẩn ROS (Grayscale 8-bit: 0=Occupied, 254=Free, 205=Unknown)
    generateRosImageCanvas() {
      const w = this.widthCells, h = this.heightCells;
      const cv = document.createElement('canvas');
      cv.width = w; cv.height = h;
      const c = cv.getContext('2d');
      const imgData = c.createImageData(w, h);
      const data = imgData.data;
      const grid = this.logOdds;
      const tOcc = this.threshOcc, tFree = this.threshFree;

      for (let gy = 0; gy < h; gy++) {
        // Trong chuẩn ROS: hàng 0 của ảnh là maxY (đỉnh), hàng h-1 là minY (đáy)
        const py = (h - 1 - gy);
        const rowOffset = py * w * 4;
        const gridRowOffset = gy * w;

        for (let gx = 0; gx < w; gx++) {
          const pixIdx = rowOffset + gx * 4;
          const val = grid[gridRowOffset + gx];
          let gray = 205; // Mặc định Unknown

          if (val > tOcc) {
            gray = 0;   // Occupied (Đen)
          } else if (val < tFree) {
            gray = 254; // Free (Trắng)
          }

          data[pixIdx] = gray;
          data[pixIdx + 1] = gray;
          data[pixIdx + 2] = gray;
          data[pixIdx + 3] = 255;
        }
      }
      c.putImageData(imgData, 0, 0);
      return cv;
    }

    // ================= XUẤT DỮ LIỆU JSON ĐẦY ĐỦ =================
    exportJson(extrinsics, zones) {
      // Nén dữ liệu lưới bằng Run-Length Encoding (RLE) để file JSON nhỏ gọn (< 20 KB)
      const w = this.widthCells, h = this.heightCells;
      const total = w * h;
      const grid = this.logOdds;
      const tOcc = this.threshOcc, tFree = this.threshFree;
      
      const rle = [];
      let curVal = -1, curCount = 0; // -1: Unknown, 0: Free, 100: Occupied

      for (let i = 0; i < total; i++) {
        let v = -1;
        if (grid[i] > tOcc) v = 100;
        else if (grid[i] < tFree) v = 0;

        if (v === curVal) {
          curCount++;
        } else {
          if (curCount > 0) rle.push([curCount, curVal]);
          curVal = v;
          curCount = 1;
        }
      }
      if (curCount > 0) rle.push([curCount, curVal]);

      const data = {
        format: 'lidar_occupancy_grid_v1',
        timestamp: new Date().toISOString(),
        resolution: this.resolution,
        width: this.widthCells,
        height: this.heightCells,
        bounds: {
          minX: this.minX, maxX: this.maxX,
          minY: this.minY, maxY: this.maxY
        },
        origin: { x: this.minX, y: this.minY, z: 0 },
        lidars: extrinsics ? extrinsics.map((e, idx) => ({
          id: idx,
          name: e.name || ('L' + (idx + 1)),
          x: +(e.dx ?? e.x ?? 0),
          y: +(e.dy ?? e.y ?? 0),
          z: +(e.z ?? 0),
          yaw: +(e.thetaDeg ?? e.yaw ?? 0)
        })) : [],
        zones: zones || [],
        rle_grid: rle
      };

      return data;
    }

    // Nạp lại bản đồ từ JSON
    loadFromJson(data) {
      if (!data || !data.bounds || !data.rle_grid) {
        throw new Error('Dữ liệu file map.json không hợp lệ.');
      }

      this.resolution = data.resolution || 0.05;
      this.minX = data.bounds.minX;
      this.maxX = data.bounds.maxX;
      this.minY = data.bounds.minY;
      this.maxY = data.bounds.maxY;

      this.widthCells = data.width;
      this.heightCells = data.height;
      this.logOdds = new Float32Array(this.widthCells * this.heightCells);

      // Giải nén RLE
      let offset = 0;
      for (const [count, val] of data.rle_grid) {
        const floatVal = (val === 100) ? 3.0 : (val === 0 ? -3.0 : 0.0);
        for (let k = 0; k < count && offset < this.logOdds.length; k++) {
          this.logOdds[offset++] = floatVal;
        }
      }

      this.frameCount = 1;
      this.dirty = true;
      this.updateCanvas();
    }
  }

  root.OccupancyGridMap = OccupancyGridMap;
})(typeof window !== 'undefined' ? window : this);
