/**
 * ==============================================================================================
 * MODULE: CAMERA-LIDAR AI FUSION SIMULATOR & VIEWER (BẢN V4: LỊCH SỬ ĐỐI TƯỢNG & CỬA SỔ RỜI POP-OUT)
 * ==============================================================================================
 * 1. LƯU LỊCH SỬ ĐỐI TƯỢNG & SOI HÌNH DẠNG (Recent Target History & Snapshot Replay):
 *    - Khi đối tượng di chuyển nhanh và đã rời khỏi vùng, thẻ của họ VẪN ĐƯỢC LƯU LẠI trong danh sách
 *      (với nhãn ⏱ ĐÃ RỜI VÙNG kèm thời gian).
 *    - Bấm vào bất kỳ thẻ nào trong lịch sử để xem lại trọn vẹn:
 *      + Kích thước thật (Dài x Rộng x Cao).
 *      + Vận tốc chạy qua cao nhất (km/h).
 *      + Mặt cắt 2D Top-down thể hiện mây điểm đa LiDAR (L1 mặt trước + L2 mặt sau).
 *      + Vị trí mà đối tượng đó đã đi qua trên mặt bằng.
 * 2. CO GIÃN PHÓNG TO / THU NHỎ & ĐƯA RA CỬA SỔ RỜI (Resizable & Pop-out Window):
 *    - ↗ CỬA SỔ RỜI (Pop-out Window): Mở Camera AI ra một cửa sổ trình duyệt độc lập riêng biệt,
 *      có thể kéo sang màn hình thứ 2 hoặc thu nhỏ tùy ý, KHÔNG BAO GIỜ BỊ CHE KHUẤT thông tin trang web.
 *    - 🗖 PHÓNG TO TOÀN MÀN HÌNH (Maximize/Restore): Mở rộng toàn khung hình khi cần quan sát chi tiết.
 *    - ⤡ CO GIÃN TỰ DO: Có thể kéo góc để phóng to/thu nhỏ khung nhìn tùy ý (Responsive Canvas 16:9).
 * 3. HỢP NHẤT ĐA LIDAR & ĐA GÓC ĐỘ QUAN SÁT:
 *    - Phối hợp đồng thời LiDAR 1 (Cyan) và LiDAR 2 (Cam).
 *    - Góc nhìn từ L1, L2, trên trần hoặc xoay 360 độ quanh mục tiêu.
 * ==============================================================================================
 */

(function (window) {
  'use strict';

  const DEG2RAD = Math.PI / 180;
  const RAD2DEG = 180 / Math.PI;

  class CameraLidarFusion {
    constructor() {
      // 1. Ngoại vi Camera (Extrinsics trong World Coordinate System)
      this.extrinsics = {
        x: -1.6,      // Tọa độ X (m) trên mặt bằng
        y: -0.8,      // Tọa độ Y (m) trên mặt bằng
        z: 2.6,       // Chiều cao gắn Camera (m)
        yaw: 72.0,    // Góc xoay ngang (độ)
        pitch: -30.0, // Góc ngẩng/cúi (độ) - âm là chúc xuống mặt đất
        roll: 0.0
      };

      // 2. Nội vi Camera (Intrinsics - Quang học ống kính)
      this.intrinsics = {
        fov: 72.0,    // Trường nhìn ngang (FOV Horizontal độ)
        width: 640,
        height: 360
      };

      // 3. Tùy chọn hiển thị & Vận hành
      this.config = {
        enabled: false,
        source: 'synthetic',     // 'synthetic' | 'webcam'
        showPoints: true,        // Chiếu mây điểm laser lên camera
        showBoxes: true,         // Vẽ hộp bao mục tiêu
        show3DBox: true,         // Vẽ hộp khối 3D lập phương
        showZones: true,         // Vẽ đường biên vùng cảnh báo trên sàn
        autoTrackPTZ: false,     // Tự động xoay camera bám theo đối tượng chính
        ptzSpeed: 0.10,          // Tốc độ lia PTZ mượt mà
        maxDist: 30.0,           // Tầm xa quan sát tối đa (m)
        showTargetDrawer: true,  // Hiện danh sách đối tượng & hình dạng bên dưới
        showRefGhost: true,      // Hiển thị khối hộp tham chiếu 3D (Ghost Wireframe Box)
        showRef2DFootprint: true // Hiển thị mặt cắt tham chiếu 2D trong khung soi hình dạng
      };

      // 4. Chế độ Quan sát Mục tiêu & Góc độ (Target & Multi-Angle Viewing)
      this.selectedTargetId = 'all'; // 'all' (Đa đối tượng) hoặc number ID (vd: 4, 36)
      this.selectedRefId = 'default_person'; // 'default_person' | 'default_car' | 'default_box' | ID từ GroundReferenceManager
      this.viewAngleMode = 'corner'; // 'corner' | 'lidar1' | 'lidar2' | 'top' | 'orbit' | 'custom'
      this.orbitAngle = 0;           // Radian góc xoay quanh vật thể
      this.orbitDistance = 2.4;      // Cự ly nhìn cận cảnh vật thể (m)
      this.orbitHeight = 1.3;        // Độ cao góc nhìn so với chân vật thể (m)
      this.orbitSpeed = 0.55;        // Tốc độ quay tự động 360° (rad/s)

      // 5. BỘ LƯU LỊCH SỬ ĐỐI TƯỢNG (TARGET HISTORY & SNAPSHOT STORAGE)
      // Lưu lại các đối tượng đã từng đi qua để có thể xem lại hình dạng ngay cả khi họ đã rời vùng
      this.targetHistory = new Map(); // id -> { id, firstSeen, lastSeen, active, leftTimeStr, x, y, z1, z2, w, h, vx, vy, maxSpeed, maxPoints, fused, timeStr }

      // 6. Trạng thái Cửa sổ rời (Pop-out Window) & Co giãn
      this.isPopout = false;
      this.popoutWin = null;
      this.isMaximized = false;
      this.isInteracting = false;
      this._isDraggingView = false;
      this.lastMouseX = 0;
      this.lastMouseY = 0;

      // 7. Trạng thái Runtime
      this.panelEl = null;
      this.canvasEl = null;
      this.ctx = null;
      this.videoEl = null;
      this.webcamStream = null;
      this.animationFrameId = null;

      // Trạng thái modal phóng to & điều khiển giao diện
      this.isEnlargeModalOpen = false;
      this.enlargeZoomScale = 100;
      this._lastTargetListUpdate = 0;
      this._forceTargetUiUpdate = false;
      this._isLoopRunning = false;
      this._loopFallbackTimer = null;
      this._onGlobalMouseMove = null;
      this._onGlobalMouseUp = null;

      // Cache bounding boxes 2D để hỗ trợ click chuột chọn mục tiêu
      this.renderedBoxes = [];

      this.loadConfig();
      this.loadTargetHistoryFromSession();
    }

    /**
     * Tải cấu hình từ LocalStorage
     */
    loadConfig() {
      try {
        const saved = localStorage.getItem('lidar_camera_fusion_cfg');
        if (saved) {
          const parsed = JSON.parse(saved);
          if (parsed.extrinsics) Object.assign(this.extrinsics, parsed.extrinsics);
          if (parsed.intrinsics) Object.assign(this.intrinsics, parsed.intrinsics);
          if (parsed.config) Object.assign(this.config, parsed.config);
          if (parsed.viewAngleMode) this.viewAngleMode = parsed.viewAngleMode;
          if (parsed.windowPos) {
            this.lastLeft = parsed.windowPos.left;
            this.lastTop = parsed.windowPos.top;
            this.lastWidth = parsed.windowPos.width;
            this.lastHeight = parsed.windowPos.height;
          }
        }
      } catch (e) {
        console.warn('[CameraFusion] Không thể đọc cấu hình cũ:', e);
      }
    }

    /**
     * Lưu cấu hình vào LocalStorage
     */
    saveConfig() {
      try {
        localStorage.setItem('lidar_camera_fusion_cfg', JSON.stringify({
          extrinsics: this.extrinsics,
          intrinsics: this.intrinsics,
          config: {
            source: this.config.source,
            showPoints: this.config.showPoints,
            showBoxes: this.config.showBoxes,
            show3DBox: this.config.show3DBox,
            showZones: this.config.showZones,
            autoTrackPTZ: this.config.autoTrackPTZ,
            showTargetDrawer: this.config.showTargetDrawer,
            showRefGhost: this.config.showRefGhost,
            showRef2DFootprint: this.config.showRef2DFootprint
          },
          viewAngleMode: this.viewAngleMode,
          windowPos: {
            left: this.lastLeft,
            top: this.lastTop,
            width: this.lastWidth,
            height: this.lastHeight
          }
        }));
      } catch (e) {}
    }

    /**
     * Tải lịch sử mục tiêu từ SessionStorage (giúp F5 / Reload trang web không bị mất dữ liệu mục tiêu cũ)
     */
    loadTargetHistoryFromSession() {
      try {
        const raw = sessionStorage.getItem('lidar_cam_target_history');
        if (raw) {
          const list = JSON.parse(raw);
          if (Array.isArray(list)) {
            for (const item of list) {
              if (item && item.id !== undefined) {
                this.targetHistory.set(item.id, item);
              }
            }
          }
        }
      } catch (e) {
        console.warn('[CameraFusion] Không thể tải targetHistory từ sessionStorage:', e);
      }
    }

    /**
     * Lưu lịch sử mục tiêu vào SessionStorage (lưu tối đa 50 đối tượng gần nhất)
     */
    saveTargetHistoryToSession() {
      try {
        if (!this.targetHistory || !this.targetHistory.size) return;
        const list = Array.from(this.targetHistory.values()).slice(-50).map(t => {
          let slimFused = null;
          if (t.fused) {
            slimFused = {
              hasPoints: !!t.fused.hasPoints,
              measuredW: t.fused.measuredW || t.w || 0.4,
              measuredH: t.fused.measuredH || t.h || 0.35,
              ptsL1: (t.fused.ptsL1 || []).slice(0, 40),
              ptsL2: (t.fused.ptsL2 || []).slice(0, 40)
            };
          }
          return {
            id: t.id,
            firstSeen: t.firstSeen,
            lastSeen: t.lastSeen,
            active: !!t.active,
            leftTimeStr: t.leftTimeStr,
            timeStr: t.timeStr,
            x: t.x, y: t.y, z1: t.z1, z2: t.z2,
            w: t.w, h: t.h, maxW: t.maxW, maxH: t.maxH,
            vx: t.vx, vy: t.vy, maxSpeed: t.maxSpeed,
            maxPoints: t.maxPoints,
            trail: (t.trail || []).slice(-30),
            fused: slimFused
          };
        });
        sessionStorage.setItem('lidar_cam_target_history', JSON.stringify(list));
      } catch (e) {}
    }

    /**
     * Khởi tạo giao diện Picture-in-Picture với thanh điều khiển Đa Mục Tiêu & Khung Soi Hình Dạng
     */
    initUI() {
      if (document.getElementById('cameraFusionPanel')) return;

      const panel = document.createElement('div');
      panel.id = 'cameraFusionPanel';
      panel.innerHTML = `
        <div class="cam-header" id="camHeader">
          <div class="cam-title">
            <span class="cam-dot"></span>
            <b>📷 CAMERA AI FUSION (Hợp Nhất Đa LiDAR)</b>
            <span id="camFps" style="font-size:10px;color:#aaa;margin-left:6px;">0 FPS</span>
          </div>
          <div class="cam-tools">
            <button id="btnCamTargets" class="${this.config.showTargetDrawer ? 'active' : ''}" title="Bật/Tắt danh sách đối tượng & xem hình dạng">📋 Đối tượng (<span id="camTargetBadge">0</span>)</button>
            <button id="btnCamPtz" title="Bật/Tắt tự động quay quét PTZ bám mục tiêu" class="${this.config.autoTrackPTZ ? 'active' : ''}">🎯 PTZ Auto</button>
            <button id="btnCamPopout" title="Mở Camera AI ra một cửa sổ riêng biệt ngoài trang web (Pop-out) để không che màn hình">↗ Cửa sổ rời</button>
            <button id="btnCamMaximize" title="Phóng to / Khôi phục kích thước cửa sổ">🗖</button>
            <button id="btnCamSrc" title="Chuyển đổi nguồn (Camera Ảo 3D / Webcam thật)">🔄 Nguồn</button>
            <button id="btnCamCalib" title="Bật/Tắt bảng căn chỉnh tọa độ Camera">⚙ Căn chỉnh</button>
            <button id="btnCamClose" title="Đóng cửa sổ Camera">✕</button>
          </div>
        </div>

        <!-- Thanh điều khiển Chọn Đối tượng & Chọn Góc nhìn (Target & Multi-Angle Bar) -->
        <div class="cam-ctrl-bar">
          <div class="ctrl-group">
            <label>🎯 Mục tiêu:</label>
            <select id="camTargetSel" title="Chọn mục tiêu cụ thể để khóa góc nhìn hoặc xem tất cả">
              <option value="all">👁️ Tất cả mục tiêu (Multi)</option>
            </select>
          </div>

          <div class="ctrl-group">
            <label>📐 Góc nhìn:</label>
            <select id="camAngleSel" title="Xem từ các góc độ khác nhau">
              <option value="corner" ${this.viewAngleMode === 'corner' ? 'selected' : ''}>📷 Góc toàn cảnh</option>
              <option value="lidar1" ${this.viewAngleMode === 'lidar1' ? 'selected' : ''}>📡 Từ LiDAR 1 (Mặt trước)</option>
              <option value="lidar2" ${this.viewAngleMode === 'lidar2' ? 'selected' : ''}>📡 Từ LiDAR 2 (Mặt sau)</option>
              <option value="top" ${this.viewAngleMode === 'top' ? 'selected' : ''}>⬇ Trên trần nhìn xuống</option>
              <option value="orbit" ${this.viewAngleMode === 'orbit' ? 'selected' : ''}>🔄 Xoay 360° quanh vật</option>
            </select>
          </div>

          <div class="ctrl-group">
            <label>📐 Tham chiếu:</label>
            <select id="camRefSel" title="Chọn mẫu hình học tham chiếu để lồng vào đối tượng so sánh kích thước">
              <option value="default_person">👤 Người (1.7×0.5×0.3m)</option>
              <option value="default_moto">🏍️ Xe máy (1.2×1.9×0.8m)</option>
              <option value="default_car">🚗 Xe ô tô (1.6×3.8×1.8m)</option>
              <option value="default_forklift">🚜 Xe nâng/AGV (2.1×2.4×1.2m)</option>
              <option value="default_box">📦 Kiện carton (0.8×0.8×0.8m)</option>
              <option value="default_pallet">🧱 Pallet hàng (1.4×1.2×1.0m)</option>
              <option value="default_animal">🐕 Thú cưng (0.4×0.6×0.3m - Lọc ảo)</option>
              <option value="none">🚫 Không lồng</option>
            </select>
          </div>

          <div style="display:flex;align-items:center;gap:4px;margin-left:auto;">
            <button id="btnTopCamCaptureRef" style="background:#e67e22;color:#fff;border:1px solid #d35400;font-weight:bold;font-size:11px;padding:3px 9px;border-radius:3px;cursor:pointer;" title="Lưu hình dạng đối tượng đang chọn thành Mẫu Tham Chiếu 2D/3D">📸 Chụp Mẫu</button>
            <button id="btnTopCamEnlarge" style="background:#0d6efd;color:#fff;border:1px solid #0b5ed7;font-weight:bold;font-size:11px;padding:3px 9px;border-radius:3px;cursor:pointer;" title="Phóng to khung soi hình dạng & tham chiếu">🔍 Phóng To</button>
          </div>

          <div class="cam-hint">
            <span>🖱️ Kéo chuột: Xoay góc • Lăn chuột: Zoom • Bấm thẻ bên dưới để soi hình dạng & Chụp mẫu</span>
          </div>
        </div>

        <!-- Khung View Camera -->
        <div class="cam-view-wrap">
          <canvas id="camCanvas" width="640" height="360"></canvas>
          <video id="camVideo" autoplay playsinline muted style="display:none;"></video>
          <div id="camHudOverlay"></div>
        </div>

        <!-- Khung Danh Sách Đối Tượng & Chi Tiết Hình Dạng (Target List & Shape Inspector) -->
        <div id="camTargetDrawer" class="cam-target-drawer" style="display:${this.config.showTargetDrawer ? 'flex' : 'none'};">
          <div class="cam-drawer-header">
            <span><b>📋 ĐỐI TƯỢNG REALTIME & LỊCH SỬ GẦN ĐÂY (<span id="camTargetHeaderCount">0</span>)</b></span>
            <span style="font-size:10px;color:#aaa;">Bấm vào đối tượng (kể cả đã rời đi) để xem lại hình dạng 2D/3D</span>
          </div>
          <div class="cam-drawer-body">
            <!-- Cột trái: Danh sách các thẻ đối tượng (Đang có mặt & Đã rời đi) -->
            <div id="camTargetList" class="cam-target-items">
              <div style="color:#777;font-style:italic;padding:12px;text-align:center;">Chưa có đối tượng nào trong tầm quét</div>
            </div>
            <!-- Cột phải: Khung phân tích hình dạng cơ bản của đối tượng đang chọn -->
            <div id="camShapePreview" class="cam-shape-preview">
              <div style="color:#777;font-style:italic;padding:16px;text-align:center;font-size:11px;">
                Chọn một đối tượng bên trái hoặc click chuột trên màn hình Camera để xem hình dạng chi tiết
              </div>
            </div>
          </div>
        </div>

        <!-- Bảng căn chỉnh tọa độ và góc nhìn (Calibration Panel) -->
        <div id="camCalibDrawer" style="display:none;">
          <div class="calib-row">
            <label>Tọa độ Cam (X, Y, Z m):</label>
            <input type="number" id="camInpX" step="0.1" value="${this.extrinsics.x}">
            <input type="number" id="camInpY" step="0.1" value="${this.extrinsics.y}">
            <input type="number" id="camInpZ" step="0.1" value="${this.extrinsics.z}">
          </div>
          <div class="calib-row">
            <label>Góc quay (Yaw, Pitch, FOV °):</label>
            <input type="number" id="camInpYaw" step="1" value="${this.extrinsics.yaw}" title="Góc xoay ngang Yaw (0-360°)">
            <input type="number" id="camInpPitch" step="1" value="${this.extrinsics.pitch}" title="Góc ngẩng/cúi Pitch (-90° chúc đất, 0° nhìn ngang)">
            <input type="number" id="camInpFov" step="1" value="${this.intrinsics.fov}" title="Góc mở ống kính (FOV)">
          </div>
          <div class="calib-actions">
            <label style="cursor:pointer;"><input type="checkbox" id="camChkPts" ${this.config.showPoints ? 'checked' : ''}> Laser Pts (Đa LiDAR)</label>
            <label style="cursor:pointer;"><input type="checkbox" id="camChkBoxes" ${this.config.showBoxes ? 'checked' : ''}> Hộp 3D</label>
            <label style="cursor:pointer;"><input type="checkbox" id="camChkZones" ${this.config.showZones ? 'checked' : ''}> Vùng Zone</label>
            <label style="cursor:pointer;"><input type="checkbox" id="camChkRefGhost" ${this.config.showRefGhost ? 'checked' : ''}> 📐 Khối tham chiếu 3D</label>
            <label style="cursor:pointer;"><input type="checkbox" id="camChkRef2D" ${this.config.showRef2DFootprint ? 'checked' : ''}> 📐 Mặt 2D</label>
            <button id="btnCamSaveCfg" style="background:#0d6efd;color:#fff;border:none;border-radius:3px;padding:2px 8px;margin-left:auto;cursor:pointer;">💾 Lưu vị trí</button>
          </div>
        </div>

        <!-- Cửa sổ phóng to Soi Hình Dạng Chi Tiết (Enlarged Shape Inspector Modal) -->
        <div id="camEnlargeModal" style="display:none;">
          <div class="enlarge-modal-header">
            <span id="camEnlargeTitle">🔍 <b>SOI CHI TIẾT HÌNH DẠNG & MẶT CẮT THAM CHIẾU</b></span>
            <div style="display:flex;align-items:center;gap:6px;margin-left:auto;">
              <button id="btnEnlargeZoomIn" title="Phóng to thêm (+)" style="font-weight:bold;font-size:11px;padding:1px 6px;">➕</button>
              <button id="btnEnlargeZoomOut" title="Thu nhỏ bớt (-)" style="font-weight:bold;font-size:11px;padding:1px 6px;">➖</button>
              <button id="btnEnlargeCaptureRef" style="background:#e67e22;color:#fff;border:1px solid #d35400;font-weight:bold;font-size:10px;padding:2px 8px;border-radius:3px;cursor:pointer;" title="Lưu hình dạng đang soi này thành Mẫu Tham Chiếu">📸 Chụp Mẫu Này</button>
              <button id="btnCloseEnlargeModal" title="Đóng cửa sổ phóng to" style="background:#444;color:#fff;border:none;border-radius:3px;padding:2px 7px;cursor:pointer;font-weight:bold;">✕</button>
            </div>
          </div>
          <div class="enlarge-modal-body">
            <div class="enlarge-canvas-wrap">
              <canvas id="camEnlargeCanvas" width="340" height="240"></canvas>
              <div class="enlarge-legend">
                <span style="color:#00e5ff;">● LiDAR 1 (Trước)</span>
                <span style="color:#ff9900;">● LiDAR 2 (Sau)</span>
                <span style="color:#2ecc71;">-- Khung tham chiếu</span>
                <span style="color:#aaa;font-style:italic;">(Cuộn chuột trên hình để Zoom)</span>
              </div>
            </div>
            <div id="camEnlargeMetrics" class="enlarge-metrics"></div>
          </div>
        </div>

        <!-- Góc và cạnh kéo co giãn kích thước (Resize Handles 8 hướng) -->
        <div class="cam-resizer cam-resizer-r" data-dir="r"></div>
        <div class="cam-resizer cam-resizer-b" data-dir="b"></div>
        <div class="cam-resizer cam-resizer-l" data-dir="l"></div>
        <div class="cam-resizer cam-resizer-t" data-dir="t"></div>
        <div class="cam-resizer cam-resizer-br" data-dir="br" title="Kéo để thay đổi kích thước">⤡</div>
        <div class="cam-resizer cam-resizer-bl" data-dir="bl"></div>
        <div class="cam-resizer cam-resizer-tr" data-dir="tr"></div>
        <div class="cam-resizer cam-resizer-tl" data-dir="tl"></div>
      `;

      document.body.appendChild(panel);
      this.panelEl = panel;

      // Phục hồi kích thước & vị trí đã lưu nếu có
      if (this.lastWidth) panel.style.width = `${this.lastWidth}px`;
      if (this.lastHeight) panel.style.height = `${this.lastHeight}px`;
      if (this.lastLeft !== undefined && this.lastTop !== undefined) {
        panel.style.left = `${this.lastLeft}px`;
        panel.style.top = `${this.lastTop}px`;
        panel.style.bottom = 'auto';
      }

      this.canvasEl = panel.querySelector('#camCanvas');
      this.ctx = this.canvasEl.getContext('2d');
      this.videoEl = panel.querySelector('#camVideo');

      this._injectCSS();
      this._bindEvents();
    }

    _injectCSS() {
      if (document.getElementById('camFusionStyle')) return;
      const st = document.createElement('style');
      st.id = 'camFusionStyle';
      st.textContent = `
        #cameraFusionPanel {
          position: fixed;
          left: 12px;
          bottom: 26px;
          width: 580px;
          height: auto;
          min-width: 380px;
          min-height: 240px;
          max-width: 98vw;
          max-height: 96vh;
          background: #181922;
          border: 1px solid #3c3f50;
          box-shadow: 0 10px 32px rgba(0,0,0,0.65);
          border-radius: 6px;
          z-index: 1200;
          display: none;
          flex-direction: column;
          overflow: hidden;
          font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
          color: #eee;
        }
        #cameraFusionPanel.show { display: flex; }

        /* Chế độ Phóng to toàn màn hình (Maximized) */
        #cameraFusionPanel.maximized {
          position: fixed !important;
          left: 10px !important;
          top: 50px !important;
          right: 10px !important;
          bottom: 26px !important;
          width: auto !important;
          height: auto !important;
          max-width: none !important;
          max-height: none !important;
          z-index: 1250 !important;
        }

        /* Chế độ Cửa sổ rời (Pop-out Window) */
        #cameraFusionPanel.popout-mode {
          display: flex !important;
          flex-direction: column !important;
          position: static !important;
          left: 0 !important;
          top: 0 !important;
          width: 100% !important;
          height: 100vh !important;
          border: none !important;
          border-radius: 0 !important;
          box-shadow: none !important;
          max-width: none !important;
          max-height: none !important;
        }
        #cameraFusionPanel.popout-mode #btnCamMaximize,
        #cameraFusionPanel.popout-mode .cam-resizer,
        #cameraFusionPanel.popout-mode .cam-resize-handle {
          display: none !important;
        }

        #cameraFusionPanel.popout-mode .cam-view-wrap,
        #cameraFusionPanel.maximized .cam-view-wrap {
          flex: 1;
          height: auto;
          min-height: 180px;
          aspect-ratio: auto;
        }
        #cameraFusionPanel.popout-mode .cam-target-drawer,
        #cameraFusionPanel.maximized .cam-target-drawer {
          height: 220px;
          min-height: 220px;
          max-height: 220px;
          flex-shrink: 0;
        }

        .cam-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 6px 10px;
          background: #12131a;
          border-bottom: 1px solid #2d2f3c;
          cursor: move;
          user-select: none;
        }
        .cam-title { font-size: 11px; display: flex; align-items: center; }
        .cam-dot { width: 8px; height: 8px; border-radius: 50%; background: #00ff66; margin-right: 6px; box-shadow: 0 0 6px #00ff66; }
        .cam-tools { display: flex; gap: 4px; }
        .cam-tools button {
          background: #252735;
          color: #ddd;
          border: 1px solid #3f4258;
          border-radius: 3px;
          font-size: 10px;
          padding: 2px 7px;
          cursor: pointer;
        }
        .cam-tools button:hover { background: #35384c; color: #fff; }
        .cam-tools button.active { background: #0d6efd; border-color: #0d6efd; color: #fff; font-weight: bold; }

        .cam-ctrl-bar {
          display: flex;
          align-items: center;
          flex-wrap: wrap;
          gap: 8px;
          padding: 4px 10px;
          background: #1f212c;
          border-bottom: 1px solid #333648;
          font-size: 11px;
        }
        .ctrl-group { display: flex; align-items: center; gap: 4px; }
        .ctrl-group label { color: #aaa; font-size: 10px; font-weight: 500; }
        .ctrl-group select {
          background: #12131a;
          color: #00e5ff;
          border: 1px solid #4a4e66;
          border-radius: 3px;
          padding: 2px 6px;
          font-size: 11px;
          font-weight: 500;
          cursor: pointer;
          max-width: 175px;
        }
        .ctrl-group select:focus { outline: none; border-color: #0d6efd; }
        .cam-hint {
          width: 100%;
          font-size: 9px;
          color: #7e84a2;
          font-style: italic;
          display: flex;
          justify-content: space-between;
        }

        .cam-view-wrap {
          position: relative;
          width: 100%;
          background: #000;
          overflow: hidden;
          cursor: grab;
          flex: 1 1 auto;
          min-height: 140px;
        }
        .cam-view-wrap.grabbing { cursor: grabbing; }
        #camCanvas {
          width: 100%;
          height: 100%;
          display: block;
          object-fit: contain;
        }

        /* KHUNG DRAWER DANH SÁCH ĐỐI TƯỢNG & SOI HÌNH DẠNG */
        .cam-target-drawer {
          background: #161720;
          border-top: 1px solid #303344;
          display: flex;
          flex-direction: column;
          flex-shrink: 0;
          height: 200px;
          min-height: 140px;
          max-height: 350px;
        }
        .cam-drawer-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 4px 10px;
          background: #101117;
          border-bottom: 1px solid #252735;
          font-size: 10px;
        }
        .cam-drawer-body {
          display: grid;
          grid-template-columns: 245px 1fr;
          gap: 6px;
          padding: 6px;
          flex: 1;
          overflow: hidden;
        }
        .cam-target-items {
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 4px;
          padding-right: 3px;
        }
        .target-item {
          background: #20222f;
          border: 1px solid #383c52;
          border-radius: 4px;
          padding: 4px 7px;
          cursor: pointer;
          font-size: 10px;
          transition: all 0.12s;
        }
        .target-item:hover { background: #272a3b; border-color: #00e5ff; }
        .target-item.active { background: #2d2a17; border-color: #ffcc00; box-shadow: 0 0 5px rgba(255,204,0,0.3); }
        .target-item.stale { opacity: 0.72; background: #1a1b24; border-style: dashed; }
        .target-item-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          font-weight: bold;
          margin-bottom: 2px;
        }
        .target-item-badge {
          background: #0d6efd;
          color: #fff;
          border-radius: 2px;
          padding: 0 4px;
          font-size: 9px;
        }
        .target-item-badge.alarm { background: #dc3545; }
        .target-item-badge.stale { background: #555; color: #ccc; }
        .target-item-meta {
          color: #aaa;
          font-size: 9px;
          display: flex;
          justify-content: space-between;
        }

        /* KHUNG XEM HÌNH DẠNG CƠ BẢN (SHAPE PREVIEW) */
        .cam-shape-preview {
          background: #111219;
          border: 1px solid #2a2d3c;
          border-radius: 4px;
          padding: 5px 8px;
          overflow-y: auto;
          display: flex;
          flex-direction: column;
          gap: 4px;
        }
        .shape-card-top {
          display: flex;
          justify-content: space-between;
          align-items: center;
          margin-bottom: 4px;
          font-size: 10px;
        }
        .shape-category {
          background: #34495e;
          color: #ecf0f1;
          border-radius: 3px;
          padding: 1px 5px;
          font-weight: bold;
        }
        .shape-content-grid {
          display: grid;
          grid-template-columns: 110px 1fr;
          gap: 6px;
          align-items: center;
        }
        .shape-canvas-box {
          display: flex;
          flex-direction: column;
          align-items: center;
        }
        #camMiniShapeCanvas {
          width: 95px;
          height: 95px;
          background: #090a0e;
          border: 1px solid #333648;
          border-radius: 3px;
        }
        .shape-canvas-lbl {
          font-size: 8px;
          color: #777;
          margin-top: 2px;
        }
        .shape-metrics-list {
          font-size: 9px;
          color: #bbb;
          display: flex;
          flex-direction: column;
          gap: 2px;
        }
        .shape-quick-btns {
          display: flex;
          align-items: center;
          gap: 3px;
          margin-top: 3px;
          padding-top: 3px;
          border-top: 1px dashed #2b2e3e;
          font-size: 9px;
          color: #888;
        }
        .shape-quick-btns button {
          background: #252735;
          color: #ccc;
          border: 1px solid #3e4256;
          border-radius: 2px;
          font-size: 9px;
          padding: 1px 5px;
          cursor: pointer;
        }
        .shape-quick-btns button:hover { background: #383c52; color: #fff; }

        #camCalibDrawer {
          background: #1a1b24;
          border-top: 1px solid #2d2f3c;
          padding: 6px 10px;
          font-size: 11px;
        }
        .calib-row {
          display: flex;
          align-items: center;
          gap: 6px;
          margin-bottom: 3px;
        }
        .calib-row label { width: 155px; color: #9aa; font-size: 10px; }
        .calib-row input {
          width: 65px;
          background: #111218;
          border: 1px solid #373a4c;
          color: #fff;
          border-radius: 3px;
          padding: 2px 4px;
          font-size: 11px;
        }
        .calib-actions {
          display: flex;
          align-items: center;
          gap: 8px;
          font-size: 10px;
          color: #888;
          margin-top: 3px;
          padding-top: 3px;
          border-top: 1px dashed #2e3142;
        }

        /* Các cạnh và góc kéo co giãn kích thước (8 hướng) */
        .cam-resizer {
          position: absolute;
          z-index: 50;
        }
        .cam-resizer-r { right: 0; top: 0; bottom: 0; width: 6px; cursor: ew-resize; }
        .cam-resizer-l { left: 0; top: 0; bottom: 0; width: 6px; cursor: ew-resize; }
        .cam-resizer-b { bottom: 0; left: 0; right: 0; height: 6px; cursor: ns-resize; }
        .cam-resizer-t { top: 0; left: 0; right: 0; height: 6px; cursor: ns-resize; }
        .cam-resizer-br {
          right: 0; bottom: 0; width: 14px; height: 14px;
          cursor: nwse-resize; user-select: none;
          display: flex; align-items: center; justify-content: center;
          font-size: 11px; color: #777; line-height: 1;
        }
        .cam-resizer-bl { left: 0; bottom: 0; width: 12px; height: 12px; cursor: nesw-resize; }
        .cam-resizer-tr { right: 0; top: 0; width: 12px; height: 12px; cursor: nesw-resize; }
        .cam-resizer-tl { left: 0; top: 0; width: 12px; height: 12px; cursor: nwse-resize; }
        .cam-resizer-br:hover { color: #00e5ff; }

        .cam-resize-handle {
          display: none;
        }

        /* Cửa sổ Modal Phóng to Soi Hình Dạng */
        #camEnlargeModal {
          position: absolute;
          left: 50%;
          top: 50%;
          transform: translate(-50%, -50%);
          width: 380px;
          max-width: 90%;
          background: #151722;
          border: 1px solid #00e5ff;
          border-radius: 6px;
          box-shadow: 0 10px 36px rgba(0,0,0,0.85);
          z-index: 450;
          display: flex;
          flex-direction: column;
          overflow: hidden;
        }
        .enlarge-modal-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 6px 10px;
          background: #0f1017;
          border-bottom: 1px solid #282b3c;
          font-size: 11px;
          color: #00ffcc;
        }
        .enlarge-modal-header button {
          background: #25283a;
          color: #fff;
          border: 1px solid #41455c;
          border-radius: 3px;
          padding: 1px 6px;
          cursor: pointer;
        }
        .enlarge-modal-header button:hover { background: #e74c3c; border-color: #e74c3c; }
        .enlarge-modal-body {
          padding: 8px;
          display: flex;
          flex-direction: column;
          gap: 6px;
        }
        .enlarge-canvas-wrap {
          position: relative;
          background: #090a0f;
          border: 1px solid #2d3044;
          border-radius: 4px;
          display: flex;
          flex-direction: column;
          align-items: center;
          padding: 4px;
        }
        #camEnlargeCanvas {
          width: 100%;
          max-width: 360px;
          height: 240px;
          display: block;
        }
        .enlarge-legend {
          display: flex;
          gap: 12px;
          font-size: 9px;
          margin-top: 4px;
        }
        .enlarge-metrics {
          display: grid;
          grid-template-columns: 1fr 1fr;
          gap: 4px;
          font-size: 10px;
          background: #1b1c28;
          padding: 6px 8px;
          border-radius: 4px;
          border: 1px solid #2c2f42;
          color: #ccc;
        }
      `;
      document.head.appendChild(st);
    }

    _bindEvents() {
      const panel = this.panelEl;
      const header = panel.querySelector('#camHeader');

      // 1. Kéo thả di chuyển cửa sổ (Window Draggable)
      let isWindowDragging = false, startX = 0, startY = 0, origLeft = 0, origTop = 0;
      header.onmousedown = (e) => {
        if (this.isPopout || this.isMaximized || e.target.tagName === 'BUTTON' || e.target.tagName === 'SELECT') return;
        isWindowDragging = true;
        startX = e.clientX;
        startY = e.clientY;
        const rect = panel.getBoundingClientRect();
        origLeft = rect.left;
        origTop = rect.top;
        document.onmousemove = (me) => {
          if (!isWindowDragging) return;
          const newLeft = origLeft + (me.clientX - startX);
          const newTop = origTop + (me.clientY - startY);
          panel.style.left = `${newLeft}px`;
          panel.style.top = `${newTop}px`;
          panel.style.bottom = 'auto';
          this.lastLeft = newLeft;
          this.lastTop = newTop;
        };
        document.onmouseup = () => {
          isWindowDragging = false;
          document.onmousemove = null;
          document.onmouseup = null;
          this.saveConfig();
        };
      };

      // 2. Co giãn linh hoạt kích thước cửa sổ đa hướng (8 hướng)
      const resizers = panel.querySelectorAll('.cam-resizer');
      resizers.forEach((rz) => {
        rz.onmousedown = (e) => {
          if (this.isPopout || this.isMaximized) return;
          e.stopPropagation();
          e.preventDefault();
          const dir = rz.getAttribute('data-dir');
          const rStartX = e.clientX;
          const rStartY = e.clientY;
          const rect = panel.getBoundingClientRect();
          const rStartLeft = rect.left;
          const rStartTop = rect.top;
          const rStartW = rect.width;
          const rStartH = rect.height;

          // Chuyển panel sang dùng top/left/width/height cố định
          panel.style.left = `${rStartLeft}px`;
          panel.style.top = `${rStartTop}px`;
          panel.style.bottom = 'auto';
          panel.style.width = `${rStartW}px`;
          panel.style.height = `${rStartH}px`;

          document.onmousemove = (me) => {
            const dx = me.clientX - rStartX;
            const dy = me.clientY - rStartY;

            let newW = rStartW;
            let newH = rStartH;
            let newLeft = rStartLeft;
            let newTop = rStartTop;

            if (dir.includes('r')) {
              newW = Math.max(380, rStartW + dx);
            }
            if (dir.includes('l')) {
              const potW = rStartW - dx;
              if (potW >= 380) {
                newW = potW;
                newLeft = rStartLeft + dx;
              } else {
                newW = 380;
                newLeft = rStartLeft + (rStartW - 380);
              }
            }
            if (dir.includes('b')) {
              newH = Math.max(240, rStartH + dy);
            }
            if (dir.includes('t')) {
              const potH = rStartH - dy;
              if (potH >= 240) {
                newH = potH;
                newTop = rStartTop + dy;
              } else {
                newH = 240;
                newTop = rStartTop + (rStartH - 240);
              }
            }

            panel.style.width = `${newW}px`;
            panel.style.height = `${newH}px`;
            panel.style.left = `${newLeft}px`;
            panel.style.top = `${newTop}px`;

            this.lastWidth = newW;
            this.lastHeight = newH;
            this.lastLeft = newLeft;
            this.lastTop = newTop;
          };

          document.onmouseup = () => {
            document.onmousemove = null;
            document.onmouseup = null;
            this.saveConfig();
          };
        };
      });

      // 3. Nút công cụ chính
      panel.querySelector('#btnCamClose').onclick = () => this.toggle(false);

      // Nút Phóng to / Thu nhỏ (Maximize / Restore)
      const btnMax = panel.querySelector('#btnCamMaximize');
      btnMax.onclick = () => {
        if (this.isPopout) return;
        this.isMaximized = !this.isMaximized;
        panel.classList.toggle('maximized', this.isMaximized);
        btnMax.textContent = this.isMaximized ? '🗗' : '🗖';
        btnMax.title = this.isMaximized ? 'Khôi phục kích thước' : 'Phóng to toàn màn hình';
      };

      // Nút Cửa sổ rời (Pop-out Window)
      const btnPopout = panel.querySelector('#btnCamPopout');
      btnPopout.onclick = () => {
        this.togglePopout();
      };

      // Bật/Tắt Drawer Danh sách đối tượng & Hình dạng
      const btnDrawer = panel.querySelector('#btnCamTargets');
      const drawer = panel.querySelector('#camTargetDrawer');
      btnDrawer.onclick = () => {
        this.config.showTargetDrawer = !this.config.showTargetDrawer;
        drawer.style.display = this.config.showTargetDrawer ? 'flex' : 'none';
        btnDrawer.classList.toggle('active', this.config.showTargetDrawer);
        this.saveConfig();
      };

      // Đóng cửa sổ phóng to Soi Hình Dạng
      const btnCloseEnlarge = panel.querySelector('#btnCloseEnlargeModal');
      if (btnCloseEnlarge) {
        btnCloseEnlarge.onclick = () => {
          const m = panel.querySelector('#camEnlargeModal');
          if (m) m.style.display = 'none';
          this.isEnlargeModalOpen = false;
        };
      }

      // Nút Chụp Mẫu ngay trên thanh điều khiển chính
      const btnTopCap = panel.querySelector('#btnTopCamCaptureRef');
      if (btnTopCap) {
        btnTopCap.onclick = () => {
          const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;
          if (!t) {
            alert('Vui lòng chọn 1 mục tiêu cụ thể (trong danh sách bên dưới hoặc click trên Camera) để chụp mẫu!');
            return;
          }
          this.captureTargetAsReference(t);
        };
      }

      // Nút Phóng To ngay trên thanh điều khiển chính
      const btnTopEnlarge = panel.querySelector('#btnTopCamEnlarge');
      if (btnTopEnlarge) {
        btnTopEnlarge.onclick = () => {
          const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;
          if (!t) {
            alert('Vui lòng chọn 1 mục tiêu cụ thể để phóng to soi hình dạng!');
            return;
          }
          this.openEnlargeModal(t);
        };
      }

      // Nút Chụp Mẫu ngay trong Cửa Sổ Phóng To
      const btnEnlargeCap = panel.querySelector('#btnEnlargeCaptureRef');
      if (btnEnlargeCap) {
        btnEnlargeCap.onclick = () => {
          const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;
          if (t) this.captureTargetAsReference(t);
        };
      }

      // Nút Zoom In/Out trong Cửa Sổ Phóng To
      const btnZIn = panel.querySelector('#btnEnlargeZoomIn');
      if (btnZIn) {
        btnZIn.onclick = () => {
          this.enlargeZoomScale = Math.min(320, (this.enlargeZoomScale || 100) + 25);
          const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;
          if (t) this._renderEnlargedShapeInspector(t);
        };
      }
      const btnZOut = panel.querySelector('#btnEnlargeZoomOut');
      if (btnZOut) {
        btnZOut.onclick = () => {
          this.enlargeZoomScale = Math.max(35, (this.enlargeZoomScale || 100) - 25);
          const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;
          if (t) this._renderEnlargedShapeInspector(t);
        };
      }

      // Cuộn chuột Zoom trực tiếp trên Canvas Phóng To
      const enlargeCv = panel.querySelector('#camEnlargeCanvas');
      if (enlargeCv) {
        enlargeCv.onwheel = (e) => {
          e.preventDefault();
          const delta = Math.sign(e.deltaY);
          this.enlargeZoomScale = Math.max(35, Math.min(320, (this.enlargeZoomScale || 100) - delta * 15));
          const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;
          if (t) this._renderEnlargedShapeInspector(t);
        };
      }

      const btnCalib = panel.querySelector('#btnCamCalib');
      const calibDrawer = panel.querySelector('#camCalibDrawer');
      btnCalib.onclick = () => {
        const isShow = calibDrawer.style.display !== 'none';
        calibDrawer.style.display = isShow ? 'none' : 'block';
        btnCalib.classList.toggle('active', !isShow);
      };

      const btnPtz = panel.querySelector('#btnCamPtz');
      btnPtz.onclick = () => {
        this.config.autoTrackPTZ = !this.config.autoTrackPTZ;
        btnPtz.classList.toggle('active', this.config.autoTrackPTZ);
      };

      panel.querySelector('#btnCamSrc').onclick = () => {
        if (this.config.source === 'synthetic') {
          this.switchSource('webcam');
        } else {
          this.switchSource('synthetic');
        }
      };

      // 4. Dropdown Chọn Mục Tiêu (Target Selection)
      const targetSel = panel.querySelector('#camTargetSel');
      targetSel.onchange = (e) => {
        const val = e.target.value;
        this.selectTarget(val === 'all' ? 'all' : parseInt(val, 10));
      };

      // 5. Dropdown Chọn Góc Nhìn (Angle Selection)
      const angleSel = panel.querySelector('#camAngleSel');
      angleSel.onchange = (e) => {
        this.viewAngleMode = e.target.value;
        this.applyViewAngle(this.viewAngleMode);
      };

      // 6. TƯƠNG TÁC CHUỘT TRÊN CANVAS (XOAY GÓC NHÌN & ZOOM & CLICK CHỌN MỤC TIÊU)
      const wrap = panel.querySelector('.cam-view-wrap');
      const cv = this.canvasEl;
      this._isDraggingView = false;   // Cờ riêng: đang kéo xoay góc nhìn trên canvas

      cv.onmousedown = (e) => {
        // Chỉ bật drag-view khi click trái trên canvas (không phải trên UI overlay)
        if (e.button !== 0) return;
        this._isDraggingView = true;
        this.isInteracting = true;
        this.lastMouseX = e.clientX;
        this.lastMouseY = e.clientY;
        wrap.classList.add('grabbing');
      };

      this._onGlobalMouseMove = (e) => {
        if (!this._isDraggingView || !this.config.enabled) return;
        const dx = e.clientX - this.lastMouseX;
        const dy = e.clientY - this.lastMouseY;
        this.lastMouseX = e.clientX;
        this.lastMouseY = e.clientY;

        if (this.selectedTargetId !== 'all') {
          // Khi đã chọn 1 đối tượng: kéo chuột để xoay orbit quanh đối tượng
          // VÀ đồng thời cho phép tự do điều chỉnh yaw/pitch
          this.orbitAngle -= dx * 0.008;
          this.orbitHeight = Math.max(0.2, Math.min(6.0, this.orbitHeight - dy * 0.01));
          // Cũng cập nhật yaw/pitch để camera có thể xoay tự do
          this.extrinsics.yaw = (this.extrinsics.yaw + dx * 0.3) % 360;
          this.extrinsics.pitch = Math.max(-88, Math.min(88, this.extrinsics.pitch - dy * 0.25));
          this.viewAngleMode = 'custom';
          const aSel = this.panelEl?.querySelector('#camAngleSel');
          if (aSel) aSel.value = 'orbit';
        } else {
          // Chế độ đa đối tượng: kéo tự do yaw/pitch
          this.extrinsics.yaw = (this.extrinsics.yaw + dx * 0.3) % 360;
          this.extrinsics.pitch = Math.max(-88, Math.min(88, this.extrinsics.pitch - dy * 0.25));
          this.viewAngleMode = 'custom';
        }
        this._updateDrawerInputs();
      };

      this._onGlobalMouseUp = () => {
        if (this._isDraggingView) {
          this._isDraggingView = false;
          this.isInteracting = false;
          const w = this.panelEl?.querySelector('.cam-view-wrap');
          if (w) w.classList.remove('grabbing');
        }
      };

      window.addEventListener('mousemove', this._onGlobalMouseMove);
      window.addEventListener('mouseup', this._onGlobalMouseUp);

      cv.onwheel = (e) => {
        e.preventDefault();
        const delta = Math.sign(e.deltaY);
        if (this.selectedTargetId !== 'all') {
          // Zoom orbit distance VÀ FOV đồng thời
          this.orbitDistance = Math.max(0.6, Math.min(15.0, this.orbitDistance + delta * 0.25));
          this.intrinsics.fov = Math.max(25, Math.min(115, this.intrinsics.fov + delta * 2.0));
          const fovInp = panel.querySelector('#camInpFov');
          if (fovInp) fovInp.value = this.intrinsics.fov.toFixed(1);
        } else {
          this.intrinsics.fov = Math.max(25, Math.min(115, this.intrinsics.fov + delta * 2.0));
          panel.querySelector('#camInpFov').value = this.intrinsics.fov.toFixed(1);
        }
      };

      // Nhấp chuột (Click) để Khóa mục tiêu trên canvas
      cv.onclick = (e) => {
        const rect = cv.getBoundingClientRect();
        const clickU = (e.clientX - rect.left) * (cv.width / rect.width);
        const clickV = (e.clientY - rect.top) * (cv.height / rect.height);

        let hitTarget = null;
        for (const b of this.renderedBoxes) {
          if (clickU >= b.minU && clickU <= b.maxU && clickV >= b.minV && clickV <= b.maxV) {
            hitTarget = b.targetId;
            break;
          }
        }

        if (hitTarget !== null) {
          this.selectTarget(hitTarget);
        }
      };

      // 7. Inputs Căn chỉnh
      const syncInputsToExtr = () => {
        this.extrinsics.x = parseFloat(panel.querySelector('#camInpX').value) || 0;
        this.extrinsics.y = parseFloat(panel.querySelector('#camInpY').value) || 0;
        this.extrinsics.z = parseFloat(panel.querySelector('#camInpZ').value) || 2.2;
        this.extrinsics.yaw = parseFloat(panel.querySelector('#camInpYaw').value) || 0;
        this.extrinsics.pitch = parseFloat(panel.querySelector('#camInpPitch').value) || 0;
        this.intrinsics.fov = parseFloat(panel.querySelector('#camInpFov').value) || 72;
      };

      ['#camInpX', '#camInpY', '#camInpZ', '#camInpYaw', '#camInpPitch', '#camInpFov'].forEach(id => {
        panel.querySelector(id).oninput = syncInputsToExtr;
      });

      panel.querySelector('#camChkPts').onchange = (e) => { this.config.showPoints = e.target.checked; };
      panel.querySelector('#camChkBoxes').onchange = (e) => { this.config.showBoxes = e.target.checked; };
      panel.querySelector('#camChkZones').onchange = (e) => { this.config.showZones = e.target.checked; };
      panel.querySelector('#camChkRefGhost').onchange = (e) => { 
        this.config.showRefGhost = e.target.checked; 
        this.saveConfig();
      };
      panel.querySelector('#camChkRef2D').onchange = (e) => { 
        this.config.showRef2DFootprint = e.target.checked; 
        this.saveConfig();
      };

      const refSel = panel.querySelector('#camRefSel');
      if (refSel) {
        refSel.onchange = (e) => {
          this.selectedRefId = e.target.value;
          this._renderShapeInspector();
          if (typeof addLog === 'function') addLog(`[CAMERA] 📐 Đã đổi mẫu tham chiếu: ${refSel.selectedOptions[0]?.text || this.selectedRefId}`);
        };
      }

      panel.querySelector('#btnCamSaveCfg').onclick = () => {
        this.saveConfig();
        if (typeof addLog === 'function') addLog('[CAMERA] 💾 Đã lưu cấu hình Camera Calib!');
      };
    }

    /**
     * Chuyển đổi giữa chế độ Cửa Sổ Rời (Pop-out Window) và gắn lại trang Web
     */
    togglePopout() {
      if (this.isPopout) {
        this.dockBack();
      } else {
        this.openPopout();
      }
    }

    openPopout() {
      if (this.popoutWin && !this.popoutWin.closed) {
        this.popoutWin.focus();
        return;
      }
      const w = 920, h = 700;
      const left = window.screenX + 50;
      const top = window.screenY + 40;
      const win = window.open('', 'CameraAIPopout', `width=${w},height=${h},left=${left},top=${top},menubar=no,toolbar=no,location=no,status=no`);
      if (!win) {
        alert('Trình duyệt đã chặn Pop-up. Vui lòng bấm vào biểu tượng trên thanh địa chỉ duyệt web để cho phép mở Pop-up.');
        return;
      }

      this.popoutWin = win;
      this.isPopout = true;

      win.document.title = '📷 Camera AI Fusion - Cửa sổ giám sát độc lập';
      const st = win.document.createElement('style');
      st.textContent = `
        *, *::before, *::after { box-sizing: border-box; }
        body { margin: 0; padding: 0; background: #0c0d12; color: #eee; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; overflow: hidden; height: 100vh; }
      ` + (document.getElementById('camFusionStyle')?.textContent || '');
      win.document.head.appendChild(st);

      // Chuyển panelEl sang document.body của cửa sổ popup
      win.document.body.appendChild(this.panelEl);
      this.panelEl.classList.add('popout-mode', 'show');
      this.panelEl.style.position = 'static';
      this.panelEl.style.width = '100%';
      this.panelEl.style.height = '100vh';

      // Cập nhật lại canvas context và kích thước theo cửa sổ mới
      this.canvasEl = this.panelEl.querySelector('#camCanvas');
      if (this.canvasEl) {
        this.ctx = this.canvasEl.getContext('2d');
      }

      const btnPopout = this.panelEl.querySelector('#btnCamPopout');
      if (btnPopout) {
        btnPopout.textContent = '↙ Gắn vào Web';
        btnPopout.classList.add('active');
        btnPopout.title = 'Gắn Camera AI trở lại vào trang web';
      }

      // Đăng ký sự kiện chuột toàn cục cho cửa sổ popup để tương tác 3D mượt mà
      if (this._onGlobalMouseMove) win.addEventListener('mousemove', this._onGlobalMouseMove);
      if (this._onGlobalMouseUp) win.addEventListener('mouseup', this._onGlobalMouseUp);

      win.onresize = () => {
        const wrap = this.panelEl?.querySelector('.cam-view-wrap');
        if (wrap && wrap.clientWidth > 50 && wrap.clientHeight > 50 && this.canvasEl) {
          this.canvasEl.width = wrap.clientWidth;
          this.canvasEl.height = wrap.clientHeight;
          this.intrinsics.width = wrap.clientWidth;
          this.intrinsics.height = wrap.clientHeight;
        }
        this.render();
      };

      win.onbeforeunload = () => {
        this.dockBack();
      };

      // Tái kích hoạt vòng lặp render trên cửa sổ popup để đảm bảo không bị browser đình chỉ khi trang chính mất focus
      this.stopLoop();
      this.startLoop();

      if (typeof addLog === 'function') addLog('[CAMERA] ↗ Đã mở Camera AI ra Cửa sổ rời độc lập (hỗ trợ kéo sang màn hình thứ 2).');
    }

    dockBack() {
      if (!this.isPopout) return;
      this.isPopout = false;

      // Hủy lắng nghe sự kiện trên popup window
      if (this.popoutWin && !this.popoutWin.closed) {
        try {
          if (this._onGlobalMouseMove) this.popoutWin.removeEventListener('mousemove', this._onGlobalMouseMove);
          if (this._onGlobalMouseUp) this.popoutWin.removeEventListener('mouseup', this._onGlobalMouseUp);
          this.popoutWin.onresize = null;
          this.popoutWin.onbeforeunload = null;
        } catch (e) {}
      }

      // Đưa panelEl trở về document.body của trang chính
      document.body.appendChild(this.panelEl);
      this.panelEl.classList.remove('popout-mode');
      this.panelEl.style.position = 'fixed';
      this.panelEl.style.width = `${this.lastWidth || 580}px`;
      this.panelEl.style.height = 'auto';
      this.panelEl.style.left = '12px';
      this.panelEl.style.bottom = '26px';
      this.panelEl.style.top = 'auto';

      // Cập nhật lại canvas context
      this.canvasEl = this.panelEl.querySelector('#camCanvas');
      if (this.canvasEl) {
        this.ctx = this.canvasEl.getContext('2d');
      }

      const btnPopout = this.panelEl.querySelector('#btnCamPopout');
      if (btnPopout) {
        btnPopout.textContent = '↗ Cửa sổ rời';
        btnPopout.classList.remove('active');
        btnPopout.title = 'Mở Camera AI ra một cửa sổ riêng biệt ngoài trang web';
      }

      if (this.popoutWin) {
        const pWin = this.popoutWin;
        this.popoutWin = null;
        try {
          if (!pWin.closed) pWin.close();
        } catch(e) {}
      }

      // Tái kích hoạt vòng lặp render trên trang chính
      this.stopLoop();
      this.startLoop();

      if (typeof addLog === 'function') addLog('[CAMERA] ↙ Đã gắn Camera AI trở lại góc trang web.');
    }

    /**
     * Chọn mục tiêu cần theo dõi
     */
    selectTarget(targetId) {
      this.selectedTargetId = targetId;
      this._forceTargetUiUpdate = true;

      const sel = this.panelEl?.querySelector('#camTargetSel');
      if (sel) sel.value = String(targetId);

      const items = this.panelEl?.querySelectorAll('.target-item');
      items?.forEach(it => {
        const id = it.getAttribute('data-id');
        it.classList.toggle('active', id === String(targetId));
      });

      if (targetId !== 'all') {
        const hist = this.targetHistory.get(targetId);
        const label = hist?.active ? `Mục tiêu #${targetId}` : `Mục tiêu #${targetId} (Lịch sử đã rời)`;
        if (typeof addLog === 'function') addLog(`[CAMERA] 🎯 Đã chọn ${label} để soi hình dạng & góc nhìn.`);
        this.applyViewAngle(this.viewAngleMode);
      }

      // Ngay lập tức cập nhật khung soi hình dạng & modal phóng to nếu đang mở
      this._renderShapeInspector();
      if (this.isEnlargeModalOpen && targetId !== 'all') {
        const t = this.targetHistory.get(targetId);
        if (t) this._renderEnlargedShapeInspector(t);
      }
    }

    /**
     * Cập nhật danh sách mục tiêu (Bao gồm CẢ Realtime Đang có mặt & Lịch sử Đã rời vùng)
     */
    /**
     * Cập nhật danh sách thẻ mục tiêu bên trái (Chỉ chạy định kỳ 250ms hoặc khi có đối tượng mới để chống đơ lag)
     */
    _updateTargetList(activeTargets) {
      const panel = this.panelEl;
      if (!panel) return;

      const totalItems = this.targetHistory.size;
      const badgeEl = panel.querySelector('#camTargetBadge');
      const headerCountEl = panel.querySelector('#camTargetHeaderCount');
      if (badgeEl) badgeEl.textContent = activeTargets.length ? `${activeTargets.length}` : (totalItems ? `${totalItems} lưu` : '0');
      if (headerCountEl) headerCountEl.textContent = totalItems;

      // 1. Cập nhật dropdown
      this._updateTargetDropdown();

      // 2. Cập nhật danh sách thẻ mục tiêu (Target items)
      const listEl = panel.querySelector('#camTargetList');
      if (listEl) {
        if (!totalItems) {
          listEl.innerHTML = '<div style="color:#777;font-style:italic;padding:12px;text-align:center;">Chưa có đối tượng nào trong tầm quét</div>';
          return;
        }

        const sortedHistory = Array.from(this.targetHistory.values()).sort((a, b) => {
          if (a.active !== b.active) return a.active ? -1 : 1;
          return b.lastSeen - a.lastSeen;
        });

        // Kiểm tra xem danh sách có thay đổi ID hoặc trạng thái không trước khi ghi đè innerHTML
        const currentKey = sortedHistory.map(i => `${i.id}_${i.active ? 1 : 0}`).join('|') + `_sel:${this.selectedTargetId}`;
        if (listEl.getAttribute('data-list-key') === currentKey) {
          return; // Danh sách không đổi, không vẽ lại DOM để tránh giật lag
        }
        listEl.setAttribute('data-list-key', currentKey);

        let html = `
          <div class="target-item ${this.selectedTargetId === 'all' ? 'active' : ''}" data-id="all">
            <div class="target-item-header">
              <span>👁️ <b>Xem tất cả mục tiêu (Multi)</b></span>
              <span class="target-item-badge">Bao quát</span>
            </div>
            <div class="target-item-meta">
              <span>Realtime: ${activeTargets.length} | Tổng lưu: ${totalItems}</span>
              <span>Toàn cảnh</span>
            </div>
          </div>
        `;

        for (const item of sortedHistory) {
          const isAct = (this.selectedTargetId === item.id);
          const isLive = !!item.active;
          const isAlarm = isLive && (!!window.alarmOn || (window._ev?.confirmed > 0));
          const badgeClass = isLive ? (isAlarm ? 'alarm' : '') : 'stale';
          const badgeText = isLive ? (isAlarm ? 'BÁO ĐỘNG' : 'DI CHUYỂN') : `ĐÃ RỜI (${item.leftTimeStr || item.timeStr})`;

          html += `
            <div class="target-item ${isAct ? 'active' : ''} ${!isLive ? 'stale' : ''}" data-id="${item.id}">
              <div class="target-item-header">
                <span>🎯 <b>Mục tiêu #${item.id}</b></span>
                <span class="target-item-badge ${badgeClass}">${badgeText}</span>
              </div>
              <div class="target-item-meta" style="display:flex;justify-content:space-between;align-items:center;">
                <span>📐 ${(item.w || 0.4).toFixed(2)}x${(item.h || 0.3).toFixed(2)}m • ${(item.maxSpeed || 0).toFixed(1)} m/s</span>
                <button class="btn-goto-blackbox" data-target-id="${item.id}" style="background:#0d6efd;color:#fff;border:none;border-radius:3px;padding:2px 7px;font-size:9px;font-weight:bold;cursor:pointer;" title="Mở đoạn phát lại sự cố của Mục tiêu #${item.id} trong Hộp đen">📼 Sự cố</button>
              </div>
            </div>
          `;
        }
        listEl.innerHTML = html;

        // Gắn sự kiện click vào từng thẻ
        listEl.querySelectorAll('.target-item').forEach(card => {
          card.onclick = () => {
            const id = card.getAttribute('data-id');
            this.selectTarget(id === 'all' ? 'all' : parseInt(id, 10));
          };
        });

        // Gắn sự kiện chuyển liên thông sang Hộp đen Blackbox theo ID mục tiêu
        listEl.querySelectorAll('.btn-goto-blackbox').forEach(btn => {
          btn.onclick = (e) => {
            e.stopPropagation();
            const tid = btn.getAttribute('data-target-id');
            if (window.eventPlaybackMgr) {
              window.eventPlaybackMgr.loadEventByTargetId(tid);
            }
          };
        });
      }
    }

    /**
     * Trích xuất mây điểm của đối tượng phân theo từng LiDAR (L1, L2)
     * Tích hợp Thuật toán Phân cụm & Lọc bỏ Bờ Tường/Mép Biên thẳng (Wall/Linear Clutter Filter)
     */
    _extractTargetFusedPoints(t, frames) {
      const rawCandidates = [];
      const searchRadius = Math.max(0.75, Math.hypot(t.w || 0.45, t.h || 0.35) * 0.85);
      const r2 = searchRadius * searchRadius;

      for (let s = 0; s < frames.length; s++) {
        const f = frames[s];
        if (!f || !f.n) continue;
        const hasXYZ = f.xyz && f.xyz.length >= f.n * 3;
        const n = f.n;

        for (let i = 0; i < n; i++) {
          let px = 0, py = 0, pz = 0;
          if (hasXYZ) {
            px = f.xyz[i * 3];
            py = f.xyz[i * 3 + 1];
            pz = f.xyz[i * 3 + 2];
          } else if (f.pts) {
            px = f.pts[i * 3];
            py = f.pts[i * 3 + 1];
          }
          const dx = px - t.x;
          const dy = py - t.y;
          if (dx * dx + dy * dy <= r2) {
            rawCandidates.push({ x: px, y: py, z: pz, sensor: s });
          }
        }
      }

      if (rawCandidates.length === 0) {
        return {
          ptsL1: [],
          ptsL2: [],
          allPts: [],
          centroidX: t.x,
          centroidY: t.y,
          measuredW: t.w || 0.40,
          measuredH: t.h || 0.35,
          hasPoints: false
        };
      }

      // --- PHƯƠNG ÁN A: PHÂN CỤM DBSCAN VÀ LOẠI BỎ VỆT THẲNG BỜ TƯỜNG (WALL FILTER) ---
      // 1. Gom các điểm thành các cụm kết nối (Cluster) với khoảng cách lân cận 14cm
      const clusterDist2 = 0.14 * 0.14;
      const visited = new Uint8Array(rawCandidates.length);
      const clusters = [];

      for (let i = 0; i < rawCandidates.length; i++) {
        if (visited[i]) continue;
        visited[i] = 1;
        const cluster = [rawCandidates[i]];
        const queue = [i];

        while (queue.length > 0) {
          const curIdx = queue.pop();
          const p1 = rawCandidates[curIdx];
          for (let j = 0; j < rawCandidates.length; j++) {
            if (!visited[j]) {
              const p2 = rawCandidates[j];
              const d2 = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
              if (d2 <= clusterDist2) {
                visited[j] = 1;
                cluster.push(p2);
                queue.push(j);
              }
            }
          }
        }
        clusters.push(cluster);
      }

      // 1.1. Xử lý trường hợp tốc độ cao khiến người dính liền bờ tường (Line-Splitting bằng RANSAC)
      // Nếu 1 cụm có quá nhiều điểm (> 15 điểm) và trải dài (> 0.7m), kiểm tra xem có đường thẳng bờ tường chạy xuyên qua không
      const refinedClusters = [];
      clusters.forEach(pts => {
        if (pts.length < 16) {
          refinedClusters.push(pts);
          return;
        }

        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        pts.forEach(p => {
          if (p.x < minX) minX = p.x;
          if (p.x > maxX) maxX = p.x;
          if (p.y < minY) minY = p.y;
          if (p.y > maxY) maxY = p.y;
        });
        const span = Math.hypot(maxX - minX, maxY - minY);
        if (span < 0.70) {
          refinedClusters.push(pts);
          return;
        }

        // Tìm đường thẳng bờ tường mạnh nhất bằng RANSAC
        let bestInliers = [];
        let bestNonInliers = [];
        const iterations = 30;
        const lineThresh = 0.06; // Khoảng cách tới đường thẳng 6cm

        for (let it = 0; it < iterations; it++) {
          const idx1 = Math.floor(Math.random() * pts.length);
          let idx2 = Math.floor(Math.random() * pts.length);
          if (idx1 === idx2) continue;
          const p1 = pts[idx1], p2 = pts[idx2];
          const ldx = p2.x - p1.x, ldy = p2.y - p1.y;
          const len = Math.hypot(ldx, ldy);
          if (len < 0.35) continue; // Đoạn mẫu phải đủ dài

          const inliers = [];
          const nonInliers = [];
          for (let k = 0; k < pts.length; k++) {
            const pk = pts[k];
            // Khoảng cách từ pk tới đường thẳng đi qua p1-p2
            const dist = Math.abs(ldy * pk.x - ldx * pk.y + p2.x * p1.y - p2.y * p1.x) / len;
            if (dist <= lineThresh) inliers.push(pk);
            else nonInliers.push(pk);
          }

          if (inliers.length > bestInliers.length) {
            bestInliers = inliers;
            bestNonInliers = nonInliers;
          }
        }

        // Nếu tìm thấy một đường thẳng bờ tường với >= 12 điểm và chiếm phần lớn dải quét
        if (bestInliers.length >= 12 && bestNonInliers.length >= 3) {
          refinedClusters.push(bestInliers); // Cụm bờ tường
          refinedClusters.push(bestNonInliers); // Cụm đối tượng người tách riêng
        } else {
          refinedClusters.push(pts);
        }
      });

      // 2. Phân tích từng cụm: Đánh giá độ dài, độ thẳng hàng (Linearity) và khoảng cách tới tâm mục tiêu
      const evaluatedClusters = refinedClusters.map(pts => {
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        let sumX = 0, sumY = 0;
        pts.forEach(p => {
          sumX += p.x; sumY += p.y;
          if (p.x < minX) minX = p.x;
          if (p.x > maxX) maxX = p.x;
          if (p.y < minY) minY = p.y;
          if (p.y > maxY) maxY = p.y;
        });
        const cX = sumX / pts.length;
        const cY = sumY / pts.length;
        const spanX = maxX - minX;
        const spanY = maxY - minY;
        const maxSpan = Math.hypot(spanX, spanY);

        // Tính phân tán dọc/ngang để kiểm tra độ thẳng hàng (Linearity)
        let varXX = 0, varYY = 0, varXY = 0;
        pts.forEach(p => {
          const dx = p.x - cX;
          const dy = p.y - cY;
          varXX += dx * dx;
          varYY += dy * dy;
          varXY += dx * dy;
        });
        const n = Math.max(1, pts.length);
        varXX /= n; varYY /= n; varXY /= n;
        // Giá trị riêng phân rã PCA 2D
        const trace = varXX + varYY;
        const det = varXX * varYY - varXY * varXY;
        const disc = Math.max(0, trace * trace / 4 - det);
        const l1 = trace / 2 + Math.sqrt(disc);
        const l2 = Math.max(0.0001, trace / 2 - Math.sqrt(disc));
        const linearity = l1 / (l1 + l2); // Càng gần 1.0 thì càng thẳng tắp thành đường thẳng

        // Nhận diện bờ tường / mép biên:
        // Cụm kéo dài > 0.55m VÀ độ thẳng hàng cao (linearity > 0.88)
        const isWall = (pts.length >= 8 && maxSpan > 0.55 && linearity > 0.88);

        // Khoảng cách từ trọng tâm cụm đến vị trí tracking của mục tiêu
        const distToTarget = Math.hypot(cX - t.x, cY - t.y);

        return {
          pts,
          cX, cY,
          minX, maxX, minY, maxY,
          maxSpan,
          linearity,
          isWall,
          distToTarget
        };
      });

      // 3. Chọn lọc cụm:
      // - Nếu có nhiều hơn 1 cụm và có cụm là bờ tường (isWall), hãy loại bỏ cụm bờ tường
      // - Ưu tiên cụm có dạng cơ thể người (không phải vệt tường thẳng dài) gần tâm mục tiêu nhất
      let validClusters = evaluatedClusters.filter(c => !c.isWall);
      if (validClusters.length === 0) {
        // Nếu tất cả các cụm đều bị coi là tường (hiếm khi xảy ra), lấy cụm gần tâm mục tiêu nhất
        validClusters = [evaluatedClusters.reduce((best, c) => c.distToTarget < best.distToTarget ? c : best, evaluatedClusters[0])];
      }

      // Giữ lại các điểm thuộc về các cụm hợp lệ (loại trừ hẳn bờ tường)
      const finalPts = [];
      validClusters.forEach(c => {
        c.pts.forEach(p => finalPts.push(p));
      });

      const ptsL1 = [];
      const ptsL2 = [];
      let sumX = 0, sumY = 0;
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;

      finalPts.forEach(p => {
        if (p.sensor === 0) ptsL1.push(p);
        else ptsL2.push(p);
        sumX += p.x;
        sumY += p.y;
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
      });

      const count = finalPts.length;
      const centroidX = count > 0 ? (sumX / count) : t.x;
      const centroidY = count > 0 ? (sumY / count) : t.y;
      const measuredW = count >= 2 ? Math.max(0.18, maxX - minX) : (t.w || 0.40);
      const measuredH = count >= 2 ? Math.max(0.18, maxY - minY) : (t.h || 0.35);

      return {
        ptsL1,
        ptsL2,
        allPts: finalPts,
        centroidX,
        centroidY,
        measuredW,
        measuredH,
        hasPoints: count > 0
      };
    }

    /**
     * VẼ RENDER KHUNG SOI HÌNH DẠNG CƠ BẢN (SHAPE INSPECTOR)
     * THIẾT KẾ PERSISTENT: KHÔNG BAO GIỜ XÓA / TẠO LẠI DOM MỖI FRAME NÊN 100% KHÔNG ĐƠ KHỰNG VÀ NÚT BẤM CỰC NHẠY!
     */
    _renderShapeInspector() {
      const previewEl = this.panelEl?.querySelector('#camShapePreview');
      if (!previewEl) return;

      if (this.selectedTargetId === 'all') {
        if (previewEl.getAttribute('data-target-id') !== 'all') {
          previewEl.setAttribute('data-target-id', 'all');
          previewEl.innerHTML = `
            <div style="color:#888;font-style:italic;padding:18px;text-align:center;font-size:10px;">
              Đang ở chế độ bao quát tất cả mục tiêu.<br>
              Bấm chọn <b>1 mục tiêu bên trái</b> (kể cả đã rời đi) hoặc click trên Camera để soi hình dạng chi tiết.
            </div>
          `;
          const modal = this.panelEl?.querySelector('#camEnlargeModal');
          if (modal) modal.style.display = 'none';
          this.isEnlargeModalOpen = false;
        }
        return;
      }

      // Tìm thông tin đối tượng từ bộ nhớ lịch sử (giữ lại ảnh snapshot hoàn chỉnh)
      const t = this.targetHistory.get(this.selectedTargetId);
      if (!t) {
        if (previewEl.getAttribute('data-target-id') !== `err_${this.selectedTargetId}`) {
          previewEl.setAttribute('data-target-id', `err_${this.selectedTargetId}`);
          previewEl.innerHTML = `
            <div style="color:#888;font-style:italic;padding:18px;text-align:center;font-size:10px;">
              Không tìm thấy dữ liệu của Mục tiêu #${this.selectedTargetId}.
            </div>
          `;
        }
        return;
      }

      const isLive = !!t.active;
      const w = t.w || 0.40;
      const d = t.h || 0.30;
      const z1 = Number.isFinite(t.z1) ? t.z1 : 0.0;
      const z2 = Number.isFinite(t.z2) && (t.z2 - z1 > 0.3) ? t.z2 : (z1 + 1.65);
      const height = z2 - z1;
      const speed = isLive ? Math.hypot(t.vx || 0, t.vy || 0) : (t.maxSpeed || 0);
      const area = w * d;
      const isAlarm = isLive && (!!window.alarmOn || (window._ev?.confirmed > 0));

      let category = '👤 Người đi bộ';
      if (w > 1.4 || d > 2.0) category = '🚗 Xe ô tô / Xe tải';
      else if (w > 0.9 && height > 1.8) category = '🚜 Xe nâng / AGV';
      else if (w > 0.6 && d > 1.2) category = '🏍️ Xe máy';
      else if (w < 0.35 && d < 0.35 && height < 0.6) category = '🐕 Thú cưng / Vật nhỏ';
      else if (height < 1.0) category = '📦 Kiện hàng / Pallet';

      const fused = t.fused || { ptsL1: [], ptsL2: [], allPts: [] };
      const statusHtml = isLive 
        ? `<span style="font-weight:bold;color:${isAlarm ? '#ff4d4d' : '#2ecc71'};">${isAlarm ? '🚨 BÁO ĐỘNG REALTIME' : '🟢 DI CHUYỂN REALTIME'}</span>`
        : `<span style="font-weight:bold;color:#f39c12;">⏱ ĐÃ RỜI VÙNG (${t.leftTimeStr || t.timeStr})</span>`;

      // 1. XÂY DỰNG DOM CẤU TRÚC 1 LẦN DUY NHẤT (Khi đổi mục tiêu) - TUYỆT ĐỐI KHÔNG RE-RENDER MỖI FRAME!
      const currentTargetKey = `${t.id}_${isLive ? '1' : '0'}`;
      if (previewEl.getAttribute('data-target-id') !== currentTargetKey) {
        previewEl.setAttribute('data-target-id', currentTargetKey);
        previewEl.innerHTML = `
          <div class="shape-card-top">
            <span>🎯 <b>MỤC TIÊU #${t.id}</b> <span id="shapeCategoryTag" class="shape-category">${category}</span></span>
            <div style="display:flex;align-items:center;gap:4px;margin-left:auto;">
              <button id="btnCamCaptureRef" style="background:#e67e22;color:#fff;border:1px solid #d35400;font-weight:bold;font-size:10px;padding:3px 8px;border-radius:3px;cursor:pointer;" title="Lưu hình dạng đối tượng này thành mẫu tham chiếu 2D/3D">📸 Chụp Mẫu</button>
              <button id="btnEnlargeShape" style="background:#0d6efd;color:#fff;border:1px solid #0b5ed7;font-weight:bold;font-size:10px;padding:3px 8px;border-radius:3px;cursor:pointer;" title="Phóng to soi chi tiết hình dạng & tham chiếu">🔍 Phóng To</button>
            </div>
          </div>
          <div id="shapeStatusLine" style="margin-bottom:3px;font-size:10px;">${statusHtml}</div>

          <div class="shape-content-grid">
            <div class="shape-canvas-box">
              <div style="position:relative;display:inline-block;cursor:pointer;" id="shapeCanvasWrap" title="Click vào để phóng to hình dạng & tham chiếu">
                <canvas id="camMiniShapeCanvas" width="95" height="95"></canvas>
                <div style="position:absolute;right:2px;top:2px;background:rgba(0,0,0,0.65);border:1px solid #00e5ff;color:#00ffcc;border-radius:2px;font-size:8px;padding:1px 3px;">🔍</div>
              </div>
              <div class="shape-canvas-lbl">${isLive ? 'Mặt cắt 2D Realtime' : 'Ảnh Snapshot 2D'} (Click phóng to)</div>
            </div>
            <div class="shape-metrics-list">
              <div id="shapeMetricDim">📐 Dài x Rộng: <b>${w.toFixed(2)} × ${d.toFixed(2)} m</b></div>
              <div id="shapeMetricArea">📏 Diện tích đáy: <b>${area.toFixed(2)} m²</b></div>
              <div id="shapeMetricHeight">↕ Chiều cao: <b>${height.toFixed(2)} m</b> (Z: ${z1.toFixed(1)}→${z2.toFixed(1)}m)</div>
              <div id="shapeMetricSpeed">⚡ Vận tốc: <b>${speed.toFixed(2)} m/s</b> (${(speed * 3.6).toFixed(1)} km/h)</div>
              <div id="shapeMetricPos">📍 Vị trí ${isLive ? 'hiện tại' : 'lúc rời'}: <b>(${t.x.toFixed(2)}, ${t.y.toFixed(2)}) m</b></div>
              <div id="shapeMetricTrail" style="color:#00ffcc;font-weight:bold;">🛤 Lộ trình: 0.00 m</div>
              <div id="shapeMetricL1" style="color:#00e5ff;font-weight:bold;">🔵 LiDAR 1: ${fused.ptsL1.length} pts (Trước)</div>
              <div id="shapeMetricL2" style="color:#ff9900;font-weight:bold;">🟠 LiDAR 2: ${fused.ptsL2.length} pts (Sau)</div>
            </div>
          </div>

          <div class="shape-quick-btns">
            <span>Góc:</span>
            <button class="btn-shape-angle" data-angle="lidar1">📡 Trước</button>
            <button class="btn-shape-angle" data-angle="lidar2">📡 Sau</button>
            <button class="btn-shape-angle" data-angle="orbit">🔄 360°</button>
            <button class="btn-shape-angle" data-angle="top">⬇ Trần</button>
          </div>
        `;

        // Gắn sự kiện các nút chuyển góc nhìn
        previewEl.querySelectorAll('.btn-shape-angle').forEach(btn => {
          btn.onclick = (e) => {
            e.stopPropagation();
            const mode = btn.getAttribute('data-angle');
            const sel = this.panelEl?.querySelector('#camAngleSel');
            if (sel) sel.value = mode;
            this.applyViewAngle(mode);
          };
        });

        // Gắn sự kiện Chụp Mẫu Tham Chiếu từ Camera AI
        const btnCap = previewEl.querySelector('#btnCamCaptureRef');
        if (btnCap) {
          btnCap.onclick = (e) => {
            e.stopPropagation();
            this.captureTargetAsReference(t);
          };
        }

        // Gắn sự kiện Phóng to Modal
        const openModalFn = (e) => {
          if (e) e.stopPropagation();
          this.openEnlargeModal(t);
        };

        const btnEnlarge = previewEl.querySelector('#btnEnlargeShape');
        if (btnEnlarge) btnEnlarge.onclick = openModalFn;

        const cvWrap = previewEl.querySelector('#shapeCanvasWrap');
        if (cvWrap) cvWrap.onclick = openModalFn;
      }

      // 2. CẬP NHẬT TEXT DỮ LIỆU ĐỘNG (Không hủy phần tử DOM)
      const catTag = previewEl.querySelector('#shapeCategoryTag');
      if (catTag) catTag.textContent = category;
      const statusEl = previewEl.querySelector('#shapeStatusLine');
      if (statusEl) statusEl.innerHTML = statusHtml;
      const dimEl = previewEl.querySelector('#shapeMetricDim');
      if (dimEl) dimEl.innerHTML = `📐 Dài x Rộng: <b>${w.toFixed(2)} × ${d.toFixed(2)} m</b>`;
      const areaEl = previewEl.querySelector('#shapeMetricArea');
      if (areaEl) areaEl.innerHTML = `📏 Diện tích đáy: <b>${area.toFixed(2)} m²</b>`;
      const hEl = previewEl.querySelector('#shapeMetricHeight');
      if (hEl) hEl.innerHTML = `↕ Chiều cao: <b>${height.toFixed(2)} m</b> (Z: ${z1.toFixed(1)}→${z2.toFixed(1)}m)`;
      const curSpeed = isLive ? Math.hypot(t.vx || 0, t.vy || 0) : 0;
      const maxSpeed = t.maxSpeed || speed;
      const spdEl = previewEl.querySelector('#shapeMetricSpeed');
      if (spdEl) {
        if (isLive) {
          spdEl.innerHTML = `⚡ Vận tốc: <b>${curSpeed.toFixed(2)} m/s</b> (${(curSpeed * 3.6).toFixed(1)} km/h) <span style="color:#00ffcc;font-size:9px;">[Max: ${(maxSpeed * 3.6).toFixed(1)} km/h]</span>`;
        } else {
          spdEl.innerHTML = `⚡ Vận tốc cực đại (Max): <b>${maxSpeed.toFixed(2)} m/s</b> (${(maxSpeed * 3.6).toFixed(1)} km/h)`;
        }
      }
      const posEl = previewEl.querySelector('#shapeMetricPos');
      if (posEl) {
        const critPosStr = (t.criticalX !== undefined && t.criticalX !== 0) ? ` <span style="color:#ffcc00;font-size:9px;" title="Vị trí tiến sâu nhất vào lòng ray">[Sâu nhất: (${t.criticalX.toFixed(2)}, ${t.criticalY.toFixed(2)})m]</span>` : '';
        posEl.innerHTML = `📍 Vị trí ${isLive ? 'hiện tại' : 'lúc rời'}: <b>(${t.x.toFixed(2)}, ${t.y.toFixed(2)}) m</b>${critPosStr}`;
      }
      const trailEl = previewEl.querySelector('#shapeMetricTrail');
      if (trailEl) {
        let totalDist = 0;
        const tr = t.trail || [];
        for (let i = 1; i < tr.length; i++) {
          const dx = tr[i].x - tr[i-1].x;
          const dy = tr[i].y - tr[i-1].y;
          totalDist += Math.hypot(dx, dy);
        }
        const trailPtsCount = tr.length;
        const trailDuration = (tr.length >= 2) ? ((tr[tr.length - 1].t - tr[0].t) / 1000) : 0;
        trailEl.innerHTML = `🛤 Lộ trình: <b>${totalDist.toFixed(2)} m</b> / ${trailDuration.toFixed(1)}s (${trailPtsCount} điểm)`;
      }
      const l1El = previewEl.querySelector('#shapeMetricL1');
      if (l1El) l1El.textContent = `🔵 LiDAR 1: ${fused.ptsL1.length} pts (Trước)`;
      const l2El = previewEl.querySelector('#shapeMetricL2');
      if (l2El) l2El.textContent = `🟠 LiDAR 2: ${fused.ptsL2.length} pts (Sau)`;

      // 3. VẼ CANVAS MINI 2D TOP-DOWN FOOTPRINT
      const miniCv = previewEl.querySelector('#camMiniShapeCanvas');
      if (miniCv) {
        const mctx = miniCv.getContext('2d');
        const mw = miniCv.width, mh = miniCv.height;
        mctx.clearRect(0, 0, mw, mh);

        const cx = mw / 2, cy = mh / 2;
        const scale = 50;

        mctx.strokeStyle = '#222533';
        mctx.lineWidth = 1;
        mctx.beginPath();
        mctx.moveTo(0, cy); mctx.lineTo(mw, cy);
        mctx.moveTo(cx, 0); mctx.lineTo(cx, mh);
        mctx.stroke();

        // Mặt cắt tham chiếu 2D
        const refModel = this.getActiveReferenceModel();
        if (this.config.showRef2DFootprint && refModel) {
          mctx.save();
          mctx.strokeStyle = refModel.color || '#2ecc71';
          mctx.lineWidth = 1.6;
          mctx.setLineDash([4, 2]);

          if (refModel.contour && refModel.contour.length >= 3) {
            mctx.beginPath();
            const refCentroidX = refModel.centroid?.x || 0;
            const refCentroidY = refModel.centroid?.y || 0;
            refModel.contour.forEach((pt, idx) => {
              const rx = cx + (pt[0] - refCentroidX) * scale;
              const ry = cy + (pt[1] - refCentroidY) * scale;
              if (idx === 0) mctx.moveTo(rx, ry);
              else mctx.lineTo(rx, ry);
            });
            mctx.closePath();
            mctx.stroke();
          } else {
            const refHwPix = (refModel.w / 2) * scale;
            const refHlPix = (refModel.l / 2) * scale;
            mctx.strokeRect(cx - refHwPix, cy - refHlPix, refHwPix * 2, refHlPix * 2);
          }
          mctx.restore();
        }

        // Viền hộp thực tế của đối tượng (Ôm sát quanh trọng tâm cụm điểm)
        const hasPoints = (fused.ptsL1?.length || 0) + (fused.ptsL2?.length || 0) > 0;
        const boxW = (hasPoints && fused.measuredW) ? Math.max(w, fused.measuredW) : w;
        const boxD = (hasPoints && fused.measuredH) ? Math.max(d, fused.measuredH) : d;
        const cX0 = (hasPoints && Number.isFinite(fused.centroidX)) ? fused.centroidX : t.x;
        const cY0 = (hasPoints && Number.isFinite(fused.centroidY)) ? fused.centroidY : t.y;

        mctx.strokeStyle = isLive ? '#ffcc00' : '#f39c12';
        mctx.lineWidth = 1.5;
        const hwPix = (boxW / 2) * scale;
        const hdPix = (boxD / 2) * scale;
        mctx.strokeRect(cx - hwPix, cy - hdPix, hwPix * 2, hdPix * 2);

        // Chấm laser L1 (Cyan)
        mctx.fillStyle = '#00e5ff';
        for (const p of fused.ptsL1) {
          const px = cx + (p.x - cX0) * scale;
          const py = cy + (p.y - cY0) * scale;
          mctx.fillRect(px - 1.5, py - 1.5, 3, 3);
        }

        // Chấm laser L2 (Orange)
        mctx.fillStyle = '#ff9900';
        for (const p of fused.ptsL2) {
          const px = cx + (p.x - cX0) * scale;
          const py = cy + (p.y - cY0) * scale;
          mctx.fillRect(px - 1.5, py - 1.5, 3, 3);
        }

        if (speed > 0.05) {
          mctx.strokeStyle = '#ffffff';
          mctx.lineWidth = 1.5;
          mctx.beginPath();
          mctx.moveTo(cx, cy);
          mctx.lineTo(cx + (t.vx || 0) * scale * 0.4, cy + (t.vy || 0) * scale * 0.4);
          mctx.stroke();
        }

        // Vẽ lộ trình 2D trên mini canvas (nét đứt xanh lá)
        const miniTrail = t.trail;
        if (miniTrail && miniTrail.length >= 2) {
          mctx.save();
          mctx.strokeStyle = '#2ecc71';
          mctx.lineWidth = 1.2;
          mctx.setLineDash([3, 2]);
          mctx.beginPath();
          let trailStarted = false;
          for (let ti = 0; ti < miniTrail.length; ti++) {
            const tp = miniTrail[ti];
            const tpx = cx + (tp.x - cX0) * scale;
            const tpy = cy + (tp.y - cY0) * scale;
            if (tpx >= 0 && tpx <= mw && tpy >= 0 && tpy <= mh) {
              if (!trailStarted) { mctx.moveTo(tpx, tpy); trailStarted = true; }
              else mctx.lineTo(tpx, tpy);
            }
          }
          mctx.stroke();
          mctx.setLineDash([]);
          // Điểm bắt đầu (xanh lá)
          const ts = miniTrail[0];
          const tsx = cx + (ts.x - cX0) * scale;
          const tsy = cy + (ts.y - cY0) * scale;
          if (tsx >= 0 && tsx <= mw && tsy >= 0 && tsy <= mh) {
            mctx.fillStyle = '#2ecc71';
            mctx.beginPath(); mctx.arc(tsx, tsy, 2, 0, Math.PI * 2); mctx.fill();
          }
          mctx.restore();
        }

        mctx.fillStyle = isLive ? '#ff0055' : '#888888';
        mctx.beginPath();
        mctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
        mctx.fill();
      }

      // 4. NẾU CỬA SỔ PHÓNG TO ĐANG MỞ, CẬP NHẬT RENDER CHO MODAL
      if (this.isEnlargeModalOpen) {
        this._renderEnlargedShapeInspector(t, fused, w, d, height, speed, z1, z2, category);
      }
    }

    /**
     * MỞ CỬA SỔ PHÓNG TO SOI HÌNH DẠNG & THAM CHIẾU
     */
    openEnlargeModal(t) {
      if (!t && this.selectedTargetId !== 'all') {
        t = this.targetHistory.get(this.selectedTargetId);
      }
      if (!t) {
        alert('Vui lòng chọn 1 mục tiêu cụ thể để phóng to soi hình dạng!');
        return;
      }

      const modal = this.panelEl?.querySelector('#camEnlargeModal');
      if (modal) {
        modal.style.display = 'flex';
        this.isEnlargeModalOpen = true;
        this._renderEnlargedShapeInspector(t);
      }
    }

    /**
     * VẼ GIAO DIỆN PHÓNG TO SOI HÌNH DẠNG CHI TIẾT (ENLARGED SHAPE MODAL)
     */
    _renderEnlargedShapeInspector(t, fused, w, d, height, speed, z1, z2, category) {
      const modal = this.panelEl?.querySelector('#camEnlargeModal');
      if (!modal || modal.style.display === 'none') return;
      if (!t) return;

      if (!fused) {
        fused = t.fused || { ptsL1: [], ptsL2: [], allPts: [] };
      }
      if (w === undefined) w = t.w || t.maxW || 0.40;
      if (d === undefined) d = t.h || t.maxH || 0.35;
      if (z1 === undefined) z1 = Number.isFinite(t.z1) ? t.z1 : 0.0;
      if (z2 === undefined) z2 = Number.isFinite(t.z2) ? t.z2 : (z1 + 1.7);
      if (height === undefined) height = Math.max(0.25, z2 - z1);
      if (speed === undefined) speed = Math.hypot(t.vx || 0, t.vy || 0);
      if (!category) {
        const area = w * d;
        if (w > 1.2 || d > 1.8) category = '🚗 Ô tô / Kiện lớn';
        else if (height > 1.3 && area < 0.6) category = '🚶 Người đi bộ';
        else if (area > 0.4) category = '📦 Hàng hóa / Xe máy';
        else category = '⚪ Vật thể / Chướng ngại';
      }

      const scale = this.enlargeZoomScale || 100;

      const titleEl = modal.querySelector('#camEnlargeTitle');
      if (titleEl) {
        const isLive = !!t.active;
        const statusStr = isLive ? '🟢 REALTIME' : `⏱ ĐÃ RỜI (${t.leftTimeStr || t.timeStr})`;
        titleEl.innerHTML = `🔍 <b>MỤC TIÊU #${t.id} [${category}]</b> <span style="font-size:10px;color:#f39c12;margin-left:6px;">${statusStr}</span> <span style="font-size:10px;color:#00e5ff;margin-left:8px;">[Thu phóng: ${Math.round(scale)}%]</span>`;
      }

      const canvas = modal.querySelector('#camEnlargeCanvas');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const W = canvas.width, H = canvas.height;
      ctx.clearRect(0, 0, W, H);

      const cx = W / 2, cy = H / 2;

      // 1. Lưới tọa độ mét (0.25m nét mảnh, 0.5m nét đậm)
      ctx.lineWidth = 1;
      for (let x = -2.5; x <= 2.5; x += 0.25) {
        const px = cx + x * scale;
        if (px < 0 || px > W) continue;
        const isMajor = Math.abs(x % 0.5) < 0.01;
        ctx.strokeStyle = isMajor ? 'rgba(60, 65, 90, 0.65)' : 'rgba(40, 45, 65, 0.35)';
        ctx.beginPath();
        ctx.moveTo(px, 0); ctx.lineTo(px, H);
        ctx.stroke();

        if (isMajor && Math.abs(x) > 0.05) {
          ctx.fillStyle = '#6c7293';
          ctx.font = '8px monospace';
          ctx.fillText(`${x > 0 ? '+' : ''}${x.toFixed(1)}m`, px + 2, H - 4);
        }
      }
      for (let y = -2.0; y <= 2.0; y += 0.25) {
        const py = cy + y * scale;
        if (py < 0 || py > H) continue;
        const isMajor = Math.abs(y % 0.5) < 0.01;
        ctx.strokeStyle = isMajor ? 'rgba(60, 65, 90, 0.65)' : 'rgba(40, 45, 65, 0.35)';
        ctx.beginPath();
        ctx.moveTo(0, py); ctx.lineTo(W, py);
        ctx.stroke();

        if (isMajor && Math.abs(y) > 0.05) {
          ctx.fillStyle = '#6c7293';
          ctx.font = '8px monospace';
          ctx.fillText(`${-y > 0 ? '+' : ''}${-y.toFixed(1)}m`, 4, py - 2);
        }
      }

      // Trục tọa độ chính (X, Y)
      ctx.strokeStyle = '#4a5173';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(0, cy); ctx.lineTo(W, cy);
      ctx.moveTo(cx, 0); ctx.lineTo(cx, H);
      ctx.stroke();

      // 2. Mẫu tham chiếu 2D (Reference Model)
      const refModel = this.getActiveReferenceModel();
      if (refModel) {
        ctx.save();
        ctx.strokeStyle = refModel.color || '#2ecc71';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([5, 3]);

        if (refModel.contour && refModel.contour.length >= 3) {
          ctx.beginPath();
          const refCentroidX = refModel.centroid?.x || 0;
          const refCentroidY = refModel.centroid?.y || 0;
          refModel.contour.forEach((pt, idx) => {
            const rx = cx + (pt[0] - refCentroidX) * scale;
            const ry = cy + (pt[1] - refCentroidY) * scale;
            if (idx === 0) ctx.moveTo(rx, ry);
            else ctx.lineTo(rx, ry);
          });
          ctx.closePath();
          ctx.stroke();
        } else {
          const refHwPix = (refModel.w / 2) * scale;
          const refHlPix = (refModel.l / 2) * scale;
          ctx.strokeRect(cx - refHwPix, cy - refHlPix, refHwPix * 2, refHlPix * 2);
        }

        ctx.fillStyle = refModel.color || '#2ecc71';
        ctx.font = 'bold 9px monospace';
        ctx.fillText(`📐 Tham chiếu: ${refModel.name} (${refModel.w}x${refModel.l}m)`, 8, 14);
        ctx.restore();
      }

      // 3. Khung bao thực tế của mục tiêu (Bounding Box - Tự động ôm sát trọng tâm cụm điểm)
      const hasPoints = (fused.ptsL1?.length || 0) + (fused.ptsL2?.length || 0) > 0;
      const boxW = (hasPoints && fused.measuredW) ? Math.max(w, fused.measuredW) : w;
      const boxD = (hasPoints && fused.measuredH) ? Math.max(d, fused.measuredH) : d;
      const cX0 = (hasPoints && Number.isFinite(fused.centroidX)) ? fused.centroidX : t.x;
      const cY0 = (hasPoints && Number.isFinite(fused.centroidY)) ? fused.centroidY : t.y;

      const isLive = !!t.active;
      ctx.strokeStyle = isLive ? '#ffcc00' : '#f39c12';
      ctx.lineWidth = 2.0;
      const hwPix = (boxW / 2) * scale;
      const hdPix = (boxD / 2) * scale;
      ctx.strokeRect(cx - hwPix, cy - hdPix, hwPix * 2, hdPix * 2);

      // Kích thước cạnh hiển thị trực tiếp
      ctx.fillStyle = '#ffea00';
      ctx.font = 'bold 10px monospace';
      ctx.fillText(`↔ Rộng: ${boxW.toFixed(2)}m`, cx - hwPix + 4, cy - hdPix - 4);
      ctx.fillText(`↕ Dài: ${boxD.toFixed(2)}m`, cx + hwPix + 4, cy);

      // 4. Các điểm laser L1 (Cyan) & L2 (Orange) - Khóa tâm tuyệt đối theo trọng tâm cụm điểm
      ctx.fillStyle = '#00e5ff';
      for (const p of (fused.ptsL1 || [])) {
        const px = cx + (p.x - cX0) * scale;
        const py = cy + (p.y - cY0) * scale;
        ctx.beginPath();
        ctx.arc(px, py, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.fillStyle = '#ff9900';
      for (const p of (fused.ptsL2 || [])) {
        const px = cx + (p.x - cX0) * scale;
        const py = cy + (p.y - cY0) * scale;
        ctx.beginPath();
        ctx.arc(px, py, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }

      // Thông báo khi mục tiêu mất dấu tia (Dự đoán quán tính)
      if (!hasPoints) {
        ctx.fillStyle = '#f39c12';
        ctx.font = 'italic 11px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('⚠ Mục tiêu khuất tia (Dự đoán quán tính)', cx, cy + hdPix + 16);
        ctx.textAlign = 'left';
      }

      // 5. Véc-tơ vận tốc
      if (speed > 0.05) {
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 2.0;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        const vxPix = (t.vx || 0) * scale * 0.5;
        const vyPix = (t.vy || 0) * scale * 0.5;
        ctx.lineTo(cx + vxPix, cy + vyPix);
        ctx.stroke();
      }

      // Điểm tâm
      ctx.fillStyle = isLive ? '#ff0055' : '#888888';
      ctx.beginPath();
      ctx.arc(cx, cy, 3.5, 0, Math.PI * 2);
      ctx.fill();

      // 6. Cập nhật Metrics Panel
      const metricsEl = modal.querySelector('#camEnlargeMetrics');
      if (metricsEl) {
        const curSpd = isLive ? speed : 0;
        const maxSpd = t.maxSpeed || speed;
        const spdHtml = isLive 
          ? `⚡ Tốc độ tức thời: <b>${curSpd.toFixed(2)} m/s</b> (${(curSpd * 3.6).toFixed(1)} km/h) <span style="color:#00ffcc;font-size:9px;">[Cực đại: ${(maxSpd * 3.6).toFixed(1)} km/h]</span>`
          : `⚡ Tốc độ: <b>${maxSpd.toFixed(2)} m/s</b> (${(maxSpd * 3.6).toFixed(1)} km/h) <span style="color:#00ffcc;font-size:9px;">(Cực đại lúc di chuyển)</span>`;
        const critPosHtml = (t.criticalX !== undefined && t.criticalX !== 0) ? ` <span style="color:#ffcc00;font-size:9px;" title="Vị trí tiến sâu vào lòng ray nhất">[Sâu nhất trong ray: (${t.criticalX.toFixed(2)}, ${t.criticalY.toFixed(2)})m]</span>` : '';

        metricsEl.innerHTML = `
          <div>📐 Kích thước đáy: <b>${boxW.toFixed(2)} × ${boxD.toFixed(2)} m</b> (Bao hợp nhất)</div>
          <div>↕ Chiều cao 3D: <b>${height.toFixed(2)} m</b> (Z: ${z1.toFixed(1)}→${z2.toFixed(1)}m)</div>
          <div>📏 Diện tích đáy: <b>${(boxW * boxD).toFixed(2)} m²</b></div>
          <div>${spdHtml}</div>
          <div>📍 Vị trí ${isLive ? 'hiện tại' : 'lúc rời'}: <b>X: ${t.x.toFixed(2)}m, Y: ${t.y.toFixed(2)}m</b>${critPosHtml}</div>
          <div>🏷 Phân loại: <b>${category}</b></div>
          <div style="color:#00e5ff;font-weight:bold;">🔵 LiDAR 1: ${(fused.ptsL1 || []).length} pts (Mặt trước)</div>
          <div style="color:#ff9900;font-weight:bold;">🟠 LiDAR 2: ${(fused.ptsL2 || []).length} pts (Mặt sau)</div>
        `;
      }
    }

    /**
     * Lấy model mẫu tham chiếu đang chọn (Kích thước W x L x H)
     */
    getActiveReferenceModel() {
      if (!this.selectedRefId || this.selectedRefId === 'none') return null;

      // 1. Mẫu tích hợp sẵn
      if (this.selectedRefId === 'default_person') {
        return { name: 'Người chuẩn', w: 0.50, l: 0.35, h: 1.70, color: '#2ecc71' };
      }
      if (this.selectedRefId === 'default_moto') {
        return { name: 'Xe máy / Xe điện', w: 0.75, l: 1.90, h: 1.15, color: '#e67e22' };
      }
      if (this.selectedRefId === 'default_car') {
        return { name: 'Ô tô / Xe tải', w: 1.80, l: 3.80, h: 1.60, color: '#3498db' };
      }
      if (this.selectedRefId === 'default_forklift') {
        return { name: 'Xe nâng / AGV', w: 1.20, l: 2.40, h: 2.10, color: '#9b59b6' };
      }
      if (this.selectedRefId === 'default_box') {
        return { name: 'Kiện carton', w: 0.80, l: 0.80, h: 0.80, color: '#f1c40f' };
      }
      if (this.selectedRefId === 'default_pallet') {
        return { name: 'Pallet hàng hóa', w: 1.00, l: 1.20, h: 1.40, color: '#1abc9c' };
      }
      if (this.selectedRefId === 'default_animal') {
        return { name: 'Thú cưng / Động vật nhỏ', w: 0.30, l: 0.60, h: 0.40, color: '#e74c3c' };
      }

      // 2. Mẫu từ GroundReferenceManager
      const mgr = window.groundRefMgr;
      if (mgr && mgr.references) {
        const found = mgr.references.find(r => r.id === this.selectedRefId);
        if (found) {
          return {
            name: found.name || found.id,
            w: found.obb?.width || found.bbox?.width || 0.5,
            l: found.obb?.length || found.bbox?.length || 0.4,
            h: found.height || 1.5,
            color: found.color || '#e67e22',
            contour: found.contour,
            centroid: found.centroid
          };
        }
      }
      return { name: 'Mặc định', w: 0.50, l: 0.35, h: 1.70, color: '#2ecc71' };
    }

    /**
     * Chụp trực tiếp đối tượng từ Camera AI lưu vào Tham Chiếu
     */
    captureTargetAsReference(targetData) {
      if (!targetData && this.selectedTargetId && this.selectedTargetId !== 'all') {
        targetData = this.targetHistory.get(this.selectedTargetId);
      }
      if (!targetData) {
        alert('Vui lòng chọn 1 mục tiêu trong danh sách trước khi chụp mẫu!');
        return;
      }
      const mgr = window.groundRefMgr;
      if (!mgr) {
        alert('Module Quản lý tham chiếu chưa sẵn sàng!');
        return;
      }

      const defaultName = `Mẫu #${targetData.id} [CamAI]`;
      const win = (this.isPopout && this.popoutWin && !this.popoutWin.closed) ? this.popoutWin : window;
      let name = null;
      try {
        name = win.prompt('Nhập tên mẫu tham chiếu:', defaultName);
      } catch (err) {
        name = window.prompt('Nhập tên mẫu tham chiếu:', defaultName);
      }
      if (name === null) return; // Người dùng ấn Hủy

      const ref = mgr.captureFromTargetData(targetData, name.trim() || defaultName);
      if (ref) {
        this.selectedRefId = ref.id;
        this._updateRefDropdown();
        this._renderShapeInspector();
        if (typeof addLog === 'function') {
          addLog(`[CAMERA-REF] 📸 Đã chụp mẫu đối tượng #${targetData.id} thành Tham Chiếu: ${ref.name} (${ref.obb.width.toFixed(2)}x${ref.obb.length.toFixed(2)}m)!`);
        }
        if (typeof beep === 'function') beep();
        try {
          win.alert(`✅ Đã chụp mẫu tham chiếu thành công!\n- Tên: ${ref.name}\n- Kích thước: ${ref.obb.width.toFixed(2)} × ${ref.obb.length.toFixed(2)} m\n- Chiều cao: ${ref.height.toFixed(2)} m`);
        } catch(e) {
          alert(`✅ Đã chụp mẫu tham chiếu thành công!\n- Tên: ${ref.name}`);
        }
      }
    }

    /**
     * Cập nhật danh sách mẫu tham chiếu trong dropdown camRefSel
     */
    _updateRefDropdown() {
      const sel = this.panelEl?.querySelector('#camRefSel');
      if (!sel) return;

      let html = `
        <option value="default_person" ${this.selectedRefId === 'default_person' ? 'selected' : ''}>👤 Người (1.7×0.5×0.3m)</option>
        <option value="default_moto" ${this.selectedRefId === 'default_moto' ? 'selected' : ''}>🏍️ Xe máy (1.2×1.9×0.8m)</option>
        <option value="default_car" ${this.selectedRefId === 'default_car' ? 'selected' : ''}>🚗 Xe ô tô (1.6×3.8×1.8m)</option>
        <option value="default_forklift" ${this.selectedRefId === 'default_forklift' ? 'selected' : ''}>🚜 Xe nâng/AGV (2.1×2.4×1.2m)</option>
        <option value="default_box" ${this.selectedRefId === 'default_box' ? 'selected' : ''}>📦 Kiện carton (0.8×0.8×0.8m)</option>
        <option value="default_pallet" ${this.selectedRefId === 'default_pallet' ? 'selected' : ''}>🧱 Pallet hàng (1.4×1.2×1.0m)</option>
        <option value="default_animal" ${this.selectedRefId === 'default_animal' ? 'selected' : ''}>🐕 Thú cưng (0.4×0.6×0.3m - Lọc ảo)</option>
      `;

      const mgr = window.groundRefMgr;
      if (mgr && mgr.references && mgr.references.length) {
        html += `<optgroup label="📸 Mẫu đã chụp (${mgr.references.length})">`;
        for (const r of mgr.references) {
          const selMark = (this.selectedRefId === r.id) ? 'selected' : '';
          const dims = `${r.obb?.width || 0.4}x${r.obb?.length || 0.3}x${r.height || 1.5}m`;
          html += `<option value="${r.id}" ${selMark}>⭐ ${r.id} (${dims})</option>`;
        }
        html += `</optgroup>`;
      }

      html += `<option value="none" ${this.selectedRefId === 'none' ? 'selected' : ''}>🚫 Không lồng</option>`;
      sel.innerHTML = html;
    }

    _updateTargetDropdown() {
      const sel = this.panelEl?.querySelector('#camTargetSel');
      if (!sel) return;

      const currentVal = this.selectedTargetId;
      const historyItems = Array.from(this.targetHistory.values()).sort((a, b) => {
        if (a.active !== b.active) return a.active ? -1 : 1;
        return b.lastSeen - a.lastSeen;
      });

      let optionsHtml = '<option value="all">👁️ Tất cả mục tiêu (Multi)</option>';
      for (const item of historyItems) {
        const isLive = !!item.active;
        const prefix = isLive ? '🎯' : '⏱ [Đã rời]';
        const selMark = (this.selectedTargetId === item.id) ? 'selected' : '';
        optionsHtml += `<option value="${item.id}" ${selMark}>${prefix} Mục tiêu #${item.id} (${(item.w || 0.4).toFixed(2)}x${(item.h || 0.3).toFixed(2)}m • ${(item.maxSpeed || 0).toFixed(1)}m/s)</option>`;
      }

      if (sel.options.length !== historyItems.length + 1) {
        sel.innerHTML = optionsHtml;
        sel.value = String(currentVal);
      }
    }

    _updateDrawerInputs() {
      const panel = this.panelEl;
      if (!panel) return;
      const elX = panel.querySelector('#camInpX'); if (elX) elX.value = this.extrinsics.x.toFixed(2);
      const elY = panel.querySelector('#camInpY'); if (elY) elY.value = this.extrinsics.y.toFixed(2);
      const elZ = panel.querySelector('#camInpZ'); if (elZ) elZ.value = this.extrinsics.z.toFixed(2);
      const elYaw = panel.querySelector('#camInpYaw'); if (elYaw) elYaw.value = this.extrinsics.yaw.toFixed(1);
      const elPitch = panel.querySelector('#camInpPitch'); if (elPitch) elPitch.value = this.extrinsics.pitch.toFixed(1);
      const elFov = panel.querySelector('#camInpFov'); if (elFov) elFov.value = this.intrinsics.fov.toFixed(1);
    }

    applyViewAngle(mode) {
      this.viewAngleMode = mode;
      const t = (this.selectedTargetId !== 'all') ? this.targetHistory.get(this.selectedTargetId) : null;

      if (mode === 'corner') {
        this.extrinsics.x = -1.6;
        this.extrinsics.y = -0.8;
        this.extrinsics.z = 2.7;
        if (t) {
          this.pointCameraAt(t.x, t.y, 0.8);
        } else {
          this.extrinsics.yaw = 72;
          this.extrinsics.pitch = -30;
        }
        this.intrinsics.fov = 74;
      } else if (mode === 'lidar1') {
        let lx = 0, ly = 0;
        if (window.extrinsics && window.extrinsics[0]) {
          lx = +(window.extrinsics[0].x ?? window.extrinsics[0].dx ?? 0);
          ly = +(window.extrinsics[0].y ?? window.extrinsics[0].dy ?? 0);
        }
        this.extrinsics.x = lx;
        this.extrinsics.y = ly;
        this.extrinsics.z = 1.4;
        if (t) {
          this.pointCameraAt(t.x, t.y, 0.8);
        } else {
          this.extrinsics.yaw = 90;
          this.extrinsics.pitch = -18;
        }
        this.intrinsics.fov = 70;
      } else if (mode === 'lidar2') {
        let lx = 0, ly = 4.2;
        if (window.extrinsics && window.extrinsics[1]) {
          lx = +(window.extrinsics[1].x ?? window.extrinsics[1].dx ?? 0);
          ly = +(window.extrinsics[1].y ?? window.extrinsics[1].dy ?? 4.2);
        }
        this.extrinsics.x = lx;
        this.extrinsics.y = ly;
        this.extrinsics.z = 1.4;
        if (t) {
          this.pointCameraAt(t.x, t.y, 0.8);
        } else {
          this.extrinsics.yaw = -90;
          this.extrinsics.pitch = -18;
        }
        this.intrinsics.fov = 70;
      } else if (mode === 'top') {
        if (t) {
          this.extrinsics.x = t.x;
          this.extrinsics.y = t.y;
        } else {
          this.extrinsics.x = 0;
          this.extrinsics.y = 2.0;
        }
        this.extrinsics.z = 4.5;
        this.extrinsics.yaw = 90;
        this.extrinsics.pitch = -88;
        this.intrinsics.fov = 85;
      } else if (mode === 'orbit') {
        if (!t && this.targetHistory.size) {
          const first = this.targetHistory.values().next().value;
          if (first) this.selectTarget(first.id);
        }
        this.intrinsics.fov = 68;
      }

      this._updateDrawerInputs();
    }

    pointCameraAt(tx, ty, tz = 0.8) {
      const dx = tx - this.extrinsics.x;
      const dy = ty - this.extrinsics.y;
      const dz = tz - this.extrinsics.z;
      const dist2D = Math.hypot(dx, dy);

      if (dist2D > 0.05) {
        this.extrinsics.yaw = Math.atan2(dy, dx) * RAD2DEG;
        this.extrinsics.pitch = Math.atan2(dz, dist2D) * RAD2DEG;
      }
    }

    async switchSource(src) {
      this.config.source = src;
      const btn = this.panelEl.querySelector('#btnCamSrc');
      if (src === 'webcam') {
        try {
          const stream = await navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 360 } });
          this.webcamStream = stream;
          this.videoEl.srcObject = stream;
          this.videoEl.play();
          btn.textContent = '📹 Webcam';
          btn.classList.add('active');
          if (typeof addLog === 'function') addLog('[CAMERA] 📹 Đã kết nối Webcam thật làm nền.');
        } catch (err) {
          console.warn('[CameraFusion] Không mở được webcam:', err);
          this.config.source = 'synthetic';
          btn.textContent = '🔄 Nguồn (Ảo)';
          btn.classList.remove('active');
          if (typeof addLog === 'function') addLog('[CAMERA] Không tìm thấy Webcam. Quay lại chế độ Camera ảo 3D.');
        }
      } else {
        if (this.webcamStream) {
          this.webcamStream.getTracks().forEach(t => t.stop());
          this.webcamStream = null;
        }
        btn.textContent = '🔄 Nguồn (Ảo)';
        btn.classList.remove('active');
      }
    }

    toggle(show = null) {
      this.initUI();
      const isShow = show !== null ? show : !this.config.enabled;
      this.config.enabled = isShow;
      this.panelEl.classList.toggle('show', isShow);

      const navBtn = document.getElementById('btnCameraFusion');
      if (navBtn) navBtn.classList.toggle('active', isShow);

      if (isShow) {
        this._forceTargetUiUpdate = true;
        this._updateTargetList(window._ev?.targets || []);
        this._renderShapeInspector();
        this.startLoop();
      } else {
        this.stopLoop();
      }
    }

    startLoop() {
      if (this._isLoopRunning) return;
      this._isLoopRunning = true;
      let lastTime = performance.now();
      let framesCount = 0;
      let fpsTimer = performance.now();

      const loop = () => {
        if (!this.config.enabled || !this._isLoopRunning) {
          this._isLoopRunning = false;
          return;
        }
        const now = performance.now();
        const dt = Math.min(0.1, (now - lastTime) / 1000);
        lastTime = now;

        try {
          this.updateCameraDynamics(dt);
          this.render();
        } catch (err) {
          console.error('[CameraFusion] Render error:', err);
        }

        framesCount++;
        if (now - fpsTimer >= 1000) {
          const fpsEl = this.panelEl?.querySelector('#camFps');
          if (fpsEl) fpsEl.textContent = `${framesCount} FPS`;
          framesCount = 0;
          fpsTimer = now;
        }

        this._scheduleNextLoop(loop);
      };

      this._scheduleNextLoop(loop);
    }

    _scheduleNextLoop(loop) {
      if (!this.config.enabled || !this._isLoopRunning) return;

      const targetWin = (this.isPopout && this.popoutWin && !this.popoutWin.closed)
        ? this.popoutWin
        : window;

      let called = false;
      const onFrame = () => {
        if (called) return;
        called = true;
        if (this._loopFallbackTimer) {
          clearTimeout(this._loopFallbackTimer);
          this._loopFallbackTimer = null;
        }
        loop();
      };

      try {
        this.animationFrameId = targetWin.requestAnimationFrame(onFrame);
      } catch (err) {
        try {
          this.animationFrameId = window.requestAnimationFrame(onFrame);
        } catch (e2) {}
      }

      // Heartbeat watchdog fallback: Nếu requestAnimationFrame bị browser đình chỉ
      // (khi tab nền, màn hình thứ 2, cửa sổ không focus), fallback timer sẽ giữ cho
      // khung hình và tương tác tiếp tục hoạt động liên tục (35-40 FPS) mà không bao giờ bị đứng hình!
      this._loopFallbackTimer = setTimeout(() => {
        if (!called && this.config.enabled && this._isLoopRunning) {
          called = true;
          loop();
        }
      }, 25);
    }

    stopLoop() {
      this._isLoopRunning = false;
      if (this.animationFrameId) {
        try {
          if (this.popoutWin && !this.popoutWin.closed) this.popoutWin.cancelAnimationFrame(this.animationFrameId);
        } catch (e) {}
        try {
          window.cancelAnimationFrame(this.animationFrameId);
        } catch (e) {}
        this.animationFrameId = null;
      }
      if (this._loopFallbackTimer) {
        clearTimeout(this._loopFallbackTimer);
        this._loopFallbackTimer = null;
      }
    }

    /**
     * BỘ THU THẬP & CẬP NHẬT LỊCH SỬ ĐỐI TƯỢNG (CHẠY NGẦM LIÊN TỤC 24/7)
     * Kể cả khi chưa bật cửa sổ Camera AI, hệ thống vẫn liên tục ghi nhận đầy đủ:
     * - Kích thước WxH, chiều cao 3D, vận tốc max, số tia laser
     * - Quỹ đạo lộ trình di chuyển (trail)
     * - Snapshot mây điểm LiDAR hợp nhất (L1 cyan + L2 orange)
     * - Thời điểm xuất hiện (timeStr) và thời điểm rời đi (leftTimeStr)
     */
    updateTargetHistory(activeTargets, curFrames = null) {
      if (!Array.isArray(activeTargets)) activeTargets = [];
      const now = Date.now();

      if (!curFrames && typeof getFrame === 'function') {
        curFrames = [];
        const count = window.lidarCount || 2;
        for (let i = 0; i < count; i++) {
          const f = getFrame(i);
          if (f) curFrames.push(f);
        }
      }
      if (!curFrames) curFrames = [];

      const activeIds = new Set();
      for (const t of activeTargets) {
        if (t.age < 2) continue;
        activeIds.add(t.id);
        const speed = Math.hypot(t.vx || 0, t.vy || 0);

        let item = this.targetHistory.get(t.id);
        const curFused = this._extractTargetFusedPoints(t, curFrames);
        const curPtCount = (curFused.ptsL1.length + curFused.ptsL2.length);

        if (!item) {
          const initW = t.w || 0.40;
          const initH = t.h || 0.35;
          const initZ1 = Number.isFinite(t.z1) ? t.z1 : 0.0;
          let initZ2 = Number.isFinite(t.z2) ? t.z2 : (initZ1 + 1.65);
          if (initZ2 - initZ1 < 0.25) {
            initZ2 = initZ1 + ((initW > 0.8 || initH > 0.8) ? 1.50 : 1.70);
          }

          item = {
            id: t.id,
            firstSeen: now,
            lastSeen: now,
            active: true,
            leftTimeStr: null,
            x: t.x,
            y: t.y,
            criticalX: t.criticalX || t.x,
            criticalY: t.criticalY || t.y,
            exitX: t.exitX || t.x,
            exitY: t.exitY || t.y,
            z1: initZ1,
            z2: initZ2,
            w: initW,
            h: initH,
            maxW: t.maxW || initW,
            maxH: t.maxH || initH,
            vx: t.vx || 0,
            vy: t.vy || 0,
            maxSpeed: Math.max(speed, t.maxSpeed || 0),
            maxPoints: Math.max(t.n || 0, curPtCount),
            timeStr: new Date(now).toLocaleTimeString('vi-VN', { hour12: false }),
            fused: curFused,
            trail: [{ x: t.x, y: t.y, z: initZ1, t: now, speed }]
          };
          this.targetHistory.set(t.id, item);
        } else {
          item.lastSeen = now;
          item.active = true;
          item.x = t.x;
          item.y = t.y;
          if (t.criticalX !== undefined) {
            item.criticalX = t.criticalX;
            item.criticalY = t.criticalY;
          }
          item.exitX = t.x;
          item.exitY = t.y;

          // Ghi nhớ quỹ đạo lộ trình di chuyển (khoảng cách tối thiểu 4cm giữa 2 điểm liên tiếp)
          if (!item.trail) item.trail = [];
          const lastPt = item.trail[item.trail.length - 1];
          const distFromLast = lastPt ? Math.hypot(t.x - lastPt.x, t.y - lastPt.y) : 999;
          if (distFromLast >= 0.04) {
            item.trail.push({ x: t.x, y: t.y, z: Number.isFinite(t.z1) ? t.z1 : 0, t: now, speed });
            if (item.trail.length > 300) item.trail.shift(); // Lưu tối đa 300 điểm lộ trình
          }

          // Giữ kích thước lớn nhất & đầy đủ nhất ghi nhận được trong suốt hành trình
          if (t.w && t.w > (item.maxW || 0)) item.maxW = t.w;
          if (t.h && t.h > (item.maxH || 0)) item.maxH = t.h;
          item.w = item.maxW || t.w || 0.40;
          item.h = item.maxH || t.h || 0.35;

          if (Number.isFinite(t.z1)) item.z1 = t.z1;
          if (Number.isFinite(t.z2)) item.z2 = t.z2;
          if (!Number.isFinite(item.z2) || (item.z2 - item.z1 < 0.25)) {
            const defHeight = (item.w > 0.8 || item.h > 0.8) ? 1.50 : 1.70;
            item.z2 = (item.z1 || 0) + defHeight;
          }

          item.vx = t.vx || 0;
          item.vy = t.vy || 0;
          if (speed > item.maxSpeed) item.maxSpeed = speed;
          if ((t.n || 0) > item.maxPoints) item.maxPoints = t.n;

          // Đánh giá chất lượng Snapshot (Ưu tiên cụm gọn gàng đúng kích thước người, không dính vệt dài)
          const calcSnapshotScore = (fusedObj, targetObj) => {
            if (!fusedObj || !fusedObj.hasPoints) return -100;
            const ptCount = (fusedObj.ptsL1?.length || 0) + (fusedObj.ptsL2?.length || 0);
            if (ptCount === 0) return -100;

            const w = fusedObj.measuredW || targetObj.w || 0.40;
            const h = fusedObj.measuredH || targetObj.h || 0.35;
            const span = Math.hypot(w, h);

            let score = Math.min(25, ptCount) * 4;
            if (span >= 0.20 && span <= 0.65) {
              score += 60;
            } else if (span > 0.85) {
              score -= (span - 0.85) * 120;
            }
            return score;
          };

          const curScore = calcSnapshotScore(curFused, t);
          const prevScore = calcSnapshotScore(item.fused, item);

          // Cập nhật snapshot nếu snapshot mới có điểm chất lượng cao hơn
          if (!item.fused || curScore > prevScore) {
            item.fused = curFused;
            item.w = curFused.measuredW || t.w || 0.35;
            item.h = curFused.measuredH || t.h || 0.35;
          }
        }
      }

      // Đánh dấu đối tượng đã rời khỏi vùng
      for (const [id, item] of this.targetHistory) {
        if (!activeIds.has(id)) {
          if (item.active) {
            item.active = false;
            item.leftTimeStr = new Date(now).toLocaleTimeString('vi-VN', { hour12: false });
          }
        }
        // Lưu lịch sử tối đa 60 đối tượng trong 30 phút
        if (now - item.lastSeen > 30 * 60 * 1000 && this.targetHistory.size > 50) {
          this.targetHistory.delete(id);
        }
      }

      // Lưu lại SessionStorage định kỳ (mỗi 3s một lần)
      if (!this._lastSessionSave || now - this._lastSessionSave > 3000) {
        this._lastSessionSave = now;
        this.saveTargetHistoryToSession();
      }

      // Cập nhật giao diện nếu Camera AI đang mở
      if (this.config.enabled) {
        const nowTs = Date.now();
        if (!this._lastTargetListUpdate || nowTs - this._lastTargetListUpdate > 250 || this._forceTargetUiUpdate) {
          this._lastTargetListUpdate = nowTs;
          this._forceTargetUiUpdate = false;
          this._updateTargetList(activeTargets);
        }
        this._renderShapeInspector();
      }
    }

    /**
     * Nạp danh sách đối tượng lịch sử từ Backend gửi về (dùng khi mới mở trang web hoặc kết nối lại)
     */
    importRecentTargets(recentTargets) {
      if (!Array.isArray(recentTargets) || !recentTargets.length) return;
      const now = Date.now();
      let changed = false;
      for (const r of recentTargets) {
        if (!r || r.id === undefined) continue;
        let item = this.targetHistory.get(r.id);
        const firstSeen = r.firstSeenMs || now;
        const lastSeen = r.lastSeenMs || now;
        const spd = r.maxSpeed || Math.hypot(r.vx || 0, r.vy || 0);

        if (!item) {
          const z1 = Number.isFinite(r.z1) ? r.z1 : 0.0;
          const z2 = Number.isFinite(r.z2) ? r.z2 : (z1 + 1.65);
          item = {
            id: r.id,
            firstSeen: firstSeen,
            lastSeen: lastSeen,
            active: !!r.active,
            leftTimeStr: r.active ? null : new Date(lastSeen).toLocaleTimeString('vi-VN', { hour12: false }),
            timeStr: new Date(firstSeen).toLocaleTimeString('vi-VN', { hour12: false }),
            x: r.x || 0,
            y: r.y || 0,
            criticalX: r.criticalX,
            criticalY: r.criticalY,
            exitX: r.exitX || r.x,
            exitY: r.exitY || r.y,
            z1: z1,
            z2: z2,
            w: r.w || 0.40,
            h: r.h || 0.35,
            maxW: r.maxW || r.w || 0.40,
            maxH: r.maxH || r.h || 0.35,
            vx: r.vx || 0,
            vy: r.vy || 0,
            maxSpeed: spd,
            maxPoints: r.pointCount || 0,
            fused: { ptsL1: [], ptsL2: [], hasPoints: false, measuredW: r.w || 0.40, measuredH: r.h || 0.35 },
            trail: [{ x: r.x || 0, y: r.y || 0, z: z1, t: lastSeen, speed: spd }]
          };
          this.targetHistory.set(r.id, item);
          changed = true;
        } else {
          if (r.active !== undefined) item.active = !!r.active;
          if (spd > (item.maxSpeed || 0)) item.maxSpeed = spd;
          if ((r.pointCount || 0) > (item.maxPoints || 0)) item.maxPoints = r.pointCount;
          if (r.criticalX !== undefined) {
            item.criticalX = r.criticalX;
            item.criticalY = r.criticalY;
          }
          if (r.maxW && r.maxW > (item.maxW || 0)) item.maxW = r.maxW;
          if (r.maxH && r.maxH > (item.maxH || 0)) item.maxH = r.maxH;
        }
      }

      if (changed) {
        this._forceTargetUiUpdate = true;
        this.saveTargetHistoryToSession();
        if (this.config.enabled) {
          this._updateTargetList(window._ev?.targets || []);
          this._renderShapeInspector();
        }
      }
    }

    updateCameraDynamics(dt) {
      const ev = window._ev;
      const activeTargets = ev?.targets || [];

      // Lấy danh sách frame thực tế
      const curFrames = [];
      const count = window.lidarCount || 2;
      if (typeof getFrame === 'function') {
        for (let i = 0; i < count; i++) {
          const f = getFrame(i);
          if (f) curFrames.push(f);
        }
      }

      // Cập nhật bộ lưu lịch sử đối tượng
      this.updateTargetHistory(activeTargets, curFrames);

      let focusedTarget = null;
      if (this.selectedTargetId !== 'all') {
        focusedTarget = this.targetHistory.get(this.selectedTargetId);
      }

      // 1. Quỹ đạo 360° xoay quanh đối tượng (kể cả đối tượng đang có mặt hoặc vị trí lúc rời)
      if (this.viewAngleMode === 'orbit' && focusedTarget) {
        // Chỉ auto-rotate khi người dùng KHÔNG đang kéo chuột
        if (!this._isDraggingView) {
          this.orbitAngle += this.orbitSpeed * dt;
        }
        while (this.orbitAngle > Math.PI * 2) this.orbitAngle -= Math.PI * 2;

        this.extrinsics.x = focusedTarget.x + Math.cos(this.orbitAngle) * this.orbitDistance;
        this.extrinsics.y = focusedTarget.y + Math.sin(this.orbitAngle) * this.orbitDistance;
        this.extrinsics.z = ((focusedTarget.z1 + focusedTarget.z2) / 2 || 0.8) + this.orbitHeight;

        this.pointCameraAt(focusedTarget.x, focusedTarget.y, 0.8);
        this._updateDrawerInputs();
        return;
      }

      // 1b. Chế độ custom khi đã chọn 1 đối tượng (người dùng kéo chuột tự do)
      // Camera vẫn orbit quanh đối tượng nhưng KHÔNG auto-rotate, cho phép kéo tự do yaw/pitch
      if (this.viewAngleMode === 'custom' && focusedTarget) {
        this.extrinsics.x = focusedTarget.x + Math.cos(this.orbitAngle) * this.orbitDistance;
        this.extrinsics.y = focusedTarget.y + Math.sin(this.orbitAngle) * this.orbitDistance;
        this.extrinsics.z = ((focusedTarget.z1 + focusedTarget.z2) / 2 || 0.8) + this.orbitHeight;

        // Khi không đang kéo, hướng camera vào đối tượng; khi đang kéo, giữ yaw/pitch tự do
        if (!this._isDraggingView) {
          this.pointCameraAt(focusedTarget.x, focusedTarget.y, 0.8);
        }
        this._updateDrawerInputs();
        return;
      }

      // 2. Chế độ bám đuổi PTZ tự động
      if (this.config.autoTrackPTZ && !this.isInteracting) {
        let t = focusedTarget?.active ? focusedTarget : null;
        if (!t && activeTargets.length) {
          t = activeTargets.reduce((best, cur) => {
            const spd = Math.hypot(cur.vx || 0, cur.vy || 0);
            return spd > (best?.speed || 0) ? { t: cur, speed: spd } : best;
          }, null)?.t || activeTargets[0];
        }

        if (t) {
          const dx = t.x - this.extrinsics.x;
          const dy = t.y - this.extrinsics.y;
          const dz = ((t.z1 + t.z2) / 2 || 0.8) - this.extrinsics.z;
          const dist2D = Math.hypot(dx, dy);

          if (dist2D > 0.2) {
            let targetYaw = Math.atan2(dy, dx) * RAD2DEG;
            let targetPitch = Math.atan2(dz, dist2D) * RAD2DEG;

            const k = Math.min(1.0, this.config.ptzSpeed * 60 * dt);
            let diffYaw = targetYaw - this.extrinsics.yaw;
            while (diffYaw > 180) diffYaw -= 360;
            while (diffYaw < -180) diffYaw += 360;

            this.extrinsics.yaw += diffYaw * k;
            this.extrinsics.pitch += (targetPitch - this.extrinsics.pitch) * k;
            this._updateDrawerInputs();
          }
        }
      }
    }

    projectWorldToPixel(wx, wy, wz = 0) {
      const dx = wx - this.extrinsics.x;
      const dy = wy - this.extrinsics.y;
      const dz = wz - this.extrinsics.z;

      const yawRad = this.extrinsics.yaw * DEG2RAD;
      const pitchRad = this.extrinsics.pitch * DEG2RAD;

      const cosY = Math.cos(yawRad), sinY = Math.sin(yawRad);
      const cosP = Math.cos(pitchRad), sinP = Math.sin(pitchRad);

      const forwardX = cosP * cosY;
      const forwardY = cosP * sinY;
      const forwardZ = sinP;

      const rightX = -sinY;
      const rightY = cosY;
      const rightZ = 0;

      const upX = rightY * forwardZ - rightZ * forwardY;
      const upY = rightZ * forwardX - rightX * forwardZ;
      const upZ = rightX * forwardY - rightY * forwardX;

      const camZ = dx * forwardX + dy * forwardY + dz * forwardZ;
      const camX = dx * rightX + dy * rightY + dz * rightZ;
      const camY = -(dx * upX + dy * upY + dz * upZ);

      if (camZ <= 0.1) {
        return { visible: false, inFront: false, u: 0, v: 0, depth: camZ };
      }

      const W = this.intrinsics.width;
      const H = this.intrinsics.height;
      const f = (W / 2) / Math.tan((this.intrinsics.fov * DEG2RAD) / 2);

      const u = (W / 2) + (f * camX / camZ);
      const v = (H / 2) + (f * camY / camZ);

      const isInside = (u >= 0 && u <= W && v >= 0 && v <= H);

      return {
        visible: isInside,
        inFront: true,
        u, v,
        depth: camZ
      };
    }

    render() {
      if (!this.panelEl) return;
      if (!this.canvasEl) {
        this.canvasEl = this.panelEl.querySelector('#camCanvas');
      }
      if (!this.canvasEl) return;
      if (!this.ctx) {
        this.ctx = this.canvasEl.getContext('2d');
      }
      const ctx = this.ctx;
      if (!ctx) return;

      const wrap = this.panelEl.querySelector('.cam-view-wrap');

      // Tự động điều chỉnh kích thước Canvas theo co giãn thực tế của panel
      if (wrap) {
        const wrapW = wrap.clientWidth;
        const wrapH = wrap.clientHeight;
        if (wrapW > 50 && wrapH > 50 && (this.canvasEl.width !== wrapW || this.canvasEl.height !== wrapH)) {
          this.canvasEl.width = wrapW;
          this.canvasEl.height = wrapH;
          this.intrinsics.width = wrapW;
          this.intrinsics.height = wrapH;
        }
      }

      const W = this.canvasEl.width;
      const H = this.canvasEl.height;
      this.renderedBoxes = [];

      // 1. Vẽ nền video / synthetic
      if (this.config.source === 'webcam' && this.videoEl && this.videoEl.readyState >= 2) {
        ctx.drawImage(this.videoEl, 0, 0, W, H);
      } else {
        this._renderSyntheticEnvironment(ctx, W, H);
      }

      // 2. Chiếu Vùng cảnh báo (Zone) lên mặt sàn quan sát
      if (this.config.showZones && window.zones) {
        this._renderZonesOnGround(ctx);
      }

      // 2.b Chiếu các Đường bao / Mặt tham chiếu 2D (Ground Reference) đã lưu lên mặt sàn
      if (window.groundRefMgr && window.groundRefMgr.references) {
        this._renderGroundReferencesOnCamera(ctx);
      }

      // 3. Chiếu Mây điểm LiDAR từ TẤT CẢ các cảm biến đang hoạt động (Multi-LiDAR Fusion)
      if (this.config.showPoints) {
        this._renderMultiLidarPointCloud(ctx);
      }

      // 4. Chiếu Bounding Box & HUD bám theo các mục tiêu di chuyển
      if (this.config.showBoxes) {
        this._renderTargetBoxes(ctx);
      }

      // 5. Vẽ HUD thông tin Camera góc trên
      this._renderCameraHUD(ctx, W, H);
    }

    /**
     * Chiếu các đường bao / mặt tham chiếu 2D & 3D Prism từ GroundReferenceManager lên Camera AI
     */
    _renderGroundReferencesOnCamera(ctx) {
      const refs = window.groundRefMgr?.references || [];
      for (const ref of refs) {
        if (!ref.visible2D && !ref.visible3D) continue;
        const col = ref.color || (ref.isAuto ? '#8e44ad' : '#e67e22');
        const contour = ref.contour || [];
        if (contour.length < 3) continue;

        const z1 = Number.isFinite(ref.zMin) ? ref.zMin : 0.0;
        const z2 = Number.isFinite(ref.zMax) ? ref.zMax : (z1 + (ref.height || 1.5));

        // 1. Chiếu đáy 2D trên mặt đất (Z=z1)
        if (ref.visible2D) {
          ctx.save();
          ctx.strokeStyle = col;
          ctx.fillStyle = col + '20';
          ctx.lineWidth = 1.8;
          ctx.setLineDash([5, 3]);

          ctx.beginPath();
          let anyInFront = false;
          contour.forEach((p, idx) => {
            const pr = this.projectWorldToPixel(p[0], p[1], z1);
            if (pr.inFront) anyInFront = true;
            if (idx === 0) ctx.moveTo(pr.u, pr.v);
            else ctx.lineTo(pr.u, pr.v);
          });
          ctx.closePath();
          if (anyInFront) {
            ctx.fill();
            ctx.stroke();
          }
          ctx.restore();
        }

        // 2. Chiếu Khối hộp 3D Lập phương / Prism Extrusion (Z1 -> Z2)
        if (ref.visible3D) {
          ctx.save();
          ctx.strokeStyle = col;
          ctx.lineWidth = 1.2;
          ctx.setLineDash([3, 3]);

          const bottomPr = contour.map(p => this.projectWorldToPixel(p[0], p[1], z1));
          const topPr = contour.map(p => this.projectWorldToPixel(p[0], p[1], z2));

          if (bottomPr.some(p => p.inFront) || topPr.some(p => p.inFront)) {
            // Nắp trên
            ctx.beginPath();
            topPr.forEach((pr, idx) => {
              if (idx === 0) ctx.moveTo(pr.u, pr.v);
              else ctx.lineTo(pr.u, pr.v);
            });
            ctx.closePath();
            ctx.stroke();

            // Các cột đứng nối đáy lên đỉnh (chỉ vẽ tối đa 16 cột để đảm bảo 60 FPS)
            const step = Math.max(1, Math.floor(contour.length / 16));
            ctx.beginPath();
            for (let i = 0; i < contour.length; i += step) {
              const b = bottomPr[i];
              const t = topPr[i];
              if (b.inFront || t.inFront) {
                ctx.moveTo(b.u, b.v);
                ctx.lineTo(t.u, t.v);
              }
            }
            ctx.stroke();
          }
          ctx.restore();
        }

        // 3. Nhãn tên mẫu tham chiếu
        if (ref.centroid) {
          const pr = this.projectWorldToPixel(ref.centroid.x, ref.centroid.y, z2);
          if (pr.inFront && pr.depth < this.config.maxDist) {
            ctx.save();
            ctx.font = 'bold 9px monospace';
            const label = `📐 ${ref.id} (${ref.obb?.width || 0.4}x${ref.obb?.length || 0.3}m)`;
            const tw = ctx.measureText(label).width + 8;
            ctx.fillStyle = 'rgba(0,0,0,0.75)';
            ctx.fillRect(pr.u - tw / 2, pr.v - 16, tw, 14);
            ctx.strokeStyle = col;
            ctx.strokeRect(pr.u - tw / 2, pr.v - 16, tw, 14);
            ctx.fillStyle = '#fff';
            ctx.fillText(label, pr.u - tw / 2 + 4, pr.v - 6);
            ctx.restore();
          }
        }
      }
    }

    _renderSyntheticEnvironment(ctx, W, H) {
      const horizonY = (H / 2) - Math.tan(this.extrinsics.pitch * DEG2RAD) * (W / 2) / Math.tan((this.intrinsics.fov * DEG2RAD) / 2);
      const grad = ctx.createLinearGradient(0, 0, 0, H);
      grad.addColorStop(0, '#0e1117');
      grad.addColorStop(Math.max(0, Math.min(1, horizonY / H)), '#161b24');
      grad.addColorStop(1, '#090b0e');
      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, W, H);

      // Lưới mặt sàn
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(65, 85, 115, 0.28)';
      ctx.beginPath();
      for (let x = -10; x <= 10; x += 1.0) {
        const p1 = this.projectWorldToPixel(x, -5, 0);
        const p2 = this.projectWorldToPixel(x, 15, 0);
        if (p1.inFront || p2.inFront) {
          ctx.moveTo(p1.u, p1.v);
          ctx.lineTo(p2.u, p2.v);
        }
      }
      for (let y = -5; y <= 15; y += 1.0) {
        const p1 = this.projectWorldToPixel(-10, y, 0);
        const p2 = this.projectWorldToPixel(10, y, 0);
        if (p1.inFront || p2.inFront) {
          ctx.moveTo(p1.u, p1.v);
          ctx.lineTo(p2.u, p2.v);
        }
      }
      ctx.stroke();

      // Vị trí các LiDAR
      const lidarCount = window.lidarCount || 2;
      for (let i = 0; i < lidarCount; i++) {
        let lx = 0, ly = 0;
        if (window.extrinsics && window.extrinsics[i]) {
          lx = +(window.extrinsics[i].x ?? window.extrinsics[i].dx ?? 0);
          ly = +(window.extrinsics[i].y ?? window.extrinsics[i].dy ?? 0);
        } else if (i === 1) { ly = 4.2; }

        const lp = this.projectWorldToPixel(lx, ly, 0);
        if (lp.inFront) {
          ctx.fillStyle = i === 0 ? '#00e5ff' : '#ff9900';
          ctx.beginPath();
          ctx.arc(lp.u, lp.v, 4, 0, Math.PI * 2);
          ctx.fill();
          ctx.fillStyle = '#fff';
          ctx.font = 'bold 9px monospace';
          ctx.fillText(`LiDAR ${i + 1}`, lp.u + 6, lp.v + 3);
        }
      }
    }

    _renderZonesOnGround(ctx) {
      const zones = window.zones || [];
      for (const z of zones) {
        const isExcl = !!z.exclusion;
        const colorStroke = isExcl ? 'rgba(230, 126, 34, 0.85)' : 'rgba(231, 76, 60, 0.9)';
        const colorFill = isExcl ? 'rgba(230, 126, 34, 0.15)' : 'rgba(231, 76, 60, 0.18)';

        ctx.strokeStyle = colorStroke;
        ctx.fillStyle = colorFill;
        ctx.lineWidth = 2;

        let pts = [];
        if (z.type === 0) {
          pts = [
            { x: z.x1, y: z.y1 },
            { x: z.x2, y: z.y1 },
            { x: z.x2, y: z.y2 },
            { x: z.x1, y: z.y2 }
          ];
        } else if (z.type === 1 && z.vertices && z.vertices.length >= 3) {
          pts = z.vertices.map(v => ({ x: v[0], y: v[1] }));
        }

        if (pts.length >= 3) {
          ctx.beginPath();
          let anyInFront = false;
          pts.forEach((p, idx) => {
            const pr = this.projectWorldToPixel(p.x, p.y, 0);
            if (pr.inFront) anyInFront = true;
            if (idx === 0) ctx.moveTo(pr.u, pr.v);
            else ctx.lineTo(pr.u, pr.v);
          });
          ctx.closePath();
          if (anyInFront) {
            ctx.fill();
            ctx.stroke();
          }
        }
      }
    }

    _renderMultiLidarPointCloud(ctx) {
      if (typeof getFrame !== 'function') return;
      const count = window.lidarCount || 2;
      const decimate = 2;

      const sensorColors = [
        'rgba(0, 229, 255, 0.75)',  // LiDAR 1: Cyan (Mặt trước)
        'rgba(255, 153, 0, 0.85)',   // LiDAR 2: Orange (Mặt sau)
        'rgba(155, 89, 182, 0.85)',  // LiDAR 3: Tím
        'rgba(46, 204, 113, 0.85)'   // LiDAR 4: Xanh lá
      ];

      for (let s = 0; s < count; s++) {
        const f = getFrame(s);
        if (!f || !f.n) continue;

        const color = sensorColors[s % sensorColors.length];
        ctx.fillStyle = color;

        const hasXYZ = f.xyz && f.xyz.length >= f.n * 3;
        const n = f.n;

        for (let i = 0; i < n; i += decimate) {
          let px = 0, py = 0, pz = 0;
          if (hasXYZ) {
            px = f.xyz[i * 3];
            py = f.xyz[i * 3 + 1];
            pz = f.xyz[i * 3 + 2];
          } else if (f.pts) {
            px = f.pts[i * 3];
            py = f.pts[i * 3 + 1];
          }

          const pr = this.projectWorldToPixel(px, py, pz);
          if (pr.visible && pr.depth < this.config.maxDist) {
            ctx.fillRect(pr.u - 1, pr.v - 1, 2.2, 2.2);
          }
        }
      }
    }

    _renderTargetBoxes(ctx) {
      const ev = window._ev;
      const activeTargets = ev?.targets || [];
      const isSingleFocus = (this.selectedTargetId !== 'all');

      // 1. Vẽ các mục tiêu đang hoạt động (Active Targets)
      for (const t of activeTargets) {
        if (t.age < 2) continue;

        const isCurrentFocused = (this.selectedTargetId === t.id);
        const alpha = isSingleFocus ? (isCurrentFocused ? 1.0 : 0.35) : 1.0;

        const w = t.w || 0.45;
        const d = t.h || 0.35;
        const z1 = Number.isFinite(t.z1) ? t.z1 : 0.0;
        const z2 = Number.isFinite(t.z2) ? t.z2 : (z1 + 1.7);

        const hw = w / 2, hd = d / 2;
        const corners3D = [
          { x: t.x - hw, y: t.y - hd, z: z1 },
          { x: t.x + hw, y: t.y - hd, z: z1 },
          { x: t.x + hw, y: t.y + hd, z: z1 },
          { x: t.x - hw, y: t.y + hd, z: z1 },
          { x: t.x - hw, y: t.y - hd, z: z2 },
          { x: t.x + hw, y: t.y - hd, z: z2 },
          { x: t.x + hw, y: t.y + hd, z: z2 },
          { x: t.x - hw, y: t.y + hd, z: z2 }
        ];

        const projected = corners3D.map(c => this.projectWorldToPixel(c.x, c.y, c.z));
        const allInFront = projected.some(p => p.inFront);
        if (!allInFront) continue;

        let minU = Infinity, maxU = -Infinity, minV = Infinity, maxV = -Infinity;
        let avgDepth = 0;
        for (const p of projected) {
          if (p.u < minU) minU = p.u;
          if (p.u > maxU) maxU = p.u;
          if (p.v < minV) minV = p.v;
          if (p.v > maxV) maxV = p.v;
          avgDepth += p.depth;
        }
        avgDepth /= projected.length;

        const boxW = maxU - minU;
        const boxH = maxV - minV;
        if (boxW < 4 || boxH < 4) continue;

        this.renderedBoxes.push({
          targetId: t.id,
          minU, maxU, minV, maxV
        });

        const isAlarm = !!window.alarmOn || (ev && ev.confirmed > 0);
        let themeColor = isAlarm ? '#ff2a2a' : '#00e5ff';
        if (isCurrentFocused) themeColor = '#ffcc00';

        ctx.save();
        ctx.globalAlpha = alpha;

        // Khung 3D Wireframe thực tế của đối tượng
        if (this.config.show3DBox) {
          ctx.strokeStyle = isCurrentFocused ? 'rgba(255, 204, 0, 0.65)' : (isAlarm ? 'rgba(255, 42, 42, 0.45)' : 'rgba(0, 229, 255, 0.4)');
          ctx.lineWidth = isCurrentFocused ? 1.8 : 1.0;
          const edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
          ];
          ctx.beginPath();
          for (const [i, j] of edges) {
            ctx.moveTo(projected[i].u, projected[i].v);
            ctx.lineTo(projected[j].u, projected[j].v);
          }
          ctx.stroke();
        }

        // Khối hộp tham chiếu 3D (3D Reference Ghost Box Wireframe)
        const refModel = this.getActiveReferenceModel();
        if (this.config.showRefGhost && refModel) {
          const rw = refModel.w || 0.50;
          const rl = refModel.l || 0.35;
          const rh = refModel.h || 1.70;
          const rhw = rw / 2, rhd = rl / 2;
          const rz1 = z1;
          const rz2 = z1 + rh;

          const refCorners3D = [
            { x: t.x - rhw, y: t.y - rhd, z: rz1 },
            { x: t.x + rhw, y: t.y - rhd, z: rz1 },
            { x: t.x + rhw, y: t.y + rhd, z: rz1 },
            { x: t.x - rhw, y: t.y + rhd, z: rz1 },
            { x: t.x - rhw, y: t.y - rhd, z: rz2 },
            { x: t.x + rhw, y: t.y - rhd, z: rz2 },
            { x: t.x + rhw, y: t.y + rhd, z: rz2 },
            { x: t.x - rhw, y: t.y + rhd, z: rz2 }
          ];

          const refProjected = refCorners3D.map(c => this.projectWorldToPixel(c.x, c.y, c.z));
          if (refProjected.some(p => p.inFront)) {
            // So sánh kích thước thực tế vs Tham chiếu để chọn màu
            const isExceeded = (w > rw * 1.25 || d > rl * 1.25 || (z2 - z1) > rh * 1.25);
            ctx.strokeStyle = isExceeded ? 'rgba(255, 75, 75, 0.85)' : 'rgba(46, 204, 113, 0.85)';
            ctx.lineWidth = 1.4;
            ctx.setLineDash([5, 3]);

            const edges = [
              [0, 1], [1, 2], [2, 3], [3, 0],
              [4, 5], [5, 6], [6, 7], [7, 4],
              [0, 4], [1, 5], [2, 6], [3, 7]
            ];
            ctx.beginPath();
            for (const [i, j] of edges) {
              ctx.moveTo(refProjected[i].u, refProjected[i].v);
              ctx.lineTo(refProjected[j].u, refProjected[j].v);
            }
            ctx.stroke();
            ctx.setLineDash([]);
          }
        }

        // Viền 2D Reticle
        ctx.strokeStyle = themeColor;
        ctx.lineWidth = isCurrentFocused ? 2.5 : 2.0;
        const cornerLen = Math.min(16, Math.min(boxW, boxH) * 0.35);

        ctx.beginPath();
        ctx.moveTo(minU, minV + cornerLen); ctx.lineTo(minU, minV); ctx.lineTo(minU + cornerLen, minV);
        ctx.moveTo(maxU - cornerLen, minV); ctx.lineTo(maxU, minV); ctx.lineTo(maxU, minV + cornerLen);
        ctx.moveTo(maxU, maxV - cornerLen); ctx.lineTo(maxU, maxV); ctx.lineTo(maxU - cornerLen, maxV);
        ctx.moveTo(minU + cornerLen, maxV); ctx.lineTo(minU, maxV); ctx.lineTo(minU, maxV - cornerLen);
        ctx.stroke();

        // HUD Tag
        const speed = Math.hypot(t.vx || 0, t.vy || 0);
        const lockPrefix = isCurrentFocused ? '🔒 KHÓA' : (isAlarm ? '🚨 CẢNH BÁO' : 'THEO DÕI');
        const tagText = `[#${t.id}] ${lockPrefix} | Cự ly: ${avgDepth.toFixed(1)}m`;
        const subText = `${w.toFixed(2)}x${d.toFixed(2)}m • V:${speed.toFixed(1)}m/s • ${t.n || 0} pts`;

        ctx.font = 'bold 10px monospace';
        const tagW = Math.max(ctx.measureText(tagText).width, ctx.measureText(subText).width) + 14;
        const tagH = 28;
        const tagX = minU;
        const tagY = Math.max(tagH + 4, minV - tagH - 4);

        ctx.fillStyle = isCurrentFocused ? 'rgba(30, 25, 0, 0.90)' : (isAlarm ? 'rgba(215, 38, 38, 0.88)' : 'rgba(10, 25, 40, 0.85)');
        ctx.fillRect(tagX, tagY, tagW, tagH);
        ctx.strokeStyle = themeColor;
        ctx.strokeRect(tagX, tagY, tagW, tagH);

        ctx.fillStyle = isCurrentFocused ? '#ffea00' : '#ffffff';
        ctx.fillText(tagText, tagX + 6, tagY + 12);
        ctx.fillStyle = isCurrentFocused ? '#ffffff' : (isAlarm ? '#ffe0e0' : '#88e0ff');
        ctx.font = '9px monospace';
        ctx.fillText(subText, tagX + 6, tagY + 24);

        const groundPin = this.projectWorldToPixel(t.x, t.y, 0);
        if (groundPin.visible) {
          ctx.strokeStyle = themeColor;
          ctx.beginPath();
          ctx.arc(groundPin.u, groundPin.v, 3, 0, Math.PI * 2);
          ctx.stroke();
        }

        ctx.restore();
      }

      // 2. Nếu đang chọn một mục tiêu trong lịch sử (đã rời vùng), TÁI HIỆN TOÀN BỘ KHỐI 3D & MÂY ĐIỂM TRỰC TIẾP TRÊN MÀN HÌNH CHÍNH
      if (isSingleFocus) {
        const histItem = this.targetHistory.get(this.selectedTargetId);
        if (histItem && !histItem.active) {
          ctx.save();
          const w = histItem.w || 0.45;
          const d = histItem.h || 0.35;
          const z1 = Number.isFinite(histItem.z1) ? histItem.z1 : 0.0;
          const z2 = Number.isFinite(histItem.z2) && (histItem.z2 - z1 > 0.3) ? histItem.z2 : (z1 + 1.65);

          const hw = w / 2, hd = d / 2;
          const corners3D = [
            { x: histItem.x - hw, y: histItem.y - hd, z: z1 },
            { x: histItem.x + hw, y: histItem.y - hd, z: z1 },
            { x: histItem.x + hw, y: histItem.y + hd, z: z1 },
            { x: histItem.x - hw, y: histItem.y + hd, z: z1 },
            { x: histItem.x - hw, y: histItem.y - hd, z: z2 },
            { x: histItem.x + hw, y: histItem.y - hd, z: z2 },
            { x: histItem.x + hw, y: histItem.y + hd, z: z2 },
            { x: histItem.x - hw, y: histItem.y + hd, z: z2 }
          ];

          const projected = corners3D.map(c => this.projectWorldToPixel(c.x, c.y, c.z));
          const anyInFront = projected.some(p => p.inFront);

          if (anyInFront) {
            let minU = Infinity, maxU = -Infinity, minV = Infinity, maxV = -Infinity;
            let avgDepth = 0;
            for (const p of projected) {
              if (p.u < minU) minU = p.u;
              if (p.u > maxU) maxU = p.u;
              if (p.v < minV) minV = p.v;
              if (p.v > maxV) maxV = p.v;
              avgDepth += p.depth;
            }
            avgDepth /= projected.length;

            const boxW = maxU - minU;
            const boxH = maxV - minV;

            // Đăng ký box để click chuột
            this.renderedBoxes.push({
              targetId: histItem.id,
              minU, maxU, minV, maxV
            });

            // 1. Khung hộp 3D Wireframe của mục tiêu lịch sử
            if (this.config.show3DBox) {
              ctx.strokeStyle = 'rgba(243, 156, 18, 0.85)';
              ctx.lineWidth = 1.6;
              ctx.setLineDash([4, 2]);
              const edges = [
                [0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]
              ];
              ctx.beginPath();
              for (const [i, j] of edges) {
                ctx.moveTo(projected[i].u, projected[i].v);
                ctx.lineTo(projected[j].u, projected[j].v);
              }
              ctx.stroke();
              ctx.setLineDash([]);
            }

            // 2. Chiếu toàn bộ mây điểm 3D đã chụp lại của mục tiêu đó (LiDAR 1 Cyan + LiDAR 2 Orange)
            const fused = histItem.fused;
            if (fused && this.config.showPoints) {
              // Laser 1 (Cyan - Mặt trước)
              ctx.fillStyle = '#00e5ff';
              for (const p of (fused.ptsL1 || [])) {
                const pr = this.projectWorldToPixel(p.x, p.y, p.z || (z1 + (z2 - z1) * 0.5));
                if (pr.visible && pr.depth < this.config.maxDist) {
                  ctx.fillRect(pr.u - 1.5, pr.v - 1.5, 3.2, 3.2);
                }
              }
              // Laser 2 (Orange - Mặt sau)
              ctx.fillStyle = '#ff9900';
              for (const p of (fused.ptsL2 || [])) {
                const pr = this.projectWorldToPixel(p.x, p.y, p.z || (z1 + (z2 - z1) * 0.5));
                if (pr.visible && pr.depth < this.config.maxDist) {
                  ctx.fillRect(pr.u - 1.5, pr.v - 1.5, 3.2, 3.2);
                }
              }
            }

            // 3. Khối hộp tham chiếu 3D (Ghost Reference Box)
            const refModel = this.getActiveReferenceModel();
            if (this.config.showRefGhost && refModel) {
              const rw = refModel.w || 0.50;
              const rl = refModel.l || 0.35;
              const rh = refModel.h || 1.70;
              const rhw = rw / 2, rhd = rl / 2;
              const rz1 = z1;
              const rz2 = z1 + rh;

              const refCorners3D = [
                { x: histItem.x - rhw, y: histItem.y - rhd, z: rz1 },
                { x: histItem.x + rhw, y: histItem.y - rhd, z: rz1 },
                { x: histItem.x + rhw, y: histItem.y + rhd, z: rz1 },
                { x: histItem.x - rhw, y: histItem.y + rhd, z: rz1 },
                { x: histItem.x - rhw, y: histItem.y - rhd, z: rz2 },
                { x: histItem.x + rhw, y: histItem.y - rhd, z: rz2 },
                { x: histItem.x + rhw, y: histItem.y + rhd, z: rz2 },
                { x: histItem.x - rhw, y: histItem.y + rhd, z: rz2 }
              ];

              const refProj = refCorners3D.map(c => this.projectWorldToPixel(c.x, c.y, c.z));
              if (refProj.some(p => p.inFront)) {
                const isExceeded = (w > rw * 1.25 || d > rl * 1.25 || (z2 - z1) > rh * 1.25);
                ctx.strokeStyle = isExceeded ? 'rgba(255, 75, 75, 0.85)' : 'rgba(46, 204, 113, 0.85)';
                ctx.lineWidth = 1.3;
                ctx.setLineDash([5, 3]);
                const edges = [
                  [0, 1], [1, 2], [2, 3], [3, 0],
                  [4, 5], [5, 6], [6, 7], [7, 4],
                  [0, 4], [1, 5], [2, 6], [3, 7]
                ];
                ctx.beginPath();
                for (const [i, j] of edges) {
                  ctx.moveTo(refProj[i].u, refProj[i].v);
                  ctx.lineTo(refProj[j].u, refProj[j].v);
                }
                ctx.stroke();
                ctx.setLineDash([]);
              }
            }

            // 4. Viền 2D Reticle góc
            ctx.strokeStyle = '#f39c12';
            ctx.lineWidth = 2.0;
            const cornerLen = Math.min(16, Math.min(boxW, boxH) * 0.35);
            ctx.beginPath();
            ctx.moveTo(minU, minV + cornerLen); ctx.lineTo(minU, minV); ctx.lineTo(minU + cornerLen, minV);
            ctx.moveTo(maxU - cornerLen, minV); ctx.lineTo(maxU, minV); ctx.lineTo(maxU, minV + cornerLen);
            ctx.moveTo(maxU, maxV - cornerLen); ctx.lineTo(maxU, maxV); ctx.lineTo(maxU - cornerLen, maxV);
            ctx.moveTo(minU + cornerLen, maxV); ctx.lineTo(minU, maxV); ctx.lineTo(minU, maxV - cornerLen);
            ctx.stroke();

            // 5. Thẻ HUD Tag
            const tagText = `[#${histItem.id}] ⏱ SNAPSHOT ĐÃ RỜI (${histItem.leftTimeStr || histItem.timeStr})`;
            const subText = `${w.toFixed(2)}x${d.toFixed(2)}m • H:${(z2 - z1).toFixed(2)}m • V_max: ${(histItem.maxSpeed || 0).toFixed(1)}m/s • ${histItem.maxPoints || 0} pts`;
            ctx.font = 'bold 10px monospace';
            const tagW = Math.max(ctx.measureText(tagText).width, ctx.measureText(subText).width) + 14;
            const tagH = 28;
            const tagX = minU;
            const tagY = Math.max(tagH + 4, minV - tagH - 4);

            ctx.fillStyle = 'rgba(35, 25, 10, 0.92)';
            ctx.fillRect(tagX, tagY, tagW, tagH);
            ctx.strokeStyle = '#f39c12';
            ctx.strokeRect(tagX, tagY, tagW, tagH);

            ctx.fillStyle = '#f39c12';
            ctx.fillText(tagText, tagX + 6, tagY + 12);
            ctx.fillStyle = '#ffffff';
            ctx.font = '9px monospace';
            ctx.fillText(subText, tagX + 6, tagY + 24);

            // 6. Chân cắm mặt sàn (Ground Pin)
            const pin = this.projectWorldToPixel(histItem.x, histItem.y, 0);
            if (pin.visible) {
              ctx.strokeStyle = '#f39c12';
              ctx.beginPath();
              ctx.arc(pin.u, pin.v, 4, 0, Math.PI * 2);
              ctx.stroke();
            }
          }
          ctx.restore();
        }
      }

      // 3. VẼ LỘ TRÌNH ĐƯỜNG ĐI 3D (TRAJECTORY PATH) CỦA MỤC TIÊU ĐANG CHỌN TRỰC TIẾP TRÊN CAMERA VIEW
      if (isSingleFocus) {
        const focusedItem = this.targetHistory.get(this.selectedTargetId);
        const trail = focusedItem?.trail;
        if (trail && trail.length >= 2) {
          ctx.save();
          ctx.lineWidth = 2.2;
          ctx.setLineDash([6, 3]);
          ctx.strokeStyle = focusedItem.active ? '#00e5ff' : '#f39c12';

          ctx.beginPath();
          let started = false;
          for (let i = 0; i < trail.length; i++) {
            const pt = trail[i];
            const pPix = this.projectWorldToPixel(pt.x, pt.y, 0.05);
            if (pPix.visible && pPix.inFront) {
              if (!started) {
                ctx.moveTo(pPix.u, pPix.v);
                started = true;
              } else {
                ctx.lineTo(pPix.u, pPix.v);
              }
            } else {
              started = false;
            }
          }
          ctx.stroke();
          ctx.setLineDash([]);

          // Vẽ điểm Bắt đầu (Start Point - Xanh lá)
          const startPt = trail[0];
          const startPix = this.projectWorldToPixel(startPt.x, startPt.y, 0.05);
          if (startPix.visible && startPix.inFront) {
            ctx.fillStyle = '#2ecc71';
            ctx.beginPath();
            ctx.arc(startPix.u, startPix.v, 4.5, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.2;
            ctx.stroke();
            ctx.fillStyle = '#2ecc71';
            ctx.font = 'bold 9px monospace';
            ctx.fillText('START', startPix.u + 6, startPix.v - 2);
          }

          // Vẽ điểm Cuối (End / Current Point - Đỏ/Cam)
          const endPt = trail[trail.length - 1];
          const endPix = this.projectWorldToPixel(endPt.x, endPt.y, 0.05);
          if (endPix.visible && endPix.inFront) {
            ctx.fillStyle = focusedItem.active ? '#ff0055' : '#f39c12';
            ctx.beginPath();
            ctx.arc(endPix.u, endPix.v, 4.5, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1.2;
            ctx.stroke();
            ctx.fillStyle = focusedItem.active ? '#ff0055' : '#f39c12';
            ctx.font = 'bold 9px monospace';
            ctx.fillText(focusedItem.active ? 'NOW' : 'END', endPix.u + 6, endPix.v - 2);
          }

          ctx.restore();
        }
      }
    }

    _renderCameraHUD(ctx, W, H) {
      ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
      ctx.fillRect(8, 8, 230, 42);
      ctx.strokeStyle = '#3a3d4f';
      ctx.strokeRect(8, 8, 230, 42);

      ctx.fillStyle = '#00ffcc';
      ctx.font = 'bold 10px monospace';
      ctx.fillText(`CAM: (${this.extrinsics.x.toFixed(1)}, ${this.extrinsics.y.toFixed(1)}, ${this.extrinsics.z.toFixed(1)})m`, 14, 21);

      ctx.fillStyle = '#bbb';
      ctx.font = '9px monospace';
      const angleName = {
        corner: 'Toàn cảnh', lidar1: 'LiDAR 1 (Trước)', lidar2: 'LiDAR 2 (Sau)',
        top: 'Trên trần', orbit: 'Quỹ đạo 360°', custom: 'Tự do (Chuột)'
      }[this.viewAngleMode] || 'Tự do';
      ctx.fillText(`GÓC: ${angleName} | YAW: ${this.extrinsics.yaw.toFixed(0)}° PITCH: ${this.extrinsics.pitch.toFixed(0)}°`, 14, 33);
      
      const targetLabel = (this.selectedTargetId === 'all') 
        ? 'Tất cả (Multi)' 
        : `Khóa #${this.selectedTargetId} ${this.targetHistory.get(this.selectedTargetId)?.active ? '' : '(Lịch sử)'}`;
      ctx.fillText(`MỤC TIÊU: ${targetLabel}`, 14, 45);

      const legendW = 150, legendH = 34;
      const legX = W - legendW - 8;
      ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
      ctx.fillRect(legX, 8, legendW, legendH);
      ctx.strokeStyle = '#3a3d4f';
      ctx.strokeRect(legX, 8, legendW, legendH);

      ctx.fillStyle = '#00e5ff';
      ctx.fillRect(legX + 8, 14, 8, 8);
      ctx.fillStyle = '#ccc';
      ctx.font = '9px monospace';
      ctx.fillText('LiDAR 1 (Mặt trước)', legX + 22, 21);

      ctx.fillStyle = '#ff9900';
      ctx.fillRect(legX + 8, 26, 8, 8);
      ctx.fillStyle = '#ccc';
      ctx.fillText('LiDAR 2 (Mặt sau)', legX + 22, 33);

      ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(W / 2 - 8, H / 2); ctx.lineTo(W / 2 + 8, H / 2);
      ctx.moveTo(W / 2, H / 2 - 8); ctx.lineTo(W / 2, H / 2 + 8);
      ctx.stroke();
    }
  }

  window.CameraLidarFusion = CameraLidarFusion;
  window.cameraFusion = new CameraLidarFusion();

})(window);
