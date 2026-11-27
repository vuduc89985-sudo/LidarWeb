/**
 * =========================================================================================
 * MODULE GHI & PHÁT LẠI SỰ KIỆN TỰ ĐỘNG (AUTO-RECORD & PLAYBACK ENGINE)
 * Chuyên dụng cho Hệ thống Giám sát & An toàn Rào chắn Đường tàu
 *
 * TÍNH NĂNG NÂNG CẤP:
 * 1. Lưu trữ CỨNG trên ổ đĩa laptop (IndexedDB): Tắt web/tắt máy không mất dữ liệu.
 * 2. Tự động kích hoạt ghi (Auto-Record on Intrusion) với Pre-buffer 50 frame.
 * 3. Hỗ trợ xem trực quan cả 2 loại dữ liệu ngay trên màn hình:
 *    - Xem Dữ Liệu THÔ (Raw Inspector): Bảng danh sách các điểm laser (X, Y, Z, Intensity) của từng LiDAR.
 *    - Xem Dữ Liệu ĐÃ XỬ LÝ (Processed Inspector): Bảng danh sách mục tiêu, hộp bao OBB, tốc độ km/h, lộ trình, cảnh báo kẹt ray.
 * 4. Chuyển đổi qua lại giữa Playback và Realtime chỉ với 1 click.
 * 5. [v6] MÔ HÌNH LƯU ĐÓNG GÓI NHỊ PHÂN CÓ KÈM METADATA:
 *    - raw_point_cloud lưu dạng Uint8Array (N×16 bytes: X,Y,Z,I float32 little-endian) trong IndexedDB.
 *    - Xuất file .bin nhị phân chuẩn công nghiệp (magic + JSON meta + binary point clouds).
 *    - Hỗ trợ import lại cả .json (cũ) lẫn .bin (mới).
 * =========================================================================================
 */

(function(global) {
  'use strict';

  class LidarEventStorageDB {
    constructor() {
      this.dbName = 'LidarRailwayBlackboxDB';
      this.dbVersion = 1;
      this.storeName = 'railway_events';
      this.db = null;
    }

    async init() {
      return new Promise((resolve, reject) => {
        const req = indexedDB.open(this.dbName, this.dbVersion);
        req.onupgradeneeded = (e) => {
          const db = e.target.result;
          if (!db.objectStoreNames.contains(this.storeName)) {
            const store = db.createObjectStore(this.storeName, { keyPath: 'id', autoIncrement: true });
            store.createIndex('timestamp', 'timestamp', { unique: false });
          }
        };
        req.onsuccess = (e) => {
          this.db = e.target.result;
          resolve(this.db);
        };
        req.onerror = (e) => reject(e);
      });
    }

    async saveEvent(eventPackage) {
      if (!this.db) await this.init();
      return new Promise((resolve, reject) => {
        const tx = this.db.transaction([this.storeName], 'readwrite');
        const store = tx.objectStore(this.storeName);
        const req = store.add(eventPackage);
        req.onsuccess = () => resolve(req.result);
        req.onerror = (e) => reject(e);
      });
    }

    async getAllEvents() {
      if (!this.db) await this.init();
      return new Promise((resolve, reject) => {
        const tx = this.db.transaction([this.storeName], 'readonly');
        const store = tx.objectStore(this.storeName);
        const req = store.getAll();
        req.onsuccess = () => resolve(req.result || []);
        req.onerror = (e) => reject(e);
      });
    }

    async deleteEvent(id) {
      if (!this.db) await this.init();
      return new Promise((resolve, reject) => {
        const tx = this.db.transaction([this.storeName], 'readwrite');
        const store = tx.objectStore(this.storeName);
        const req = store.delete(id);
        req.onsuccess = () => resolve(true);
        req.onerror = (e) => reject(e);
      });
    }
  }

  class LidarEventPlaybackManager {
    constructor() {
      this.isRecording = false;
      this.isPlaying = false;
      this.isPaused = false;
      this.autoRecordEnabled = true;
      this.alarmClearDurationMs = 2500;
      this.lastAlarmTime = 0;

      this.preBuffer = [];
      this.preBufferMax = 50;

      this.recordedFrames = [];
      this.savedEventList = [];
      this.playbackIndex = 0;
      this.playbackTimer = null;
      this.playbackSpeed = 1.0;
      this.recordStartTime = 0;
      this.maxRecordFrames = 2000;
      this.maxStoredEvents = 50;

      this.db = new LidarEventStorageDB();

      this.panelEl = null;
      this.inspectorEl = null;
      this.activeTab = 'processed'; // 'raw' hoặc 'processed'
      this.timelineSlider = null;
      this.btnPlay = null;
      this.btnPause = null;
      this.chkAuto = null;

      this.onPlaybackFrame = null;

      this._initUI();
      this._initDataInspectorModal();
      this._loadStoredEventsFromDisk();
    }

    async _loadStoredEventsFromDisk() {
      try {
        const events = await this.db.getAllEvents();
        if (events && events.length) {
          this.savedEventList = events;
          this._refreshEventsDropdown();
          if (typeof window.addLog === 'function') {
            window.addLog(`[BLACKBOX] Đã nạp ${events.length} sự cố đường tàu đã lưu từ ổ đĩa máy tính.`);
          }
        }
      } catch (e) {
        console.warn('[BLACKBOX DB]', e);
      }
    }

    _refreshEventsDropdown() {
      const p = this.panelEl;
      if (!p) return;
      const selEvents = p.querySelector('#pbSavedEventsSel');
      selEvents.innerHTML = `<option value="">📂 Danh sách sự cố (${this.savedEventList.length})</option>`;
      this.savedEventList.forEach((ev, idx) => {
        const opt = document.createElement('option');
        opt.value = idx;
        opt.textContent = ev.title || `Sự cố #${idx + 1}`;
        selEvents.appendChild(opt);
      });
    }

    _initUI() {
      if (document.getElementById('lidarPlaybackPanel')) return;

      const panel = document.createElement('div');
      panel.id = 'lidarPlaybackPanel';
      panel.style.cssText = `
        position: fixed;
        bottom: 12px;
        left: 50%;
        transform: translateX(-50%);
        background: rgba(16, 20, 30, 0.96);
        border: 1px solid #3b4566;
        border-radius: 8px;
        padding: 8px 14px;
        color: #fff;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace;
        font-size: 12px;
        box-shadow: 0 8px 28px rgba(0,0,0,0.7);
        z-index: 2000;
        display: none;
        flex-direction: column;
        gap: 6px;
        min-width: 650px;
        backdrop-filter: blur(8px);
      `;

      panel.innerHTML = `
        <div style="display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #28314a;padding-bottom:5px;">
          <div style="display:flex;align-items:center;gap:8px;">
            <span style="color:#00ffcc;font-weight:bold;font-size:13px;">📼 HỘP ĐEN RÀO CHẮN ĐƯỜNG TÀU (BLACKBOX)</span>
            <span id="pbModeBadge" style="background:#28a745;color:#fff;padding:2px 7px;border-radius:3px;font-size:10px;font-weight:bold;">STANDBY (AUTO ON)</span>
            <span style="background:#17a2b8;color:#fff;padding:2px 5px;border-radius:3px;font-size:9px;">💾 LƯU TRỰC TIẾP Ổ ĐĨA</span>
          </div>
          <div style="display:flex;align-items:center;gap:10px;">
            <label style="color:#ffcc00;font-weight:bold;cursor:pointer;display:flex;align-items:center;gap:4px;font-size:11px;">
              <input type="checkbox" id="pbChkAutoRecord" checked style="accent-color:#ffcc00;cursor:pointer;">
              ⚡ Tự động ghi khi có vật cản
            </label>
            <button id="pbBtnClose" style="background:transparent;border:none;color:#aaa;cursor:pointer;font-size:14px;font-weight:bold;">✕</button>
          </div>
        </div>

        <!-- Thanh trượt dòng thời gian Timeline -->
        <div style="display:flex;align-items:center;gap:10px;">
          <span id="pbCurTime" style="font-family:monospace;color:#f39c12;font-size:11px;min-width:55px;">00:00.0</span>
          <input type="range" id="pbTimeline" min="0" max="0" value="0" style="flex:1;cursor:pointer;accent-color:#00e5ff;">
          <span id="pbTotalTime" style="font-family:monospace;color:#888;font-size:11px;min-width:55px;">00:00.0</span>
        </div>

        <!-- Các nút bấm điều khiển -->
        <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:6px;">
          <div style="display:flex;align-items:center;gap:5px;">
            <button id="pbBtnRec" style="background:#dc3545;color:#fff;border:1px solid #b02a37;border-radius:4px;padding:3px 10px;font-weight:bold;cursor:pointer;">⏺ GHI THỦ CÔNG</button>
            <button id="pbBtnStopRec" style="background:#555;color:#fff;border:1px solid #444;border-radius:4px;padding:3px 8px;cursor:pointer;display:none;">⏹ DỪNG</button>
            <span style="border-left:1px solid #3b4566;height:16px;margin:0 4px;"></span>
            <button id="pbBtnStepBack" style="background:#202738;color:#fff;border:1px solid #3b4566;border-radius:3px;padding:3px 8px;cursor:pointer;" title="Lùi 1 frame">⏮</button>
            <button id="pbBtnPlay" style="background:#0d6efd;color:#fff;border:1px solid #0b5ed7;border-radius:3px;padding:3px 12px;font-weight:bold;cursor:pointer;">▶ PHÁT</button>
            <button id="pbBtnPause" style="background:#e67e22;color:#fff;border:1px solid #d35400;border-radius:3px;padding:3px 10px;font-weight:bold;cursor:pointer;display:none;">⏸ TẠM DỪNG</button>
            <button id="pbBtnStepForward" style="background:#202738;color:#fff;border:1px solid #3b4566;border-radius:3px;padding:3px 8px;cursor:pointer;" title="Tiến 1 frame">⏭</button>
            <button id="pbBtnLive" style="background:#28a745;color:#fff;border:1px solid #1e7e34;border-radius:3px;padding:3px 10px;font-weight:bold;cursor:pointer;" title="Thoát chế độ xem lại và quay trở về xem Realtime trực tiếp">🔴 VỀ REALTIME</button>
            <select id="pbSpeedSel" style="background:#141926;color:#00ffcc;border:1px solid #3b4566;border-radius:3px;padding:2px 4px;font-size:11px;">
              <option value="0.25">0.25x (Rất chậm)</option>
              <option value="0.5">0.5x (Chậm)</option>
              <option value="1.0" selected>1.0x (Chuẩn)</option>
              <option value="2.0">2.0x (Nhanh)</option>
              <option value="4.0">4.0x (Siêu nhanh)</option>
            </select>
          </div>

          <div style="display:flex;align-items:center;gap:6px;">
            <button id="pbBtnInspectData" style="background:#fd7e14;color:#fff;border:1px solid #d35400;border-radius:4px;padding:3px 9px;cursor:pointer;font-weight:bold;" title="Mở bảng soi chi tiết: Dữ liệu Thô (XYZI) & Dữ liệu Đã xử lý (AI Objects)">🔍 Soi Dữ Liệu</button>
            <select id="pbSavedEventsSel" style="background:#141926;color:#ffcc00;border:1px solid #e67e22;border-radius:3px;padding:2px 4px;font-size:11px;max-width:160px;">
              <option value="">📂 Danh sách sự cố (0)</option>
            </select>
            <button id="pbBtnExport" style="background:#198754;color:#fff;border:1px solid #146c43;border-radius:4px;padding:3px 8px;cursor:pointer;font-weight:bold;" title="Xuất file JSON (metadata + điểm thô dạng text)">📄 Xuất JSON</button>
            <button id="pbBtnExportBin" style="background:#6610f2;color:#fff;border:1px solid #520dc2;border-radius:4px;padding:3px 8px;cursor:pointer;font-weight:bold;" title="Xuất file nhị phân chuẩn công nghiệp: 4B magic + JSON meta + binary float32 XYZI">💾 Xuất .BIN</button>
            <button id="pbBtnImport" style="background:#495057;color:#fff;border:1px solid #343a40;border-radius:4px;padding:3px 8px;cursor:pointer;" title="Mở file sự kiện .json hoặc .bin đã lưu trước đó để phát lại">📂 Nạp File</button>
            <input type="file" id="pbFileInput" accept=".json,.bin" style="display:none;">
          </div>
        </div>

        <div id="pbStatsLine" style="font-size:10px;color:#9aa5c2;display:flex;justify-content:space-between;margin-top:2px;">
          <span>Frames: <b id="pbFrameCount" style="color:#fff;">0</b> | Điểm mây thô (Raw pts): <b id="pbRawPtsCount" style="color:#00e5ff;">0</b></span>
          <span>Mục tiêu AI: <b id="pbTargetCount" style="color:#ffcc00;">0</b> | Trạng thái rào chắn: <b id="pbZoneStatus" style="color:#2ecc71;">AN TOÀN</b></span>
        </div>
      `;

      document.body.appendChild(panel);
      this.panelEl = panel;

      this._bindEvents();
    }

    _bindEvents() {
      const p = this.panelEl;
      p.querySelector('#pbBtnClose').onclick = () => this.hide();

      this.chkAuto = p.querySelector('#pbChkAutoRecord');
      this.chkAuto.onchange = () => {
        this.autoRecordEnabled = this.chkAuto.checked;
        const badge = p.querySelector('#pbModeBadge');
        if (!this.isRecording) {
          badge.textContent = this.autoRecordEnabled ? 'STANDBY (AUTO ON)' : 'STANDBY (AUTO OFF)';
          badge.style.background = this.autoRecordEnabled ? '#28a745' : '#6c757d';
        }
      };

      const btnRec = p.querySelector('#pbBtnRec');
      const btnStopRec = p.querySelector('#pbBtnStopRec');
      btnRec.onclick = () => this.startRecording(false);
      btnStopRec.onclick = () => this.stopRecording();

      this.btnPlay = p.querySelector('#pbBtnPlay');
      this.btnPause = p.querySelector('#pbBtnPause');
      this.btnPlay.onclick = () => this.play();
      this.btnPause.onclick = () => this.pause();

      p.querySelector('#pbBtnStepBack').onclick = () => this.step(-1);
      p.querySelector('#pbBtnStepForward').onclick = () => this.step(1);
      p.querySelector('#pbBtnLive').onclick = () => this.returnToLive();

      // Nút mở Bảng soi dữ liệu Thô & Đã xử lý
      p.querySelector('#pbBtnInspectData').onclick = () => this.toggleInspectorModal();

      const spd = p.querySelector('#pbSpeedSel');
      spd.onchange = () => {
        this.playbackSpeed = parseFloat(spd.value) || 1.0;
        if (this.isPlaying) {
          this.pause();
          this.play();
        }
      };

      this.timelineSlider = p.querySelector('#pbTimeline');
      this.timelineSlider.oninput = () => {
        const idx = parseInt(this.timelineSlider.value, 10);
        this.seekTo(idx);
      };

      const selEvents = p.querySelector('#pbSavedEventsSel');
      selEvents.onchange = () => {
        const evIndex = parseInt(selEvents.value, 10);
        if (!isNaN(evIndex) && this.savedEventList[evIndex]) {
          this.loadEventPackage(this.savedEventList[evIndex]);
        }
      };

      p.querySelector('#pbBtnExport').onclick = () => this.exportEventFile();
      p.querySelector('#pbBtnExportBin').onclick = () => this.exportBinaryFile();
      const fileInp = p.querySelector('#pbFileInput');
      p.querySelector('#pbBtnImport').onclick = () => fileInp.click();
      fileInp.onchange = (e) => {
        const file = e.target.files[0];
        if (file) this.importEventFile(file);
      };
    }

    /**
     * KHỞI TẠO BẢNG SOI DỮ LIỆU (INSPECTOR MODAL: RAW & PROCESSED)
     */
    _initDataInspectorModal() {
      if (document.getElementById('lidarDataInspectorModal')) return;

      const modal = document.createElement('div');
      modal.id = 'lidarDataInspectorModal';
      modal.style.cssText = `
        position: fixed;
        top: 60px;
        right: 20px;
        width: 480px;
        height: 520px;
        background: rgba(14, 18, 28, 0.98);
        border: 1px solid #00e5ff;
        border-radius: 8px;
        box-shadow: 0 10px 32px rgba(0,0,0,0.8);
        z-index: 2100;
        display: none;
        flex-direction: column;
        color: #fff;
        font-family: monospace;
        backdrop-filter: blur(8px);
      `;

      modal.innerHTML = `
        <div style="display:flex;align-items:center;justify-content:space-between;padding:8px 12px;background:#182033;border-bottom:1px solid #2a3552;border-radius:8px 8px 0 0;">
          <div style="display:flex;align-items:center;gap:8px;">
            <span style="color:#00ffcc;font-weight:bold;font-size:12px;">📊 BẢNG SOI DỮ LIỆU ĐƯỜNG TÀU</span>
            <span id="inspFrameBadge" style="background:#0d6efd;color:#fff;padding:1px 5px;border-radius:2px;font-size:10px;">FRAME #0</span>
          </div>
          <button id="inspBtnClose" style="background:transparent;border:none;color:#aaa;cursor:pointer;font-size:14px;font-weight:bold;">✕</button>
        </div>

        <!-- Tabs chuyển đổi giữa Thô và Đã xử lý -->
        <div style="display:flex;background:#0f1422;border-bottom:1px solid #2a3552;">
          <button id="inspTabProcessed" style="flex:1;padding:7px;background:#1e273e;color:#ffcc00;border:none;border-bottom:2px solid #ffcc00;font-weight:bold;cursor:pointer;font-size:11px;">
            🤖 DỮ LIỆU ĐÃ XỬ LÝ (Mục tiêu & An toàn)
          </button>
          <button id="inspTabRaw" style="flex:1;padding:7px;background:transparent;color:#8892b0;border:none;border-bottom:2px solid transparent;font-weight:bold;cursor:pointer;font-size:11px;">
            📡 DỮ LIỆU THÔ (Tia Laser XYZI)
          </button>
        </div>

        <!-- Nội dung hiển thị -->
        <div id="inspContentArea" style="flex:1;overflow-y:auto;padding:10px;font-size:11px;line-height:1.4;">
          <!-- Render động ở đây -->
        </div>

        <div style="padding:6px 12px;background:#0f1422;border-top:1px solid #2a3552;display:flex;justify-content:space-between;font-size:10px;color:#888;">
          <span>Định dạng: Unified Schema v2.0 | Raw: Binary float32 (N×16B)</span>
          <span>Bấm Play hoặc kéo Timeline để xem giá trị nhảy theo thời gian</span>
        </div>
      `;

      document.body.appendChild(modal);
      this.inspectorEl = modal;

      modal.querySelector('#inspBtnClose').onclick = () => this.toggleInspectorModal(false);

      const tabProc = modal.querySelector('#inspTabProcessed');
      const tabRaw = modal.querySelector('#inspTabRaw');

      tabProc.onclick = () => {
        this.activeTab = 'processed';
        tabProc.style.background = '#1e273e';
        tabProc.style.color = '#ffcc00';
        tabProc.style.borderBottom = '2px solid #ffcc00';
        tabRaw.style.background = 'transparent';
        tabRaw.style.color = '#8892b0';
        tabRaw.style.borderBottom = '2px solid transparent';
        this._renderInspectorContent();
      };

      tabRaw.onclick = () => {
        this.activeTab = 'raw';
        tabRaw.style.background = '#1e273e';
        tabRaw.style.color = '#00e5ff';
        tabRaw.style.borderBottom = '2px solid #00e5ff';
        tabProc.style.background = 'transparent';
        tabProc.style.color = '#8892b0';
        tabProc.style.borderBottom = '2px solid transparent';
        this._renderInspectorContent();
      };
    }

    toggleInspectorModal(forceState) {
      if (!this.inspectorEl) return;
      const cur = this.inspectorEl.style.display !== 'none';
      const nxt = (forceState !== undefined) ? forceState : !cur;
      this.inspectorEl.style.display = nxt ? 'flex' : 'none';
      if (nxt) {
        this._renderInspectorContent();
      }
    }

    _renderInspectorContent() {
      if (!this.inspectorEl || this.inspectorEl.style.display === 'none') return;
      const content = this.inspectorEl.querySelector('#inspContentArea');
      const badge = this.inspectorEl.querySelector('#inspFrameBadge');

      const f = this.recordedFrames[this.playbackIndex] || (this.preBuffer.length ? this.preBuffer[this.preBuffer.length - 1] : null);
      if (!f) {
        content.innerHTML = `<div style="color:#aaa;text-align:center;margin-top:40px;">Chưa có dữ liệu frame nào được chọn.<br>Hãy kết nối LiDAR hoặc mở sự cố để xem.</div>`;
        return;
      }

      badge.textContent = `FRAME #${this.playbackIndex + 1} (${((f.header?.relTimeMs || 0)/1000).toFixed(1)}s)`;

      if (this.activeTab === 'processed') {
        // HIỂN THỊ DỮ LIỆU ĐÃ XỬ LÝ (PROCESSED OBJECTS & SAFETY)
        const pd = f.processed_data || {};
        const targets = pd.targets || [];
        const isAlarm = pd.alarmLevel > 0;

        let html = `
          <div style="background:#192236;padding:8px;border-radius:4px;margin-bottom:8px;border-left:4px solid ${isAlarm ? '#ff4d4d' : '#2ecc71'};">
            <div>🚨 CẤP ĐỘ NGUY HIỂM: <b style="color:${isAlarm ? '#ff4d4d' : '#2ecc71'};">${pd.railwayDanger || 'SAFE'} (Level: ${pd.alarmLevel})</b></div>
            <div>🎯 SỐ MỤC TIÊU PHÁT HIỆN: <b style="color:#ffcc00;">${pd.targetsCount || targets.length} vật thể</b></div>
          </div>
        `;

        if (targets.length === 0) {
          html += `<div style="color:#7f8c8d;padding:10px;">Lòng đường ray thông thoáng, không có vật cản vi phạm.</div>`;
        } else {
          targets.forEach((t, i) => {
            html += `
              <div style="background:#141a29;border:1px solid #2a3552;border-radius:4px;padding:8px;margin-bottom:8px;">
                <div style="display:flex;justify-content:space-between;color:#00ffcc;font-weight:bold;margin-bottom:4px;">
                  <span>#${i+1} [ID: ${t.id}] - ${t.class}</span>
                  <span style="color:#ffcc00;">${t.velocity?.speedKmh || 0} km/h</span>
                </div>
                <div>📍 Tọa độ tâm: <b>X: ${t.center?.x}m, Y: ${t.center?.y}m, Z: ${t.center?.z}m</b></div>
                <div>📐 Kích thước 3D: <b>${t.dimensions?.width}m (Rộng) × ${t.dimensions?.length}m (Dài) × ${t.dimensions?.height}m (Cao)</b></div>
                <div>⚡ Vận tốc vector: <b>Vx: ${t.velocity?.vx} m/s, Vy: ${t.velocity?.vy} m/s</b></div>
                <div>🛤 Lộ trình đã đi: <b>${(t.trajectory || []).length} điểm tọa độ</b></div>
              </div>
            `;
          });
        }
        content.innerHTML = html;
      } else {
        // HIỂN THỊ DỮ LIỆU THÔ (RAW POINT CLOUD TỪNG TIA LASER) - decode từ binary Uint8Array
        const raw = f.raw_point_cloud || {};
        const s1 = raw.sensors ? raw.sensors[0] : null;
        const s2 = raw.sensors ? raw.sensors[1] : null;

        const cnt1 = s1?.pointsCount || 0;
        const cnt2 = s2?.pointsCount || 0;
        let html = `
          <div style="background:#192236;padding:8px;border-radius:4px;margin-bottom:8px;">
            <div>🔵 LiDAR 1 (Trước): <b>${cnt1} tia laser</b>
              <span style="color:#555;font-size:9px;margin-left:6px;">${cnt1 > 0 ? (cnt1 * 16 / 1024).toFixed(1) + ' KB binary' : ''}</span></div>
            <div>🟠 LiDAR 2 (Sau): <b>${cnt2} tia laser</b>
              <span style="color:#555;font-size:9px;margin-left:6px;">${cnt2 > 0 ? (cnt2 * 16 / 1024).toFixed(1) + ' KB binary' : ''}</span></div>
          </div>
          <div style="font-weight:bold;color:#00e5ff;margin-bottom:4px;">BẢNG TỌA ĐỘ TIA LASER THÔ (Mẫu 50 điểm đầu tiên):</div>
          <table style="width:100%;border-collapse:collapse;font-size:10px;text-align:left;">
            <thead>
              <tr style="background:#1b2438;color:#00ffcc;">
                <th style="padding:3px;border:1px solid #2a3552;">Sensor</th>
                <th style="padding:3px;border:1px solid #2a3552;">X (m)</th>
                <th style="padding:3px;border:1px solid #2a3552;">Y (m)</th>
                <th style="padding:3px;border:1px solid #2a3552;">Z (m)</th>
                <th style="padding:3px;border:1px solid #2a3552;">Intensity</th>
              </tr>
            </thead>
            <tbody>
        `;

        // Helper: decode Uint8Array binary → list of {x,y,z,intensity}
        const decodeBinary = (bin, maxPts) => {
          const pts = [];
          if (!bin || bin.length < 16) return pts;
          const buf = bin.buffer ? bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength) : bin;
          const dv = new DataView(buf);
          const total = Math.floor(dv.byteLength / 16);
          const count = Math.min(total, maxPts);
          for (let i = 0; i < count; i++) {
            const off = i * 16;
            pts.push({
              x: dv.getFloat32(off, true).toFixed(3),
              y: dv.getFloat32(off + 4, true).toFixed(3),
              z: dv.getFloat32(off + 8, true).toFixed(3),
              intensity: dv.getFloat32(off + 12, true).toFixed(1)
            });
          }
          return pts;
        };

        // Lấy 25 điểm từ mỗi sensor
        const pts1 = s1?.binary ? decodeBinary(s1.binary, 25) :
                     (s1?.points ? s1.points.slice(0, 25) : []);
        const pts2 = s2?.binary ? decodeBinary(s2.binary, 25) :
                     (s2?.points ? s2.points.slice(0, 25) : []);

        pts1.forEach(p => {
          html += `
            <tr style="border-bottom:1px solid #1f283d;">
              <td style="padding:2px;color:#00e5ff;">LiDAR 1</td>
              <td style="padding:2px;">${p.x}</td>
              <td style="padding:2px;">${p.y}</td>
              <td style="padding:2px;">${p.z}</td>
              <td style="padding:2px;color:#aaa;">${p.intensity}</td>
            </tr>
          `;
        });
        pts2.forEach(p => {
          html += `
            <tr style="border-bottom:1px solid #1f283d;">
              <td style="padding:2px;color:#ff9900;">LiDAR 2</td>
              <td style="padding:2px;">${p.x}</td>
              <td style="padding:2px;">${p.y}</td>
              <td style="padding:2px;">${p.z}</td>
              <td style="padding:2px;color:#aaa;">${p.intensity}</td>
            </tr>
          `;
        });

        const totalBytes = ((cnt1 + cnt2) * 16);
        html += `
            </tbody>
          </table>
          <div style="color:#888;font-size:9px;margin-top:6px;">
            * Dữ liệu raw lưu dạng <b style="color:#a78bfa;">Binary float32 IEEE-754</b> — ${(totalBytes/1024).toFixed(1)} KB/frame.
            Bấm <b>"💾 Xuất .BIN"</b> để tải file nhị phân chuẩn công nghiệp.
          </div>
        `;

        content.innerHTML = html;
      }
    }

    show() {
      if (this.panelEl) this.panelEl.style.display = 'flex';
    }

    hide() {
      if (this.panelEl) this.panelEl.style.display = 'none';
      if (this.inspectorEl) this.inspectorEl.style.display = 'none';
    }

    toggle() {
      if (this.panelEl) {
        if (this.panelEl.style.display === 'none' || !this.panelEl.style.display) this.show();
        else this.hide();
      }
    }

    startRecording(isAuto = false) {
      if (this.isRecording) return;
      this.isRecording = true;
      this.isPlaying = false;
      this.recordStartTime = Date.now();

      this.recordedFrames = [...this.preBuffer];

      const p = this.panelEl;
      p.querySelector('#pbBtnRec').style.display = 'none';
      p.querySelector('#pbBtnStopRec').style.display = 'inline-block';
      const badge = p.querySelector('#pbModeBadge');
      badge.textContent = isAuto ? '🚨 AUTO RECORDING (VẬT CẢN)...' : 'RECORDING 🔴';
      badge.style.background = '#dc3545';

      if (typeof window.addLog === 'function') {
        window.addLog(`[BLACKBOX] ${isAuto ? '⚡ TỰ ĐỘNG GHI SỰ CỐ' : 'Bắt đầu ghi thủ công'}: Đã nạp ${this.recordedFrames.length} frame đệm trước đó.`);
      }
    }

    async stopRecording() {
      if (!this.isRecording) return;
      this.isRecording = false;
      const p = this.panelEl;
      p.querySelector('#pbBtnRec').style.display = 'inline-block';
      p.querySelector('#pbBtnStopRec').style.display = 'none';

      const badge = p.querySelector('#pbModeBadge');
      badge.textContent = this.autoRecordEnabled ? 'STANDBY (AUTO ON)' : 'STANDBY';
      badge.style.background = '#28a745';

      if (this.recordedFrames.length > 0) {
        const evName = `Sự cố #${this.savedEventList.length + 1} (${new Date().toLocaleTimeString('vi-VN')} - ${this.recordedFrames.length} frames)`;
        const evData = {
          title: evName,
          timestamp: Date.now(),
          recordedAt: new Date().toISOString(),
          frames: [...this.recordedFrames]
        };

        try {
          await this.db.saveEvent(evData);
        } catch (err) {
          console.warn('[BLACKBOX DB SAVE ERR]', err);
        }

        this.savedEventList.push(evData);

        if (this.savedEventList.length > this.maxStoredEvents) {
          const oldest = this.savedEventList.shift();
          if (oldest && oldest.id) {
            this.db.deleteEvent(oldest.id).catch(() => {});
          }
        }

        this._refreshEventsDropdown();

        const selEvents = p.querySelector('#pbSavedEventsSel');
        selEvents.value = this.savedEventList.length - 1;

        this.timelineSlider.max = Math.max(0, this.recordedFrames.length - 1);
        this.timelineSlider.value = 0;
        this.playbackIndex = 0;
        this._updateTimeDisplay();

        if (typeof window.addLog === 'function') {
          window.addLog(`[BLACKBOX] ĐÃ LƯU VÀO Ổ ĐĨA CỨNG: ${evName}`);
        }
      }
    }

    _buildUnifiedFrame(rawPointData, processedData) {
      const now = Date.now();
      const ptsL1 = rawPointData?.ptsL1 || [];
      const ptsL2 = rawPointData?.ptsL2 || [];

      // ── BINARY PACK: Mỗi điểm = 4 × float32 = 16 bytes (Little-Endian, giống protocol WebSocket v3)
      const packToBinary = (pts) => {
        if (!pts || pts.length === 0) return new Uint8Array(0);
        const buf = new ArrayBuffer(pts.length * 16);
        const dv = new DataView(buf);
        pts.forEach((p, i) => {
          const off = i * 16;
          dv.setFloat32(off,      p.x         || 0, true);  // X
          dv.setFloat32(off + 4,  p.y         || 0, true);  // Y
          dv.setFloat32(off + 8,  p.z         || 0, true);  // Z
          dv.setFloat32(off + 12, p.intensity  || 0, true); // Intensity
        });
        return new Uint8Array(buf);
      };

      const bin1 = packToBinary(ptsL1);
      const bin2 = packToBinary(ptsL2);

      return {
        header: {
          timestamp: now,
          relTimeMs: this.isRecording ? (now - this.recordStartTime) : 0,
          systemStatus: 'ONLINE',
          alarmActive: !!processedData?.alarmOn
        },
        raw_point_cloud: {
          sensors: [
            {
              sourceId: 0,
              name: 'LiDAR 1 (Mặt trước)',
              pointsCount: ptsL1.length,
              binaryBytes: bin1.length,  // số byte = pointsCount × 16
              binary: bin1               // Uint8Array — IndexedDB lưu native, không cần base64
            },
            {
              sourceId: 1,
              name: 'LiDAR 2 (Mặt sau)',
              pointsCount: ptsL2.length,
              binaryBytes: bin2.length,
              binary: bin2
            }
          ],
          extrinsics: window.extrinsics ? JSON.parse(JSON.stringify(window.extrinsics)) : []
        },
        processed_data: {
          alarmLevel: processedData?.alarmOn ? 3 : 0,
          railwayDanger: processedData?.alarmOn ? 'CRITICAL_TRACK_OBSTACLE' : 'SAFE_CLEAR',
          targetsCount: (processedData?.targets || []).length,
          targets: (processedData?.targets || []).map(t => {
            const speed = +(Math.hypot(t.vx || 0, t.vy || 0)).toFixed(2);
            return {
              id: t.id,
              class: t.category || (t.w > 0.8 || t.h > 0.8 ? 'Phương tiện' : 'Người đi bộ'),
              center: {
                x: +t.x.toFixed(3),
                y: +t.y.toFixed(3),
                z: +(t.z1 || 0).toFixed(2)
              },
              dimensions: {
                width: +(t.w || 0.4).toFixed(2),
                length: +(t.h || 0.4).toFixed(2),
                height: +((t.z2 || 1.7) - (t.z1 || 0)).toFixed(2)
              },
              velocity: {
                vx: +(t.vx || 0).toFixed(2),
                vy: +(t.vy || 0).toFixed(2),
                speed: speed,
                speedKmh: +(speed * 3.6).toFixed(1)
              },
              trajectory: t.trail ? t.trail.slice(-50).map(tp => ({ x: +tp.x.toFixed(2), y: +tp.y.toFixed(2) })) : []
            };
          }),
          safetyZones: window.zones ? JSON.parse(JSON.stringify(window.zones)) : []
        }
      };
    }

    captureRealtimeFrame(rawPointData, processedData) {
      const unifiedFrame = this._buildUnifiedFrame(rawPointData, processedData);

      this.preBuffer.push(unifiedFrame);
      if (this.preBuffer.length > this.preBufferMax) {
        this.preBuffer.shift();
      }

      const hasAlarm = unifiedFrame.processed_data.alarmLevel > 0;
      if (this.autoRecordEnabled) {
        if (hasAlarm) {
          this.lastAlarmTime = Date.now();
          if (!this.isRecording) {
            this.startRecording(true);
          }
        } else if (this.isRecording) {
          const elapsedSinceAlarm = Date.now() - this.lastAlarmTime;
          if (elapsedSinceAlarm > this.alarmClearDurationMs) {
            this.stopRecording();
          }
        }
      }

      if (this.isRecording) {
        if (this.recordedFrames.length >= this.maxRecordFrames) {
          this.stopRecording();
          return;
        }
        this.recordedFrames.push(unifiedFrame);

        const countEl = this.panelEl.querySelector('#pbFrameCount');
        if (countEl) countEl.textContent = this.recordedFrames.length;
        const ptsCountEl = this.panelEl.querySelector('#pbRawPtsCount');
        if (ptsCountEl) {
          const s1 = unifiedFrame.raw_point_cloud.sensors[0]?.pointsCount || 0;
          const s2 = unifiedFrame.raw_point_cloud.sensors[1]?.pointsCount || 0;
          ptsCountEl.textContent = s1 + s2;
        }
      }

      // Cập nhật bảng soi nếu đang mở
      if (this.inspectorEl && this.inspectorEl.style.display !== 'none' && !this.isPlaying) {
        this._renderInspectorContent();
      }
    }

    play() {
      if (this.recordedFrames.length === 0) {
        alert('Chưa có dữ liệu để phát lại. Hãy chọn sự cố trong danh sách hoặc nạp file!');
        return;
      }
      this.isPlaying = true;
      this.isPaused = false;
      this.btnPlay.style.display = 'none';
      this.btnPause.style.display = 'inline-block';
      const badge = this.panelEl.querySelector('#pbModeBadge');
      badge.textContent = `PLAYING ▶ (${this.playbackSpeed}x)`;
      badge.style.background = '#007bff';

      const intervalMs = Math.max(10, Math.floor(40 / this.playbackSpeed));

      clearInterval(this.playbackTimer);
      this.playbackTimer = setInterval(() => {
        if (this.playbackIndex >= this.recordedFrames.length - 1) {
          this.pause();
          this.playbackIndex = this.recordedFrames.length - 1;
          this._applyCurrentPlaybackFrame();
          return;
        }
        this.playbackIndex++;
        this.timelineSlider.value = this.playbackIndex;
        this._applyCurrentPlaybackFrame();
      }, intervalMs);
    }

    pause() {
      this.isPlaying = false;
      this.isPaused = true;
      clearInterval(this.playbackTimer);
      this.btnPlay.style.display = 'inline-block';
      this.btnPause.style.display = 'none';
      const badge = this.panelEl.querySelector('#pbModeBadge');
      badge.textContent = 'PAUSED ⏸';
      badge.style.background = '#e67e22';
    }

    returnToLive() {
      this.pause();
      const badge = this.panelEl.querySelector('#pbModeBadge');
      badge.textContent = this.autoRecordEnabled ? 'STANDBY (AUTO ON)' : 'STANDBY (AUTO OFF)';
      badge.style.background = this.autoRecordEnabled ? '#28a745' : '#6c757d';

      if (typeof window.latestRealtimeEv !== 'undefined' && window.latestRealtimeEv) {
        window._ev = window.latestRealtimeEv;
      }
      if (typeof window.addLog === 'function') {
        window.addLog('[BLACKBOX] Đã thoát chế độ xem lại, quay về giám sát Realtime trực tiếp.');
      }
    }

    seekTo(index) {
      if (index < 0) index = 0;
      if (index >= this.recordedFrames.length) index = this.recordedFrames.length - 1;
      this.playbackIndex = index;
      this.timelineSlider.value = index;
      this._applyCurrentPlaybackFrame();
    }

    step(direction) {
      this.pause();
      this.seekTo(this.playbackIndex + direction);
    }

    loadEventPackage(evPackage) {
      if (!evPackage || !evPackage.frames) return;
      this.recordedFrames = evPackage.frames;
      this.playbackIndex = 0;
      this.timelineSlider.max = Math.max(0, this.recordedFrames.length - 1);
      this.timelineSlider.value = 0;
      this._updateTimeDisplay();
      this._applyCurrentPlaybackFrame();
      this.show();
    }

    _applyCurrentPlaybackFrame() {
      if (this.playbackIndex < 0 || this.playbackIndex >= this.recordedFrames.length) return;
      const f = this.recordedFrames[this.playbackIndex];
      this._updateTimeDisplay();

      const s1 = f.raw_point_cloud?.sensors[0]?.pointsCount || 0;
      const s2 = f.raw_point_cloud?.sensors[1]?.pointsCount || 0;
      const ptsCountEl = this.panelEl.querySelector('#pbRawPtsCount');
      if (ptsCountEl) ptsCountEl.textContent = s1 + s2;

      const tgCountEl = this.panelEl.querySelector('#pbTargetCount');
      if (tgCountEl) tgCountEl.textContent = f.processed_data?.targetsCount || 0;

      const zStatusEl = this.panelEl.querySelector('#pbZoneStatus');
      if (zStatusEl) {
        if (f.processed_data?.alarmLevel > 0) {
          zStatusEl.textContent = '🚨 NGUY HIỂM / CÓ VẬT CẢN';
          zStatusEl.style.color = '#ff4d4d';
        } else {
          zStatusEl.textContent = 'AN TOÀN';
          zStatusEl.style.color = '#2ecc71';
        }
      }

      if (this.inspectorEl && this.inspectorEl.style.display !== 'none') {
        this._renderInspectorContent();
      }

      if (typeof this.onPlaybackFrame === 'function') {
        this.onPlaybackFrame(f);
      }
    }

    _updateTimeDisplay() {
      const p = this.panelEl;
      if (!p || this.recordedFrames.length === 0) return;
      const curFrame = this.recordedFrames[this.playbackIndex];
      const lastFrame = this.recordedFrames[this.recordedFrames.length - 1];

      const curSec = ((curFrame?.header?.relTimeMs || 0) / 1000).toFixed(1);
      const totalSec = ((lastFrame?.header?.relTimeMs || 0) / 1000).toFixed(1);

      p.querySelector('#pbCurTime').textContent = `${curSec}s`;
      p.querySelector('#pbTotalTime').textContent = `${totalSec}s`;
    }

    exportEventFile() {
      if (this.recordedFrames.length === 0) {
        alert('Chưa có dữ liệu sự kiện để xuất!');
        return;
      }

      // Xuất JSON — raw binary Uint8Array → chuyển thành mảng số để JSON.stringify được
      const serializeFrame = (f) => {
        const out = JSON.parse(JSON.stringify(f, (key, val) => {
          if (val instanceof Uint8Array) {
            return { _type: 'Uint8Array', data: Array.from(val) };
          }
          return val;
        }));
        return out;
      };

      const eventPackage = {
        package_info: {
          system_name: 'LidarWeb Railway Crossing Safety System',
          format_version: '2.0-binary-metadata',
          created_at: new Date().toISOString(),
          total_frames: this.recordedFrames.length,
          duration_sec: ((this.recordedFrames[this.recordedFrames.length - 1]?.header?.relTimeMs || 0) / 1000).toFixed(2),
          railway_location: 'KM15+200 - Giao cat Duong Sat',
          raw_storage: 'Uint8Array binary float32 IEEE-754 LE (N×16 bytes/point)',
          data_sections: ['header', 'raw_point_cloud', 'processed_data']
        },
        frames: this.recordedFrames.map(serializeFrame)
      };

      const jsonStr = JSON.stringify(eventPackage, null, 2);
      const blob = new Blob([jsonStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      const timeStr = new Date().toISOString().replace(/[:.]/g, '-');
      a.href = url;
      a.download = `railway_blackbox_event_${timeStr}.json`;
      document.body.appendChild(a);
      a.click();
      setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 1000);

      if (typeof window.addLog === 'function') {
        window.addLog(`[BLACKBOX] Đã xuất JSON (nhị phân serialize): railway_blackbox_event_${timeStr}.json (${(blob.size / 1024 / 1024).toFixed(2)} MB)`);
      }
    }

    /**
     * XUẤT FILE NHỊ PHÂN CHUẨN CÔNG NGHIỆP (.bin)
     * Cấu trúc file:
     *   Bytes  0- 3: Magic "LDRB" (4 bytes ASCII)
     *   Bytes  4- 7: uint32 LE = tổng số frame
     *   Mỗi frame:
     *     [0-3]   uint32 LE = độ dài JSON metadata (meta_len)
     *     [4..4+meta_len-1] UTF-8 JSON (header + processed_data, không có raw)
     *     [meta_len+4 .. +3] uint32 LE = số điểm LiDAR1 (N1)
     *     [+4 .. +N1*16-1]   N1 × 16 bytes: float32 X,Y,Z,I (LE)
     *     [+N1*16 .. +3]     uint32 LE = số điểm LiDAR2 (N2)
     *     [+4 .. +N2*16-1]   N2 × 16 bytes: float32 X,Y,Z,I (LE)
     */
    exportBinaryFile() {
      if (this.recordedFrames.length === 0) {
        alert('Chưa có dữ liệu sự kiện để xuất!');
        return;
      }

      try {
        const enc = new TextEncoder();
        const chunks = [];

        // Magic header
        chunks.push(new Uint8Array([0x4C, 0x44, 0x52, 0x42])); // "LDRB"

        // Tổng số frame (uint32 LE)
        const frameCntBuf = new ArrayBuffer(4);
        new DataView(frameCntBuf).setUint32(0, this.recordedFrames.length, true);
        chunks.push(new Uint8Array(frameCntBuf));

        this.recordedFrames.forEach((f) => {
          // ── JSON metadata cho frame này (header + processed_data, bỏ raw binary)
          const metaObj = {
            header: f.header,
            processed_data: f.processed_data,
            extrinsics: f.raw_point_cloud?.extrinsics || []
          };
          const metaJson = enc.encode(JSON.stringify(metaObj));

          // 4 bytes: độ dài JSON
          const metaLenBuf = new ArrayBuffer(4);
          new DataView(metaLenBuf).setUint32(0, metaJson.length, true);
          chunks.push(new Uint8Array(metaLenBuf));
          chunks.push(metaJson);

          // LiDAR 1 binary
          const s1 = f.raw_point_cloud?.sensors[0];
          const bin1 = (s1?.binary instanceof Uint8Array) ? s1.binary : new Uint8Array(0);
          const n1Buf = new ArrayBuffer(4);
          new DataView(n1Buf).setUint32(0, s1?.pointsCount || 0, true);
          chunks.push(new Uint8Array(n1Buf));
          chunks.push(bin1);

          // LiDAR 2 binary
          const s2 = f.raw_point_cloud?.sensors[1];
          const bin2 = (s2?.binary instanceof Uint8Array) ? s2.binary : new Uint8Array(0);
          const n2Buf = new ArrayBuffer(4);
          new DataView(n2Buf).setUint32(0, s2?.pointsCount || 0, true);
          chunks.push(new Uint8Array(n2Buf));
          chunks.push(bin2);
        });

        const blob = new Blob(chunks, { type: 'application/octet-stream' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        const timeStr = new Date().toISOString().replace(/[:.]/g, '-');
        a.href = url;
        a.download = `railway_blackbox_${timeStr}.bin`;
        document.body.appendChild(a);
        a.click();
        setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 1000);

        if (typeof window.addLog === 'function') {
          const totalPts = this.recordedFrames.reduce((sum, f) => {
            return sum + (f.raw_point_cloud?.sensors[0]?.pointsCount || 0)
                       + (f.raw_point_cloud?.sensors[1]?.pointsCount || 0);
          }, 0);
          window.addLog(`[BLACKBOX] Đã xuất file nhị phân chuẩn ITS: railway_blackbox_${timeStr}.bin | ${this.recordedFrames.length} frames | ${totalPts.toLocaleString()} điểm laser | ${(blob.size / 1024 / 1024).toFixed(2)} MB`);
        }
      } catch (err) {
        alert('Lỗi xuất file nhị phân: ' + err.message);
        console.error('[BLACKBOX BIN EXPORT]', err);
      }
    }

    importEventFile(file) {
      const ext = file.name.split('.').pop().toLowerCase();

      if (ext === 'bin') {
        // ── ĐỌC FILE NHỊ PHÂN .bin
        const reader = new FileReader();
        reader.onload = (e) => {
          try {
            const buf = e.target.result;
            const dv = new DataView(buf);
            const dec = new TextDecoder();

            // Kiểm tra magic "LDRB"
            const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
            if (magic !== 'LDRB') {
              alert('File .bin không hợp lệ: Magic header sai!');
              return;
            }

            const totalFrames = dv.getUint32(4, true);
            const frames = [];
            let pos = 8;

            for (let fi = 0; fi < totalFrames; fi++) {
              // JSON metadata
              const metaLen = dv.getUint32(pos, true); pos += 4;
              const metaJson = dec.decode(new Uint8Array(buf, pos, metaLen)); pos += metaLen;
              const meta = JSON.parse(metaJson);

              // LiDAR 1
              const n1 = dv.getUint32(pos, true); pos += 4;
              const bin1 = new Uint8Array(buf, pos, n1 * 16); pos += n1 * 16;

              // LiDAR 2
              const n2 = dv.getUint32(pos, true); pos += 4;
              const bin2 = new Uint8Array(buf, pos, n2 * 16); pos += n2 * 16;

              // Tái tạo unified frame
              frames.push({
                header: meta.header,
                raw_point_cloud: {
                  sensors: [
                    { sourceId: 0, name: 'LiDAR 1 (Mặt trước)', pointsCount: n1, binaryBytes: n1 * 16, binary: new Uint8Array(bin1) },
                    { sourceId: 1, name: 'LiDAR 2 (Mặt sau)',   pointsCount: n2, binaryBytes: n2 * 16, binary: new Uint8Array(bin2) }
                  ],
                  extrinsics: meta.extrinsics || []
                },
                processed_data: meta.processed_data
              });
            }

            const pkg = { title: file.name, frames };
            this.loadEventPackage(pkg);
            const badge = this.panelEl.querySelector('#pbModeBadge');
            badge.textContent = `LOADED .BIN (${frames.length} frames)`;
            badge.style.background = '#6610f2';

            if (typeof window.addLog === 'function') {
              window.addLog(`[BLACKBOX] Đã nạp file nhị phân .bin: ${file.name} — ${frames.length} frames.`);
            }
          } catch (err) {
            alert('Lỗi đọc file nhị phân: ' + err.message);
            console.error('[BLACKBOX BIN IMPORT]', err);
          }
        };
        reader.readAsArrayBuffer(file);

      } else {
        // ── ĐỌC FILE JSON .json (backward compatible)
        const reader = new FileReader();
        reader.onload = (e) => {
          try {
            const parsed = JSON.parse(e.target.result, (key, val) => {
              // Khôi phục Uint8Array nếu bị serialize thành { _type: 'Uint8Array', data: [...] }
              if (val && val._type === 'Uint8Array' && Array.isArray(val.data)) {
                return new Uint8Array(val.data);
              }
              return val;
            });
            if (!parsed.frames || !Array.isArray(parsed.frames)) {
              alert('File không hợp lệ: Không tìm thấy dữ liệu frames!');
              return;
            }
            this.loadEventPackage(parsed);
            const badge = this.panelEl.querySelector('#pbModeBadge');
            badge.textContent = `LOADED (${parsed.frames.length} frames)`;
            badge.style.background = '#6f42c1';

            if (typeof window.addLog === 'function') {
              window.addLog(`[BLACKBOX] Đã nạp file JSON: ${file.name} — ${parsed.frames.length} frames.`);
            }
          } catch (err) {
            alert('Lỗi giải mã file: ' + err.message);
          }
        };
        reader.readAsText(file);
      }
    }
  }

  global.LidarEventPlaybackManager = LidarEventPlaybackManager;

})(window);
