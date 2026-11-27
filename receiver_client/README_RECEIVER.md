# HƯỚNG DẪN BÀN GIAO & TÍCH HỢP RECEIVER CHUYÊN SÂU
**Hệ thống Giám sát & Cảnh báo Rào chắn Đường tàu (LiDAR Railway Safety)**

---

## 📦 Bộ phần mềm Receiver bàn giao gồm:

1. **`RUN_PROFESSIONAL_RECEIVER.bat`** (Khuyên dùng): 
   * Bộ nhận thời gian thực độc lập cho Windows (Click đúp chạy ngay, không cần cài đặt Python).
   * Tự động mở cổng Firewall TCP 9100, hiển thị bảng điều khiển tốc độ, vị trí, cấp độ cảnh báo.
   * **Đặc biệt:** Hỗ trợ **kéo thả bất kỳ file `.bin` sự cố nào vào file `.bat` này** để tự động giải mã tọa độ mây điểm và xuất ra file CSV.

2. **`lidar_full_receiver.py`**:
   * Mã nguồn Python chuyên sâu có sẵn các hàm Hook (`on_target`, `on_alarm`, `on_raw_points`).
   * Phù hợp cho đối tác lập trình muốn gắn thêm logic điều khiển Camera PTZ quay quét, kích hoạt rơ-le còi đèn, hoặc lưu vào cơ sở dữ liệu của đối tác.

3. **`decode_incident_bin.py`**:
   * Công cụ dòng lệnh phân tích sâu các tệp nhị phân `.bin` trong thư mục `recordings/events/`.
   * Cú pháp: `python decode_incident_bin.py <ten_file.bin> --csv --json`

4. **`decode_incident_ps.ps1`**:
   * Script giải mã nhị phân chạy trực tiếp trên Windows PowerShell không cần cài đặt Python.

---

## 🚀 Cách kết nối nhận dữ liệu trong 3 bước:

### Bước 1: Khởi động Receiver trên máy Đối tác
* Copy toàn bộ thư mục `receiver_client` sang máy tính đối tác.
* Click đúp file **`RUN_PROFESSIONAL_RECEIVER.bat`**.
* Màn hình sẽ hiện ra địa chỉ IP của máy đối tác, ví dụ: `-> IP may nay: 192.168.1.50 (Port: 9100)`.

### Bước 2: Cấu hình trên LiDAR Gateway
* Mở giao diện Web của LiDAR Gateway: `http://localhost:8080/`.
* Vào mục **Hệ thống** ➔ **Visual Config** ➔ Nhập IP của máy đối tác (`192.168.1.50`) và Port (`9100`).
* Nhấn **Lưu & Bắt đầu gửi**.

### Bước 3: Kiểm tra dữ liệu
* Cửa sổ Receiver của đối tác sẽ lập tức nhận các gói tin cảnh báo thời gian thực:
  ```text
  [14:05:01] 🎯 MỤC TIÊU #16 [CRITICAL - TRONG RAY] | Vị trí: X=1.12m, Y=3.45m | Tốc độ: 4.2 km/h | 28 tia laser
  ```

---

## 📑 Cấu trúc gói tin sự kiện (JSON Lines qua TCP 9100):
```json
{
  "type": "object_event",
  "eventType": "detected",
  "objectId": 16,
  "eventId": "10312-9342490347571-34",
  "object": {
    "x": 1.12,
    "y": 3.45,
    "zMin": 0.0,
    "zMax": 1.7,
    "vx": 0.85,
    "vy": 0.42,
    "width": 0.55,
    "depth": 0.40,
    "pointCount": 28
  },
  "sensorIds": [0]
}
```

---

## 💾 Giải mã file sự cố nhị phân (.bin):
Khi cần điều tra bằng chứng sự cố:
* Kéo file `event_xxx.bin` thả đè lên file `RUN_PROFESSIONAL_RECEIVER.bat`.
* Hệ thống sẽ tự động bóc tách và sinh ra file `.csv` chứa toàn bộ tọa độ từng tia laser ($X, Y, Z, Intensity$) của khoảnh khắc vi phạm.
