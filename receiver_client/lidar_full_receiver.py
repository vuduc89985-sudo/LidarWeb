#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================================================
LIDAR RAILWAY SAFETY SYSTEM - PROFESSIONAL RECEIVER CLIENT
Ung dung Nhan Du Lieu Chuyen Sau Da Kenh cho Doi tac & Trung tam Dieu hanh
Ho tro:
1. Kenh Canh Bao & Muc Tieu AI (TCP JSON Lines / Port 9100 kem ACK xac nhan).
2. Kenh May Diem Tho 3D Nhi Phan (WebSocket Stream / Port 9001).
3. Bo Hook/Callback san sang cho doi tac tich hop (PTZ Camera, Coi Den, PLC).
========================================================================================
"""

import socket
import struct
import json
import threading
import time
import sys
import os

class LidarProfessionalReceiver:
    def __init__(self, gateway_ip="127.0.0.1", visual_tcp_port=9100, ws_port=9001):
        self.gateway_ip = gateway_ip
        self.visual_tcp_port = visual_tcp_port
        self.ws_port = ws_port
        self.running = False

        # Thong ke
        self.total_events_received = 0
        self.current_alarm_level = 0
        self.active_targets = {}

        # Callbacks cho doi tac tuy bien
        self.on_alarm_callback = None
        self.on_target_callback = None
        self.on_raw_points_callback = None

    def set_on_alarm(self, callback):
        """Gan ham xu ly khi co bao dong (Level 0, 1, 2, 3)"""
        self.on_alarm_callback = callback

    def set_on_target(self, callback):
        """Gan ham xu ly khi co muc tieu moi xuat hien hoac di chuyen"""
        self.on_target_callback = callback

    def set_on_raw_points(self, callback):
        """Gan ham xu ly khi nhan duoc may diem tho 3D"""
        self.on_raw_points_callback = callback

    def start_visual_tcp_listener(self):
        """Lang nghe luong su kien tu LidarWebGateway (Visual TCP Stream)"""
        def listener_thread():
            server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                server_sock.bind(("0.0.0.0", self.visual_tcp_port))
                server_sock.listen(5)
                print(f"[TCP VISUAL] 🟢 Dang lang nghe luong su kien tren cong {self.visual_tcp_port}...")

                while self.running:
                    server_sock.settimeout(1.0)
                    try:
                        conn, addr = server_sock.accept()
                    except socket.timeout:
                        continue
                    except Exception:
                        break

                    print(f"[TCP VISUAL] 🔗 Da ket noi tu Gateway: {addr[0]}:{addr[1]}")
                    conn.settimeout(5.0)
                    buffer = ""

                    while self.running:
                        try:
                            data = conn.recv(4096)
                            if not data:
                                break
                            buffer += data.decode("utf-8", errors="ignore")
                            while "\n" in buffer:
                                line, buffer = buffer.split("\n", 1)
                                line = line.strip()
                                if not line:
                                    continue
                                self._handle_visual_line(conn, line)
                        except socket.timeout:
                            continue
                        except Exception as e:
                            print(f"[TCP VISUAL] ⚠️ Mat ket noi: {e}")
                            break
                    conn.close()
            finally:
                server_sock.close()

        t = threading.Thread(target=listener_thread, daemon=True)
        t.start()

    def _handle_visual_line(self, conn, line):
        try:
            msg = json.loads(line)
        except Exception:
            return

        self.total_events_received += 1
        now_str = time.strftime("%H:%M:%S")

        # 1. Gui phan hoi ACK (Visual ACK Protocol) de Gateway do do tre
        event_id = msg.get("eventId")
        if event_id:
            ack_pkt = json.dumps({
                "type": "ack",
                "eventId": str(event_id),
                "receivedAtUnixMs": int(time.time() * 1000)
            }) + "\n"
            try:
                conn.sendall(ack_pkt.encode("utf-8"))
            except Exception:
                pass

        # 2. Xu ly su kien doi tuong
        if msg.get("type") == "object_event":
            obj = msg.get("object", {})
            event_type = msg.get("eventType", "detected")
            obj_id = msg.get("objectId") or obj.get("id")
            x = obj.get("x", 0)
            y = obj.get("y", 0)
            vx = obj.get("vx", 0)
            vy = obj.get("vy", 0)
            speed = obj.get("speedKmh", (vx**2 + vy**2)**0.5 * 3.6) # km/h
            max_speed = obj.get("maxSpeedKmh", speed)
            w = obj.get("width", 0.4)
            h = obj.get("depth", 0.4)
            pts = obj.get("pointCount", 0)

            # Danh gia cap do an toan
            # Neu toa do roi vao long ray -> Cap 3
            danger_status = "CRITICAL" if abs(x) < 1.5 else "WARNING"
            danger_color = "\033[91m" if danger_status == "CRITICAL" else "\033[93m"
            reset_color = "\033[0m"

            crit_x = obj.get("criticalX")
            crit_y = obj.get("criticalY")
            crit_str = f" (Giua ray: X={crit_x:.2f}, Y={crit_y:.2f})" if crit_x is not None else ""

            print(f"[{now_str}] 🎯 {event_type.upper():<8} | {danger_color}MUC TIEU #{obj_id:<3} [{danger_status}]{reset_color} | Vi tri: X={x:5.2f}m, Y={y:5.2f}m{crit_str} | Toc do: {speed:4.1f} km/h (Max: {max_speed:4.1f} km/h) | Kich thuoc: {w:.2f}x{h:.2f}m | Tia: {pts:3d} pts")

            # Goi hook
            if self.on_target_callback:
                self.on_target_callback(obj_id, x, y, speed, danger_status, obj)

    def start(self):
        self.running = True
        print("=" * 80)
        print("🚀 HE THONG RECEIVER CHUYEN SAU - LIDAR RAILWAY SAFETY")
        print("=" * 80)
        print(f"  • Cong nhan su kien doi tuong (Visual TCP): {self.visual_tcp_port}")
        print(f"  • Dia chi Gateway du kien                : {self.gateway_ip}")
        print("=" * 80)
        print("Dang cho du lieu tu LidarWebGateway... (Nhan Ctrl+C de dung)\n")

        self.start_visual_tcp_listener()

        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nDang dung Receiver...")
            self.running = False

# ================= VI DU TICH HOP CHO DOI TAC =================
if __name__ == "__main__":
    receiver = LidarProfessionalReceiver(visual_tcp_port=9100)

    # Vi du Doi tac gan ham xu ly rieng:
    def my_custom_trigger(target_id, x, y, speed_kmh, danger_status, raw_obj):
        if danger_status == "CRITICAL":
            # Vi du: Kich hoat coi bao, gui lenh dieu khien PTZ camera quay toi vi tri x, y
            pass

    receiver.set_on_target(my_custom_trigger)
    receiver.start()