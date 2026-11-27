#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================================================
LIDAR RAILWAY SAFETY SYSTEM - INCIDENT BINARY DECODER
Bo giai ma file nhi phan su co (.bin) chuyen dung cho Doi tac & Don vi tich hop
Chuan giao thuc: LDRB (LiDAR Railway Blackbox v2.0)
Chay bang Python tieu chuan (Khong can cai dat them bat ky thu vien ben ngoai nao)
========================================================================================
"""

import sys
import os
import struct
import json
import argparse

def parse_incident_bin(file_path, export_csv=False, export_json=False, max_display_frames=5):
    if not os.path.isfile(file_path):
        print(f"[ERROR] Khong tim thay file: {file_path}")
        return False

    file_size = os.path.getsize(file_path)
    print("=" * 75)
    print(f"📁 TEP NHI PHAN SU CO: {os.path.basename(file_path)}")
    print(f"📊 Dung luong tep    : {file_size / (1024*1024):.2f} MB ({file_size:,} bytes)")
    print("=" * 75)

    import io
    if file_path.endswith(".zip"):
        import zipfile
        try:
            with zipfile.ZipFile(file_path, "r") as z:
                bin_names = [n for n in z.namelist() if n.endswith(".bin")]
                if not bin_names:
                    print(f"[ERROR] Khong tim thay file .bin nao trong tep zip: {file_path}")
                    return False
                f_stream = io.BytesIO(z.read(bin_names[0]))
        except Exception as e:
            print(f"[ERROR] Loi doc file zip: {e}")
            return False
    else:
        f_stream = open(file_path, "rb")

    with f_stream as f:
        # 1. Doc Magic Header 4 bytes
        magic = f.read(4)
        if magic != b'LDRB':
            print(f"[ERROR] Magic Header khong hop le: {magic}. Ky vong 'LDRB'!")
            return False

        # 2. Doc tong so frames (uint32 LE)
        total_frames = struct.unpack("<I", f.read(4))[0]
        print(f"🎞️ TONG SO FRAME SU CO: {total_frames} frames (bao gom ca pre-buffer 10s)")
        print("-" * 75)

        frames_meta = []
        max_alarm = 0
        all_target_ids = set()
        total_pts_l1 = 0
        total_pts_l2 = 0

        for frame_idx in range(total_frames):
            # 3. Doc do dai metadata JSON
            meta_len_data = f.read(4)
            if len(meta_len_data) < 4:
                break
            meta_len = struct.unpack("<I", meta_len_data)[0]

            # 4. Doc chuoi JSON
            meta_str = f.read(meta_len).decode('utf-8', errors='ignore')
            try:
                meta = json.loads(meta_str)
            except Exception:
                meta = {}

            # 5. Doc LiDAR 1
            n1 = struct.unpack("<I", f.read(4))[0]
            bin1 = f.read(n1 * 16)

            # 6. Doc LiDAR 2
            n2 = struct.unpack("<I", f.read(4))[0]
            bin2 = f.read(n2 * 16)

            total_pts_l1 += n1
            total_pts_l2 += n2

            # Thong ke
            al = meta.get("alarmLevel", 0)
            if al > max_alarm:
                max_alarm = al

            targets = meta.get("targets", [])
            for tg in targets:
                if "id" in tg:
                    all_target_ids.add(tg["id"])

            frames_meta.append({
                "frame": frame_idx + 1,
                "alarmLevel": al,
                "targets": targets,
                "n1": n1,
                "n2": n2
            })

            # Hien thi mau vai frame dau va frame co bao dong cao nhat
            if frame_idx < max_display_frames or al >= 3:
                danger_str = "CRITICAL (NGUY HIEM TREN RAY)" if al >= 3 else ("WARNING (CANH BAO)" if al == 2 else "SAFE")
                target_desc = ", ".join([f"Muc tieu #{t.get('id')} ({t.get('w',0):.2f}x{t.get('h',0):.2f}m, speed={t.get('vx',0):.1f}m/s)" for t in targets]) or "Khong co vat can"
                print(f"▶ Frame #{frame_idx+1:03d} | Muc an toan: [{al}] {danger_str:<30} | L1: {n1:4d} pts | L2: {n2:4d} pts")
                if targets:
                    print(f"    └─ Chi tiet vat the: {target_desc}")

                # Xuat CSV may diem tho cua frame bao dong (neu co yeu cau)
                if export_csv and (frame_idx == 0 or al >= 3):
                    csv_name = f"{os.path.splitext(file_path)[0]}_frame{frame_idx+1}_points.csv"
                    with open(csv_name, "w", encoding="utf-8") as csv_f:
                        csv_f.write("sensor,x,y,z,intensity\n")
                        # Giai ma L1
                        pts1 = struct.unpack(f"<{n1*4}f", bin1)
                        for i in range(n1):
                            csv_f.write(f"1,{pts1[i*4]:.3f},{pts1[i*4+1]:.3f},{pts1[i*4+2]:.3f},{pts1[i*4+3]:.1f}\n")
                        # Giai ma L2
                        pts2 = struct.unpack(f"<{n2*4}f", bin2)
                        for i in range(n2):
                            csv_f.write(f"2,{pts2[i*4]:.3f},{pts2[i*4+1]:.3f},{pts2[i*4+2]:.3f},{pts2[i*4+3]:.1f}\n")
                    print(f"    💾 Da trich xuat {n1+n2} diem tho ra file CSV: {os.path.basename(csv_name)}")

        print("-" * 75)
        print("📋 TONG KET GOI SU CO:")
        print(f"  • Muc do nguy hiem cao nhat: CAP {max_alarm}")
        print(f"  • Danh sach Muc tieu lien quan: {sorted(list(all_target_ids))}")
        print(f"  • Tong diem may LiDAR 1: {total_pts_l1:,} diem")
        print(f"  • Tong diem may LiDAR 2: {total_pts_l2:,} diem")
        print(f"  • Tong diem may toan su co: {total_pts_l1 + total_pts_l2:,} diem")

        if export_json:
            json_name = f"{os.path.splitext(file_path)[0]}_summary.json"
            with open(json_name, "w", encoding="utf-8") as jf:
                json.dump({
                    "file": os.path.basename(file_path),
                    "total_frames": total_frames,
                    "max_alarm_level": max_alarm,
                    "targets_involved": sorted(list(all_target_ids)),
                    "frames": frames_meta
                }, jf, indent=2, ensure_ascii=False)
            print(f"  💾 Da xuat toan bo bao cao JSON: {os.path.basename(json_name)}")

    print("=" * 75)
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Bo giai ma su co nhi phan LiDAR Railway Blackbox (.bin)")
    parser.add_argument("file", nargs="?", help="Duong dan toi file .bin can giai ma")
    parser.add_argument("--csv", action="store_true", help="Trich xuat toa do diem tho ra file CSV")
    parser.add_argument("--json", action="store_true", help="Xuat bao cao su co chi tiet ra file JSON")

    args = parser.parse_args()

    target_file = args.file
    if not target_file:
        # Tu dong tim file su co moi nhat trong thu muc neu khong truyen tham so
        events_dir = os.path.join(os.path.dirname(__file__), "..", "backend", "LidarWebGateway", "recordings", "events")
        if os.path.isdir(events_dir):
            files = [os.path.join(events_dir, f) for f in os.listdir(events_dir) if f.endswith(".bin")]
            if files:
                files.sort(key=os.path.getmtime, reverse=True)
                target_file = files[0]
                print(f"[TU DONG] Chon file su co moi nhat: {os.path.basename(target_file)}")

    if not target_file:
        print("Cach dung: python decode_incident_bin.py <duong_dan_file.bin> [--csv] [--json]")
        sys.exit(1)

    parse_incident_bin(target_file, export_csv=args.csv, export_json=args.json)