#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================================================================
VLASER - CÔNG CỤ TẠO LICENSE NỘI BỘ (KEY GENERATOR TOOL)
Dành riêng cho Nhà cung cấp giải pháp - Không bàn giao file này cho khách hàng!
========================================================================================
Hỗ trợ 3 cách nhập fingerprint:
  1. Truyền mã kích hoạt VLSR-...: --machine-code "VLSR-..."
  2. Đọc từ file ma_may.txt hoặc hwid.txt: --hwid-file ma_may.txt
  3. Truyền trực tiếp: --guid, --mac, --vol
========================================================================================
"""

import json
import hmac
import hashlib
import base64
import re
import os
import sys
import argparse
from datetime import datetime, timezone, timedelta

# Khóa bí mật nội bộ dùng chung với Gateway C++ và generate_license.ps1
HMAC_SECRET_KEY = b"VLASER_RAILWAY_SAFETY_OFFLINE_SECRET_KEY_2026"
HWID_XOR_KEY = b"VLASER_HWID_OBFUSCATE_2026"


def decode_machine_code(code: str):
    """Giải mã chuỗi VLSR-... (XOR + Base64) thành dict {machine_guid, volume_serial_c, mac_addr}"""
    clean = code.strip()
    if clean.startswith("VLSR-"):
        clean = clean[5:]
    enc_bytes = base64.b64decode(clean)
    dec_bytes = bytes(b ^ HWID_XOR_KEY[i % len(HWID_XOR_KEY)] for i, b in enumerate(enc_bytes))
    raw_str = dec_bytes.decode("utf-8")
    parts = raw_str.split("|")
    if len(parts) < 3:
        raise ValueError("Mã kích hoạt không đủ 3 thành phần (GUID|VOL|MAC)")
    return {
        "machine_guid": parts[0].strip(),
        "volume_serial_c": parts[1].strip(),
        "mac_addr": parts[2].strip(),
    }


def parse_hwid_file(filepath: str):
    """Đọc file ma_may.txt (chứa VLSR-...) hoặc file hwid.txt cũ (key=value)"""
    try:
        with open(filepath, "r", encoding="utf-8-sig") as f:
            content = f.read()
    except FileNotFoundError:
        print(f"❌ Không tìm thấy file: {filepath}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Lỗi đọc file: {e}")
        sys.exit(1)

    m = re.search(r"VLSR-[A-Za-z0-9+/=]+", content)
    if m:
        try:
            result = decode_machine_code(m.group(0))
        except Exception as e:
            print(f"❌ Lỗi giải mã mã VLSR trong file: {e}")
            sys.exit(1)
    else:
        result = {}
        for line in content.splitlines():
            line = line.strip()
            if line.startswith("#") or not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip()
            if key in ("machine_guid", "mac_addr", "volume_serial_c"):
                result[key] = value

    missing = [k for k in ("machine_guid", "mac_addr", "volume_serial_c")
               if not result.get(k) or result.get(k) == "UNKNOWN"]
    if missing:
        print(f"⚠️  Thông tin phần cứng thiếu hoặc không hợp lệ: {', '.join(missing)}")
        print("    Hãy yêu cầu khách chạy lại GET_HWID.bat")
        sys.exit(1)

    return result


def generate_license(customer, mb_uuid, volume_serial, mac_addr, days_valid=365, output_file="license.key"):
    now_utc = datetime.now(timezone.utc)
    exp_utc = now_utc + timedelta(days=days_valid)

    payload = {
        "license_id": f"VLASER-{int(now_utc.timestamp())}",
        "customer": customer,
        "device_fingerprint": {
            "machine_guid": mb_uuid.strip(),
            "volume_serial_c": volume_serial.strip(),
            "mac_addr": mac_addr.strip(),
        },
        "issued_at_utc": now_utc.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "expires_at_utc": exp_utc.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "expires_timestamp_utc": int(exp_utc.timestamp()),
        "features": [
            "MULTI_LIDAR_FUSION",
            "OBJECT_TRACKING",
            "RAIL_SAFETY_ALARM",
            "TCP_RECEIVER_9100",
            "BLACKBOX_RECORD",
        ],
        "version": "1.0",
    }

    # Chuẩn hóa Canonical JSON khớp 100% với nlohmann::json::dump() của C++ (sort keys, UTF-8, compact)
    canonical_json = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)

    # Ký HMAC-SHA256
    signature = hmac.new(HMAC_SECRET_KEY, canonical_json.encode("utf-8"), hashlib.sha256).hexdigest()

    full_license = {
        "payload": payload,
        "signature": signature,
    }

    out_dir = os.path.dirname(os.path.abspath(output_file))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(full_license, f, indent=2, ensure_ascii=False)

    print("=" * 70)
    print("✅ ĐÃ TẠO LICENSE THÀNH CÔNG!")
    print(f" -> Khách hàng   : {customer}")
    print(f" -> Hạn sử dụng  : {exp_utc.strftime('%Y-%m-%d')} ({days_valid} ngày)")
    print(f" -> Fingerprint  : GUID={mb_uuid[:8]}... | MAC={mac_addr} | VOL={volume_serial}")
    print(f" -> File xuất    : {os.path.abspath(output_file)}")
    print("=" * 70)


def interactive_mode():
    print("=" * 70)
    print("    VLASER - TẠO LICENSE BẢN QUYỀN (CHẾ ĐỘ TƯƠNG TÁC)")
    print("=" * 70)
    print()

    raw_input = input("1. Dán mã VLSR-... hoặc đường dẫn file ma_may.txt / hwid.txt: ").strip().strip('"').strip("'")
    if not raw_input:
        raw_input = "ma_may.txt"

    if os.path.isfile(raw_input):
        hw = parse_hwid_file(raw_input)
    elif raw_input.startswith("VLSR-"):
        hw = decode_machine_code(raw_input)
    else:
        guid = input("Nhập Machine GUID : ").strip()
        mac = input("Nhập MAC Address  : ").strip()
        vol = input("Nhập Volume Serial: ").strip()
        hw = {"machine_guid": guid, "mac_addr": mac, "volume_serial_c": vol}

    guid = hw["machine_guid"]
    mac = hw["mac_addr"]
    vol = hw["volume_serial_c"]

    print()
    print(f"  Machine GUID  : {guid}")
    print(f"  MAC Address   : {mac}")
    print(f"  Volume Serial : {vol}")
    print()

    customer = input("2. Tên khách hàng (mặc định: Khách hàng Vlaser): ").strip() or "Khách hàng Vlaser"
    days_input = input("3. Số ngày hiệu lực (mặc định 365, nhập 0 = vĩnh viễn): ").strip()
    try:
        days = int(days_input)
        if days <= 0:
            days = 36500
    except ValueError:
        days = 365

    out = input("4. Đường dẫn file xuất (mặc định: license.key): ").strip().strip('"') or "license.key"
    print()
    generate_license(customer, guid, vol, mac, days, out)


if __name__ == "__main__":
    if len(sys.argv) == 1:
        interactive_mode()
        input("\nNhấn Enter để thoát...")
        sys.exit(0)

    parser = argparse.ArgumentParser(description="VLASER - Tool tạo license bản quyền (Dành cho nhà cung cấp)")
    parser.add_argument("--machine-code", help="Chuỗi mã kích hoạt VLSR-...")
    parser.add_argument("--hwid-file", help="Đường dẫn file ma_may.txt hoặc hwid.txt")
    parser.add_argument("--guid", help="Machine GUID")
    parser.add_argument("--mac", help="MAC Address")
    parser.add_argument("--vol", help="Volume Serial C")
    parser.add_argument("--customer", default="Khách hàng Vlaser", help="Tên khách hàng")
    parser.add_argument("--days", type=int, default=365, help="Số ngày hiệu lực (0 = vĩnh viễn)")
    parser.add_argument("--out", default="license.key", help="Đường dẫn file license.key xuất ra")

    args = parser.parse_args()
    days = 36500 if args.days <= 0 else args.days

    if args.machine_code:
        hw = decode_machine_code(args.machine_code)
    elif args.hwid_file:
        hw = parse_hwid_file(args.hwid_file)
    elif args.guid and args.mac and args.vol:
        hw = {"machine_guid": args.guid, "mac_addr": args.mac, "volume_serial_c": args.vol}
    else:
        print("❌ Cần chỉ định --machine-code, --hwid-file HOẶC cả 3 tham số --guid, --mac, --vol")
        sys.exit(1)

    generate_license(args.customer, hw["machine_guid"], hw["volume_serial_c"], hw["mac_addr"], days, args.out)
