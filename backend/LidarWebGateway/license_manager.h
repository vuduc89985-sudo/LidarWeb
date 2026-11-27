#pragma once
/**
 * ==============================================================================
 * MODULE: HARDWARE FINGERPRINT & LICENSE VERIFICATION (OFFLINE HMAC-SHA256)
 * ==============================================================================
 * Ràng buộc phần cứng chịu lỗi (Fault-tolerant matching >= 2/3 linh kiện):
 *  - F1: MachineGuid (Registry Windows)
 *  - F2: Volume Serial C: (Ổ hệ điều hành)
 *  - F3: MAC Address Card mạng LAN cố định (Ethernet)
 *
 * Xác thực Offline:
 *  - Kiểm tra chữ ký HMAC-SHA256 (Win32 BCrypt) trên Canonical Payload JSON
 *  - Kiểm tra thời hạn UTC, Grace Period (7 ngày) và chống lùi đồng hồ (Registry)
 * ==============================================================================
 */

#include <windows.h>
#include <iphlpapi.h>
#include <bcrypt.h>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <cctype>
#include <nlohmann/json.hpp>

#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "bcrypt.lib")

namespace LicenseSystem {

using json = nlohmann::json;

static const std::string HMAC_SECRET_KEY = "VLASER_RAILWAY_SAFETY_OFFLINE_SECRET_KEY_2026";
static const int64_t GRACE_PERIOD_SECONDS = 7LL * 86400LL;   // 7 ngày gia hạn
static const int64_t CLOCK_ROLLBACK_TOLERANCE_SEC = 300LL;   // Cho phép lệch 5 phút (NTP)

// Cấu trúc Fingerprint phần cứng
struct HardwareInfo {
    std::string machineGuid;    // F1
    std::string volumeSerialC;  // F2
    std::string macAddress;     // F3
    std::string compositeHwid;  // Mã tóm tắt hiển thị cho khách
};

// Trạng thái License
struct LicenseStatus {
    bool isValid = false;
    bool isDemoMode = true;
    bool inGracePeriod = false;
    std::string customer = "DEMO / UNREGISTERED";
    std::string licenseId = "NONE";
    std::string expiresAtUtc = "NONE";
    std::vector<std::string> features;
    std::string message = "Running in Demo Mode (Raw Points only)";
};

// ------------------------------------------------------------------------------
// 1. THU THẬP VÂN TAY PHẦN CỨNG CHỊU LỖI (WIN32 API)
// ------------------------------------------------------------------------------
inline std::string GetRegistryMachineGuid() {
    HKEY hKey;
    char buffer[256] = {0};
    DWORD bufSize = sizeof(buffer);
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SOFTWARE\\Microsoft\\Cryptography", 0, KEY_READ | KEY_WOW64_64KEY, &hKey) == ERROR_SUCCESS) {
        if (RegQueryValueExA(hKey, "MachineGuid", nullptr, nullptr, (LPBYTE)buffer, &bufSize) == ERROR_SUCCESS) {
            RegCloseKey(hKey);
            return std::string(buffer);
        }
        RegCloseKey(hKey);
    }
    return "UNKNOWN_GUID";
}

inline std::string GetVolumeSerialC() {
    DWORD serialNumber = 0;
    if (GetVolumeInformationA("C:\\", nullptr, 0, &serialNumber, nullptr, nullptr, nullptr, 0)) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%04X-%04X", (serialNumber >> 16) & 0xFFFF, serialNumber & 0xFFFF);
        return std::string(buf);
    }
    return "UNKNOWN_VOL";
}

inline std::string GetPrimaryMacAddress() {
    ULONG bufLen = sizeof(IP_ADAPTER_INFO);
    std::vector<BYTE> buf(bufLen);
    PIP_ADAPTER_INFO pAdapterInfo = reinterpret_cast<IP_ADAPTER_INFO*>(buf.data());

    if (GetAdaptersInfo(pAdapterInfo, &bufLen) == ERROR_BUFFER_OVERFLOW) {
        buf.resize(bufLen);
        pAdapterInfo = reinterpret_cast<IP_ADAPTER_INFO*>(buf.data());
    }

    if (GetAdaptersInfo(pAdapterInfo, &bufLen) == NO_ERROR) {
        std::string fallbackMac;
        while (pAdapterInfo) {
            if (pAdapterInfo->AddressLength == 6) {
                bool nonZero = false;
                for (UINT i = 0; i < 6; ++i) {
                    if (pAdapterInfo->Address[i] != 0) { nonZero = true; break; }
                }
                if (nonZero) {
                    char mac[32];
                    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                        pAdapterInfo->Address[0], pAdapterInfo->Address[1], pAdapterInfo->Address[2],
                        pAdapterInfo->Address[3], pAdapterInfo->Address[4], pAdapterInfo->Address[5]);
                    if (pAdapterInfo->Type == MIB_IF_TYPE_ETHERNET) {
                        return std::string(mac);
                    }
                    if (fallbackMac.empty() && pAdapterInfo->Type != MIB_IF_TYPE_LOOPBACK) {
                        fallbackMac = mac;
                    }
                }
            }
            pAdapterInfo = pAdapterInfo->Next;
        }
        if (!fallbackMac.empty()) return fallbackMac;
    }
    return "UNKNOWN_MAC";
}

// Băm FNV-1a 64-bit đơn giản tạo mã rút gọn
inline uint64_t HashFnv1a(const std::string& str) {
    uint64_t hash = 14695981039346656037ULL;
    for (char c : str) {
        hash ^= (uint64_t)(unsigned char)c;
        hash *= 1099511628211ULL;
    }
    return hash;
}

inline HardwareInfo CollectHardwareFingerprint() {
    HardwareInfo hw;
    hw.machineGuid = GetRegistryMachineGuid();
    hw.volumeSerialC = GetVolumeSerialC();
    hw.macAddress = GetPrimaryMacAddress();

    std::string combined = hw.machineGuid + "|" + hw.volumeSerialC + "|" + hw.macAddress;
    uint64_t h = HashFnv1a(combined);
    char code[32];
    snprintf(code, sizeof(code), "VLASER-%04X-%04X-%04X",
        (unsigned int)((h >> 32) & 0xFFFF),
        (unsigned int)((h >> 16) & 0xFFFF),
        (unsigned int)(h & 0xFFFF));
    hw.compositeHwid = code;
    return hw;
}

// ------------------------------------------------------------------------------
// 2. HÀM MÃ HÓA HMAC-SHA256 (WIN32 BCRYPT) & SO SÁNH HẰNG THỜI GIAN
// ------------------------------------------------------------------------------
inline std::string ComputeHmacSha256Hex(const std::string& message, const std::string& key) {
    BCRYPT_ALG_HANDLE hAlg = nullptr;
    BCRYPT_HASH_HANDLE hHash = nullptr;
    DWORD hashObjSize = 0, dataSize = 0, hashLen = 0;

    if (BCryptOpenAlgorithmProvider(&hAlg, BCRYPT_SHA256_ALGORITHM, nullptr, BCRYPT_ALG_HANDLE_HMAC_FLAG) != 0) {
        return "";
    }

    auto cleanup = [&]() {
        if (hHash) BCryptDestroyHash(hHash);
        if (hAlg) BCryptCloseAlgorithmProvider(hAlg, 0);
    };

    if (BCryptGetProperty(hAlg, BCRYPT_OBJECT_LENGTH, (PUCHAR)&hashObjSize, sizeof(DWORD), &dataSize, 0) != 0 ||
        BCryptGetProperty(hAlg, BCRYPT_HASH_LENGTH, (PUCHAR)&hashLen, sizeof(DWORD), &dataSize, 0) != 0) {
        cleanup();
        return "";
    }

    std::vector<BYTE> hashObj(hashObjSize);
    std::vector<BYTE> hashOut(hashLen);

    if (BCryptCreateHash(hAlg, &hHash, hashObj.data(), hashObjSize,
            (PUCHAR)key.data(), (ULONG)key.size(), 0) != 0) {
        cleanup();
        return "";
    }

    if (!message.empty()) {
        if (BCryptHashData(hHash, (PUCHAR)message.data(), (ULONG)message.size(), 0) != 0) {
            cleanup();
            return "";
        }
    }

    if (BCryptFinishHash(hHash, hashOut.data(), hashLen, 0) != 0) {
        cleanup();
        return "";
    }

    cleanup();

    static const char hexDigits[] = "0123456789abcdef";
    std::string hex;
    hex.reserve(hashLen * 2);
    for (BYTE b : hashOut) {
        hex.push_back(hexDigits[(b >> 4) & 0x0F]);
        hex.push_back(hexDigits[b & 0x0F]);
    }
    return hex;
}

inline bool ConstantTimeHexEquals(const std::string& a, const std::string& b) {
    if (a.size() != b.size() || a.empty()) return false;
    unsigned char diff = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        unsigned char ca = (unsigned char)std::tolower((unsigned char)a[i]);
        unsigned char cb = (unsigned char)std::tolower((unsigned char)b[i]);
        diff |= (ca ^ cb);
    }
    return diff == 0;
}

// ------------------------------------------------------------------------------
// 3. CƠ CHẾ CHỐNG LÙI ĐỒNG HỒ (HIGH-WATERMARK TIMESTAMP TRONG REGISTRY)
// ------------------------------------------------------------------------------
inline bool CheckAndUpdateAntiRollback(int64_t nowUtc, std::string& errOut) {
    const char* regPath = "SOFTWARE\\Vlaser\\LidarWebGateway";
    HKEY hKey = nullptr;
    DWORD disp = 0;
    if (RegCreateKeyExA(HKEY_CURRENT_USER, regPath, 0, nullptr, REG_OPTION_NON_VOLATILE,
            KEY_READ | KEY_WRITE, nullptr, &hKey, &disp) != ERROR_SUCCESS) {
        // Không ghi được registry thì bỏ qua kiểm tra rollback, không khóa oan
        return true;
    }

    char tsBuf[64] = {0};
    char sigBuf[128] = {0};
    DWORD tsLen = sizeof(tsBuf);
    DWORD sigLen = sizeof(sigBuf);

    bool hasStored = (RegQueryValueExA(hKey, "LastSeenUtc", nullptr, nullptr, (LPBYTE)tsBuf, &tsLen) == ERROR_SUCCESS) &&
                     (RegQueryValueExA(hKey, "LastSeenSig", nullptr, nullptr, (LPBYTE)sigBuf, &sigLen) == ERROR_SUCCESS);

    int64_t highWatermark = nowUtc;
    if (hasStored) {
        std::string storedTsStr(tsBuf);
        std::string storedSigStr(sigBuf);
        std::string expectedSig = ComputeHmacSha256Hex("HWM|" + storedTsStr, HMAC_SECRET_KEY);
        if (ConstantTimeHexEquals(storedSigStr, expectedSig)) {
            try {
                int64_t storedTs = std::stoll(storedTsStr);
                if (nowUtc + CLOCK_ROLLBACK_TOLERANCE_SEC < storedTs) {
                    RegCloseKey(hKey);
                    errOut = "Phát hiện lùi đồng hồ hệ thống -> Chạy chế độ DEMO";
                    return false;
                }
                if (storedTs > highWatermark) {
                    highWatermark = storedTs;
                }
            } catch (...) {}
        }
    }

    std::string newTsStr = std::to_string(highWatermark);
    std::string newSig = ComputeHmacSha256Hex("HWM|" + newTsStr, HMAC_SECRET_KEY);
    RegSetValueExA(hKey, "LastSeenUtc", 0, REG_SZ, (const BYTE*)newTsStr.c_str(), (DWORD)(newTsStr.size() + 1));
    RegSetValueExA(hKey, "LastSeenSig", 0, REG_SZ, (const BYTE*)newSig.c_str(), (DWORD)(newSig.size() + 1));
    RegCloseKey(hKey);
    return true;
}

// ------------------------------------------------------------------------------
// 4. SO KHỚP CHỊU LỖI >= 2/3 LINH KIỆN
// ------------------------------------------------------------------------------
inline bool MatchFingerprint(const HardwareInfo& current, const json& licHw) {
    int matches = 0;
    if (licHw.contains("machine_guid") && licHw["machine_guid"].is_string() &&
        licHw["machine_guid"].get<std::string>() == current.machineGuid) {
        matches++;
    }
    if (licHw.contains("volume_serial_c") && licHw["volume_serial_c"].is_string() &&
        licHw["volume_serial_c"].get<std::string>() == current.volumeSerialC) {
        matches++;
    }
    if (licHw.contains("mac_addr") && licHw["mac_addr"].is_string() &&
        licHw["mac_addr"].get<std::string>() == current.macAddress) {
        matches++;
    }
    return matches >= 2;
}

// ------------------------------------------------------------------------------
// 5. XÁC THỰC FILE LICENSE.KEY OFFLINE
// ------------------------------------------------------------------------------
inline LicenseStatus VerifyLicenseFile(const std::string& licPath, const HardwareInfo& hw) {
    LicenseStatus st;
    st.isValid = false;
    st.isDemoMode = true;
    st.inGracePeriod = false;

    std::ifstream f(licPath);
    if (!f.is_open()) {
        st.message = "Không tìm thấy file 'license.key' -> Chạy chế độ DEMO (chỉ xem điểm thô)";
        return st;
    }

    try {
        json root = json::parse(f);
        if (!root.contains("payload") || !root["payload"].is_object() ||
            !root.contains("signature") || !root["signature"].is_string()) {
            st.message = "File license sai cấu trúc -> Chạy chế độ DEMO";
            return st;
        }

        const json& payload = root["payload"];
        std::string sig = root["signature"].get<std::string>();

        // 1. Kiểm tra chữ ký số HMAC-SHA256 trên Canonical JSON (nlohmann::json::dump() mặc định sort keys, UTF-8, compact)
        std::string canonicalJson = payload.dump();
        std::string expectedSig = ComputeHmacSha256Hex(canonicalJson, HMAC_SECRET_KEY);
        if (expectedSig.empty() || !ConstantTimeHexEquals(sig, expectedSig)) {
            st.message = "Chữ ký số license không hợp lệ (hoặc file đã bị chỉnh sửa) -> Chạy chế độ DEMO";
            return st;
        }

        // 2. So khớp phần cứng chịu lỗi (>= 2/3)
        if (!payload.contains("device_fingerprint") || !payload["device_fingerprint"].is_object() ||
            !MatchFingerprint(hw, payload["device_fingerprint"])) {
            st.message = "Mã phần cứng máy tính không khớp với license -> Chạy chế độ DEMO";
            return st;
        }

        // 3. Kiểm tra chống lùi đồng hồ & thời hạn UTC (+ Grace Period 7 ngày)
        auto now = std::chrono::system_clock::now();
        int64_t nowTime = (int64_t)std::chrono::system_clock::to_time_t(now);

        std::string rollbackErr;
        if (!CheckAndUpdateAntiRollback(nowTime, rollbackErr)) {
            st.message = rollbackErr;
            return st;
        }

        if (payload.contains("expires_at_utc") && payload["expires_at_utc"].is_string()) {
            st.expiresAtUtc = payload["expires_at_utc"].get<std::string>();
        }
        if (payload.contains("expires_timestamp_utc") && payload["expires_timestamp_utc"].is_number_integer()) {
            int64_t expTs = payload["expires_timestamp_utc"].get<int64_t>();
            if (nowTime > expTs) {
                int64_t overdue = nowTime - expTs;
                if (overdue <= GRACE_PERIOD_SECONDS) {
                    st.inGracePeriod = true;
                } else {
                    st.message = "License đã hết hạn (" + st.expiresAtUtc + ") -> Chạy chế độ DEMO";
                    return st;
                }
            }
        }

        // 4. Nạp thông tin & tính năng
        if (payload.contains("customer") && payload["customer"].is_string()) {
            st.customer = payload["customer"].get<std::string>();
        }
        if (payload.contains("license_id") && payload["license_id"].is_string()) {
            st.licenseId = payload["license_id"].get<std::string>();
        }
        if (payload.contains("features") && payload["features"].is_array()) {
            for (const auto& feat : payload["features"]) {
                if (feat.is_string()) st.features.push_back(feat.get<std::string>());
            }
        }

        st.isValid = true;
        st.isDemoMode = false;
        if (st.inGracePeriod) {
            int64_t remainDays = std::max<int64_t>(1, (GRACE_PERIOD_SECONDS - (nowTime - payload["expires_timestamp_utc"].get<int64_t>()) + 86399) / 86400);
            st.message = "License đã hết hạn (" + st.expiresAtUtc + "), đang trong Grace Period (còn " + std::to_string(remainDays) + " ngày): " + st.licenseId;
        } else {
            st.message = "License hợp lệ: " + st.licenseId + " (" + st.customer + ")";
        }
        return st;

    } catch (const std::exception& e) {
        st.message = std::string("Lỗi đọc license: ") + e.what() + " -> Chạy chế độ DEMO";
        return st;
    }
}

} // namespace LicenseSystem
