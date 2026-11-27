// lidar_web_gateway_v3.cpp
// Backend headless: WebSocket + LiDAR SDK
// v3: (1) Zone da hinh: rect/polygon/circle/sector/freehand + sua dinh
//     (2) IP dong: nhap IP tren web, them/bot LiDAR, luu lidar_config.cfg
//     (3) Stage latency telemetry and optional Visual receiver transport
//     (4) Da bo sung day du thong tin Z / do cao (dz) vao CSV va CSV Player
//     (5) Hop nhat uu diem Code 1: Zone.sourceId doc lap voi Zone.id + tuong thich cau hinh cu
//     (6) CSV Player tu dong nhan dien header, tranh nham format 11 cot

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif
#ifndef WEBSOCKETPP_CPP11_STL
#define WEBSOCKETPP_CPP11_STL
#endif

#ifndef _WEBSOCKETPP_CPP11_STL_
#define _WEBSOCKETPP_CPP11_STL_
#endif

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <Windows.h>
#undef min
#undef max
#endif
#include <iphlpapi.h>
#include <icmpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#include <stdio.h>
#include <stdarg.h>
#include <thread>
#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <mutex>
#include <atomic>
#include <deque>
#include <condition_variable>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <unordered_map>
#include <chrono>
#include <climits>
#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include "license_manager.h"
#include "ladar_api.h"

// Biến toàn cục lưu trạng thái bản quyền của Gateway
static LicenseSystem::LicenseStatus gLicStatus;
static LicenseSystem::HardwareInfo gHwInfo;
static std::mutex gLicMutex;
static std::atomic<bool> gIsDemoMode{ true };
#pragma comment(lib, "ladarsdk_gd.lib")

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using WsServer = websocketpp::server<websocketpp::config::asio>;
using ConnHdl = websocketpp::connection_hdl;

static void QueueWebJson(const json& j);
static void BroadcastText(const std::string& s);
static void BroadcastExtr(int id);
static void BroadcastLidarConfig();

// ================= 1. CAU HINH =================
static const int MAX_LIDARS = 16;  // >>> v3: ho tro toi da 16 LiDAR <<<
static const char* LIDAR_CFG_FILE = "lidar_config.cfg";  // >>> v3: file cau hinh IP <<<
const size_t MAX_HISTORY_FRAMES = 500;
std::atomic<bool> gIsPaused(false);
std::atomic<bool> gNewDataAvailable(false);
int gSelectedLidar = 0;
std::atomic<int> gActiveLidarCount(0);  // >>> v3: so LiDAR hien tai <<<

const float PI_F = 3.14159265358979f;
inline float Deg2Rad(float d) { return d * PI_F / 180.0f; }
inline float Rad2Deg(float r) { return r * 180.0f / PI_F; }
const float ANGLE_START = -45.0f;
const float ANGLE_INC = 0.0625f;
inline int GetPointIndexFromAngle(float angleDeg)
{
    return (int)roundf((angleDeg - ANGLE_START) / ANGLE_INC);
}

#include "extrinsic3d.h"
#include "latency_stats.h"
#include "visual_ack_protocol.h"
struct Ptf { float x, y; };
// >>> v3.1: ten khong dau cach (de luu vao lidar_config.cfg) <<<
static std::string SanitizeName(const std::string& s)
{
    std::string r = s;
    for (auto& c : r) if (c == ' ' || c == '\t' || c == '\r' || c == '\n') c = '_';
    if (r.empty()) r = "LiDAR";
    return r;
}

// ================= 2. LOG =================
static FILE* gLogFile = NULL;
static std::mutex gLogMutex;
static HANDLE gConOut = NULL;

void LogPrintf(const char* fmt, ...)
{
    char buf[1024]; va_list ap;
    va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    std::lock_guard<std::mutex> lk(gLogMutex);
    if (gConOut) { DWORD w = 0; WriteFile(gConOut, buf, (DWORD)strlen(buf), &w, NULL); }
    if (gLogFile) { fputs(buf, gLogFile); fflush(gLogFile); }
}

// ================= 6. PARAM LOC =================
static const int PERSISTENCE_FRAMES = 3;
static const float MIN_TARGET_W = 0.06f, MIN_TARGET_H = 0.06f;
static const float MAX_TARGET_W = 3.0f, MAX_TARGET_H = 3.0f;
static const int MIN_CLUSTER_POINTS = 7;
static const float ZONE_BUFFER = 0.30f;

// ================= 7. OCCLUSION =================
static int gLowPointFrames = 0;
static bool gOcclusionAlarm = false;
static float gAvgPointCount = 0.0f;
static int gAvgCount = 0;
static const int OCCLUSION_MIN_POINTS = 200;
static const int OCCLUSION_PERSIST_FRAMES = 25;

// ================= 8. MOTION =================
struct DynTarget {
    int id; float x, y, vx, vy, w, h, z1, z2; int n, age;
    float spawnX = 0, spawnY = 0;
    float maxDistFromSpawn = 0;
    int miss = 0;
    float maxW = 0, maxH = 0;
    float maxSpeed = 0;
    float criticalX = 0, criticalY = 0;
    float exitX = 0, exitY = 0;
};
static std::vector<DynTarget> gTargets;
static std::unordered_map<long long, int> gBgSeen;
static int gBgFrames = 0;
static int gNextTargetId = 1;
static bool gPrevAlarm = false;

// Bộ lưu lịch sử mục tiêu gần đây (cho Camera AI & Web Client phục hồi đối tượng đã qua)
struct TargetHistoryRecord {
    int id = 0;
    float x = 0, y = 0, z1 = 0, z2 = 0;
    float w = 0, h = 0;
    float maxW = 0, maxH = 0;
    float vx = 0, vy = 0;
    float curSpeed = 0;
    float maxSpeed = 0;
    int pointCount = 0;
    int age = 0;
    float criticalX = 0, criticalY = 0;
    float exitX = 0, exitY = 0;
    uint64_t firstSeenMs = 0;
    uint64_t lastSeenMs = 0;
    bool active = true;
};
static std::mutex gTargetHistoryMutex;
static std::vector<TargetHistoryRecord> gRecentTargets;

// Kiem tra muc tieu thuc su di chuyen (loai bo rung lac nhap nhay tai cho tren bo tuong)
inline bool IsRealMotionTarget(const DynTarget& tg)
{
    float speed = std::hypot(tg.vx, tg.vy);
    // Vat the ro net (>= 14 diem nhu ban tay, co the nguoi, AGV): luon la muc tieu hop le
    if (tg.n >= 14) return true;
    // Vat the nho (7 - 13 diem): phai co do dich chuyen thuc te (>= 0.08m ~ 8cm)
    // hoac van toc duy tri on dinh (>= 0.12 m/s sau it nhat 3 frame)
    return (tg.maxDistFromSpawn >= 0.08f) || (speed >= 0.12f && tg.age >= 3);
}

void ResetMotionModel()
{
    gBgSeen.clear(); gBgFrames = 0; gTargets.clear(); gPrevAlarm = false;
}

// ================= 9. CLUSTER =================
static bool gClusterMode = false;
static DWORD64 gClusterLast = 0;
struct Box { float x1, y1, x2, y2; int n; };
static std::vector<Box> gBoxes;
static float gNearest = 0.0f;
static int gNObj = 0;

// ================= 10. PICK =================
static Ptf gPickRef[2], gPickTarget[2];
static int gPickNRef = 0, gPickNTarget = 0;

// ================= 11/12/13. ICP / RMSE / CFG =================
static bool gIcpRunning = false;
static std::atomic<bool> gAutoAlignRunning{ false };
void RunGlobalAutoAlign(int targetId);
static std::mutex gRmseMutex;
static bool gRmseActive = false;
static float gRmseTrue = 0;
static int gRmseCount = 0;
static double gRmseSum = 0, gRmseSumSq = 0;
static const int gRmseN = 100;
static const char* EXTR_CFG = "extrinsics.cfg";

// ================= 14. POINT CLOUD =================
struct PointCloudData
{
    std::vector<float> x_coords, y_coords, z_coords, local_x, local_y, local_z;
    std::vector<float> intensities, angles_deg, distances;
    int frame_id = 0;
    float range_min = 0, range_max = 0, angle_min = 0, angle_max = 0;
    float angle_increment = 0, scan_time = 0, time_increment = 0;
    int32_t wave_count = 0;
    uint32_t ts_secs = 0, ts_usecs = 0;
    long long backend_arrival_steady_us = 0;
    bool source_timestamp_valid = false;
    bool has_param = false;
    float origin_x = 0, origin_y = 0, origin_z = 0;
    float origin_roll = 0, origin_pitch = 0, origin_yaw = 0;
    int source_id = 0;
};
using FramePtr = std::shared_ptr<PointCloudData>;

// ================= 15. DO TRE =================
static long long NowUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static LatencyStats gCommandRttStats, gDetectionStats, gTrackingFusionStats, gVisualAckStats;
static std::atomic<unsigned long long> gMetricBatchId{ 0 };
static std::mutex gMetricFrameMutex;
static json gMetricFrames = json::array();
static std::atomic<bool> gVisualConfigured{ false };
static std::atomic<unsigned long long> gVisualDroppedCount{ 0 };
static std::atomic<unsigned long long> gVisualGeneration{ 0 };
static std::atomic<long long> gCmdSentUs{ 0 };
static std::atomic<uint16_t>  gCmdExpect{ 0 };
static std::atomic<long long> gConnectUs{ 0 };
static std::atomic<bool>      gGotFirstFrame{ false };

static json StatsJson(const char* name, const char* status, const char* start, const char* end,
    const LatencyStats::Snapshot& s)
{
    json j = { {"name",name},{"status",status},{"start",start},{"end",end},
        {"unit","ms"},{"available",s.available},{"sampleCount",s.sampleCount} };
    if (s.available) {
        j["last"] = s.last; j["mean"] = s.mean; j["median"] = s.median;
        j["p95"] = s.p95; j["p99"] = s.p99; j["min"] = s.min; j["max"] = s.max;
    }
    return j;
}

static json BuildLatencyTelemetry()
{
    json metrics = json::array();
    metrics.push_back(StatsJson("detection_processing_ms", "measured_directly",
        "after frame batch is available to detector", "candidate clusters are ready", gDetectionStats.Get()));
    metrics.push_back(StatsJson("tracking_fusion_processing_ms", "measured_directly",
        "multi-sensor point aggregation starts", "track update completes; sum of disjoint fusion and tracking stages", gTrackingFusionStats.Get()));
    metrics.push_back(StatsJson("visual_send_ack_rtt_ms", gVisualAckStats.Get().available ? "measured_directly" : "not_measured",
        "LiDARWeb begins writing event to configured Visual TCP stream", "matching ACK arrives from Visual receiver", gVisualAckStats.Get()));
    metrics.push_back(StatsJson("visual_one_way_receive_ms", "not_measured",
        "LiDARWeb send timestamp", "Visual receive timestamp", LatencyStats::Snapshot{}));
    metrics.push_back(StatsJson("visual_classification_ms", "not_measured",
        "Visual receiver accepts event", "Visual classification completes", LatencyStats::Snapshot{}));
    metrics.push_back(StatsJson("lidar_command_rtt_ms", "measured_directly",
        "LiDARWeb issues standby/wakeup command", "matching LiDAR response is received", gCommandRttStats.Get()));
    metrics.push_back(StatsJson("lidar_frame_age_estimate_ms", "not_measured",
        "LiDAR scan timestamp (clock/scan convention unverified)", "backend monotonic frame arrival", LatencyStats::Snapshot{}));
    json j; j["type"] = "latency_metrics"; j["metrics"] = metrics;
    j["batchId"] = gMetricBatchId.load();
    { std::lock_guard<std::mutex> lock(gMetricFrameMutex); j["frames"] = gMetricFrames; }
    j["visualConfigured"] = gVisualConfigured.load();
    j["visualDroppedEventCount"] = gVisualDroppedCount.load();
    return j;
}

void MeasureCommandLatency(ILadar* ladar, bool sleep)
{
    if (!ladar) return;
    gCmdSentUs = NowUs();
    if (sleep) { gCmdExpect = 7140; ladar->Standby(1); LogPrintf("[DO TRE] Gui lenh SLEEP...\n"); }
    else { gCmdExpect = 110;  ladar->Standby(0); LogPrintf("[DO TRE] Gui lenh WAKE UP...\n"); }
}

void CheckCommandResponse(uint16_t cmdWord)
{
    long long t0 = gCmdSentUs.load();
    if (t0 && cmdWord == gCmdExpect.load())
    {
        if (NowUs() - t0 > 5000000) { gCmdSentUs = 0; return; }
        gCommandRttStats.Record((NowUs() - t0) / 1000.0); gCmdSentUs = 0;
        QueueWebJson(BuildLatencyTelemetry());
    }
}

void NoteFrameArrival()
{
    if (!gGotFirstFrame.exchange(true))
    {
        long long t0 = gConnectUs.load();
        if (t0) LogPrintf("[DO TRE] Ket noi -> frame dau: %.2f ms\n", (NowUs() - t0) / 1000.0);
    }
    // LiDAR timestamp is retained as source metadata, but is not subtracted from
    // backend time until clock offset and scan timestamp semantics are verified.
}

// Optional JSON-lines TCP adapter for a separate Visual host. All socket I/O is
// owned by this worker, so detector and LiDAR callback threads never await ACKs.
class VisualTcpAdapter
{
public:
    void Start() { worker_ = std::thread([this] { Run(); }); }
    void Stop()
    {
        { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
    void Configure(const std::string& host, unsigned short port)
    {
        std::lock_guard<std::mutex> lock(mutex_); host_ = host; port_ = port;
        gVisualDroppedCount += queue_.size(); queue_.clear(); gVisualGeneration.fetch_add(1);
        gVisualConfigured = !host.empty() && port != 0;
        gVisualAckStats.Reset(); cv_.notify_all();
    }
    bool Enqueue(const json& message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (host_.empty() || !port_ || stopping_) return false;
        if (queue_.size() >= kMaxQueue) { ++dropped_; ++gVisualDroppedCount; return false; }
        queue_.push_back(message); cv_.notify_one(); return true;
    }
    std::string Host() const { std::lock_guard<std::mutex> lock(mutex_); return host_; }
    unsigned short Port() const { std::lock_guard<std::mutex> lock(mutex_); return port_; }
    unsigned long long Dropped() const { std::lock_guard<std::mutex> lock(mutex_); return dropped_; }
private:
    static const size_t kMaxQueue = 128;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<json> queue_;
    std::string host_;
    unsigned short port_ = 0;
    bool stopping_ = false;
    unsigned long long dropped_ = 0;
    std::thread worker_;
    static void Close(SOCKET& s) { if (s != INVALID_SOCKET) { closesocket(s); s = INVALID_SOCKET; } }
    SOCKET Connect(const std::string& host, unsigned short port)
    {
        addrinfo hints{}; hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM; hints.ai_protocol = IPPROTO_TCP;
        addrinfo* list = nullptr; const std::string service = std::to_string(port);
        if (getaddrinfo(host.c_str(), service.c_str(), &hints, &list) != 0) return INVALID_SOCKET;
        SOCKET s = INVALID_SOCKET;
        for (auto p = list; p; p = p->ai_next) {
            s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
            if (s == INVALID_SOCKET) continue;
            u_long nonBlocking = 1; ioctlsocket(s, FIONBIO, &nonBlocking);
            int connected = ::connect(s, p->ai_addr, (int)p->ai_addrlen);
            if (connected != 0 && WSAGetLastError() == WSAEWOULDBLOCK) {
                fd_set writeSet; FD_ZERO(&writeSet); FD_SET(s, &writeSet); timeval timeout{}; timeout.tv_usec = 500000;
                connected = select(0, nullptr, &writeSet, nullptr, &timeout);
                if (connected > 0) { int error = 0; int len = sizeof(error); getsockopt(s, SOL_SOCKET, SO_ERROR, (char*)&error, &len); connected = error == 0 ? 0 : SOCKET_ERROR; }
                else connected = SOCKET_ERROR;
            }
            if (connected == 0) {
                nonBlocking = 0; ioctlsocket(s, FIONBIO, &nonBlocking);
                DWORD sendTimeout = 250; setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char*)&sendTimeout, sizeof(sendTimeout));
                break;
            }
            Close(s);
        }
        freeaddrinfo(list); return s;
    }
    static bool SendAll(SOCKET s, const std::string& text)
    {
        size_t offset = 0;
        while (offset < text.size()) {
            int n = ::send(s, text.data() + offset, (int)(text.size() - offset), 0);
            if (n <= 0) return false;
            offset += (size_t)n;
        }
        return true;
    }
    bool WaitForAck(SOCKET s, const std::string& expectedEventId)
    {
        VisualAckMatcher matcher; matcher.Begin(expectedEventId, NowUs(), 750000);
        std::string input; char buf[2048];
        while (matcher.Pending()) {
            fd_set set; FD_ZERO(&set); FD_SET(s, &set); timeval tv{}; tv.tv_usec = 20000;
            int ready = select(0, &set, nullptr, nullptr, &tv);
            if (ready == SOCKET_ERROR) return false;
            if (!ready) { if (matcher.Expire(NowUs()) == VisualAckMatcher::Result::timeout) return false; continue; }
            int n = recv(s, buf, sizeof(buf), 0); if (n <= 0) return false;
            input.append(buf, buf + n); size_t eol;
            while ((eol = input.find('\n')) != std::string::npos) {
                std::string line = input.substr(0, eol); input.erase(0, eol + 1);
                try {
                    json ack = json::parse(line);
                    auto result = matcher.Accept(ack, NowUs());
                    if (result == VisualAckMatcher::Result::matched) return true;
                    if (result == VisualAckMatcher::Result::timeout) return false;
                }
                catch (...) { /* Ignore malformed/unrelated ACKs until timeout. */ }
            }
        }
        return false;
    }
    void Run()
    {
        SOCKET s = INVALID_SOCKET; std::string liveHost; unsigned short livePort = 0;
        for (;;) {
            json message; std::string host; unsigned short port; unsigned long long generation;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return stopping_ || !queue_.empty(); });
                if (stopping_) break;
                host = host_; port = port_; generation = gVisualGeneration.load();
                if (host != liveHost || port != livePort) { Close(s); liveHost.clear(); livePort = 0; }
                if (queue_.empty() || host.empty() || !port) continue;
                message = std::move(queue_.front()); queue_.pop_front();
            }
            if (s == INVALID_SOCKET) {
                s = Connect(host, port);
                if (s == INVALID_SOCKET) {
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (!stopping_ && host_ == host && port_ == port) {
                            if (queue_.size() < kMaxQueue) queue_.push_front(std::move(message)); else { ++dropped_; ++gVisualDroppedCount; }
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue;
                }
                liveHost = host; livePort = port;
            }
            const std::string eventId = message.value("eventId", "");
            message["sentAtUnixMs"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            message["sentAtMonotonicUs"] = NowUs();
            const std::string payload = message.dump() + "\n";
            const long long sentAtUs = NowUs();
            if (!SendAll(s, payload) || !WaitForAck(s, eventId)) {
                LogPrintf("[VISUAL] ACK timeout/disconnect eventId=%s; no receive latency recorded\n", eventId.c_str());
                Close(s); liveHost.clear(); livePort = 0; continue;
            }
            if (generation == gVisualGeneration.load())
                gVisualAckStats.Record((NowUs() - sentAtUs) / 1000.0);
            QueueWebJson(BuildLatencyTelemetry());
        }
        Close(s);
    }
};
static VisualTcpAdapter gVisualAdapter;
static std::atomic<unsigned long long> gVisualEventSequence{ 1 };
static std::string NewVisualEventId()
{
    return std::to_string(GetCurrentProcessId()) + "-" + std::to_string(NowUs()) + "-" +
        std::to_string(gVisualEventSequence.fetch_add(1));
}

int GetNowString(char* sNow, int size)
{
    SYSTEMTIME now; GetLocalTime(&now);
    sprintf_s(sNow, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
        now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond, now.wMilliseconds);
    return (int)strlen(sNow);
}

// ================= v3: ZONE DA HINH =================
// >>> v3: Zone ho tro nhieu loai hinh <<<
enum ZoneType { ZT_RECT = 0, ZT_POLYGON = 1, ZT_CIRCLE = 2, ZT_SECTOR = 3, ZT_FREEHAND = 4 };

struct Zone
{
    bool local = false;
    bool exclusion = false;
    int id = 0;
    int sourceId = 0;  // LiDAR whose local coordinate frame owns this Zone
    int type = ZT_RECT;  // ZoneType
    // Rectangle (giu nguyen tuong thich cu)
    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    // Polygon / Freehand: danh sach dinh
    std::vector<Ptf> vertices;
    // Circle: tam + ban kinh
    float cx = 0, cy = 0, radius = 0;
    // Sector: tam + ban kinh + goc bat dau/ket thuc (radian)
    float sx = 0, sy = 0, sRadius = 0, sAngleStart = 0, sAngleEnd = 0;

    // >>> v3: Kiem tra diem co nam trong zone khong <<<
    bool ContainsPoint(float px, float py) const
    {
        switch (type)
        {
        case ZT_RECT:
            return px >= x1 && px <= x2 && py >= y1 && py <= y2;

        case ZT_POLYGON:
        case ZT_FREEHAND:
        {
            if (vertices.size() < 3) return false;
            // Ray casting algorithm
            bool inside = false;
            int n = (int)vertices.size();
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                float xi = vertices[i].x, yi = vertices[i].y;
                float xj = vertices[j].x, yj = vertices[j].y;
                if (((yi > py) != (yj > py)) &&
                    (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
                    inside = !inside;
            }
            return inside;
        }

        case ZT_CIRCLE:
        {
            float dx = px - cx, dy = py - cy;
            return (dx * dx + dy * dy) <= (radius * radius);
        }

        case ZT_SECTOR:
        {
            float dx = px - sx, dy = py - sy;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist > sRadius) return false;
            float angle = atan2f(dy, dx);
            // Chuan hoa goc vao [0, 2*PI)
            while (angle < 0) angle += 2.0f * PI_F;
            while (angle >= 2.0f * PI_F) angle -= 2.0f * PI_F;
            float aStart = sAngleStart, aEnd = sAngleEnd;
            while (aStart < 0) aStart += 2.0f * PI_F;
            while (aEnd < 0) aEnd += 2.0f * PI_F;
            if (aStart <= aEnd)
                return angle >= aStart && angle <= aEnd;
            else
                return angle >= aStart || angle <= aEnd;
        }

        default:
            return false;
        }
    }
};

static std::vector<Zone> gZones;
static std::mutex gZoneMutex;

// ================= 16. IFrameSource =================
class IFrameSource
{
public:
    virtual ~IFrameSource() {}
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;
    virtual FramePtr GetFrameAtOffset(int offset) = 0;
    virtual void SetExtr(const Extrinsic3D& e) = 0;
    virtual Extrinsic3D GetExtr() = 0;
    virtual std::string GetName() const = 0;
    virtual ILadar* GetLadar() { return nullptr; }
    virtual void SetEnableCsvRecord(bool e) { (void)e; }
    virtual void SetPaused(bool p) { (void)p; }
    virtual void SetName(const std::string& n) { (void)n; }   // >>> v3.1: doi ten <<<
    // >>> v3: ho tro reconnect voi IP moi <<<
    virtual bool Reconnect(const std::string& newIp, uint16_t newPort) { return false; }
    virtual std::string GetIp() const { return ""; }
    virtual uint16_t GetPort() const { return 0; }
};

// ================= 17. CSV PLAYER =================
class CsvLidarPlayer : public IFrameSource
{
    std::string csvPath, name;
    std::string csvHeader;
    std::ifstream csvFile;
    std::thread playerThread;
    std::atomic<bool> isRunning{ false }, isPaused{ false };
    FramePtr building = std::make_shared<PointCloudData>();
    FramePtr latest;
    std::deque<FramePtr> history;
    std::mutex dataMutex, extrMutex;
    Extrinsic3D extr;
    int sourceId = 0;
public:
    CsvLidarPlayer(int id, const std::string& n, const std::string& p, const Extrinsic3D& e = Extrinsic3D())
        : name(n), csvPath(p), extr(e), sourceId(id) {}
    ~CsvLidarPlayer() { Stop(); }
    std::string GetName() const override { return name; }
    void SetName(const std::string& n) override { name = SanitizeName(n); }
    void SetExtr(const Extrinsic3D& e) override { std::lock_guard<std::mutex> lk(extrMutex); extr = e; }
    Extrinsic3D GetExtr() override { std::lock_guard<std::mutex> lk(extrMutex); return extr; }
    void SetPaused(bool p) override { isPaused = p; }
    bool Start() override
    {
        if (isRunning.load()) return true;
        csvFile.open(csvPath);
        if (!csvFile.is_open()) { LogPrintf("[ERROR] Cannot open CSV: %s\n", csvPath.c_str()); return false; }
        std::getline(csvFile, csvHeader);
        isRunning = true;
        playerThread = std::thread(&CsvLidarPlayer::PlayerLoop, this);
        LogPrintf("[OK] CSV Started: %s\n", csvPath.c_str());
        return true;
    }
    void Stop() override
    {
        if (!isRunning.load()) return;
        isRunning = false;
        if (playerThread.joinable()) playerThread.join();
        if (csvFile.is_open()) csvFile.close();
    }
    bool IsRunning() const override { return isRunning.load(); }
    FramePtr GetFrameAtOffset(int o) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (o <= 0 || history.empty()) return latest;
        int idx = (int)history.size() - 1 - o;
        if (idx < 0) idx = 0;
        return history[idx];
    }
private:
    void PlayerLoop()
    {
        int lastFrameId = -1, frameSeq = 0, replayed = 0;
        auto startTime = std::chrono::high_resolution_clock::now();
        Extrinsic3D curExtr; Transform3D transform(curExtr);
        while (isRunning.load())
        {
            if (isPaused.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
            std::string line;
            if (!std::getline(csvFile, line))
            {
                csvFile.clear(); csvFile.seekg(0);
                std::getline(csvFile, line);
                startTime = std::chrono::high_resolution_clock::now();
                replayed = 0; lastFrameId = -1; continue;
            }
            std::stringstream ss(line); std::string token;
            std::vector<std::string> tok;
            while (std::getline(ss, token, ',')) tok.push_back(token);

            // >>> CAP NHAT:
            //     Ho tro 4 format CSV:
            //       10 cot = cu, khong Source_ID, khong Local_Z
            //       11 cot = cu, co Source_ID, khong Local_Z
            //       11 cot = moi, khong Source_ID, co Local_Z
            //       12 cot = moi, co Source_ID, co Local_Z
            //
            //     Vi format 11 cot bi mo ho, uu tien nhan dien theo header
            //     (Start() da doc header) va fallback theo so cot.
            bool hasSrc = false;
            bool hasLocalZ = false;

            try
            {
                // Header CSV duoc luu rieng de phan biet 11-cot old/new.
                // Neu header khong co (file cu/khac), dung fallback an toan.
                // Chuoi header duoc cache trong member csvHeader.
                if (!csvHeader.empty()) {
                    hasSrc = (csvHeader.find("Source_ID") != std::string::npos);
                    hasLocalZ = (csvHeader.find("Local_Z") != std::string::npos);
                }
                else {
                    if (tok.size() == 12) {
                        hasSrc = true; hasLocalZ = true;
                    }
                    else if (tok.size() == 11) {
                        // Ambiguous: assume new no-source format only if Local_Z
                        // can be parsed consistently; otherwise use old source format.
                        hasSrc = false; hasLocalZ = true;
                    }
                    else {
                        hasSrc = (tok.size() >= 11);
                        hasLocalZ = false;
                    }
                }

                int base = hasSrc ? 1 : 0;
                int required = base + (hasLocalZ ? 11 : 10);
                if ((int)tok.size() < required) continue;

                int frameId = std::stoi(tok[base + 0]);
                uint32_t tsS = (uint32_t)std::stoul(tok[base + 1]);
                uint32_t tsU = (uint32_t)std::stoul(tok[base + 2]);
                float angleDeg = std::stof(tok[base + 4]);
                float distance = std::stof(tok[base + 5]);
                float x = std::stof(tok[base + 6]);
                float y = std::stof(tok[base + 7]);

                float lz = 0.0f;
                float inten = 0.0f;
                float scanTime = 0.0f;

                if (hasLocalZ) {
                    lz = std::stof(tok[base + 8]);
                    inten = std::stof(tok[base + 9]);
                    scanTime = std::stof(tok[base + 10]);
                }
                else {
                    inten = std::stof(tok[base + 8]);
                    scanTime = std::stof(tok[base + 9]);
                }

                if (frameId != lastFrameId)
                {
                    if (lastFrameId >= 0)
                    {
                        auto expT = startTime + std::chrono::milliseconds(replayed * 40);
                        auto nowT = std::chrono::high_resolution_clock::now();
                        if (expT > nowT) std::this_thread::sleep_until(expT);
                    }
                    lastFrameId = frameId; replayed++;
                    std::lock_guard<std::mutex> lock(dataMutex);
                    int prev = (int)building->x_coords.size();
                    if (prev > 0)
                    {
                        building->frame_id = frameSeq++;
                        history.push_back(building);
                        if (history.size() > MAX_HISTORY_FRAMES) history.pop_front();
                        latest = building;
                    }
                    FramePtr f = std::make_shared<PointCloudData>();
                    f->source_id = sourceId; f->ts_secs = tsS; f->ts_usecs = tsU;
                    curExtr = GetExtr(); transform = Transform3D(curExtr);
                    f->origin_x = curExtr.x; f->origin_y = curExtr.y; f->origin_yaw = curExtr.yaw;
                    f->origin_z = curExtr.z; f->origin_roll = curExtr.roll; f->origin_pitch = curExtr.pitch;
                    f->range_max = 80.0f;
                    f->angle_min = Deg2Rad(-45.0f); f->angle_max = Deg2Rad(225.0f);
                    f->angle_increment = Deg2Rad(0.0625f);
                    f->scan_time = scanTime;
                    f->time_increment = prev ? scanTime / (float)prev : 0;
                    f->wave_count = 1; f->has_param = true;
                    building = f;
                    gNewDataAvailable = true;
                }
                std::lock_guard<std::mutex> lock(dataMutex);
                building->local_x.push_back(x);
                building->local_y.push_back(y);
                building->local_z.push_back(lz); // <<< THEM: Luu local_z tu CSV

                const auto world = transform.Apply(x, y);
                building->x_coords.push_back(world.x);
                building->y_coords.push_back(world.y);
                building->z_coords.push_back(world.z);

                building->intensities.push_back(inten);
                building->angles_deg.push_back(angleDeg);
                building->distances.push_back(distance);
            }
            catch (...) { continue; }
        }
    }
};

// ================= 18. REAL LIDAR =================
class RealLidarSource;
static void PrintLidarInfo(int sourceId, const char* name, ILadar* ladar_, const ladar_data_t& data, int curId);
static std::map<ILadar*, RealLidarSource*> gLidarMap;
static std::mutex gLidarMapMutex;
std::ofstream gCsvFile;
std::mutex gCsvMutex;
std::atomic<bool> gIsRecording(false);

class RealLidarSource : public IFrameSource
{
    std::string name, ip;
    uint16_t port;
    int sourceId;
    ILadar* ladar = nullptr;
    ILadarPoller* poller = nullptr;
    std::thread pollerThread;
    std::thread syncThread;
    std::atomic<bool> isRunning{ false };
    FramePtr latest;
    std::deque<FramePtr> history;
    std::mutex dataMutex, extrMutex;
    Extrinsic3D extr;
    std::atomic<int> frameCount{ 0 };
public:
    bool enableCsvRecord = true;

    RealLidarSource(int id, const std::string& n, const std::string& ip_, uint16_t p, const Extrinsic3D& e = Extrinsic3D())
        : name(n), ip(ip_), port(p), sourceId(id), extr(e) {}
    ~RealLidarSource() { Stop(); }

    std::string GetName() const override { return name; }
    void SetName(const std::string& n) override { name = SanitizeName(n); }
    std::string GetIp() const override { return ip; }
    uint16_t GetPort() const override { return port; }
    void SetExtr(const Extrinsic3D& e) override { std::lock_guard<std::mutex> lk(extrMutex); extr = e; }
    Extrinsic3D GetExtr() override { std::lock_guard<std::mutex> lk(extrMutex); return extr; }
    ILadar* GetLadar() override { return ladar; }
    void SetEnableCsvRecord(bool e) override { enableCsvRecord = e; }
    bool IsRunning() const override { return isRunning.load(); }

    FramePtr GetFrameAtOffset(int o) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (o <= 0 || history.empty()) return latest;
        int idx = (int)history.size() - 1 - o;
        if (idx < 0) idx = 0;
        return history[idx];
    }

    // >>> v3: Reconnect voi IP/port moi <<<
    bool Reconnect(const std::string& newIp, uint16_t newPort) override
    {
        LogPrintf("[IP] %s: Reconnect %s:%d -> %s:%d\n", name.c_str(), ip.c_str(), port, newIp.c_str(), newPort);
        Stop();
        ip = newIp;
        port = newPort;
        frameCount = 0;
        gGotFirstFrame = false;
        gConnectUs = NowUs();
        return Start();
    }

    static void OnFrameCallback(ILadar* ladar_, const uint8_t* frame, int frameLength)
    {
        const long long backendArrivalUs = NowUs();
        RealLidarSource* self = nullptr;
        {
            std::lock_guard<std::mutex> lock(gLidarMapMutex);
            auto it = gLidarMap.find(ladar_);
            if (it == gLidarMap.end()) return;
            self = it->second;
        }
        uint16_t cmdWord = (uint16_t)(frame[2] | (frame[3] << 8));
        CheckCommandResponse(cmdWord);
        if (gIsPaused.load()) return;
        if (cmdWord != 110) return;
        ladar_data_t data;
        if (ladar_->GetData(data, frame, frameLength) <= 0) return;
        if (self->sourceId == 0) NoteFrameArrival();
        int curId = self->frameCount.load();
        PrintLidarInfo(self->sourceId, self->name.c_str(), ladar_, data, curId);
        if (self->sourceId == 0 && gRmseActive)
        {
            int idx = GetPointIndexFromAngle(0.0f);
            if (idx >= 0 && idx < (int)data.ranges.size())
            {
                float d = data.ranges[idx];
                if (d > 0 && d < data.range_max)
                {
                    std::lock_guard<std::mutex> lk(gRmseMutex);
                    double err = (double)d - (double)gRmseTrue;
                    gRmseSum += err; gRmseSumSq += err * err; gRmseCount++;
                    if (gRmseCount >= gRmseN)
                    {
                        double mean = gRmseSum / gRmseCount;
                        double rmse = sqrt(gRmseSumSq / gRmseCount);
                        double var = gRmseSumSq / gRmseCount - mean * mean;
                        if (var < 0) var = 0;
                        LogPrintf("[RMSE] N=%d | TB=%.3f | RMSE=%.3f | std=%.3f | true=%.3f\n",
                            gRmseCount, mean, rmse, sqrt(var), gRmseTrue);
                        json r; r["type"] = "rmse"; r["mean"] = mean; r["rmse"] = rmse;
                        r["std"] = sqrt(var); r["true"] = gRmseTrue;
                        QueueWebJson(r);
                        gRmseActive = false;
                    }
                }
            }
        }
        FramePtr f = std::make_shared<PointCloudData>();
        Extrinsic3D e = self->GetExtr();
        Transform3D transform(e);
        f->source_id = self->sourceId; f->frame_id = curId;
        f->backend_arrival_steady_us = backendArrivalUs;
        f->range_min = data.range_min; f->range_max = data.range_max;
        f->angle_min = data.angle_min; f->angle_max = data.angle_max;
        f->angle_increment = data.angle_increment;
        f->scan_time = data.scan_time; f->time_increment = data.time_increment;
        f->wave_count = data.wave_count;
        f->ts_secs = data.ts_secs; f->ts_usecs = data.ts_usecs;
        f->source_timestamp_valid = (data.flags & dsTimeStamp) != 0;
        f->origin_x = e.x; f->origin_y = e.y; f->origin_yaw = e.yaw;
        f->origin_z = e.z; f->origin_roll = e.roll; f->origin_pitch = e.pitch;
        f->has_param = true;
        float angle = data.angle_min;
        for (size_t i = 0; i < data.ranges.size(); i++)
        {
            float d = data.ranges[i];
            if (d > 0.0f && d < data.range_max)
            {
                float lx = d * cosf(angle), ly = d * sinf(angle);
                f->local_x.push_back(lx); f->local_y.push_back(ly);
                f->local_z.push_back(0); // TR70 2D: diem do nam tren mat phang local Z=0; e.z la do cao cam bien
                const auto world = transform.Apply(lx, ly);
                f->x_coords.push_back(world.x); f->y_coords.push_back(world.y); f->z_coords.push_back(world.z);
                f->angles_deg.push_back(angle * 180.0f / PI_F);
                f->distances.push_back(d);
                f->intensities.push_back(i < data.intensities.size() ? data.intensities[i] : 0.0f);
            }
            angle += data.angle_increment;
        }
        {
            std::lock_guard<std::mutex> lock(self->dataMutex);
            self->history.push_back(f);
            if (self->history.size() > MAX_HISTORY_FRAMES) self->history.pop_front();
            self->latest = f;
        }
        if (gIsRecording.load() && self->enableCsvRecord)
        {
            std::lock_guard<std::mutex> lk(gCsvMutex);
            if (gCsvFile.is_open())
            {
                float a2 = data.angle_min;
                for (size_t i = 0; i < data.ranges.size(); i++)
                {
                    if (data.ranges[i] > 0 && data.ranges[i] < data.range_max)
                    {
                        float lx = data.ranges[i] * cosf(a2), ly = data.ranges[i] * sinf(a2);
                        float inten = i < data.intensities.size() ? data.intensities[i] : 0.0f;
                        gCsvFile << (self->sourceId + 1) << "," << curId << ","
                            << data.ts_secs << "," << data.ts_usecs << "," << i << ","
                            << std::fixed << std::setprecision(4) << (a2 * 180.0f / PI_F) << ","
                            << data.ranges[i] << "," << lx << "," << ly << ","
                            << std::setprecision(4) << e.z << ","   // <<< THEM: Local_Z (do cao dz)
                            << std::setprecision(2) << inten << ","
                            << std::setprecision(4) << data.scan_time << "\n";
                    }
                    a2 += data.angle_increment;
                }
            }
        }
        self->frameCount++;
        gNewDataAvailable = true;
    }

    static void OnStatusCallback(ILadar* ladar_, uint32_t, uint32_t status)
    {
        RealLidarSource* self = nullptr;
        {
            std::lock_guard<std::mutex> lock(gLidarMapMutex);
            auto it = gLidarMap.find(ladar_);
            if (it == gLidarMap.end()) return;
            self = it->second;
        }
        char sNow[64]; GetNowString(sNow, sizeof(sNow));
        const char* ss[] = { "closed", "connecting...", "connected", "connect error" };
        LogPrintf("[%s] %s: %s\n", sNow, self->name.c_str(), status <= 3 ? ss[status] : "unknown");
        if (status == csConnected)
        {
            int ok = 0;
            for (int i = 0; i < 5 && !(ok = (ladar_->SetTime() > 0)); i++) Sleep(300);
            LogPrintf("[DO TRE] SetTime %s\n", ok ? "OK" : "FAIL");
        }
    }

    bool Start() override
    {
        if (isRunning.load()) return true;
        poller = CreateLadarPoller();
        if (!poller) return false;
        ladar = poller->AddLadar(ip.c_str(), port, OnFrameCallback, OnStatusCallback);
        if (!ladar) { DeleteLadarPoller(poller); poller = nullptr; return false; }
        { std::lock_guard<std::mutex> lock(gLidarMapMutex); gLidarMap[ladar] = this; }
        isRunning = true;
        pollerThread = std::thread([this]() { poller->Run(); });
        syncThread = std::thread([this]() {
            int tick = 0;
            while (isRunning.load()) {
                Sleep(1000); if (++tick < 60) continue;
                tick = 0;
                if (ladar && ladar->IsConnected()) {
                    int ok = ladar->SetTime() > 0;
                    LogPrintf("[DO TRE] %s: SetTime dinh ky %s\n", name.c_str(), ok ? "OK" : "FAIL");
                }
            }
            });
        LogPrintf("[OK] LiDAR %s:%d (%s)\n", ip.c_str(), port, name.c_str());
        return true;
    }

    void Stop() override
    {
        if (!isRunning.load()) return;
        isRunning = false;
        if (syncThread.joinable()) syncThread.join();
        if (ladar) { std::lock_guard<std::mutex> lock(gLidarMapMutex); gLidarMap.erase(ladar); }
        if (poller)
        {
            poller->Stop();
            if (pollerThread.joinable()) pollerThread.join();
            DeleteLadarPoller(poller); poller = nullptr;
        }
        ladar = nullptr;
    }
};

// >>> PrintLidarInfo (giu nguyen) <<<
static void PrintLidarInfo(int sourceId, const char* name, ILadar* ladar_, const ladar_data_t& data, int curId)
{
    int period = (sourceId == 0) ? 50 : 200;
    if (curId % period != 0) return;
    char sNow[64]; GetNowString(sNow, sizeof(sNow));
    LogPrintf("\n");
    LogPrintf("+--------------------------------------------------------------+\n");
    LogPrintf("| [%s] LiDAR %d - %s | Frame #%d\n", sNow, sourceId + 1, name, curId);
    LogPrintf("+--------------------------------------------------------------+\n");
    LogPrintf("| [SCAN]\n");
    LogPrintf("|   Timestamp    : %u.%06u s\n", data.ts_secs, data.ts_usecs);
    LogPrintf("|   Scan time    : %.4f s\n", data.scan_time);
    LogPrintf("|   Time incr    : %.6f s\n", data.time_increment);
    LogPrintf("|   Wave count   : %d\n", data.wave_count);
    LogPrintf("|   Points/frame : %zu\n", data.ranges.size());
    LogPrintf("| [GEOMETRY]\n");
    LogPrintf("|   Angle range  : %7.2f -> %7.2f deg (incr %.4f deg)\n",
        Rad2Deg(data.angle_min), Rad2Deg(data.angle_max), Rad2Deg(data.angle_increment));
    LogPrintf("|   Range        : %5.2f m -> %6.2f m\n", data.range_min, data.range_max);
    LogPrintf("|   Flags        : 0x%02X (%s%s%s%s%s)\n", data.flags,
        (data.flags & dsSyncInfo) ? "SyncInfo " : "",
        (data.flags & dsTimeStamp) ? "TimeStamp " : "",
        (data.flags & dsLogin) ? "Login " : "",
        (data.flags & dsParam) ? "Param " : "",
        (data.flags & dsDataFmt) ? "DataFmt " : "");
    ladar_param_t param;
    ladar_->GetParam(param);
    LogPrintf("| [ladar_param_t]\n");
    LogPrintf("|   flags        : 0x%08X\n", param.flags);
    if (param.flags & dsParam)
    {
        LogPrintf("|   radius       : %.1f m\n", param.radius);
        LogPrintf("|   ticks        : %.4f s (chu ky quet)\n", param.ticks);
    }
    if (param.flags & dsDataFmt)
    {
        LogPrintf("|   angle_start  : %.2f deg\n", Rad2Deg(param.angle_start));
        LogPrintf("|   angle_stop   : %.2f deg\n", Rad2Deg(param.angle_stop));
        LogPrintf("|   angle_step   : %.4f deg\n", Rad2Deg(param.angle_step));
        LogPrintf("|   wave_count   : %d\n", param.wave_count);
        LogPrintf("|   wave_types   : [%u, %u]\n", param.wave_types[0], param.wave_types[1]);
        LogPrintf("|   intensity    : %s\n", param.intensity ? "YES" : "NO");
    }
    LogPrintf("| [SAMPLE 7 GOC]\n");
    LogPrintf("|   %-7s | %-8s | %-10s | %-9s | %-5s\n",
        "Goc", "Pt_Idx", "Distance", "Intensity", "HopLe");
    LogPrintf("|   --------+----------+------------+-----------+------\n");
    const float TEST_ANGLES[7] = { -45.0f, 0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f };
    for (int i = 0; i < 7; i++)
    {
        float a = TEST_ANGLES[i];
        int idx = GetPointIndexFromAngle(a);
        if (idx >= 0 && idx < (int)data.ranges.size())
        {
            float dist = data.ranges[idx];
            float inten = (!data.intensities.empty() && idx < (int)data.intensities.size())
                ? data.intensities[idx] : 0.0f;
            bool valid = (dist > 0.0f && dist < data.range_max);
            LogPrintf("|   %6.2f | %8d | %9.4f m | %9.2f | %-5s\n",
                a, idx, dist, inten, valid ? "CO" : "KHONG");
        }
        else
            LogPrintf("|   %6.2f | %-8s | %-10s | %-9s | %-5s\n", a, "N/A", "N/A", "N/A", "OUT");
    }
    LogPrintf("+--------------------------------------------------------------+\n\n");
}

// ================= 19. SOURCES =================
std::vector<std::shared_ptr<IFrameSource>> gSources;
static std::recursive_mutex gSourcesMutex;

// >>> v3: Cau hinh LiDAR dong <<<
struct LidarConfigEntry
{
    std::string ip;
    uint16_t port = 1112;
    std::string name;
};

static void SaveLidarConfig()
{
    FILE* f = fopen(LIDAR_CFG_FILE, "w");
    if (!f) return;
    for (size_t i = 0; i < gSources.size(); i++)
    {
        auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[i]);
        if (src)
            fprintf(f, "%s %d %s\n", src->GetIp().c_str(), src->GetPort(), src->GetName().c_str());
        else
            fprintf(f, "csv 0 %s\n", gSources[i]->GetName().c_str());
    }
    fclose(f);
    LogPrintf("[CFG] Da luu %s (%d LiDAR)\n", LIDAR_CFG_FILE, (int)gSources.size());
    QueueWebJson({ {"type","log"},{"text","[CFG] Da luu lidar_config.cfg"} });
}

static bool LoadLidarConfig(std::vector<LidarConfigEntry>& entries)
{
    FILE* f = fopen(LIDAR_CFG_FILE, "r");
    if (!f) return false;
    char ipBuf[64], nameBuf[64];
    int port;
    while (fscanf(f, "%63s %d %63s", ipBuf, &port, nameBuf) == 3)
    {
        LidarConfigEntry e;
        e.ip = ipBuf; e.port = (uint16_t)port; e.name = nameBuf;
        entries.push_back(e);
    }
    fclose(f);
    return !entries.empty();
}

static void BroadcastLidarConfig()
{
    json arr = json::array();
    for (size_t i = 0; i < gSources.size(); i++)
    {
        auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[i]);
        json item;
        item["id"] = (int)i;
        item["name"] = gSources[i]->GetName();
        if (src)
        {
            item["ip"] = src->GetIp();
            item["port"] = src->GetPort();
            item["connected"] = src->GetLadar() && src->GetLadar()->IsConnected();
        }
        else
        {
            item["ip"] = "csv";
            item["port"] = 0;
            item["connected"] = gSources[i]->IsRunning();
        }
        Extrinsic3D e = gSources[i]->GetExtr();
        item["dx"] = e.x; item["dy"] = e.y; item["thetaDeg"] = Rad2Deg(e.yaw);
        item["x"] = e.x; item["y"] = e.y; item["z"] = e.z; item["roll"] = Rad2Deg(e.roll); item["pitch"] = Rad2Deg(e.pitch); item["yaw"] = Rad2Deg(e.yaw);
        arr.push_back(item);
    }
    json out; out["type"] = "lidar_config"; out["lidars"] = arr; out["count"] = (int)gSources.size();
    QueueWebJson(out);
}

// ================= WEB CORE =================
static WsServer g_server;
static std::set<ConnHdl, std::owner_less<ConnHdl>> g_conns;
static std::mutex g_connMutex;
static std::mutex gWebJsonMutex;
static std::vector<std::string> gWebJsonQueue;

static void appendU8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }
static void appendU16(std::vector<uint8_t>& b, uint16_t v)
{
    b.push_back(v & 0xFF); b.push_back((v >> 8) & 0xFF);
}
static void appendU32(std::vector<uint8_t>& b, uint32_t v)
{
    b.push_back(v & 0xFF); b.push_back((v >> 8) & 0xFF); b.push_back((v >> 16) & 0xFF); b.push_back((v >> 24) & 0xFF);
}
static void appendF32(std::vector<uint8_t>& b, float v)
{
    static_assert(sizeof(float) == 4, "Protocol requires IEEE-754 float32");
    uint32_t bits; memcpy(&bits, &v, 4); appendU32(b, bits);
}

static bool HasClients() { std::lock_guard<std::mutex> lk(g_connMutex); return !g_conns.empty(); }
static void BroadcastText(const std::string& s)
{
    std::lock_guard<std::mutex> lk(g_connMutex);
    for (auto h : g_conns)
    {
        websocketpp::lib::error_code ec; g_server.send(h, s, websocketpp::frame::opcode::value::text, ec);
    }
}
static void BroadcastBinary(const std::vector<uint8_t>& p)
{
    if (p.empty()) return;
    std::lock_guard<std::mutex> lk(g_connMutex);
    for (auto h : g_conns)
    {
        websocketpp::lib::error_code ec; g_server.send(h, p.data(), p.size(), websocketpp::frame::opcode::value::binary, ec);
    }
}
static void QueueWebJson(const json& j)
{
    std::lock_guard<std::mutex> lk(gWebJsonMutex); gWebJsonQueue.push_back(j.dump());
}
static void DrainWebJson()
{
    std::vector<std::string> out;
    { std::lock_guard<std::mutex> lk(gWebJsonMutex); out.swap(gWebJsonQueue); }
    for (auto& s : out) BroadcastText(s);
}
static void BroadcastExtr(int id)
{
    if (id < 0 || id >= (int)gSources.size()) return;
    Extrinsic3D e = gSources[id]->GetExtr();
    json j; j["type"] = "extr"; j["id"] = id;
    j["dx"] = e.x; j["dy"] = e.y; j["thetaDeg"] = Rad2Deg(e.yaw);
    j["x"] = e.x; j["y"] = e.y; j["z"] = e.z; j["roll"] = Rad2Deg(e.roll); j["pitch"] = Rad2Deg(e.pitch); j["yaw"] = Rad2Deg(e.yaw);
    QueueWebJson(j);
}

// ================= 20. EXTRINSIC HELPERS =================
IFrameSource* GetSelectedSource()
{
    if (gSelectedLidar >= 0 && gSelectedLidar < (int)gSources.size())
        return gSources[gSelectedLidar].get();
    return nullptr;
}

void SaveExtrConfig()
{
    FILE* f = fopen(EXTR_CFG, "w");
    if (!f) return;
    for (int i = 0; i < (int)gSources.size(); i++)
    {
        Extrinsic3D e = gSources[i]->GetExtr();
        fprintf(f, "%.4f %.4f %.4f %.4f %.4f %.4f\n", e.x, e.y, e.z, Rad2Deg(e.roll), Rad2Deg(e.pitch), Rad2Deg(e.yaw));
    }
    fclose(f);
    LogPrintf("[CFG] Da luu %s (%d dong)\n", EXTR_CFG, (int)gSources.size());
    QueueWebJson({ {"type","log"},{"text","[CFG] Da luu extrinsics.cfg"} });
}

// ================= MAP EXPORT & SAVE =================
static std::vector<uint8_t> Base64Decode(const std::string& in)
{
    static const int B64Index[256] = {
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,
        52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,
        -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
        15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,
        -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
        41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
    };
    std::vector<uint8_t> out;
    size_t start = 0;
    size_t comma = in.find(',');
    if (comma != std::string::npos) start = comma + 1;

    int val = 0, valb = -8;
    for (size_t i = start; i < in.size(); i++)
    {
        uint8_t c = (uint8_t)in[i];
        if (c == '=') break;
        int d = B64Index[c];
        if (d < 0) continue;
        val = (val << 6) + d;
        valb += 6;
        if (valb >= 0)
        {
            out.push_back((uint8_t)((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return out;
}

static void SaveMapFiles(const std::string& name, const std::string& yamlStr, const std::string& jsonStr, const std::string& pngB64)
{
    try {
        std::filesystem::create_directories("maps");
        std::string baseName = name.empty() ? "map_latest" : name;

        // Luu ban do chinh: map_latest
        if (!yamlStr.empty()) {
            std::ofstream f("maps/" + baseName + ".yaml");
            if (f.is_open()) f << yamlStr;
        }
        if (!jsonStr.empty()) {
            std::ofstream f("maps/" + baseName + ".json");
            if (f.is_open()) f << jsonStr;
        }
        if (!pngB64.empty()) {
            auto bytes = Base64Decode(pngB64);
            if (!bytes.empty()) {
                std::ofstream f("maps/" + baseName + ".png", std::ios::binary);
                if (f.is_open()) f.write((char*)bytes.data(), bytes.size());
            }
        }

        // Luu kem ban sao luu co dau thoi gian (Timestamp)
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm tmNow;
        localtime_s(&tmNow, &now);
        char tsBuf[64];
        strftime(tsBuf, sizeof(tsBuf), "%Y%m%d_%H%M%S", &tmNow);
        std::string tsName = "map_" + std::string(tsBuf);

        if (!yamlStr.empty()) {
            std::ofstream f("maps/" + tsName + ".yaml");
            if (f.is_open()) f << yamlStr;
        }
        if (!jsonStr.empty()) {
            std::ofstream f("maps/" + tsName + ".json");
            if (f.is_open()) f << jsonStr;
        }
        if (!pngB64.empty()) {
            auto bytes = Base64Decode(pngB64);
            if (!bytes.empty()) {
                std::ofstream f("maps/" + tsName + ".png", std::ios::binary);
                if (f.is_open()) f.write((char*)bytes.data(), bytes.size());
            }
        }

        LogPrintf("[MAP] Da luu ban do vao thu muc maps/%s (.png, .yaml, .json)\n", baseName.c_str());
        QueueWebJson({ {"type","log"},{"text","[MAP] ✔ Da luu ban do vao maps/" + baseName + " (.png, .yaml, .json) tren may chu"} });
    }
    catch (const std::exception& e) {
        LogPrintf("[MAP ERROR] Loi khi luu ban do: %s\n", e.what());
    }
}

bool LoadExtrConfig(std::vector<Extrinsic3D>& extrs)
{
    std::ifstream f(EXTR_CFG);
    if (!f) return false;
    auto loaded = extrs;
    std::string line; size_t i = 0;
    while (i < loaded.size() && std::getline(f, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos || line[line.find_first_not_of(" \t\r")] == '#') continue;
        if (!ParseExtrinsic(line, loaded[i++])) return false;
    }
    if (i != loaded.size()) return false;
    extrs = loaded; return true;
}

void AdjustSelectedExtr(float ddx, float ddy, float dthRad)
{
    IFrameSource* src = GetSelectedSource();
    if (!src) return;
    Extrinsic3D e = src->GetExtr();
    e.x += ddx; e.y += ddy; e.yaw += dthRad;
    src->SetExtr(e);
    ResetMotionModel();
    LogPrintf("[EXTRINSIC %s] dx=%.2f dy=%.2f th=%.2f\n", src->GetName().c_str(), e.x, e.y, Rad2Deg(e.yaw));
    BroadcastExtr(gSelectedLidar);
}

void PrintSelectedExtr()
{
    IFrameSource* src = GetSelectedSource();
    if (!src) return;
    Extrinsic3D e = src->GetExtr();
    char b[240];
    sprintf_s(b, "[EXTRINSIC %s] { x=%.3f m, y=%.3f m, z=%.3f m, roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg }",
        src->GetName().c_str(), e.x, e.y, e.z, Rad2Deg(e.roll), Rad2Deg(e.pitch), Rad2Deg(e.yaw));
    LogPrintf("%s\n", b);
    QueueWebJson({ {"type","log"},{"text",b} });
}

void ApplyPickCalib()
{
    if (gSelectedLidar == 0) { LogPrintf("[PICK] Chon L2..L4.\n"); return; }
    Ptf v1a = gPickRef[0], v1b = gPickRef[1], v2a = gPickTarget[0], v2b = gPickTarget[1];
    float a1 = atan2f(v1b.y - v1a.y, v1b.x - v1a.x);
    float a2 = atan2f(v2b.y - v2a.y, v2b.x - v2a.x);
    float th = a1 - a2, c = cosf(th), s = sinf(th);
    float tx = v1a.x - (c * v2a.x - s * v2a.y);
    float ty = v1a.y - (s * v2a.x + c * v2a.y);
    IFrameSource* src = GetSelectedSource();
    if (!src) return;
    Extrinsic3D e = src->GetExtr(); e.x = tx; e.y = ty; e.yaw = th;
    src->SetExtr(e);
    ResetMotionModel();
    LogPrintf("[CALIB PICK] %s dx=%.3f dy=%.3f th=%.2f\n", src->GetName().c_str(), tx, ty, Rad2Deg(th));
    BroadcastExtr(gSelectedLidar);
    gPickNRef = 0; gPickNTarget = 0;
}

void RunIcpRefine();
void RunGlobalAutoAlign(int targetId);

// ================= SPATIAL HASH & POINT-TO-LINE PCA ICP =================
struct WallPoint
{
    Ptf pt;
    Ptf normal;
    float linearity = 0.0f; // 0 = scatter/corner, 1 = straight wall
};

struct SpatialGrid2D
{
    float minX = 0, minY = 0, res = 0.35f;
    int nx = 0, ny = 0;
    std::vector<std::vector<int>> buckets;
    std::vector<WallPoint> wallPts;

    void Build(const std::vector<Ptf>& pts, float cellSize = 0.35f)
    {
        res = cellSize;
        if (pts.empty()) return;
        minX = 1e9f; float maxX = -1e9f;
        minY = 1e9f; float maxY = -1e9f;
        for (const auto& p : pts)
        {
            minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
        }
        minX -= 1.0f; minY -= 1.0f;
        maxX += 1.0f; maxY += 1.0f;
        nx = std::max(1, (int)std::ceil((maxX - minX) / res));
        ny = std::max(1, (int)std::ceil((maxY - minY) / res));
        buckets.assign(nx * ny, {});
        for (int i = 0; i < (int)pts.size(); i++)
        {
            int gx = (int)std::floor((pts[i].x - minX) / res);
            int gy = (int)std::floor((pts[i].y - minY) / res);
            if (gx >= 0 && gx < nx && gy >= 0 && gy < ny)
                buckets[gy * nx + gx].push_back(i);
        }

        // Tinh toan PCA Wall Normal doc lap cho tung diem tren T (khong bi lech theo diem P)
        wallPts.resize(pts.size());
        for (size_t i = 0; i < pts.size(); i++)
        {
            wallPts[i].pt = pts[i];
            wallPts[i].normal = { 0, 0 };
            wallPts[i].linearity = 0;

            float px = pts[i].x, py = pts[i].y;
            int gx = (int)std::floor((px - minX) / res);
            int gy = (int)std::floor((py - minY) / res);

            double sumX = 0, sumY = 0;
            int count = 0;
            std::vector<int> neigh;
            for (int dy = -1; dy <= 1; dy++)
            {
                int cy = gy + dy;
                if (cy < 0 || cy >= ny) continue;
                for (int dx = -1; dx <= 1; dx++)
                {
                    int cx = gx + dx;
                    if (cx < 0 || cx >= nx) continue;
                    for (int idx : buckets[cy * nx + cx])
                    {
                        float d2 = (px - pts[idx].x) * (px - pts[idx].x) + (py - pts[idx].y) * (py - pts[idx].y);
                        if (d2 <= 0.09f) // ban kinh 0.30m
                        {
                            sumX += pts[idx].x;
                            sumY += pts[idx].y;
                            count++;
                            neigh.push_back(idx);
                        }
                    }
                }
            }

            if (count >= 4)
            {
                double mx = sumX / count, my = sumY / count;
                double cxx = 0, cyy = 0, cxy = 0;
                for (int idx : neigh)
                {
                    double dx = pts[idx].x - mx;
                    double dy = pts[idx].y - my;
                    cxx += dx * dx;
                    cyy += dy * dy;
                    cxy += dx * dy;
                }
                cxx /= count; cyy /= count; cxy /= count;

                double trace = cxx + cyy;
                double disc = sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
                double l1 = (trace + disc) * 0.5;
                double l2 = (trace - disc) * 0.5;

                float lin = (l1 > 1e-6) ? (float)((l1 - l2) / l1) : 0.0f;
                if (lin >= 0.70f)
                {
                    double nx_val = -cxy;
                    double ny_val = cxx - l2;
                    double len = hypot(nx_val, ny_val);
                    if (len > 1e-6)
                    {
                        wallPts[i].normal = { (float)(nx_val / len), (float)(ny_val / len) };
                        wallPts[i].linearity = lin;
                    }
                }
            }
        }
    }

    int FindNearest(float px, float py, const std::vector<Ptf>& pts, float maxDist, float& outDistSq) const
    {
        int gx = (int)std::floor((px - minX) / res);
        int gy = (int)std::floor((py - minY) / res);
        int bestIdx = -1;
        float bestD2 = maxDist * maxDist;
        int rad = std::max(1, (int)std::ceil(maxDist / res));
        for (int dy = -rad; dy <= rad; dy++)
        {
            int cy = gy + dy;
            if (cy < 0 || cy >= ny) continue;
            for (int dx = -rad; dx <= rad; dx++)
            {
                int cx = gx + dx;
                if (cx < 0 || cx >= nx) continue;
                for (int idx : buckets[cy * nx + cx])
                {
                    float d2 = (px - pts[idx].x) * (px - pts[idx].x) + (py - pts[idx].y) * (py - pts[idx].y);
                    if (d2 < bestD2)
                    {
                        bestD2 = d2;
                        bestIdx = idx;
                    }
                }
            }
        }
        outDistSq = bestD2;
        return bestIdx;
    }
};

static std::pair<float, int> PointToLineIcpStep(
    const std::vector<Ptf>& T,
    const SpatialGrid2D& gridT,
    const std::vector<Ptf>& S,
    float& curX, float& curY, float& curYaw,
    float maxDist)
{
    float c = cosf(curYaw), s = sinf(curYaw);
    double A[3][3] = { 0 };
    double b[3] = { 0 };
    int inliers = 0;
    double sumDistSq = 0;

    for (const auto& sp : S)
    {
        float px = c * sp.x - s * sp.y + curX;
        float py = s * sp.x + c * sp.y + curY;

        float d2A = 0;
        int idxA = gridT.FindNearest(px, py, T, maxDist, d2A);
        if (idxA < 0) continue;

        const auto& wpA = gridT.wallPts[idxA];
        float dA = sqrtf(d2A);

        float nx = 0, ny = 0;
        float signedDist = 0;

        if (wpA.linearity >= 0.70f)
        {
            nx = wpA.normal.x;
            ny = wpA.normal.y;
            signedDist = (px - wpA.pt.x) * nx + (py - wpA.pt.y) * ny;
            if (signedDist < 0) {
                nx = -nx; ny = -ny; signedDist = -signedDist;
            }
        }
        else
        {
            if (dA < 1e-4f) continue;
            nx = (px - wpA.pt.x) / dA;
            ny = (py - wpA.pt.y) / dA;
            signedDist = dA;
        }

        if (signedDist > maxDist) continue;

        float cross = px * ny - py * nx;
        A[0][0] += nx * nx;
        A[0][1] += nx * ny;
        A[0][2] += nx * cross;
        A[1][1] += ny * ny;
        A[1][2] += ny * cross;
        A[2][2] += cross * cross;

        b[0] -= signedDist * nx;
        b[1] -= signedDist * ny;
        b[2] -= signedDist * cross;

        sumDistSq += signedDist * signedDist;
        inliers++;
    }

    if (inliers < 15) return { 999.0f, inliers };

    A[1][0] = A[0][1];
    A[2][0] = A[0][2];
    A[2][1] = A[1][2];

    A[0][0] += 1e-3 * A[0][0] + 1e-4;
    A[1][1] += 1e-3 * A[1][1] + 1e-4;
    A[2][2] += 1e-3 * A[2][2] + 1e-4;

    double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
        - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (std::abs(det) < 1e-12) return { (float)sqrt(sumDistSq / inliers), inliers };

    double invDet = 1.0 / det;
    double invA[3][3];
    invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    invA[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * invDet;
    invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    invA[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * invDet;
    invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    invA[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * invDet;
    invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    invA[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * invDet;
    invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

    float dtx = (float)(invA[0][0] * b[0] + invA[0][1] * b[1] + invA[0][2] * b[2]);
    float dty = (float)(invA[1][0] * b[0] + invA[1][1] * b[1] + invA[1][2] * b[2]);
    float dth = (float)(invA[2][0] * b[0] + invA[2][1] * b[1] + invA[2][2] * b[2]);

    dtx = std::max(-0.30f, std::min(0.30f, dtx));
    dty = std::max(-0.30f, std::min(0.30f, dty));
    dth = std::max(-0.12f, std::min(0.12f, dth));

    curX += dtx;
    curY += dty;
    curYaw += dth;
    while (curYaw > PI_F) curYaw -= 2.0f * PI_F;
    while (curYaw < -PI_F) curYaw += 2.0f * PI_F;

    return { (float)sqrt(sumDistSq / inliers), inliers };
}

static std::pair<float, int> RunPointToLineIcpLoop(
    const std::vector<Ptf>& T,
    const SpatialGrid2D& gridT,
    const std::vector<Ptf>& S,
    float& curX, float& curY, float& curYaw,
    int nIterations = 25)
{
    std::pair<float, int> res = { 999.0f, 0 };
    for (int iter = 0; iter < nIterations; iter++)
    {
        float maxD = (iter < 5) ? 0.60f : ((iter < 12) ? 0.25f : ((iter < 18) ? 0.12f : 0.06f));
        res = PointToLineIcpStep(T, gridT, S, curX, curY, curYaw, maxD);
    }
    return res;
}

void RunIcpRefine()
{
    if (gIcpRunning) return;
    if (gSelectedLidar == 0) { LogPrintf("[ICP] Chon L2..L4.\n"); return; }
    if (gSources.empty() || gSelectedLidar >= (int)gSources.size()) return;
    FramePtr f1, f2;
    Extrinsic3D e1, e2;
    {
        std::lock_guard<std::recursive_mutex> lk(gSourcesMutex);
        if (gSources[0] && gSources[0]->IsRunning()) {
            f1 = gSources[0]->GetFrameAtOffset(0);
            e1 = gSources[0]->GetExtr();
        }
        if (gSources[gSelectedLidar] && gSources[gSelectedLidar]->IsRunning()) {
            f2 = gSources[gSelectedLidar]->GetFrameAtOffset(0);
            e2 = gSources[gSelectedLidar]->GetExtr();
        }
    }
    if (!f1 || !f2 || f1->local_x.empty() || f2->local_x.empty()) {
        LogPrintf("[ICP] Khong du du lieu.\n");
        return;
    }
    gIcpRunning = true;

    // Xu ly hoan toan trong he toa do Local cua LiDAR 1 -> bat bien voi vi tri e1 cua LiDAR 1!
    std::vector<Ptf> T_local, S_local;
    for (size_t i = 0; i < f1->local_x.size(); i += 2) {
        float lx = f1->local_x[i], ly = f1->local_y[i];
        if (std::hypotf(lx, ly) > 0.15f) T_local.push_back({ lx, ly });
    }
    for (size_t i = 0; i < f2->local_x.size(); i += 2) {
        float lx = f2->local_x[i], ly = f2->local_y[i];
        if (std::hypotf(lx, ly) > 0.15f) S_local.push_back({ lx, ly });
    }

    SpatialGrid2D gridT;
    gridT.Build(T_local, 0.35f);

    // Chuyen vi tri hien tai cua LiDAR 2 ve toa do tuong doi so voi LiDAR 1
    float c1 = cosf(-e1.yaw), s1 = sinf(-e1.yaw);
    float dxW = e2.x - e1.x, dyW = e2.y - e1.y;
    float curRelX = c1 * dxW - s1 * dyW;
    float curRelY = s1 * dxW + c1 * dyW;
    float curRelYaw = e2.yaw - e1.yaw;

    auto r0 = PointToLineIcpStep(T_local, gridT, S_local, curRelX, curRelY, curRelYaw, 0.60f);
    auto rFinal = RunPointToLineIcpLoop(T_local, gridT, S_local, curRelX, curRelY, curRelYaw, 25);

    // Chuyen nguoc ve World theo e1
    float c1w = cosf(e1.yaw), s1w = sinf(e1.yaw);
    e2.x = e1.x + c1w * curRelX - s1w * curRelY;
    e2.y = e1.y + s1w * curRelX + c1w * curRelY;
    e2.yaw = e1.yaw + curRelYaw;
    while (e2.yaw > PI_F) e2.yaw -= 2.0f * PI_F;
    while (e2.yaw < -PI_F) e2.yaw += 2.0f * PI_F;

    gSources[gSelectedLidar]->SetExtr(e2);
    ResetMotionModel();

    LogPrintf("[ICP] %s Point-to-Line RMS %.3f -> %.3f | World dx=%.3f dy=%.3f th=%.2f (%d inliers)\n",
        gSources[gSelectedLidar]->GetName().c_str(), r0.first, rFinal.first, e2.x, e2.y, Rad2Deg(e2.yaw), rFinal.second);
    json r; r["type"] = "icp"; r["id"] = gSelectedLidar;
    r["dx"] = e2.x; r["dy"] = e2.y; r["thetaDeg"] = Rad2Deg(e2.yaw);
    r["rmsBefore"] = r0.first; r["rmsAfter"] = rFinal.first;
    r["inliers"] = rFinal.second;
    QueueWebJson(r);
    BroadcastExtr(gSelectedLidar);
    gIcpRunning = false;
}

// ================= 12. AUTO-ALIGN GLOBAL SCAN MATCHING (FRAME-INVARIANT) =================
void RunGlobalAutoAlign(int targetId)
{
    if (gAutoAlignRunning.exchange(true))
    {
        QueueWebJson({ {"type","log"},{"text","[AUTO ALIGN] Dang trong qua trinh can chinh, vui long doi..."} });
        return;
    }
    if (targetId <= 0 || targetId >= (int)gSources.size())
    {
        QueueWebJson({ {"type","log"},{"text","[AUTO ALIGN] Vui long chon LiDAR can ghep (L2, L3...) voi LiDAR 1"} });
        gAutoAlignRunning = false;
        return;
    }

    QueueWebJson({ {"type","auto_align"},{"status","start"},{"id", targetId} });
    LogPrintf("[AUTO ALIGN] Bat dau can chinh tu dong L%d -> L1 (Frame-Invariant Global CSM + Point-to-Line ICP)...\n", targetId + 1);

    auto tStart = std::chrono::high_resolution_clock::now();

    FramePtr fRef, fSrc;
    Extrinsic3D eRef;
    {
        std::lock_guard<std::recursive_mutex> lk(gSourcesMutex);
        if (gSources.size() > 0 && gSources[0]->IsRunning()) {
            fRef = gSources[0]->GetFrameAtOffset(0);
            eRef = gSources[0]->GetExtr();
        }
        if (targetId < (int)gSources.size() && gSources[targetId]->IsRunning()) {
            fSrc = gSources[targetId]->GetFrameAtOffset(0);
        }
    }

    if (!fRef || !fSrc || fRef->local_x.empty() || fSrc->local_x.empty())
    {
        LogPrintf("[AUTO ALIGN] Khong du du lieu point cloud tu 2 LiDAR.\n");
        QueueWebJson({ {"type","log"},{"text","[AUTO ALIGN] Khong du du lieu tu 2 LiDAR de can chinh"} });
        QueueWebJson({ {"type","auto_align"},{"status","error"},{"id", targetId} });
        gAutoAlignRunning = false;
        return;
    }

    // 1. Trich xuat points HOAN TOAN TRONG HE TOA DO LOCAL CUA LIDAR 1 (T_local) va LIDAR target (S_local)
    // Dieu nay dam bao thuat toan 100% bat bien (invariant) du LiDAR 1 dat o (0,0) hay (10,0) hay bat ky dau!
    std::vector<Ptf> T_local, S_local;
    for (size_t i = 0; i < fRef->local_x.size(); i += 2)
    {
        float lx = fRef->local_x[i], ly = fRef->local_y[i];
        if (std::hypotf(lx, ly) > 0.15f)
        {
            T_local.push_back({ lx, ly });
        }
    }
    for (size_t i = 0; i < fSrc->local_x.size(); i += 2)
    {
        float lx = fSrc->local_x[i], ly = fSrc->local_y[i];
        if (std::hypotf(lx, ly) > 0.15f)
        {
            S_local.push_back({ lx, ly });
        }
    }

    if (T_local.size() < 30 || S_local.size() < 30)
    {
        LogPrintf("[AUTO ALIGN] So diem qua it (%zu, %zu).\n", T_local.size(), S_local.size());
        QueueWebJson({ {"type","log"},{"text","[AUTO ALIGN] Diem quet qua it, khong the tim vi tri"} });
        QueueWebJson({ {"type","auto_align"},{"status","error"},{"id", targetId} });
        gAutoAlignRunning = false;
        return;
    }

    // Xay dung SpatialGrid2D voi PCA Wall Normal tren T_local
    SpatialGrid2D gridT;
    gridT.Build(T_local, 0.35f);

    // 2. Xay dung 2D Likelihood Field Grid tu Target (L1 local frame, tam luon luon o (0,0))
    // Kich thuoc 50m x 50m, do phan giai res = 0.12m (12cm) -> 416 x 416 o
    const float GRID_RES = 0.12f;
    const float GRID_MIN = -25.0f;
    const float GRID_MAX = 25.0f;
    const int GRID_N = (int)((GRID_MAX - GRID_MIN) / GRID_RES); // 416
    std::vector<uint8_t> grid(GRID_N * GRID_N, 0);

    auto w2g = [&](float x, float y, int& gx, int& gy) {
        gx = (int)floorf((x - GRID_MIN) / GRID_RES);
        gy = (int)floorf((y - GRID_MIN) / GRID_RES);
        return (gx >= 0 && gx < GRID_N && gy >= 0 && gy < GRID_N);
        };

    for (const auto& pt : T_local)
    {
        int cx, cy;
        if (!w2g(pt.x, pt.y, cx, cy)) continue;
        for (int dy = -3; dy <= 3; dy++)
        {
            int gy = cy + dy;
            if (gy < 0 || gy >= GRID_N) continue;
            for (int dx = -3; dx <= 3; dx++)
            {
                int gx = cx + dx;
                if (gx < 0 || gx >= GRID_N) continue;
                int d2 = dx * dx + dy * dy;
                uint8_t val = 0;
                if (d2 == 0) val = 100;
                else if (d2 <= 1) val = 80;
                else if (d2 <= 4) val = 50;
                else if (d2 <= 9) val = 20;
                int idx = gy * GRID_N + gx;
                if (val > grid[idx]) grid[idx] = val;
            }
        }
    }

    // Subsample Source cho Coarse Search (~70 diem)
    std::vector<Ptf> S_coarse;
    size_t step_c = std::max((size_t)1, S_local.size() / 70);
    for (size_t i = 0; i < S_local.size(); i += step_c) S_coarse.push_back(S_local[i]);

    // 3. Coarse Search: 120 goc (buoc 3 do), vi tri -15m..+15m quanh L1 (buoc 0.45m)
    const int N_ANGLES = 120;
    const float D_THETA = 3.0f; // 3 do
    std::vector<std::vector<Ptf>> S_rot(N_ANGLES);
    for (int a = 0; a < N_ANGLES; a++)
    {
        float rad = Deg2Rad((float)a * D_THETA - 180.0f);
        float c = cosf(rad), s = sinf(rad);
        S_rot[a].reserve(S_coarse.size());
        for (const auto& p : S_coarse)
        {
            S_rot[a].push_back({ c * p.x - s * p.y, s * p.x + c * p.y });
        }
    }

    struct Candidate {
        float dx, dy, yaw;
        int score;
    };
    std::vector<Candidate> topCandidates;

    const float POS_MIN = -16.0f;
    const float POS_MAX = 16.0f;
    const float POS_STEP_C = 0.45f;

    auto tryAddCandidate = [&](float tx, float ty, float yaw, int score) {
        if (score <= 0) return;
        for (auto& c : topCandidates)
        {
            float dPos = std::hypotf(tx - c.dx, ty - c.dy);
            float dYaw = std::abs(yaw - c.yaw);
            while (dYaw > PI_F) dYaw = std::abs(dYaw - 2.0f * PI_F);
            if (dPos < 1.2f && dYaw < Deg2Rad(20.0f))
            {
                if (score > c.score) c = { tx, ty, yaw, score };
                return;
            }
        }
        topCandidates.push_back({ tx, ty, yaw, score });
        std::sort(topCandidates.begin(), topCandidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.score > b.score;
            });
        if (topCandidates.size() > 10) topCandidates.resize(10);
        };

    for (int a = 0; a < N_ANGLES; a++)
    {
        float yaw = Deg2Rad((float)a * D_THETA - 180.0f);
        const auto& spts = S_rot[a];
        for (float tx = POS_MIN; tx <= POS_MAX; tx += POS_STEP_C)
        {
            for (float ty = POS_MIN; ty <= POS_MAX; ty += POS_STEP_C)
            {
                int score = 0;
                int hitCount = 0;
                for (const auto& sp : spts)
                {
                    int gx, gy;
                    if (w2g(sp.x + tx, sp.y + ty, gx, gy))
                    {
                        uint8_t v = grid[gy * GRID_N + gx];
                        score += v;
                        if (v >= 50) hitCount++;
                    }
                }
                if (hitCount >= 10)
                {
                    tryAddCandidate(tx, ty, yaw, score);
                }
            }
        }
    }

    if (topCandidates.empty())
    {
        LogPrintf("[AUTO ALIGN] Coarse search khong tim thay ung vien phu hop.\n");
        QueueWebJson({ {"type","log"},{"text","[AUTO ALIGN] Khong tim thay vung trung khop giua 2 LiDAR (thieu overlap)"} });
        QueueWebJson({ {"type","auto_align"},{"status","error"},{"id", targetId} });
        gAutoAlignRunning = false;
        return;
    }

    LogPrintf("[AUTO ALIGN] Coarse search tim duoc %d ung vien hang dau (trong he L1):\n", (int)topCandidates.size());
    for (size_t i = 0; i < topCandidates.size(); i++)
    {
        LogPrintf("  #%d: relDx=%.2f relDy=%.2f relYaw=%.1f deg (Score=%d)\n",
            (int)i + 1, topCandidates[i].dx, topCandidates[i].dy, Rad2Deg(topCandidates[i].yaw), topCandidates[i].score);
    }

    // 4. Fine Grid Search & Point-to-Line ICP cho tung ung vien
    std::vector<Ptf> S_fine;
    size_t step_f = std::max((size_t)1, S_local.size() / 180);
    for (size_t i = 0; i < S_local.size(); i += step_f) S_fine.push_back(S_local[i]);

    std::vector<Ptf> S_icp;
    size_t step_icp = std::max((size_t)1, S_local.size() / 350);
    for (size_t i = 0; i < S_local.size(); i += step_icp) S_icp.push_back(S_local[i]);

    float bestRelX = 0, bestRelY = 0, bestRelYaw = 0;
    float bestRms = 999.0f;
    int bestInliers = 0;
    float bestQuality = -1.0f;

    for (size_t ci = 0; ci < topCandidates.size(); ci++)
    {
        const auto& cand = topCandidates[ci];
        float fineDx = cand.dx;
        float fineDy = cand.dy;
        float fineYaw = cand.yaw;
        int fineScore = 0;

        for (float dth = -3.5f; dth <= 3.5f; dth += 0.4f)
        {
            float yaw = cand.yaw + Deg2Rad(dth);
            float c = cosf(yaw), s = sinf(yaw);
            std::vector<Ptf> spts;
            spts.reserve(S_fine.size());
            for (const auto& p : S_fine) spts.push_back({ c * p.x - s * p.y, s * p.x + c * p.y });

            for (float tx = cand.dx - 0.55f; tx <= cand.dx + 0.55f; tx += 0.04f)
            {
                for (float ty = cand.dy - 0.55f; ty <= cand.dy + 0.55f; ty += 0.04f)
                {
                    int score = 0;
                    for (const auto& sp : spts)
                    {
                        int gx, gy;
                        if (w2g(sp.x + tx, sp.y + ty, gx, gy))
                        {
                            score += grid[gy * GRID_N + gx];
                        }
                    }
                    if (score > fineScore)
                    {
                        fineScore = score;
                        fineDx = tx;
                        fineDy = ty;
                        fineYaw = yaw;
                    }
                }
            }
        }

        // Chay Point-to-Line ICP 25 vong lap voi PCA Wall Normals
        float curX = fineDx;
        float curY = fineDy;
        float curYaw = fineYaw;
        auto icpRes = RunPointToLineIcpLoop(T_local, gridT, S_icp, curX, curY, curYaw, 25);

        float rms = icpRes.first;
        int inliers = icpRes.second;
        float quality = (float)inliers / (rms + 0.010f);

        LogPrintf("  [Cand #%d] Fine: (%.2f, %.2f, %.1f) -> Point-to-Line ICP: relDx=%.3f relDy=%.3f relYaw=%.2f | RMS=%.3f inliers=%d Quality=%.1f\n",
            (int)ci + 1, fineDx, fineDy, Rad2Deg(fineYaw), curX, curY, Rad2Deg(curYaw), rms, inliers, quality);

        if (quality > bestQuality)
        {
            bestQuality = quality;
            bestRelX = curX;
            bestRelY = curY;
            bestRelYaw = curYaw;
            bestRms = rms;
            bestInliers = inliers;
        }
    }

    if (bestInliers < 20 || bestRms > 0.35f)
    {
        LogPrintf("[AUTO ALIGN] Chat luong ghep chua dat (RMS=%.3f, inliers=%d).\n", bestRms, bestInliers);
        QueueWebJson({ {"type","log"},{"text","[AUTO ALIGN] Chat luong ghep thap (khong tim thay duong tuong tuong dong)"} });
        QueueWebJson({ {"type","auto_align"},{"status","error"},{"id", targetId} });
        gAutoAlignRunning = false;
        return;
    }

    // 5. Chuyen vi tri tuong doi (relX, relY, relYaw) sang World theo eRef cua LiDAR 1!
    float c1w = cosf(eRef.yaw), s1w = sinf(eRef.yaw);
    float worldX = eRef.x + c1w * bestRelX - s1w * bestRelY;
    float worldY = eRef.y + s1w * bestRelX + c1w * bestRelY;
    float worldYaw = eRef.yaw + bestRelYaw;
    while (worldYaw > PI_F) worldYaw -= 2.0f * PI_F;
    while (worldYaw < -PI_F) worldYaw += 2.0f * PI_F;

    auto tEnd = std::chrono::high_resolution_clock::now();
    long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count();

    // 6. Cap nhat Extrinsic chinh xac vao gSources
    {
        std::lock_guard<std::recursive_mutex> lk(gSourcesMutex);
        if (targetId < (int)gSources.size() && gSources[targetId])
        {
            Extrinsic3D e = gSources[targetId]->GetExtr();
            e.x = worldX;
            e.y = worldY;
            e.yaw = worldYaw;
            gSources[targetId]->SetExtr(e);
        }
    }
    ResetMotionModel();

    LogPrintf("[AUTO ALIGN] HOAN TAT CHINH XAC CAO (%lld ms)! L%d -> L1: rel=(%.3f, %.3f, %.2f deg) -> World=(%.3f, %.3f, %.2f deg) | RMS=%.3f m (%d inliers)\n",
        elapsedMs, targetId + 1, bestRelX, bestRelY, Rad2Deg(bestRelYaw), worldX, worldY, Rad2Deg(worldYaw), bestRms, bestInliers);

    char buf[256];
    sprintf_s(buf, "[AUTO ALIGN] Thanh cong (%lld ms): World X=%.3f m, Y=%.3f m, Yaw=%.2f deg | RMS=%.3f m (%d diem hop le)",
        elapsedMs, worldX, worldY, Rad2Deg(worldYaw), bestRms, bestInliers);
    QueueWebJson({ {"type","log"},{"text", buf} });

    json res;
    res["type"] = "auto_align";
    res["status"] = "done";
    res["success"] = true;
    res["id"] = targetId;
    res["dx"] = worldX;
    res["dy"] = worldY;
    res["thetaDeg"] = Rad2Deg(worldYaw);
    res["rms"] = bestRms;
    res["inliers"] = bestInliers;
    res["elapsedMs"] = elapsedMs;
    QueueWebJson(res);

    BroadcastExtr(targetId);
    gAutoAlignRunning = false;
}

// ================= 23. RECORDING =================
void ToggleRecording()
{
    if (!gIsRecording.load())
    {
        char fn[100]; time_t now = time(NULL); struct tm t;
        localtime_s(&t, &now);
        strftime(fn, sizeof(fn), "lidar_full_data_%Y%m%d_%H%M%S.csv", &t);
        std::lock_guard<std::mutex> lk(gCsvMutex);
        gCsvFile.open(fn);
        if (gCsvFile.is_open())
        {
            // >>> CAP NHAT: Them Local_Z vao header
            gCsvFile << "Source_ID,Frame_ID,Timestamp_sec,Timestamp_usec,Point_Index,Angle_Degree,Distance_m,Local_X,Local_Y,Local_Z,Intensity,Scan_Time\n";
            gIsRecording = true;
            LogPrintf("[INFO] Ghi CSV: %s\n", fn);
        }
    }
    else
    {
        { std::lock_guard<std::mutex> lk(gCsvMutex); if (gCsvFile.is_open()) gCsvFile.close(); }
        gIsRecording = false;
        LogPrintf("[INFO] Dung ghi CSV.\n");
    }
}

// ================= 26. PROCESSING (v3: dung vector thay array) =================
static void CheckOcclusion(const std::vector<FramePtr>& frames)
{
    if (gIsPaused.load()) return;
    int total = 0;
    for (size_t i = 0; i < frames.size(); i++) if (frames[i]) total += (int)frames[i]->x_coords.size();
    if (total == 0) return;
    gAvgPointCount = (gAvgCount == 0) ? (float)total : 0.98f * gAvgPointCount + 0.02f * total;
    gAvgCount++;
    if (gAvgCount < 50) return;
    float th = std::max((float)OCCLUSION_MIN_POINTS, 0.30f * gAvgPointCount);
    if ((float)total < th) gLowPointFrames++; else gLowPointFrames = 0;
    bool should = (gLowPointFrames >= OCCLUSION_PERSIST_FRAMES);
    if (should && !gOcclusionAlarm)
    {
        LogPrintf("[OCCLUSION] CANH BAO: diem giam con %d (TB %.0f)\n", total, gAvgPointCount);
        gOcclusionAlarm = true;
    }
    else if (!should && gOcclusionAlarm)
    {
        LogPrintf("[OCCLUSION] Het canh bao.\n");
        gOcclusionAlarm = false;
    }
}

// >>> v3: UpdateMotionDetection dung Zone::ContainsPoint <<<
static void UpdateMotionDetection(const std::vector<FramePtr>& frames, DWORD64 nowMs)
{
    const long long fusionStartUs = NowUs();
    static DWORD64 lastMs = 0;
    float dt = (lastMs && nowMs > lastMs) ? (nowMs - lastMs) / 1000.0f : 0.05f;
    if (dt < 0.01f) dt = 0.01f; if (dt > 1.0f) dt = 1.0f;
    lastMs = nowMs;
    bool hasInclusionZone = false;
    for (const auto& z : gZones) {
        if (!z.exclusion) { hasInclusionZone = true; break; }
    }
    if (!hasInclusionZone) {
        if (!gTargets.empty() || gBgFrames) ResetMotionModel();
        return;
    }
    const unsigned long long batchId = gMetricBatchId.fetch_add(1) + 1;
    json frameRefs = json::array(), sourceTimestamps = json::array(), metricFrames = json::array();
    for (const auto& f : frames) if (f) {
        frameRefs.push_back({ {"sensorId",f->source_id},{"frameId",f->frame_id} });
        metricFrames.push_back({ {"sensorId",f->source_id},{"frameId",f->frame_id},
            {"backendArrivalMonotonicUs",f->backend_arrival_steady_us},
            {"sourceTimestampValid",f->source_timestamp_valid},
            {"sourceTimestampUs",f->source_timestamp_valid ? json((uint64_t)f->ts_secs * 1000000ULL + f->ts_usecs) : json(nullptr)} });
        sourceTimestamps.push_back({ {"sensorId",f->source_id},
            {"timestampValid",f->source_timestamp_valid},
            {"timestampUs",f->source_timestamp_valid ? json((uint64_t)f->ts_secs * 1000000ULL + f->ts_usecs) : json(nullptr)},
            {"timestampSemantic","unverified"},
            {"backendArrivalMonotonicUs",f->backend_arrival_steady_us} });
    }
    { std::lock_guard<std::mutex> lock(gMetricFrameMutex); gMetricFrames = metricFrames; }
    std::vector<Ptf> pts;
    std::vector<float> ptsZ;
    std::vector<int> ptsSensor;
    for (size_t i = 0; i < frames.size(); i++)
    {
        if (!frames[i]) continue;
        const auto& xs = frames[i]->x_coords, ys = frames[i]->y_coords, zs = frames[i]->z_coords;
        const auto& lxs = frames[i]->local_x, lys = frames[i]->local_y;
        for (size_t k = 0; k < xs.size(); k++)
        {
            bool inInclusion = false;
            bool inExclusion = false;
            for (const auto& z : gZones)
            {
                bool inside = (!z.local && z.ContainsPoint(xs[k], ys[k])) ||
                              (z.local && z.sourceId == (int)i && z.ContainsPoint(lxs[k], lys[k]));
                if (inside) {
                    if (z.exclusion) inExclusion = true;
                    else inInclusion = true;
                }
            }
            if (inInclusion && !inExclusion) {
                pts.push_back(Ptf{ xs[k], ys[k] });
                ptsZ.push_back(k < zs.size() ? zs[k] : 0.0f);
                ptsSensor.push_back((int)i);
            }
        }
    }
    const long long fusionEndUs = NowUs();
    const long long detectionStartUs = fusionEndUs;
    // 35 cm clustering cells: du rong de noi mat truoc va mat sau cua nguoi (20-30cm)
    const float CS = 0.35f;
    auto key = [](int ix, int iy) { return ((long long)(ix + 2048) << 32) | (uint32_t)(iy + 2048); };
    gBgFrames++;
    // Mo hinh nen dong thich nghi (Sliding window): giam nua sau moi 120 frames (~5 giay)
    // Giup bo tuong bi che khuat nhanh chong tro lai trang thai nen tinh
    if (gBgFrames >= 120) {
        gBgFrames = 60;
        for (auto it = gBgSeen.begin(); it != gBgSeen.end(); ) {
            it->second /= 2;
            if (it->second == 0) it = gBgSeen.erase(it);
            else ++it;
        }
    }
    std::unordered_map<long long, char> occ;
    for (auto& p : pts) occ[key((int)floorf(p.x / CS), (int)floorf(p.y / CS))] = 1;
    for (auto& kv : occ) gBgSeen[kv.first]++;
    std::vector<Ptf> dyn;
    std::vector<float> dynZ;
    std::vector<int> dynSensor;
    for (size_t pi = 0; pi < pts.size(); pi++)
    {
        auto& p = pts[pi];
        long long k = key((int)floorf(p.x / CS), (int)floorf(p.y / CS));
        int seen = gBgSeen.count(k) ? gBgSeen[k] : 0;
        bool isStatic = (gBgFrames > 15) && (seen * 100 >= 65 * gBgFrames);
        if (!isStatic) {
            dyn.push_back(p);
            dynZ.push_back(ptsZ[pi]);
            dynSensor.push_back(ptsSensor[pi]);
        }
    }

    // --- MULTI-SENSOR SAFE RAY SHADOW FILTER ---
    // QUAN TRONG: Chi xet bong che tren cung 1 LiDAR tu goc nhin cua chinh LiDAR do.
    // Diem tu cac LiDAR khac nhau (nhu LiDAR 1 thay mat truoc, LiDAR 2 thay mat sau)
    // la cac be mat thuc cua vat the 3D, TUYET DOI KHONG DUOC coi la bong che cua nhau!
    std::vector<Ptf> cleanDyn;
    std::vector<float> cleanDynZ;
    cleanDyn.reserve(dyn.size());
    cleanDynZ.reserve(dynZ.size());

    for (size_t i = 0; i < dyn.size(); i++) {
        int sI = dynSensor[i];
        float oxI = (sI < (int)gSources.size()) ? gSources[sI]->GetExtr().x : 0.0f;
        float oyI = (sI < (int)gSources.size()) ? gSources[sI]->GetExtr().y : 0.0f;
        float dxI = dyn[i].x - oxI, dyI = dyn[i].y - oyI;
        float dI = sqrtf(dxI * dxI + dyI * dyI);
        float aI = atan2f(dyI, dxI);
        bool isShadow = false;

        for (size_t j = 0; j < pts.size(); j++) {
            // Chi so sanh voi diem pts cua CUNG SENSOR sI
            if (ptsSensor[j] != sI) continue;
            float dxJ = pts[j].x - oxI, dyJ = pts[j].y - oyI;
            float dJ = sqrtf(dxJ * dxJ + dyJ * dyJ);
            float aJ = atan2f(dyJ, dxJ);
            // Neu co diem cua cung sensor o gan hon nam cung huong tia (|aI - aJ| < 0.12 rad ~ 6.8 deg)
            if (dI > dJ + 0.35f && fabs(aI - aJ) < 0.12f) {
                isShadow = true;
                break;
            }
        }
        if (!isShadow) {
            cleanDyn.push_back(dyn[i]);
            cleanDynZ.push_back(dynZ[i]);
        }
    }
    dyn = std::move(cleanDyn);
    dynZ = std::move(cleanDynZ);
    // -------------------------------------------------------------------
    std::unordered_map<long long, std::vector<int>> grid;
    for (int i = 0; i < (int)dyn.size(); i++) grid[key((int)floorf(dyn[i].x / CS), (int)floorf(dyn[i].y / CS))].push_back(i);
    std::vector<char> vis(dyn.size(), 0);
    struct SubCluster {
        float x1, y1, x2, y2;
        float sx, sy;
        int cnt;
        float zLo, zHi;
    };
    std::vector<SubCluster> subs;
    for (auto& kv : grid)
        for (int seed : kv.second)
        {
            if (vis[seed]) continue;
            std::vector<int> st{ seed }; vis[seed] = 1;
            float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f, sx = 0, sy = 0; int cnt = 0;
            float zLo = 1e9f, zHi = -1e9f;
            while (!st.empty())
            {
                int cur = st.back(); st.pop_back();
                float px = dyn[cur].x, py = dyn[cur].y, pz = dynZ[cur];
                x1 = std::min(x1, px); y1 = std::min(y1, py);
                x2 = std::max(x2, px); y2 = std::max(y2, py);
                zLo = std::min(zLo, pz); zHi = std::max(zHi, pz);
                sx += px; sy += py; cnt++;
                int ix = (int)floorf(px / CS), iy = (int)floorf(py / CS);
                for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
                {
                    auto it = grid.find(key(ix + dx, iy + dy));
                    if (it == grid.end()) continue;
                    for (int j : it->second) if (!vis[j]) { vis[j] = 1; st.push_back(j); }
                }
            }
            float w = x2 - x1, h = y2 - y1;
            // Lưu các cụm con tiềm năng (từ >= 3 điểm) để cho phép nối thông 2 mặt trước - sau đối diện
            if (cnt >= 3 && w <= MAX_TARGET_W && h <= MAX_TARGET_H)
                subs.push_back({ x1, y1, x2, y2, sx, sy, cnt, zLo, zHi });
        }

    // --- NỐI THÔNG BỀ MẶT ĐỐI DIỆN ĐA LIDAR BẰNG DSU (CỰC NHANH O(M^2) < 0.01ms) ---
    // Trong môi trường đông người ngoài trời:
    // 1. Nối 2 bề mặt ngực (cam) và lưng (xanh) của cùng 1 người xuyên qua khoảng rỗng bên trong (khe <= 35cm).
    // 2. Không bao giờ vô tình gom 2 người đứng gần nhau vì giới hạn kích thước người kết hợp <= 0.80m.
    std::vector<int> parent(subs.size());
    for (size_t i = 0; i < subs.size(); i++) parent[i] = (int)i;
    auto findRoot = [&](auto& self, int i) -> int {
        return parent[i] == i ? i : (parent[i] = self(self, parent[i]));
    };

    for (size_t i = 0; i < subs.size(); i++) {
        for (size_t j = i + 1; j < subs.size(); j++) {
            float gapX = std::max(0.0f, std::max(subs[i].x1, subs[j].x1) - std::min(subs[i].x2, subs[j].x2));
            float gapY = std::max(0.0f, std::max(subs[i].y1, subs[j].y1) - std::min(subs[i].y2, subs[j].y2));
            float gapDist2 = gapX * gapX + gapY * gapY;
            if (gapDist2 <= 0.35f * 0.35f) {
                float combW = std::max(subs[i].x2, subs[j].x2) - std::min(subs[i].x1, subs[j].x1);
                float combH = std::max(subs[i].y2, subs[j].y2) - std::min(subs[i].y1, subs[j].y1);
                if (combW <= 0.80f && combH <= 0.80f) {
                    int ri = findRoot(findRoot, (int)i);
                    int rj = findRoot(findRoot, (int)j);
                    if (ri != rj) parent[rj] = ri;
                }
            }
        }
    }

    std::unordered_map<int, SubCluster> merged;
    for (size_t i = 0; i < subs.size(); i++) {
        int r = findRoot(findRoot, (int)i);
        auto it = merged.find(r);
        if (it == merged.end()) {
            merged[r] = subs[i];
        } else {
            it->second.x1 = std::min(it->second.x1, subs[i].x1);
            it->second.y1 = std::min(it->second.y1, subs[i].y1);
            it->second.x2 = std::max(it->second.x2, subs[i].x2);
            it->second.y2 = std::max(it->second.y2, subs[i].y2);
            it->second.sx += subs[i].sx;
            it->second.sy += subs[i].sy;
            it->second.cnt += subs[i].cnt;
            it->second.zLo = std::min(it->second.zLo, subs[i].zLo);
            it->second.zHi = std::max(it->second.zHi, subs[i].zHi);
        }
    }

    struct RawT { float x, y, w, h; int n; float z1, z2; };
    std::vector<RawT> raw;
    for (const auto& kv : merged) {
        const auto& c = kv.second;
        float w = c.x2 - c.x1, h = c.y2 - c.y1;
        if (c.cnt >= MIN_CLUSTER_POINTS && w >= MIN_TARGET_W && h >= MIN_TARGET_H &&
            w <= MAX_TARGET_W && h <= MAX_TARGET_H) {
            raw.push_back({ c.sx / c.cnt, c.sy / c.cnt, w, h, c.cnt, c.zLo, c.zHi });
        }
    }
    const long long detectionEndUs = NowUs();
    const long long trackingStartUs = detectionEndUs;
    const std::vector<DynTarget> previousTargets = gTargets;
    std::vector<DynTarget> next;
    std::vector<char> used(gTargets.size(), 0);
    for (auto& r : raw)
    {
        int best = -1; float bd = 1.6f;
        for (size_t i = 0; i < gTargets.size(); i++)
            if (!used[i])
            {
                // Dự đoán vị trí quán tính theo vận tốc để bám dính ID cực chuẩn khi di chuyển nhanh hoặc đổi hướng
                float predX = gTargets[i].x + gTargets[i].vx * dt;
                float predY = gTargets[i].y + gTargets[i].vy * dt;
                float d = sqrtf((predX - r.x) * (predX - r.x) + (predY - r.y) * (predY - r.y));
                if (d < bd) { bd = d; best = (int)i; }
            }
        DynTarget t;
        if (best >= 0)
        {
            used[best] = 1; t = gTargets[best];
            t.age = gTargets[best].age + 1;
            t.miss = 0;
            t.vx = 0.6f * t.vx + 0.4f * ((r.x - t.x) / dt);
            t.vy = 0.6f * t.vy + 0.4f * ((r.y - t.y) / dt);
            t.x = r.x; t.y = r.y;

            // Làm mượt kích thước vật lý (EMA: 70% cũ + 30% mới) để loại bỏ co giật cục bộ
            t.w = 0.70f * t.w + 0.30f * r.w;
            t.h = 0.70f * t.h + 0.30f * r.h;
            if (r.w > t.maxW && r.w <= 2.5f) t.maxW = r.w;
            if (r.h > t.maxH && r.h <= 2.5f) t.maxH = r.h;
            t.n = (int)(0.7f * t.n + 0.3f * r.n);
            if (r.n > t.n) t.n = r.n;
            t.z1 = r.z1; t.z2 = r.z2;

            float spd = std::hypot(t.vx, t.vy);
            if (spd > t.maxSpeed) t.maxSpeed = spd;

            // Ghi nhận điểm tiến sâu vào lòng ray nhất (|X| nhỏ nhất)
            if (t.criticalX == 0.0f || std::abs(t.x) < std::abs(t.criticalX)) {
                t.criticalX = t.x;
                t.criticalY = t.y;
            }
            t.exitX = t.x;
            t.exitY = t.y;

            float distFromSpawn = std::hypot(r.x - t.spawnX, r.y - t.spawnY);
            if (distFromSpawn > t.maxDistFromSpawn) t.maxDistFromSpawn = distFromSpawn;
            next.push_back(t);
        }
        else
        {
            t.id = gNextTargetId++; t.x = r.x; t.y = r.y; t.vx = 0; t.vy = 0;
            t.w = r.w; t.h = r.h; t.maxW = r.w; t.maxH = r.h;
            t.n = r.n; t.age = 1; t.z1 = r.z1; t.z2 = r.z2;
            t.maxSpeed = 0.0f;
            t.criticalX = r.x; t.criticalY = r.y;
            t.exitX = r.x; t.exitY = r.y;
            t.spawnX = r.x; t.spawnY = r.y; t.maxDistFromSpawn = 0.0f;
            t.miss = 0;
            next.push_back(t);
        }
    }
    // Duy trì đối tượng tạm thời bị khuất tia (Dead reckoning coasting tối đa 15 frames ~ 0.65s)
    for (size_t i = 0; i < gTargets.size(); i++) {
        if (!used[i] && gTargets[i].age >= PERSISTENCE_FRAMES && gTargets[i].miss < 15) {
            DynTarget t = gTargets[i];
            t.miss++;
            t.x += t.vx * dt;
            t.y += t.vy * dt;
            next.push_back(t);
        }
    }
    gTargets = next;
    const long long trackingEndUs = NowUs();
    gDetectionStats.Record((detectionEndUs - detectionStartUs) / 1000.0);
    // This metric is the sum of two disjoint stages: cross-sensor point aggregation
    // and track association/update. Detection clustering is excluded.
    gTrackingFusionStats.Record(((fusionEndUs - fusionStartUs) + (trackingEndUs - trackingStartUs)) / 1000.0);

    json sensorIds = json::array();
    for (const auto& f : frames) if (f) sensorIds.push_back(f->source_id);
    if (gBgFrames > 15) {
        for (const auto& t : gTargets) if (t.age >= PERSISTENCE_FRAMES && IsRealMotionTarget(t)) {
            auto prior = std::find_if(previousTargets.begin(), previousTargets.end(), [&t](const DynTarget& p) { return p.id == t.id; });
            const bool newlyConfirmed = prior == previousTargets.end() || prior->age < PERSISTENCE_FRAMES || !IsRealMotionTarget(*prior);
            const bool moved = prior != previousTargets.end() && std::hypot(t.x - prior->x, t.y - prior->y) >= 0.20f;
            if (!newlyConfirmed && !moved) continue;
            json event;
            event["type"] = "object_event"; event["eventId"] = NewVisualEventId();
            event["objectId"] = t.id; event["batchId"] = batchId; event["frameIds"] = frameRefs; event["sensorIds"] = sensorIds;
            event["sourceTimestamps"] = sourceTimestamps;
            // Object x/y are already expressed in the fused LiDAR world frame used by
            // the occupancy map. Publish the frame explicitly so receivers can place
            // the object on a copy of the same map without guessing the coordinates.
            event["mapPosition"] = { {"frameId", "lidar_fusion_map"}, {"units", "m"},
                {"x", t.x}, {"y", t.y}, {"zMin", t.z1}, {"zMax", t.z2} };
            event["eventType"] = newlyConfirmed ? "detected" : "updated";

            float spdKmh = std::hypot(t.vx, t.vy) * 3.6f;
            float maxSpdKmh = t.maxSpeed * 3.6f;
            float displayW = (t.maxW > t.w) ? (0.6f * t.w + 0.4f * t.maxW) : t.w;
            float displayH = (t.maxH > t.h) ? (0.6f * t.h + 0.4f * t.maxH) : t.h;

            event["object"] = {
                {"x", t.x}, {"y", t.y},
                {"vx", t.vx}, {"vy", t.vy},
                {"speedKmh", spdKmh},
                {"maxSpeedKmh", maxSpdKmh},
                {"width", displayW}, {"depth", displayH},
                {"maxW", t.maxW}, {"maxH", t.maxH},
                {"zMin", t.z1}, {"zMax", t.z2},
                {"pointCount", t.n}, {"ageFrames", t.age},
                {"criticalX", t.criticalX}, {"criticalY", t.criticalY},
                {"confirmed", true}
            };
            if (!gIsDemoMode.load()) {
                gVisualAdapter.Enqueue(event);
            }
        }
    }
    int confirmed = 0;
    if (gBgFrames > 15) {
        for (auto& tg : gTargets) if (tg.age >= PERSISTENCE_FRAMES && IsRealMotionTarget(tg)) confirmed++;

        // Cập nhật bộ lưu lịch sử mục tiêu gần đây (giúp Camera AI & Web Client phục hồi đối tượng đã rời đi)
        {
            std::lock_guard<std::mutex> lk(gTargetHistoryMutex);
            uint64_t nowEpochMs = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            for (const auto& t : gTargets) {
                if (t.age < PERSISTENCE_FRAMES || !IsRealMotionTarget(t)) continue;
                float spd = std::hypot(t.vx, t.vy);
                auto it = std::find_if(gRecentTargets.begin(), gRecentTargets.end(),
                    [&t](const TargetHistoryRecord& r) { return r.id == t.id; });
                if (it != gRecentTargets.end()) {
                    it->lastSeenMs = nowEpochMs;
                    it->x = t.x; it->y = t.y; it->z1 = t.z1; it->z2 = t.z2;
                    it->w = t.w; it->h = t.h;
                    if (t.maxW > it->maxW) it->maxW = t.maxW;
                    if (t.maxH > it->maxH) it->maxH = t.maxH;
                    it->vx = t.vx; it->vy = t.vy;
                    it->curSpeed = spd;
                    if (spd > it->maxSpeed) it->maxSpeed = spd;
                    if (t.n > it->pointCount) it->pointCount = t.n;
                    it->age = t.age;
                    it->criticalX = t.criticalX; it->criticalY = t.criticalY;
                    it->exitX = t.exitX; it->exitY = t.exitY;
                    it->active = true;
                } else {
                    TargetHistoryRecord rec;
                    rec.id = t.id;
                    rec.x = t.x; rec.y = t.y; rec.z1 = t.z1; rec.z2 = t.z2;
                    rec.w = t.w; rec.h = t.h;
                    rec.maxW = t.maxW; rec.maxH = t.maxH;
                    rec.vx = t.vx; rec.vy = t.vy;
                    rec.curSpeed = spd;
                    rec.maxSpeed = spd;
                    rec.pointCount = t.n;
                    rec.age = t.age;
                    rec.criticalX = t.criticalX; rec.criticalY = t.criticalY;
                    rec.exitX = t.exitX; rec.exitY = t.exitY;
                    rec.firstSeenMs = nowEpochMs;
                    rec.lastSeenMs = nowEpochMs;
                    rec.active = true;
                    gRecentTargets.push_back(rec);
                    if (gRecentTargets.size() > 60) {
                        gRecentTargets.erase(gRecentTargets.begin());
                    }
                }
            }
            for (auto& r : gRecentTargets) {
                if (!r.active) continue;
                bool found = false;
                for (const auto& t : gTargets) {
                    if (t.id == r.id) { found = true; break; }
                }
                if (!found) {
                    r.active = false;
                }
            }
        }
    }
    bool alarm = (confirmed > 0);
    if (alarm && !gPrevAlarm) LogPrintf("[CANH BAO] %d muc tieu DI CHUYEN trong vung!\n", confirmed);
    if (!alarm && gPrevAlarm) LogPrintf("[AN TOAN] Vung an toan.\n");
    gPrevAlarm = alarm;
}

void ComputeClusters(const std::vector<FramePtr>& frames)
{
    std::vector<Ptf> pts;
    for (size_t i = 0; i < frames.size(); i++)
        if (frames[i])
            for (size_t j = 0; j < frames[i]->x_coords.size(); j += 2)
                pts.push_back({ frames[i]->x_coords[j], frames[i]->y_coords[j] });
    const float CS = 0.25f;
    auto key = [](int ix, int iy) { return ((long long)(ix + 2048) << 32) | (uint32_t)(iy + 2048); };
    std::unordered_map<long long, std::vector<int>> grid;
    for (int i = 0; i < (int)pts.size(); i++) grid[key((int)floorf(pts[i].x / CS), (int)floorf(pts[i].y / CS))].push_back(i);
    std::vector<char> vis(pts.size(), 0);
    gBoxes.clear(); gNearest = 1e9f;
    for (auto& kv : grid)
        for (int seed : kv.second)
        {
            if (vis[seed]) continue;
            std::vector<int> st{ seed }; vis[seed] = 1;
            float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f; int cnt = 0;
            while (!st.empty())
            {
                int cur = st.back(); st.pop_back();
                float px = pts[cur].x, py = pts[cur].y;
                x1 = std::min(x1, px); y1 = std::min(y1, py);
                x2 = std::max(x2, px); y2 = std::max(y2, py); cnt++;
                int ix = (int)floorf(px / CS), iy = (int)floorf(py / CS);
                for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
                {
                    auto it = grid.find(key(ix + dx, iy + dy));
                    if (it == grid.end()) continue;
                    for (int j : it->second) if (!vis[j]) { vis[j] = 1; st.push_back(j); }
                }
            }
            if (cnt >= 6)
            {
                gBoxes.push_back({ x1, y1, x2, y2, cnt });
                float d = sqrtf(((x1 + x2) / 2) * ((x1 + x2) / 2) + ((y1 + y2) / 2) * ((y1 + y2) / 2));
                if (d < gNearest) gNearest = d;
            }
        }
    gNObj = (int)gBoxes.size();
}

// ================= SERIALIZE + EVENTS + TICK =================
static std::shared_ptr<std::vector<uint8_t>> SerializeFrameV3(const PointCloudData& f)
{
    size_t n = f.local_x.size();
    auto buf = std::make_shared<std::vector<uint8_t>>();
    buf->reserve(76 + n * 16);
    appendU8(*buf, 3); appendU8(*buf, (uint8_t)f.source_id); appendU16(*buf, 0);
    appendU32(*buf, (uint32_t)f.frame_id);
    appendU32(*buf, f.ts_secs); appendU32(*buf, f.ts_usecs);
    appendF32(*buf, f.origin_x); appendF32(*buf, f.origin_y); appendF32(*buf, f.origin_z);
    appendF32(*buf, f.origin_roll); appendF32(*buf, f.origin_pitch); appendF32(*buf, f.origin_yaw);
    appendF32(*buf, f.angle_min); appendF32(*buf, f.angle_max); appendF32(*buf, f.angle_increment);
    appendF32(*buf, f.range_min); appendF32(*buf, f.range_max);
    appendF32(*buf, f.scan_time); appendF32(*buf, f.time_increment);
    appendU32(*buf, (uint32_t)f.wave_count);
    appendU32(*buf, (uint32_t)n);
    for (size_t i = 0; i < n; i++)
    {
        appendF32(*buf, f.x_coords[i]); appendF32(*buf, f.y_coords[i]); appendF32(*buf, f.z_coords[i]);
        appendF32(*buf, i < f.intensities.size() ? f.intensities[i] : 0.0f);
    }
    return buf;
}

// >>> v3: GetFrames tra ve vector thay vi array <<<
static std::vector<FramePtr> GetFrames(int offset)
{
    std::vector<FramePtr> fr(gSources.size());
    for (size_t i = 0; i < gSources.size(); i++)
        if (gSources[i] && gSources[i]->IsRunning())
        {
            auto raw = gSources[i]->GetFrameAtOffset(offset);
            if (!raw) continue;
            auto e = gSources[i]->GetExtr();
            if (raw->origin_x == e.x && raw->origin_y == e.y && raw->origin_z == e.z && raw->origin_roll == e.roll && raw->origin_pitch == e.pitch && raw->origin_yaw == e.yaw && raw->source_id == (int)i) { fr[i] = raw; continue; }
            auto f = std::make_shared<PointCloudData>(*raw); Transform3D transform(e);
            f->source_id = (int)i;
            f->origin_x = e.x; f->origin_y = e.y; f->origin_z = e.z;
            f->origin_roll = e.roll; f->origin_pitch = e.pitch; f->origin_yaw = e.yaw;
            for (size_t k = 0; k < f->local_x.size(); ++k) {
                auto p = transform.Apply(f->local_x[k], f->local_y[k]);
                f->x_coords[k] = p.x; f->y_coords[k] = p.y; f->z_coords[k] = p.z;
            }
            fr[i] = f;
        }
    return fr;
}

// ================= ALARM LEVEL ENGINE & STORAGE ARCHITECTURE =================
// Phan cap bao dong 4 muc chuan ITS/Railway Safety:
//   0 = SAFE       — Khong co bat thuong
//   1 = WATCH      — Co muc tieu vung ngoai hoac che khuat laser bat thuong
//   2 = WARNING    — Muc tieu xac nhan di chuyen / tien vao vung bao ve
//   3 = CRITICAL   — Vat can tren ray / dung im / nhieu muc tieu vi pham
static std::atomic<int> gAlarmLevel{ 0 };

static int ComputeAlarmLevel(int confirmed, int totalTargets)
{
    if (confirmed >= 2) return 3; // Nhieu vat the vi pham -> Nguy hiem khot
    if (confirmed == 1) {
        // Neu vat the ton tai lau (dung yen tren ray) -> Cap 3
        for (const auto& t : gTargets) {
            if (t.age >= 15 && IsRealMotionTarget(t)) return 3;
        }
        return 2; // Vat the moi tien vao -> Canh bao
    }
    if (totalTargets > 0) return 1; // Co muc tieu tiem nang nhung chua du dieu kien
    if (gOcclusionAlarm) return 1;  // Che khuat laser bat thuong -> Theo doi
    return 0;
}

// ================= EVENT AUTO-PACKAGER =================
// Tu dong dong goi su co vao recordings/events/ khi alarmLevel >= 2
// Format file .bin: Magic "LDRB" + uint32 total_frames + [metaLen + metaJson + n1 + bin1 + n2 + bin2]
static const int EVENT_PRE_BUFFER_FRAMES  = 300;  // ~10 giay pre-buffer
static const int EVENT_POST_ALARM_FRAMES  = 150;  // ~5 giay post-buffer
static const size_t MAX_EVENT_FRAMES      = 1500; // Gioi han ~60 giay

struct RawFrameSnapshot {
    uint32_t ts_secs, ts_usecs;
    int      alarmLevel;
    std::vector<uint8_t> bin0;
    std::vector<uint8_t> bin1;
    uint32_t n0, n1;
    std::string metaJson;
};

static std::deque<RawFrameSnapshot> gPreBuffer;
static std::mutex gPreBufMutex;
static bool gEventCapturing = false;
static int  gEventPostFramesLeft = 0;
static std::vector<RawFrameSnapshot> gEventFrames;
static std::mutex gEventMutex;
static int  gEventAlarmLevel = 0;

static std::vector<uint8_t> PackPointsToBin(const PointCloudData& f)
{
    size_t n = f.x_coords.size();
    std::vector<uint8_t> buf(n * 16);
    float* p = reinterpret_cast<float*>(buf.data());
    for (size_t i = 0; i < n; i++) {
        p[i*4+0] = f.x_coords[i];
        p[i*4+1] = f.y_coords[i];
        p[i*4+2] = (i < f.z_coords.size()) ? f.z_coords[i] : 0.0f;
        p[i*4+3] = (i < f.intensities.size()) ? f.intensities[i] : 0.0f;
    }
    return buf;
}

// ================= QUOTA & COMPRESSION SETTINGS =================
static const bool     ENABLE_RAW_RECORDER   = false;                                // TAT HOAN TOAN ghi raw 24/7 (Che do van hanh thuc te - Chi ghi Event)
static const uint64_t RAW_MAX_QUOTA_BYTES   = 15ULL * 1024ULL * 1024ULL * 1024ULL; // Tran 15 GB cho recordings/raw
static const uint64_t EVENT_MAX_QUOTA_BYTES = 5ULL  * 1024ULL * 1024ULL * 1024ULL; // Tran 5 GB cho recordings/events
static const int      RAW_KEEP_DAYS         = 7;                                    // Giu raw 7 ngay
static const int      EVENT_KEEP_DAYS       = 14;                                   // Giu events 14 ngay

static void CompressFileInBackground(const std::string& filePath)
{
    std::thread([filePath]() {
        namespace fs = std::filesystem;
        try {
            if (!fs::exists(filePath)) return;
            fs::path p(filePath);
            fs::path dir = p.parent_path();
            fs::path filename = p.filename();
            fs::path zipPath = dir / (p.stem().string() + ".zip");

            // Dung tar.exe tich hop san trong Windows de nen lossless
            std::string cmd = "tar.exe -a -c -f \"" + zipPath.string() + "\" -C \"" + dir.string() + "\" \"" + filename.string() + "\"";

            STARTUPINFOA si{};
            si.cb = sizeof(si);
            si.dwFlags = STARTF_USESHOWWINDOW;
            si.wShowWindow = SW_HIDE;
            PROCESS_INFORMATION pi{};

            std::vector<char> cmdBuf(cmd.begin(), cmd.end());
            cmdBuf.push_back('\0');

            if (CreateProcessA(NULL, cmdBuf.data(), NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
                WaitForSingleObject(pi.hProcess, 120000); // Cho toi da 120 giay
                DWORD exitCode = 1;
                GetExitCodeProcess(pi.hProcess, &exitCode);
                CloseHandle(pi.hProcess);
                CloseHandle(pi.hThread);

                if (exitCode == 0 && fs::exists(zipPath) && fs::file_size(zipPath) > 0) {
                    uint64_t origSz = fs::file_size(filePath);
                    uint64_t zipSz  = fs::file_size(zipPath);
                    fs::remove(filePath); // Xoa file .bin goc sau khi da nen thanh cong
                    LogPrintf("[COMPRESS] Da nen lossless: %s (%llu MB) -> %s (%llu MB, tiet kiem %.1f%%)\n",
                        filename.string().c_str(), origSz / (1024 * 1024),
                        zipPath.filename().string().c_str(), zipSz / (1024 * 1024),
                        (origSz > 0) ? (100.0 * (1.0 - (double)zipSz / (double)origSz)) : 0.0);
                }
            }
        } catch (...) {}
    }).detach();
}

static void EnforceFolderQuota(const std::string& dirPath, uint64_t maxBytes, const std::string& tag)
{
    namespace fs = std::filesystem;
    try {
        if (!fs::exists(dirPath)) return;
        struct FileInfo {
            fs::path path;
            uint64_t size;
            fs::file_time_type lastWrite;
        };
        std::vector<FileInfo> files;
        uint64_t totalBytes = 0;

        for (const auto& entry : fs::directory_iterator(dirPath)) {
            if (!entry.is_regular_file()) continue;
            auto ext = entry.path().extension().string();
            if (ext != ".bin" && ext != ".zip" && ext != ".gz") continue;
            uint64_t sz = (uint64_t)entry.file_size();
            totalBytes += sz;
            files.push_back({ entry.path(), sz, entry.last_write_time() });
        }

        if (totalBytes > maxBytes) {
            std::sort(files.begin(), files.end(), [](const FileInfo& a, const FileInfo& b) {
                return a.lastWrite < b.lastWrite;
            });

            uint64_t targetBytes = (uint64_t)(maxBytes * 0.9); // Giam ve duoi 90% tran
            for (const auto& fi : files) {
                if (totalBytes <= targetBytes) break;
                try {
                    fs::remove(fi.path);
                    totalBytes = (totalBytes >= fi.size) ? (totalBytes - fi.size) : 0;
                    LogPrintf("[%s_QUOTA] Da tu dong xoa file cu de giu duoi tran (%llu MB): %s\n",
                        tag.c_str(), maxBytes / (1024 * 1024), fi.path.filename().string().c_str());
                } catch (...) {}
            }
        }
    } catch (...) {}
}

static void SaveEventBin(const std::vector<RawFrameSnapshot>& frames, int level)
{
    namespace fs = std::filesystem;
    try { fs::create_directories("recordings/events"); } catch (...) {}

    // Trich xuat ID muc tieu de dat ten file khop voi Camera AI
    std::string targetTag = "";
    std::set<int> tids;
    for (const auto& fr : frames) {
        if (fr.metaJson.empty()) continue;
        try {
            json mj = json::parse(fr.metaJson);
            if (mj.contains("targets") && mj["targets"].is_array()) {
                for (const auto& tg : mj["targets"]) {
                    if (tg.contains("id")) tids.insert(tg["id"].get<int>());
                }
            }
        } catch (...) {}
    }
    if (!tids.empty()) {
        targetTag = "_Target";
        for (int id : tids) targetTag += std::to_string(id) + "-";
        if (targetTag.back() == '-') targetTag.pop_back();
    }

    char fn[256]; time_t now = time(NULL); struct tm t;
    localtime_s(&t, &now);
    const char* label = (level >= 3) ? "CRITICAL" : (level == 2) ? "WARNING" : "WATCH";
    snprintf(fn, sizeof(fn), "recordings/events/event_%04d%02d%02d_%02d%02d%02d%s_L%d_%s.bin",
        t.tm_year+1900, t.tm_mon+1, t.tm_mday,
        t.tm_hour, t.tm_min, t.tm_sec, targetTag.c_str(), level, label);

    std::ofstream f(fn, std::ios::binary);
    if (!f.is_open()) { LogPrintf("[EVENT] Loi mo file su co: %s\n", fn); return; }

    f.write("LDRB", 4);
    uint32_t nf = (uint32_t)frames.size();
    f.write(reinterpret_cast<const char*>(&nf), 4);

    for (const auto& fr : frames) {
        uint32_t mlen = (uint32_t)fr.metaJson.size();
        f.write(reinterpret_cast<const char*>(&mlen), 4);
        if (mlen > 0) f.write(fr.metaJson.data(), mlen);

        f.write(reinterpret_cast<const char*>(&fr.n0), 4);
        if (fr.n0 > 0 && !fr.bin0.empty()) f.write(reinterpret_cast<const char*>(fr.bin0.data()), fr.bin0.size());

        f.write(reinterpret_cast<const char*>(&fr.n1), 4);
        if (fr.n1 > 0 && !fr.bin1.empty()) f.write(reinterpret_cast<const char*>(fr.bin1.data()), fr.bin1.size());
    }
    f.close();

    size_t szKB = (size_t)fs::file_size(fn) / 1024;
    LogPrintf("[EVENT] Da tu dong dong goi su co: %s | %u frames | %zu KB\n", fn, nf, szKB);

    // 1. Nen lossless file su co trong background
    CompressFileInBackground(fn);

    // 2. Kiem tra tran dung luong events
    EnforceFolderQuota("recordings/events", EVENT_MAX_QUOTA_BYTES, "EVENT");
}

static void UpdateEventPackager(const std::vector<FramePtr>& frames, int alarmLevel, const std::string& eventsMetaJson)
{
    // Loc frame duplicate: Chi xu ly khi co frame moi tu LiDAR de tranh ghi trung
    static std::vector<uint32_t> lastPackagerFrameIds;
    bool anyNewFrame = false;
    if (lastPackagerFrameIds.size() != frames.size()) {
        lastPackagerFrameIds.assign(frames.size(), 0xFFFFFFFF);
        anyNewFrame = true;
    }
    for (size_t i = 0; i < frames.size(); i++) {
        if (frames[i] && frames[i]->frame_id != lastPackagerFrameIds[i]) {
            lastPackagerFrameIds[i] = frames[i]->frame_id;
            anyNewFrame = true;
        }
    }
    if (!anyNewFrame) return;

    RawFrameSnapshot snap;
    time_t now = time(NULL);
    snap.ts_secs  = (uint32_t)now;
    snap.ts_usecs = 0;
    snap.alarmLevel = alarmLevel;
    snap.metaJson = eventsMetaJson;
    snap.n0 = snap.n1 = 0;

    for (size_t i = 0; i < frames.size() && i < 2; i++) {
        if (!frames[i]) continue;
        auto bin = PackPointsToBin(*frames[i]);
        if (i == 0) { snap.bin0 = std::move(bin); snap.n0 = (uint32_t)frames[i]->x_coords.size(); }
        else        { snap.bin1 = std::move(bin); snap.n1 = (uint32_t)frames[i]->x_coords.size(); }
    }

    {
        std::lock_guard<std::mutex> lk(gPreBufMutex);
        gPreBuffer.push_back(snap);
        while (gPreBuffer.size() > (size_t)EVENT_PRE_BUFFER_FRAMES)
            gPreBuffer.pop_front();
    }

    std::lock_guard<std::mutex> lk(gEventMutex);
    if (!gEventCapturing) {
        if (alarmLevel >= 2) {
            gEventCapturing = true;
            gEventAlarmLevel = alarmLevel;
            gEventPostFramesLeft = EVENT_POST_ALARM_FRAMES;
            {
                std::lock_guard<std::mutex> pb(gPreBufMutex);
                gEventFrames.assign(gPreBuffer.begin(), gPreBuffer.end());
            }
            LogPrintf("[EVENT] Phat hien su co cap %d! Bat dau tu dong ghi kem %zu pre-frames...\n",
                alarmLevel, gEventFrames.size());
        }
    } else {
        if (alarmLevel >= 2) {
            gEventAlarmLevel = std::max(gEventAlarmLevel, alarmLevel);
            gEventPostFramesLeft = EVENT_POST_ALARM_FRAMES;
        }

        gEventFrames.push_back(snap);
        bool overtime = (gEventFrames.size() >= MAX_EVENT_FRAMES);
        bool postDone = (alarmLevel < 2 && --gEventPostFramesLeft <= 0);

        if (overtime || postDone) {
            auto framesToSave = std::move(gEventFrames);
            int lvl = gEventAlarmLevel;
            gEventCapturing = false;
            gEventFrames.clear();
            std::thread([framesToSave = std::move(framesToSave), lvl]() {
                SaveEventBin(framesToSave, lvl);
            }).detach();
        }
    }
}

// ================= CIRCULAR RAW RECORDER =================
// Ghi toan bo frames raw 24/7 vao recordings/raw/, xoay file moi 1 gio, tu xoa cu hon 7 ngay
static std::atomic<bool>  gRawRecorderRun{ false };
static std::thread        gRawRecorderThread;
static std::mutex         gRawRecMutex;
struct RawWriteRequest { std::vector<uint8_t> data; };
static std::deque<RawWriteRequest> gRawWriteQueue;
static std::condition_variable gRawRecCV;

static std::vector<uint8_t> SerializeRawFrame(const PointCloudData& f)
{
    size_t n = f.x_coords.size();
    std::vector<uint8_t> buf(24 + n * 16);
    buf[0] = 'F'; buf[1] = 'R'; buf[2] = 'M'; buf[3] = '3';
    buf[4] = (uint8_t)f.source_id;
    buf[5] = 0; buf[6] = 0; buf[7] = 0;
    *reinterpret_cast<uint32_t*>(&buf[8])  = (uint32_t)f.frame_id;
    *reinterpret_cast<uint32_t*>(&buf[12]) = f.ts_secs;
    *reinterpret_cast<uint32_t*>(&buf[16]) = f.ts_usecs;
    *reinterpret_cast<uint32_t*>(&buf[20]) = (uint32_t)n;

    float* p = reinterpret_cast<float*>(&buf[24]);
    for (size_t i = 0; i < n; i++) {
        p[i*4+0] = f.x_coords[i];
        p[i*4+1] = f.y_coords[i];
        p[i*4+2] = (i < f.z_coords.size()) ? f.z_coords[i] : 0.0f;
        p[i*4+3] = (i < f.intensities.size()) ? f.intensities[i] : 0.0f;
    }
    return buf;
}

static void PurgeOldRawFiles(int keepDays = RAW_KEEP_DAYS)
{
    namespace fs = std::filesystem;
    try {
        if (!fs::exists("recordings/raw")) return;
        time_t now = time(NULL);
        time_t cutoff = now - (time_t)(keepDays * 24 * 3600);
        for (const auto& entry : fs::directory_iterator("recordings/raw")) {
            if (!entry.is_regular_file()) continue;
            auto ext = entry.path().extension().string();
            if (ext != ".bin" && ext != ".zip") continue;
            std::string name = entry.path().stem().string();
            if (name.rfind("raw_", 0) == 0 && name.size() >= 15) {
                int y = 0, m = 0, d = 0, h = 0;
                if (sscanf_s(name.c_str(), "raw_%04d%02d%02d_%02d", &y, &m, &d, &h) == 4) {
                    struct tm ft{};
                    ft.tm_year = y - 1900;
                    ft.tm_mon  = m - 1;
                    ft.tm_mday = d;
                    ft.tm_hour = h;
                    time_t ftime = mktime(&ft);
                    if (ftime > 0 && ftime < cutoff) {
                        fs::remove(entry.path());
                        LogPrintf("[RAW_REC] Da xoa file raw cu (>%d ngay): %s\n", keepDays, entry.path().filename().string().c_str());
                    }
                }
            }
        }
    } catch (...) {}
}

static void PurgeOldEventFiles(int keepDays = EVENT_KEEP_DAYS)
{
    namespace fs = std::filesystem;
    try {
        if (!fs::exists("recordings/events")) return;
        time_t now = time(NULL);
        time_t cutoff = now - (time_t)(keepDays * 24 * 3600);
        for (const auto& entry : fs::directory_iterator("recordings/events")) {
            if (!entry.is_regular_file()) continue;
            auto ext = entry.path().extension().string();
            if (ext != ".bin" && ext != ".zip") continue;
            std::string name = entry.path().stem().string();
            if (name.rfind("event_", 0) == 0 && name.size() >= 21) {
                int y = 0, m = 0, d = 0, H = 0, M = 0, S = 0;
                if (sscanf_s(name.c_str(), "event_%04d%02d%02d_%02d%02d%02d", &y, &m, &d, &H, &M, &S) == 6) {
                    struct tm ft{};
                    ft.tm_year = y - 1900;
                    ft.tm_mon  = m - 1;
                    ft.tm_mday = d;
                    ft.tm_hour = H;
                    ft.tm_min  = M;
                    ft.tm_sec  = S;
                    time_t ftime = mktime(&ft);
                    if (ftime > 0 && ftime < cutoff) {
                        fs::remove(entry.path());
                        LogPrintf("[EVENT] Da xoa file event cu (>%d ngay): %s\n", keepDays, entry.path().filename().string().c_str());
                    }
                }
            }
        }
    } catch (...) {}
}

static void RawRecorderThreadFunc()
{
    namespace fs = std::filesystem;
    try { fs::create_directories("recordings/raw"); } catch (...) {}

    std::ofstream curFile;
    std::string curFileName = "";
    int curHour = -1;
    DWORD64 lastPurge = 0;

    while (gRawRecorderRun.load()) {
        std::unique_lock<std::mutex> lk(gRawRecMutex);
        gRawRecCV.wait_for(lk, std::chrono::milliseconds(100),
            [] { return !gRawWriteQueue.empty() || !gRawRecorderRun.load(); });

        std::deque<RawWriteRequest> batch;
        batch.swap(gRawWriteQueue);
        lk.unlock();

        time_t now = time(NULL); struct tm t; localtime_s(&t, &now);
        if (t.tm_hour != curHour) {
            std::string prevHourFile = curFileName;
            if (curFile.is_open()) {
                curFile.close();
                if (!prevHourFile.empty()) {
                    CompressFileInBackground(prevHourFile);
                }
            }
            char fn[256];
            snprintf(fn, sizeof(fn), "recordings/raw/raw_%04d%02d%02d_%02d.bin",
                t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour);
            curFile.open(fn, std::ios::binary | std::ios::app);
            curHour = t.tm_hour;
            curFileName = fn;
            if (curFile.is_open())
                LogPrintf("[RAW_REC] Mo file raw 24/7 moi: %s\n", fn);

            EnforceFolderQuota("recordings/raw", RAW_MAX_QUOTA_BYTES, "RAW");
        }

        if (curFile.is_open()) {
            for (auto& req : batch)
                curFile.write(reinterpret_cast<const char*>(req.data.data()), req.data.size());
            curFile.flush();
        }

        DWORD64 nowMs = GetTickCount64();
        if (nowMs - lastPurge > 3600000ULL) {
            lastPurge = nowMs;
            PurgeOldRawFiles(RAW_KEEP_DAYS);
            PurgeOldEventFiles(EVENT_KEEP_DAYS);
            EnforceFolderQuota("recordings/raw", RAW_MAX_QUOTA_BYTES, "RAW");
            EnforceFolderQuota("recordings/events", EVENT_MAX_QUOTA_BYTES, "EVENT");
        }
    }
    if (curFile.is_open()) {
        curFile.close();
        if (!curFileName.empty()) {
            CompressFileInBackground(curFileName);
        }
    }
    LogPrintf("[RAW_REC] Raw recorder da dung.\n");
}

static void EnqueueRawFrame(const std::vector<uint8_t>& frameBytes)
{
    std::lock_guard<std::mutex> lk(gRawRecMutex);
    if (gRawWriteQueue.size() < 500)
        gRawWriteQueue.push_back({ frameBytes });
    gRawRecCV.notify_one();
}

static void StartRawRecorder()
{
    if (!ENABLE_RAW_RECORDER) {
        LogPrintf("[RAW_REC] Circular Raw Recorder da duoc TAT (Che do van hanh thuc te - Chi ghi Event).\n");
        return;
    }
    if (gRawRecorderRun.load()) return;
    gRawRecorderRun = true;
    gRawRecorderThread = std::thread(RawRecorderThreadFunc);
    LogPrintf("[RAW_REC] Bat dau Circular Raw Recorder (24/7, xoay file moi 1 gio, luu 7 ngay).\n");
}

static void StopRawRecorder()
{
    if (!gRawRecorderRun.load()) return;
    gRawRecorderRun = false;
    gRawRecCV.notify_all();
    if (gRawRecorderThread.joinable()) gRawRecorderThread.join();
}

static json BuildEventsJson()
{
    json j;
    j["type"] = "events";
    j["recording"] = gIsRecording.load();
    j["occlusion"] = gOcclusionAlarm;

    // ================= LICENSE GATE CHO TÍNH NĂNG SỰ KIỆN & CẢNH BÁO =================
    // Nếu ở chế độ DEMO (chưa có license hoặc hết hạn):
    // - Khóa phân cấp an toàn ray (ép về SAFE = 0)
    // - Khóa danh sách đối tượng nhận diện mục tiêu (không gửi targets)
    // - Thông báo trạng thái DEMO lên giao diện Web
    LicenseSystem::LicenseStatus lic;
    {
        std::lock_guard<std::mutex> lk(gLicMutex);
        lic = gLicStatus;
    }
    j["licenseValid"] = lic.isValid;
    j["isDemoMode"] = lic.isDemoMode;
    j["inGracePeriod"] = lic.inGracePeriod;
    j["licenseMsg"] = lic.message;

    if (lic.isDemoMode) {
        j["alarmLevel"] = 0;
        j["alarmLabel"] = "DEMO MODE (Chưa kích hoạt bản quyền)";
        j["targets"] = json::array();
        j["recent_targets"] = json::array();
        return j;
    }
    j["clusterMode"] = gClusterMode;
    j["sel"] = gSelectedLidar;
    j["paused"] = gIsPaused.load();
    json tgs = json::array(); int confirmed = 0;
    if (gBgFrames > 15) {
        for (auto& t : gTargets)
        {
            bool isReal = IsRealMotionTarget(t);
            if (t.age >= PERSISTENCE_FRAMES && isReal) confirmed++;
            tgs.push_back({
                {"id", t.id},
                {"x", t.x}, {"y", t.y},
                {"vx", t.vx}, {"vy", t.vy},
                {"w", t.w}, {"h", t.h},
                {"maxW", t.maxW}, {"maxH", t.maxH},
                {"maxSpeed", t.maxSpeed},
                {"criticalX", t.criticalX}, {"criticalY", t.criticalY},
                {"exitX", t.exitX}, {"exitY", t.exitY},
                {"n", t.n}, {"age", t.age},
                {"z1", t.z1}, {"z2", t.z2},
                {"realMotion", isReal}
            });
        }
    }
    j["targets"] = tgs; j["confirmed"] = confirmed;

    int lvl = ComputeAlarmLevel(confirmed, (int)tgs.size());
    gAlarmLevel.store(lvl);
    j["alarmLevel"] = lvl;
    j["alarmOn"] = (lvl >= 2);
    j["dangerStatus"] = (lvl == 3) ? "CRITICAL" : (lvl == 2) ? "WARNING" : (lvl == 1) ? "WATCH" : "SAFE";

    json bxs = json::array();
    for (auto& b : gBoxes) bxs.push_back({ {"x1",b.x1},{"y1",b.y1},{"x2",b.x2},{"y2",b.y2},{"n",b.n} });
    j["boxes"] = bxs; j["nearest"] = gNearest; j["nObj"] = gNObj;

    // Gửi kèm danh sách đối tượng lịch sử gần đây (phục vụ Camera AI hiển thị đầy đủ ngay cả khi mở muộn)
    json rts = json::array();
    {
        std::lock_guard<std::mutex> lk(gTargetHistoryMutex);
        for (const auto& r : gRecentTargets) {
            rts.push_back({
                {"id", r.id},
                {"x", r.x}, {"y", r.y}, {"z1", r.z1}, {"z2", r.z2},
                {"w", r.w}, {"h", r.h},
                {"maxW", r.maxW}, {"maxH", r.maxH},
                {"vx", r.vx}, {"vy", r.vy},
                {"curSpeed", r.curSpeed},
                {"maxSpeed", r.maxSpeed},
                {"criticalX", r.criticalX}, {"criticalY", r.criticalY},
                {"exitX", r.exitX}, {"exitY", r.exitY},
                {"pointCount", r.pointCount},
                {"active", r.active},
                {"firstSeenMs", r.firstSeenMs},
                {"lastSeenMs", r.lastSeenMs}
            });
        }
    }
    j["recent_targets"] = rts;
    return j;
}

static std::atomic<bool> gRun{ true };

static void TickLoop()
{
    // >>> v3: lastSent dung vector dong <<<
    std::vector<FramePtr> lastSent;
    std::vector<uint32_t> lastRawFrameIds;
    DWORD64 lastEvent = 0;
    DWORD64 lastTelemetry = 0;
    DWORD64 lastLicenseCheck = 0;
    while (gRun.load())
    {
        DWORD64 nowMs = GetTickCount64();
        if (nowMs - lastLicenseCheck >= 60000) {
            lastLicenseCheck = nowMs;
            LicenseSystem::LicenseStatus newSt = LicenseSystem::VerifyLicenseFile("license.key", gHwInfo);
            bool changed = false;
            {
                std::lock_guard<std::mutex> lk(gLicMutex);
                if (newSt.isDemoMode != gLicStatus.isDemoMode || newSt.inGracePeriod != gLicStatus.inGracePeriod || newSt.message != gLicStatus.message) {
                    changed = true;
                }
                gLicStatus = newSt;
                gIsDemoMode.store(newSt.isDemoMode);
            }
            if (changed) {
                LogPrintf("[LICENSE] Cap nhat trang thai runtime: %s\n", newSt.message.c_str());
            }
        }
        {
            std::lock_guard<std::recursive_mutex> sourcesLock(gSourcesMutex);
            auto frames = GetFrames(0);
            {
                std::lock_guard<std::mutex> lk(gZoneMutex);
                CheckOcclusion(frames);
                UpdateMotionDetection(frames, nowMs);
                if (gClusterMode && nowMs - gClusterLast > 500) { gClusterLast = nowMs; ComputeClusters(frames); }
            }

            // Lay trang thai su kien va phan cap bao dong (4 muc)
            json evJson = BuildEventsJson();
            int curAlarmLevel = evJson.value("alarmLevel", 0);
            std::string curEventsJsonStr = evJson.dump();

            // 1. Kiem tra va tu dong dong goi su co vao recordings/events/
            UpdateEventPackager(frames, curAlarmLevel, curEventsJsonStr);

            // 2. Ghi vao Circular Raw Recorder (neu duoc bat)
            if (ENABLE_RAW_RECORDER) {
                if (lastRawFrameIds.size() != frames.size())
                    lastRawFrameIds.assign(frames.size(), 0xFFFFFFFF);
                for (size_t i = 0; i < frames.size(); i++) {
                    const auto& fr = frames[i];
                    if (fr && !fr->x_coords.empty() && fr->frame_id != lastRawFrameIds[i]) {
                        lastRawFrameIds[i] = fr->frame_id;
                        EnqueueRawFrame(SerializeRawFrame(*fr));
                    }
                }
            }

            if (HasClients())
            {
                lastSent.resize(frames.size());
                for (size_t i = 0; i < frames.size() && i < lastSent.size(); i++)
                    if (frames[i] && (!lastSent[i] || frames[i]->frame_id != lastSent[i]->frame_id || frames[i]->source_id != lastSent[i]->source_id || frames[i]->origin_x != lastSent[i]->origin_x || frames[i]->origin_y != lastSent[i]->origin_y || frames[i]->origin_z != lastSent[i]->origin_z || frames[i]->origin_roll != lastSent[i]->origin_roll || frames[i]->origin_pitch != lastSent[i]->origin_pitch || frames[i]->origin_yaw != lastSent[i]->origin_yaw))
                    {
                        lastSent[i] = frames[i];
                        BroadcastBinary(*SerializeFrameV3(*frames[i]));
                    }
                if (nowMs - lastEvent > 100) { lastEvent = nowMs; BroadcastText(curEventsJsonStr); }
                if (nowMs - lastTelemetry >= 1000) { lastTelemetry = nowMs; BroadcastText(BuildLatencyTelemetry().dump()); }
            }
        }
        DrainWebJson();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
// >>> v3.2: PING - thoi gian phan hoi cua 1 goi tin mang (ICMP echo) <<<
static void DoPing(int id)
{
    if (id < 0 || id >= (int)gSources.size()) return;
    std::string ip = gSources[id]->GetIp();
    in_addr bin{};
    IPAddr addr = (InetPtonA(AF_INET, ip.c_str(), &bin) == 1)
        ? (IPAddr)bin.S_un.S_addr : INADDR_NONE;
    int ms = -1;
    HANDLE h = IcmpCreateFile();
    if (h != INVALID_HANDLE_VALUE && addr != INADDR_NONE)
    {
        char data[32]; memset(data, 'L', sizeof(data));
        DWORD bufSz = sizeof(ICMP_ECHO_REPLY) + sizeof(data) + 8;
        std::vector<char> buf(bufSz);
        for (int i = 0; i < 4; i++)
        {
            PICMP_ECHO_REPLY rep = (PICMP_ECHO_REPLY)buf.data();
            DWORD r = IcmpSendEcho2(h, NULL, NULL, NULL, addr, data, (WORD)sizeof(data),
                NULL, rep, bufSz, 1000);
            if (r > 0) { ms = (int)rep->RoundTripTime; break; }
        }
        IcmpCloseHandle(h);
    }
    LogPrintf("[PING] %s: %d ms\n", ip.c_str(), ms);
    json j; j["type"] = "ping"; j["id"] = id; j["ip"] = ip; j["ms"] = ms;
    QueueWebJson(j);
}
// ================= HANDLE MESSAGE (v3: them lenh moi) =================
static void HandleMessage(ConnHdl hdl, WsServer::message_ptr msg)
{
    std::lock_guard<std::recursive_mutex> sourcesLock(gSourcesMutex);
    (void)hdl;
    try
    {
        auto j = json::parse(msg->get_payload());
        std::string cmd = j.value("cmd", "");

        if (cmd == "select") { gSelectedLidar = j.at("id").get<int>(); BroadcastExtr(gSelectedLidar); }
        else if (cmd == "extr_apply")
        {
            int id = j.at("id").get<int>();
            if (id >= 0 && id < (int)gSources.size())
            {
                Extrinsic3D e = gSources[id]->GetExtr();
                e.x = j.value("x", j.value("dx", e.x)); e.y = j.value("y", j.value("dy", e.y));
                e.z = j.value("z", e.z);
                e.roll = Deg2Rad(j.value("roll", Rad2Deg(e.roll)));
                e.pitch = Deg2Rad(j.value("pitch", Rad2Deg(e.pitch)));
                e.yaw = Deg2Rad(j.value("yaw", j.value("thetaDeg", Rad2Deg(e.yaw))));
                if (!std::isfinite(e.x) || !std::isfinite(e.y) || !std::isfinite(e.z) || !std::isfinite(e.roll) || !std::isfinite(e.pitch) || !std::isfinite(e.yaw)) return;
                gSources[id]->SetExtr(e);
                ResetMotionModel();
                BroadcastExtr(id);
            }
        }
        else if (cmd == "extr_adjust")
            AdjustSelectedExtr((float)j.value("ddx", 0.0), (float)j.value("ddy", 0.0), Deg2Rad((float)j.value("dth", 0.0)));
        else if (cmd == "extr_save") SaveExtrConfig();
        else if (cmd == "extr_print") PrintSelectedExtr();
        else if (cmd == "record") ToggleRecording();
        else if (cmd == "pause")
        {
            bool p = j.value("paused", false);
            gIsPaused = p;
            for (auto& src : gSources) src->SetPaused(p);
            LogPrintf("[INFO] %s\n", p ? "PAUSED - backend NGUNG nhan/xu ly du lieu"
                : "RESUMED - backend chay lai");
        }
        else if (cmd == "cluster") gClusterMode = !gClusterMode;
        else if (cmd == "rmse_start")
        {
            float v = (float)j.value("true", 0.0);
            if (v <= 0) QueueWebJson({ {"type","log"},{"text","[RMSE] Nhap true > 0"} });
            else
            {
                std::lock_guard<std::mutex> lk(gRmseMutex);
                gRmseTrue = v; gRmseSum = 0; gRmseSumSq = 0; gRmseCount = 0; gRmseActive = true;
            }
        }
        else if (cmd == "icp") RunIcpRefine();
        else if (cmd == "auto_align")
        {
            int targetId = j.value("id", gSelectedLidar);
            std::thread([targetId]() {
                RunGlobalAutoAlign(targetId);
                }).detach();
        }
        else if (cmd == "map_save")
        {
            std::string name = j.value("name", "map_latest");
            std::string yaml = j.value("yaml", "");
            std::string jsonContent = j.value("json", "");
            std::string pngB64 = j.value("png_base64", "");
            SaveMapFiles(name, yaml, jsonContent, pngB64);
        }
        else if (cmd == "lidar_command_rtt")
        {
            if (!gSources.empty() && gSources[0]->IsRunning())
                MeasureCommandLatency(gSources[0]->GetLadar(), j.value("sleep", false));
        }
        else if (cmd == "get_latency_metrics") QueueWebJson(BuildLatencyTelemetry());
        else if (cmd == "visual_config")
        {
            std::string host = j.value("host", "");
            int portValue = j.value("port", 0);
            if (host.size() > 255 || portValue < 0 || portValue > 65535) return;
            const auto port = (unsigned short)portValue;
            gVisualAdapter.Configure(host, port);
            LogPrintf("[VISUAL] endpoint %s\n", host.empty() || !port ? "disabled" : "configured (JSONL/TCP)");
            QueueWebJson(BuildLatencyTelemetry());
        }
        else if (cmd == "pick_apply")
        {
            auto rf = j.at("ref"), tg = j.at("target");
            for (int k = 0; k < 2; k++)
            {
                gPickRef[k] = { (float)rf[k]["x"].get<double>(), (float)rf[k]["y"].get<double>() };
                gPickTarget[k] = { (float)tg[k]["x"].get<double>(), (float)tg[k]["y"].get<double>() };
            }
            gPickNRef = 2; gPickNTarget = 2;
            ApplyPickCalib();
        }
        // >>> v3: zones_set ho tro nhieu loai hinh <<<
        else if (cmd == "zones_set")
        {
            std::lock_guard<std::mutex> lk(gZoneMutex);
            gZones.clear();
            for (auto& z : j.at("zones"))
            {
                Zone zn;
                zn.local = (z.value("scope", "fusion") == "local");
                zn.exclusion = z.value("exclusion", false);
                zn.id = z.value("id", 0);

                // Code 1 compatibility:
                // - New local Zones explicitly carry sourceId.
                // - Old local Zones used id as the LiDAR index.
                // - Fusion/world Zones do not belong to a specific LiDAR.
                zn.sourceId = (zn.local && z.contains("sourceId") && !z["sourceId"].is_null())
                    ? z["sourceId"].get<int>()
                    : (zn.local ? zn.id : 0);

                zn.type = z.value("type", 0);  // >>> v3: type <<<

                if (zn.type == ZT_RECT)
                {
                    // Tuong thich cu: x1,y1,x2,y2
                    zn.x1 = (float)z.at("x1").get<double>();
                    zn.y1 = (float)z.at("y1").get<double>();
                    zn.x2 = (float)z.at("x2").get<double>();
                    zn.y2 = (float)z.at("y2").get<double>();
                }
                else if (zn.type == ZT_POLYGON || zn.type == ZT_FREEHAND)
                {
                    // >>> v3: Polygon/Freehand nhan mang vertices <<<
                    if (z.contains("vertices"))
                        for (auto& v : z["vertices"])
                            zn.vertices.push_back({ (float)v[0].get<double>(), (float)v[1].get<double>() });
                }
                else if (zn.type == ZT_CIRCLE)
                {
                    // >>> v3: Circle nhan cx, cy, radius <<<
                    zn.cx = (float)z.at("cx").get<double>();
                    zn.cy = (float)z.at("cy").get<double>();
                    zn.radius = (float)z.at("radius").get<double>();
                }
                else if (zn.type == ZT_SECTOR)
                {
                    // >>> v3: Sector nhan sx, sy, sRadius, sAngleStart, sAngleEnd <<<
                    zn.sx = (float)z.at("sx").get<double>();
                    zn.sy = (float)z.at("sy").get<double>();
                    zn.sRadius = (float)z.at("sRadius").get<double>();
                    zn.sAngleStart = Deg2Rad((float)z.at("sAngleStart").get<double>());
                    zn.sAngleEnd = Deg2Rad((float)z.at("sAngleEnd").get<double>());
                }
                gZones.push_back(zn);
            }
            ResetMotionModel();
            LogPrintf("[ZONE] Nhan %d vung canh bao (da hinh)\n", (int)gZones.size());
        }
        else if (cmd == "zone_clear")
        {
            std::lock_guard<std::mutex> lk(gZoneMutex);
            gZones.clear();
            ResetMotionModel();
        }
        // >>> v3: Cap nhat dinh zone (vertex editing) <<<
        else if (cmd == "zone_update")
        {
            std::lock_guard<std::mutex> lk(gZoneMutex);
            int zid = j.at("zoneId").get<int>();
            for (size_t i = 0; i < gZones.size(); i++)
            {
                if (gZones[i].id == zid)
                {
                    if (j.contains("vertices"))
                    {
                        gZones[i].vertices.clear();
                        for (auto& v : j["vertices"])
                            gZones[i].vertices.push_back({ (float)v[0].get<double>(), (float)v[1].get<double>() });
                    }
                    if (j.contains("x1")) gZones[i].x1 = (float)j["x1"].get<double>();
                    if (j.contains("y1")) gZones[i].y1 = (float)j["y1"].get<double>();
                    if (j.contains("x2")) gZones[i].x2 = (float)j["x2"].get<double>();
                    if (j.contains("y2")) gZones[i].y2 = (float)j["y2"].get<double>();
                    if (j.contains("cx")) gZones[i].cx = (float)j["cx"].get<double>();
                    if (j.contains("cy")) gZones[i].cy = (float)j["cy"].get<double>();
                    if (j.contains("radius")) gZones[i].radius = (float)j["radius"].get<double>();
                    if (j.contains("sRadius")) gZones[i].sRadius = (float)j["sRadius"].get<double>();
                    if (j.contains("sAngleStart")) gZones[i].sAngleStart = Deg2Rad((float)j["sAngleStart"].get<double>());
                    if (j.contains("sAngleEnd")) gZones[i].sAngleEnd = Deg2Rad((float)j["sAngleEnd"].get<double>());
                    if (j.contains("exclusion")) gZones[i].exclusion = j["exclusion"].get<bool>();

                    // Keep local Zone ownership independently from Zone ID.
                    if (j.contains("sourceId") && !j["sourceId"].is_null())
                        gZones[i].sourceId = j["sourceId"].get<int>();

                    break;
                }
            }
            ResetMotionModel();
            LogPrintf("[ZONE] Cap nhat zone %d\n", zid);
        }
        // >>> v3: Cau hinh IP LiDAR dong <<<
        else if (cmd == "lidar_set_ip")
        {
            int id = j.at("id").get<int>();
            std::string newIp = j.at("ip").get<std::string>();
            uint16_t newPort = (uint16_t)j.value("port", 1112);
            if (id >= 0 && id < (int)gSources.size())
            {
                auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[id]);
                if (src)
                {
                    LogPrintf("[IP] Thay doi LiDAR %d: %s:%d -> %s:%d\n", id + 1,
                        src->GetIp().c_str(), src->GetPort(), newIp.c_str(), newPort);
                    src->Reconnect(newIp, newPort);
                    SaveLidarConfig();
                    BroadcastLidarConfig();
                }
            }
        }
        // >>> v3: Them LiDAR moi <<<
        else if (cmd == "lidar_add")
        {
            if ((int)gSources.size() >= MAX_LIDARS)
            {
                QueueWebJson({ {"type","log"},{"text","[ERROR] Toi da 16 LiDAR"} });
            }
            else
            {
                std::string newIp = j.value("ip", "192.168.201.15");
                uint16_t newPort = (uint16_t)j.value("port", 1112);
                int newId = (int)gSources.size();
                std::string nm = SanitizeName(j.value("name", ""));
                if (nm.empty() || j.value("name", "") == "") { char buf[32]; sprintf_s(buf, "LiDAR_%d", newId + 1); nm = buf; }
                Extrinsic3D defExtr;
                auto src = std::make_shared<RealLidarSource>(newId, nm, newIp, newPort, defExtr);
                src->SetEnableCsvRecord(true);
                gSources.push_back(src);
                src->Start();
                gActiveLidarCount = (int)gSources.size();
                SaveLidarConfig();
                SaveExtrConfig();
                BroadcastLidarConfig();
                LogPrintf("[IP] Them LiDAR %d: %s:%d\n", newId + 1, newIp.c_str(), newPort);
            }
        }
        // >>> v3: Xoa LiDAR <<<
        else if (cmd == "lidar_remove")
        {
            int id = j.at("id").get<int>();
            if (id >= 0 && id < (int)gSources.size() && gSources.size() > 1)
            {
                gSources[id]->Stop();
                gSources.erase(gSources.begin() + id);
                // Cap nhat source_id cho cac source con lai
                for (size_t i = 0; i < gSources.size(); i++)
                {
                    // Khong co SetSourceId, nen ta chi log
                }
                if (gSelectedLidar >= (int)gSources.size())
                    gSelectedLidar = (int)gSources.size() - 1;
                gActiveLidarCount = (int)gSources.size();
                SaveLidarConfig();
                SaveExtrConfig();
                BroadcastLidarConfig();
                LogPrintf("[IP] Xoa LiDAR %d. Con lai: %d\n", id + 1, (int)gSources.size());
            }
        }
        // >>> v3: Lay cau hinh LiDAR <<<
        else if (cmd == "lidar_get_config")
        {
            BroadcastLidarConfig();
        }
        // >>> v3: Reconnect 1 LiDAR <<<
        else if (cmd == "lidar_reconnect")
        {
            int id = j.at("id").get<int>();
            if (id >= 0 && id < (int)gSources.size())
            {
                auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[id]);
                if (src)
                {
                    src->Reconnect(src->GetIp(), src->GetPort());
                    LogPrintf("[IP] Reconnect LiDAR %d\n", id + 1);
                }
            }
        }
        // >>> v3.1: DOI TEN LiDAR (khong reconnect) <<<
        else if (cmd == "lidar_rename")
        {
            int id = j.at("id").get<int>();
            std::string nm = j.value("name", "");
            if (id >= 0 && id < (int)gSources.size() && !nm.empty())
            {
                gSources[id]->SetName(nm);
                LogPrintf("[IP] Doi ten LiDAR %d -> %s\n", id + 1, gSources[id]->GetName().c_str());
                SaveLidarConfig();        // luu vao lidar_config.cfg
                BroadcastLidarConfig();   // cap nhat web
            }
        }
        else if (cmd == "ping") DoPing(j.value("id", 0));
        else if (cmd == "get_state" || cmd == "get_target_history")
        {
            json arr = json::array();
            for (size_t i = 0; i < gSources.size(); i++)
            {
                Extrinsic3D e = gSources[i]->GetExtr();
                arr.push_back({ {"id",(int)i},{"name",gSources[i]->GetName()},
                    {"dx",e.x},{"dy",e.y},{"thetaDeg",Rad2Deg(e.yaw)},
                    {"x",e.x},{"y",e.y},{"z",e.z},{"roll",Rad2Deg(e.roll)},{"pitch",Rad2Deg(e.pitch)},{"yaw",Rad2Deg(e.yaw)} });
            }
            json out; out["type"] = (cmd == "get_target_history") ? "target_history" : "state";
            out["lidars"] = arr;
            json rts = json::array();
            {
                std::lock_guard<std::mutex> lk(gTargetHistoryMutex);
                for (const auto& r : gRecentTargets) {
                    rts.push_back({
                        {"id", r.id},
                        {"x", r.x}, {"y", r.y}, {"z1", r.z1}, {"z2", r.z2},
                        {"w", r.w}, {"h", r.h},
                        {"maxW", r.maxW}, {"maxH", r.maxH},
                        {"vx", r.vx}, {"vy", r.vy},
                        {"curSpeed", r.curSpeed},
                        {"maxSpeed", r.maxSpeed},
                        {"criticalX", r.criticalX}, {"criticalY", r.criticalY},
                        {"exitX", r.exitX}, {"exitY", r.exitY},
                        {"pointCount", r.pointCount},
                        {"active", r.active},
                        {"firstSeenMs", r.firstSeenMs},
                        {"lastSeenMs", r.lastSeenMs}
                    });
                }
            }
            out["recent_targets"] = rts;
            BroadcastText(out.dump());
        }
    }
    catch (...) {}
}

// ================= MAIN (v3: doc lidar_config.cfg) =================
int main(int argc, char** argv)
{
    WSADATA winsockData{};
    if (WSAStartup(MAKEWORD(2, 2), &winsockData) != 0) {
        fprintf(stderr, "WSAStartup failed\n"); return 1;
    }
    {
        HANDLE hOrig = GetStdHandle(STD_OUTPUT_HANDLE);
        HANDLE hDup = NULL;
        if (hOrig && hOrig != INVALID_HANDLE_VALUE)
        {
            DuplicateHandle(GetCurrentProcess(), hOrig,
                GetCurrentProcess(), &hDup, 0, FALSE, DUPLICATE_SAME_ACCESS);
        }
        gConOut = hDup ? hDup : hOrig;
    }
    freopen("lidar_spam_stdout.log", "w", stdout);
    freopen("lidar_spam_stderr.log", "w", stderr);
    // [FIX] Gioi han lidar_web_log.txt: neu vuot 10 MB thi xoa va mo moi (tranh phong to vo han)
    { struct stat _st{}; if (stat("lidar_web_log.txt", &_st) == 0 && _st.st_size > 10LL * 1024 * 1024) remove("lidar_web_log.txt"); }
    gLogFile = fopen("lidar_web_log.txt", "a");
    LogPrintf("\n=== LiDAR WEB GATEWAY v3 ===\n");

    // ================= KIỂM TRA BẢN QUYỀN OFFLINE (LICENSE GATE) =================
    gHwInfo = LicenseSystem::CollectHardwareFingerprint();
    {
        std::lock_guard<std::mutex> lk(gLicMutex);
        gLicStatus = LicenseSystem::VerifyLicenseFile("license.key", gHwInfo);
        gIsDemoMode.store(gLicStatus.isDemoMode);
    }
    
    LogPrintf("------------------------------------------------------------------\n");
    LogPrintf("[LICENSE] Ma phan cung (HWID) : %s\n", gHwInfo.compositeHwid.c_str());
    LogPrintf("[LICENSE] Trang thai ban quyen : %s\n", gLicStatus.message.c_str());
    if (gLicStatus.isDemoMode) {
        LogPrintf("[LICENSE] >>> CHU Y: Dang chay CHE DO DEMO. Cảnh bao an toan ray & TCP 9100 bi vo hieu hoa. <<<\n");
    } else {
        LogPrintf("[LICENSE] >>> XAC THUC THANH CONG: Mo khoa toan bo he thong an toan duong sat. <<<\n");
    }
    LogPrintf("------------------------------------------------------------------\n");

    // >>> v3: Doc cau hinh IP tu lidar_config.cfg, neu khong co thi dung mac dinh <<<
    std::vector<LidarConfigEntry> cfgEntries;
    if (!LoadLidarConfig(cfgEntries))
    {
        // Mac dinh 4 LiDAR
        const char* DEF_IPS[4] = { "192.168.201.15","192.168.201.16","192.168.201.17","192.168.201.18" };
        for (int i = 0; i < 4; i++)
        {
            LidarConfigEntry e;
            e.ip = DEF_IPS[i]; e.port = 1112;
            char nm[32]; sprintf_s(nm, "TR70_Real%d", i + 1);
            e.name = nm;
            cfgEntries.push_back(e);
        }
        LogPrintf("[CFG] Khong tim thay %s, dung mac dinh 4 LiDAR\n", LIDAR_CFG_FILE);
    }
    else
        LogPrintf("[CFG] Nap %s: %d LiDAR\n", LIDAR_CFG_FILE, (int)cfgEntries.size());

    int nLidars = (int)cfgEntries.size();
    if (nLidars > MAX_LIDARS) nLidars = MAX_LIDARS;

    std::vector<Extrinsic3D> extrs(nLidars);
    // Extrinsics mac dinh
    if (nLidars >= 4)
    {
        extrs[0] = { 0,0,0 }; extrs[1] = { 5,0,0 };
        extrs[2] = { 0,5,0,0,0,Deg2Rad(90) }; extrs[3] = { 5,5,0,0,0,Deg2Rad(180) };
    }
    if (LoadExtrConfig(extrs)) LogPrintf("[CFG] Nap %s\n", EXTR_CFG);

    if (argc > 1)
    {
        LogPrintf("[INIT] CSV: %s\n", argv[1]);
        gSources.push_back(std::make_shared<CsvLidarPlayer>(0, "CSV_Player", argv[1], extrs.empty() ? Extrinsic3D() : extrs[0]));
    }
    else
    {
        for (int i = 0; i < nLidars; i++)
        {
            auto src = std::make_shared<RealLidarSource>(i, cfgEntries[i].name,
                cfgEntries[i].ip, cfgEntries[i].port, extrs[i]);
            src->SetEnableCsvRecord(true);
            gSources.push_back(src);
        }
    }

    gActiveLidarCount = (int)gSources.size();
    gConnectUs = NowUs();
    for (auto& s : gSources) s->Start();

    try
    {
        g_server.init_asio();
        // [FIX] Tat toan bo verbose log cua websocketpp -> khong con ghi vao lidar_spam_stdout.log
        g_server.clear_access_channels(websocketpp::log::alevel::all);
        g_server.clear_error_channels(websocketpp::log::elevel::all);
        g_server.set_error_channels(websocketpp::log::elevel::fatal);
        g_server.set_reuse_addr(true);
        g_server.set_open_handler([](ConnHdl h) {
            std::lock_guard<std::mutex> lk(g_connMutex); g_conns.insert(h);
            LogPrintf("[WEB] client connect\n");
            // >>> v3: Gui cau hinh LiDAR khi client ket noi <<<
            BroadcastLidarConfig();
            });
        g_server.set_close_handler([](ConnHdl h) {
            std::lock_guard<std::mutex> lk(g_connMutex); g_conns.erase(h);
            LogPrintf("[WEB] client disconnect\n");
            });
        g_server.set_message_handler([](ConnHdl h, WsServer::message_ptr m) { HandleMessage(h, m); });
        g_server.listen(9001);
        g_server.start_accept();
        LogPrintf("WebSocket listening ws://0.0.0.0:9001\n");
    }
    catch (std::exception& e) { LogPrintf("[ERROR] WS: %s\n", e.what()); return 1; }

    gVisualAdapter.Start();
    StartRawRecorder();
    std::thread wsTh([]() { try { g_server.run(); } catch (...) {} });
    std::thread tick(TickLoop);
    LogPrintf("Press q + Enter to quit.\n");
    char c;
    while (std::cin.get(c)) if (c == 'q' || c == 'Q') break;
    gRun = false;
    if (tick.joinable()) tick.join();
    StopRawRecorder();
    g_server.stop_listening(); g_server.stop();
    if (wsTh.joinable()) wsTh.join();
    gVisualAdapter.Stop();
    if (gIsRecording.load()) ToggleRecording();
    for (auto& s : gSources) s->Stop();
    if (gLogFile) fclose(gLogFile);
    WSACleanup();
    return 0;
}
