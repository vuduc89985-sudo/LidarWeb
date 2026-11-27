#pragma once
#ifndef OCCUPANCY_GRID2D_H
#define OCCUPANCY_GRID2D_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <string>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>

// Thang do log-odds dang so nguyen (int8_t tu -127 den 127)
//  0: Chua quet (Unknown - xam 205)
// < 0: Vung trong (Free Space - trang 254)
// > 0: Vat can / Tuong (Occupied - den 0)
class OccupancyGrid2D
{
public:
    float origin_x = -40.0f;  // Toa do goc duoi-trai trong World (m)
    float origin_y = -40.0f;
    float width_m  = 80.0f;   // Chieu rong khong gian (m)
    float height_m = 80.0f;   // Chieu dai khong gian (m)
    float resolution = 0.05f; // Do phan giai: 0.05m = 5cm / cell

    int width_cells = 0;
    int height_cells = 0;

    std::vector<int8_t> grid;
    mutable std::mutex grid_mutex;

    // Tham so tang giam xac suat
    const int8_t LOG_ODDS_FREE = -3;   // Giam diem khi tia xuyen qua
    const int8_t LOG_ODDS_OCC  = 20;   // Cong diem khi tia cham vat can
    const int8_t LOG_ODDS_MIN  = -100;
    const int8_t LOG_ODDS_MAX  = 100;

    OccupancyGrid2D(float res = 0.05f, float size_m = 80.0f)
    {
        Init(res, size_m);
    }

    void Init(float res, float size_m)
    {
        std::lock_guard<std::mutex> lk(grid_mutex);
        resolution = res;
        width_m = size_m;
        height_m = size_m;
        origin_x = -size_m / 2.0f;
        origin_y = -size_m / 2.0f;

        width_cells = (int)std::ceil(width_m / resolution);
        height_cells = (int)std::ceil(height_m / resolution);

        grid.assign(width_cells * height_cells, 0);
    }

    void Reset()
    {
        std::lock_guard<std::mutex> lk(grid_mutex);
        std::fill(grid.begin(), grid.end(), 0);
    }

    // Chuyen toa do World (m) -> Toa do luoi (cell index)
    inline bool WorldToGrid(float wx, float wy, int& gx, int& gy) const
    {
        gx = (int)std::floor((wx - origin_x) / resolution);
        gy = (int)std::floor((wy - origin_y) / resolution);
        return (gx >= 0 && gx < width_cells && gy >= 0 && gy < height_cells);
    }

    // Bresenham 2D Ray-Casting: ban 1 tia tu sensor den diem phan xa
    void UpdateRay(float x0, float y0, float x1, float y1, bool isHit)
    {
        int gx0, gy0, gx1, gy1;
        if (!WorldToGrid(x0, y0, gx0, gy0)) return;
        
        bool hitInBounds = WorldToGrid(x1, y1, gx1, gy1);
        if (!hitInBounds)
        {
            // Neu vat can vuot ra ngoai bien ban do, kẹp toa do lai
            gx1 = std::max(0, std::min(width_cells - 1, gx1));
            gy1 = std::max(0, std::min(height_cells - 1, gy1));
        }

        int dx = std::abs(gx1 - gx0);
        int dy = std::abs(gy1 - gy0);
        int sx = (gx0 < gx1) ? 1 : -1;
        int sy = (gy0 < gy1) ? 1 : -1;
        int err = dx - dy;

        int curX = gx0;
        int curY = gy0;

        // Truy vet cac o tia xuyen qua (Free Space)
        while (curX != gx1 || curY != gy1)
        {
            int idx = curY * width_cells + curX;
            int val = grid[idx] + LOG_ODDS_FREE;
            grid[idx] = (int8_t)std::max((int)LOG_ODDS_MIN, val);

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; curX += sx; }
            if (e2 < dx)  { err += dx; curY += sy; }
        }

        // Cap nhat o cuoi cung (Occupied / Vat can) neu tia va cham that su
        if (isHit && hitInBounds)
        {
            int idx = gy1 * width_cells + gx1;
            int val = grid[idx] + LOG_ODDS_OCC;
            grid[idx] = (int8_t)std::min((int)LOG_ODDS_MAX, val);
        }
    }

    // Xuat ban do ra file PGM (anh nhi phan P5) + YAML chuan ROS
    bool ExportPgmYaml(const std::string& baseName = "lidar_map")
    {
        std::lock_guard<std::mutex> lk(grid_mutex);
        std::string pgmPath = baseName + ".pgm";
        std::string yamlPath = baseName + ".yaml";

        std::ofstream pgm(pgmPath, std::ios::binary);
        if (!pgm.is_open()) return false;

        // Header PGM P5
        pgm << "P5\n# ROS Occupancy Grid Map\n" << width_cells << " " << height_cells << "\n255\n";

        // Ghi pixel: quet tu dong tren xuong duoi
        for (int y = height_cells - 1; y >= 0; y--)
        {
            for (int x = 0; x < width_cells; x++)
            {
                int8_t val = grid[y * width_cells + x];
                uint8_t pixel;
                if (val == 0)       pixel = 205; // Unknown: Xam nhạt
                else if (val < 0)   pixel = 254; // Free: Trang
                else                pixel = 0;   // Occupied: Den tuyen (vat can)
                pgm.put((char)pixel);
            }
        }
        pgm.close();

        // Ghi file YAML
        std::ofstream yml(yamlPath);
        if (!yml.is_open()) return false;
        yml << "image: " << baseName << ".pgm\n";
        yml << "resolution: " << resolution << "\n";
        yml << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
        yml << "negate: 0\n";
        yml << "occupied_thresh: 0.65\n";
        yml << "free_thresh: 0.196\n";
        yml.close();

        return true;
    }

    // Dong goi du lieu map thu nho (Downsampled) de truyen Web sieu nhe
    nlohmann::json GetWebSnapshot(int targetSize = 800)
    {
        std::lock_guard<std::mutex> lk(grid_mutex);
        nlohmann::json j;
        j["type"] = "map_data";
        j["origin_x"] = origin_x;
        j["origin_y"] = origin_y;
        j["width_m"] = width_m;
        j["height_m"] = height_m;
        j["res"] = resolution;

        // Downsample de payload nhe tam vai chuc KB
        int step = (width_cells > targetSize) ? (width_cells / targetSize) : 1;
        int outW = width_cells / step;
        int outH = height_cells / step;

        j["w"] = outW;
        j["h"] = outH;

        std::string bytes;
        bytes.reserve(outW * outH);

        for (int y = 0; y < outH; y++)
        {
            int srcY = y * step;
            for (int x = 0; x < outW; x++)
            {
                int srcX = x * step;
                int8_t val = grid[srcY * width_cells + srcX];
                uint8_t pix = (val == 0) ? 127 : ((val < 0) ? 255 : 0);
                bytes.push_back((char)pix);
            }
        }

        // Base64 encode don gian de truyen qua JSON
        static const char b64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string b64;
        int val = 0, valb = -6;
        for (uint8_t c : bytes) {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0) {
                b64.push_back(b64chars[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6) b64.push_back(b64chars[((val << 8) >> (valb + 8)) & 0x3F]);
        while (b64.size() % 4) b64.push_back('=');

        j["data"] = b64;
        return j;
    }
};

#endif // OCCUPANCY_GRID2D_H