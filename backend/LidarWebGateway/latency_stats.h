#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

// Bounded, thread-safe rolling latency statistics. All inputs and outputs are ms.
class LatencyStats
{
public:
    struct Snapshot {
        bool available = false;
        double last = 0, mean = 0, median = 0, p95 = 0, p99 = 0, min = 0, max = 0;
        size_t sampleCount = 0;
    };

    explicit LatencyStats(size_t capacity = 1000) : capacity_(capacity ? capacity : 1) {}

    void Record(double ms)
    {
        if (!std::isfinite(ms) || ms < 0) return;
        std::lock_guard<std::mutex> lock(mutex_);
        samples_.push_back(ms);
        if (samples_.size() > capacity_) samples_.pop_front();
    }

    Snapshot Get() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Snapshot out;
        if (samples_.empty()) return out;
        out.available = true;
        out.last = samples_.back();
        out.sampleCount = samples_.size();
        std::vector<double> sorted(samples_.begin(), samples_.end());
        std::sort(sorted.begin(), sorted.end());
        double sum = 0;
        for (double value : sorted) sum += value;
        auto percentile = [&sorted](double p) {
            const size_t index = static_cast<size_t>(std::ceil(p * sorted.size())) - 1;
            return sorted[std::min(index, sorted.size() - 1)];
        };
        out.mean = sum / sorted.size();
        out.min = sorted.front(); out.max = sorted.back();
        const size_t mid = sorted.size() / 2;
        out.median = sorted.size() % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2.0;
        out.p95 = percentile(0.95); out.p99 = percentile(0.99);
        return out;
    }

    void Reset() { std::lock_guard<std::mutex> lock(mutex_); samples_.clear(); }

private:
    const size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<double> samples_;
};
