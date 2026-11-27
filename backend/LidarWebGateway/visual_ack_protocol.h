#pragma once
#include <cmath>
#include <string>
#include <utility>
#include <nlohmann/json.hpp>

class VisualAckMatcher
{
public:
    enum class Result { matched, wrong_event, invalid, duplicate, timeout, waiting, unexpected };

    void Begin(std::string eventId, long long sendMonotonicUs, long long timeoutUs = 750000)
    {
        expected_ = std::move(eventId); sentAtUs_ = sendMonotonicUs;
        deadlineUs_ = sendMonotonicUs + timeoutUs; pending_ = true;
    }
    Result Accept(const nlohmann::json& ack, long long nowMonotonicUs)
    {
        if (!pending_) {
            if (ack.is_object() && ack.contains("eventId") && ack["eventId"].is_string() &&
                ack["eventId"].get<std::string>() == completed_) return Result::duplicate;
            return Result::unexpected;
        }
        if (nowMonotonicUs >= deadlineUs_) { pending_ = false; return Result::timeout; }
        if (!ack.is_object() || !ack.contains("type") || !ack["type"].is_string() || ack["type"] != "ack" || !ack.contains("eventId") ||
            !ack["eventId"].is_string() || !ack.contains("receivedAtUnixMs") ||
            !ack["receivedAtUnixMs"].is_number() || !std::isfinite(ack["receivedAtUnixMs"].get<double>()) ||
            ack["receivedAtUnixMs"].get<double>() < 0) return Result::invalid;
        if (ack["eventId"].get<std::string>() != expected_) return Result::wrong_event;
        completed_ = expected_; pending_ = false; return Result::matched;
    }
    Result Expire(long long nowMonotonicUs)
    {
        if (pending_ && nowMonotonicUs >= deadlineUs_) { pending_ = false; return Result::timeout; }
        return pending_ ? Result::waiting : Result::unexpected;
    }
    bool Pending() const { return pending_; }
    long long SentAtUs() const { return sentAtUs_; }

private:
    std::string expected_, completed_;
    long long sentAtUs_ = 0, deadlineUs_ = 0;
    bool pending_ = false;
};
