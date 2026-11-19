#pragma once

#include <chrono>

using FrameClock = std::chrono::steady_clock;

// These are software receipt times, not sensor exposure timestamps.
inline bool freshFramePair(FrameClock::time_point left, FrameClock::time_point right,
                           FrameClock::time_point now) {
    if (left == FrameClock::time_point{} || right == FrameClock::time_point{} ||
        left > now || right > now) return false;
    const auto skew = left > right ? left - right : right - left;
    return now - left <= std::chrono::seconds(1) && now - right <= std::chrono::seconds(1) &&
           skew <= std::chrono::milliseconds(100);
}
