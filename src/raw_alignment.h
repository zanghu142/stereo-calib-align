#pragma once

#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

struct RawAlignment {
    bool valid = false;
    double mean_abs_y_px = 0.0;
    double max_abs_y_px = 0.0;
};

inline RawAlignment measureRawAlignment(const std::vector<cv::Point2f>& left,
                                         const std::vector<cv::Point2f>& right) {
    RawAlignment result;
    if (left.empty() || left.size() != right.size()) return result;
    for (std::size_t i = 0; i < left.size(); ++i) {
        if (!std::isfinite(left[i].x) || !std::isfinite(left[i].y) ||
            !std::isfinite(right[i].x) || !std::isfinite(right[i].y)) return {};
        const double error = std::abs(static_cast<double>(right[i].y) - left[i].y);
        result.mean_abs_y_px += error;
        result.max_abs_y_px = std::max(result.max_abs_y_px, error);
    }
    result.mean_abs_y_px /= left.size();
    result.valid = true;
    return result;
}

inline bool rawAlignmentPasses(const RawAlignment& result) {
    return result.valid && std::isfinite(result.max_abs_y_px) &&
           std::isfinite(result.mean_abs_y_px) && result.mean_abs_y_px >= 0.0 &&
           result.mean_abs_y_px <= result.max_abs_y_px &&
           result.max_abs_y_px >= 0.0 && result.max_abs_y_px <= 2.0;
}

inline bool jointFisheyeLevelPasses(bool physical_passes, const RawAlignment& raw) {
    return physical_passes && rawAlignmentPasses(raw);
}
