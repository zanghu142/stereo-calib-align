#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// A fixed large window can include the next chessboard corner when the board
// is small or foreshortened. Keep it well inside the closest corner spacing.
inline int cornerRefinementRadius(const std::vector<cv::Point2f>& corners, int preferred) {
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < corners.size(); ++i)
        for (std::size_t j = i + 1; j < corners.size(); ++j)
            minimum_distance = std::min(minimum_distance, cv::norm(corners[i] - corners[j]));
    if (!std::isfinite(minimum_distance) || minimum_distance < 8.0) return 0;
    return std::min(preferred, std::max(2, static_cast<int>(minimum_distance * 0.25)));
}

inline bool refineScaledBoardCorners(const cv::Mat& image, cv::Size detection_size,
                                     double requested_scale, std::vector<cv::Point2f>& corners) {
    if (image.empty() || detection_size.width <= 0 || detection_size.height <= 0 ||
        corners.empty() || !std::isfinite(requested_scale) || requested_scale <= 0.0) return false;
    const double sx = static_cast<double>(image.cols) / detection_size.width;
    const double sy = static_cast<double>(image.rows) / detection_size.height;
    for (auto& p : corners) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
        // Match cv::resize pixel-center geometry and actual rounded dimensions.
        p.x = static_cast<float>((p.x + 0.5) * sx - 0.5);
        p.y = static_cast<float>((p.y + 0.5) * sy - 0.5);
    }
    const int preferred = static_cast<int>(std::clamp(4.0 / requested_scale, 7.0, 31.0));
    const int radius = cornerRefinementRadius(corners, preferred);
    if (radius == 0) return false;
    for (const auto& p : corners)
        if (p.x < radius || p.y < radius || p.x >= image.cols - radius || p.y >= image.rows - radius)
            return false;
    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else if (image.channels() == 1) gray = image;
    else return false;
    cv::cornerSubPix(gray, corners, {radius, radius}, {-1, -1},
        {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 60, 0.001});
    return true;
}
