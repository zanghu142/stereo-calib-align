#pragma once

#include <opencv2/core.hpp>
#include <cmath>
#include <limits>
#include <vector>

// OpenCV's pinhole projection ignores skew; accepting it silently changes K.
inline bool validCameraIntrinsics(const cv::Mat& matrix, const cv::Mat& distortion,
                                  bool fisheye) {
    if (matrix.rows != 3 || matrix.cols != 3 || matrix.channels() != 1 ||
        distortion.empty() || distortion.channels() != 1 ||
        (distortion.rows != 1 && distortion.cols != 1)) return false;
    const auto count = distortion.total();
    if (fisheye ? count != 4 : (count != 4 && count != 5 && count != 8 && count != 12 && count != 14))
        return false;
    cv::Mat k, d;
    matrix.convertTo(k, CV_64F);
    distortion.convertTo(d, CV_64F);
    return cv::checkRange(k) && cv::checkRange(d) && k.at<double>(0,0) > 0.0 &&
        k.at<double>(1,1) > 0.0 && std::abs(k.at<double>(0,1)) <= 1e-9 &&
        std::abs(k.at<double>(1,0)) <= 1e-9 && std::abs(k.at<double>(2,0)) <= 1e-9 &&
        std::abs(k.at<double>(2,1)) <= 1e-9 && std::abs(k.at<double>(2,2)-1.0) <= 1e-9;
}

inline bool validBoardGeometry(cv::Size board, float square) {
    const auto count = static_cast<long long>(board.width) * board.height;
    return board.width >= 2 && board.height >= 2 && count <= std::numeric_limits<int>::max() &&
        std::isfinite(square) && square > 0.0F &&
        std::isfinite(static_cast<float>(board.width-1)*square) &&
        std::isfinite(static_cast<float>(board.height-1)*square);
}

inline bool validCalibrationCorners(const std::vector<cv::Point2f>& points, cv::Size board,
                                     cv::Size image = {}) {
    const auto count = static_cast<long long>(board.width) * board.height;
    if (board.width < 2 || board.height < 2 || count <= 0 ||
        points.size() != static_cast<std::size_t>(count)) return false;
    for (const auto& p : points)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
            (image.width > 0 && p.x >= image.width) || (image.height > 0 && p.y >= image.height))
            return false;
    return true;
}

inline bool validCalibrationSamples(const std::vector<std::vector<cv::Point2f>>& samples,
                                     cv::Size board, float square, cv::Size image) {
    if (!validBoardGeometry(board, square) || image.width <= 0 || image.height <= 0 || samples.size() < 8)
        return false;
    for (const auto& sample : samples)
        if (!validCalibrationCorners(sample, board, image)) return false;
    return true;
}
