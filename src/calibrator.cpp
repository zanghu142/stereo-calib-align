#include "calibrator.h"
#include "corner_refinement.h"
#include "calibration_validation.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

std::vector<cv::Point3f> makeBoardObjectPoints(cv::Size board_size, float square_size) {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(board_size.area()));
    for (int y = 0; y < board_size.height; ++y) {
        for (int x = 0; x < board_size.width; ++x) {
            points.emplace_back(static_cast<float>(x) * square_size,
                                static_cast<float>(y) * square_size,
                                0.0F);
        }
    }
    return points;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if ((values.size() & 1U) != 0U) return upper;
    std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
    return 0.5 * (upper + values[middle - 1]);
}

bool calibrateMonoPoints(const std::vector<std::vector<cv::Point2f>>& image_points,
                         const std::vector<cv::Point3f>& board_points,
                         cv::Size image_size,
                         MonoCalibrationResult& result) {
    if (image_points.size() < 8) return false;
    std::vector<std::vector<cv::Point3f>> object_points(image_points.size(), board_points);
    result = MonoCalibrationResult{};
    result.image_size = image_size;
    result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    result.dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    result.reprojection_error = cv::calibrateCamera(
        object_points, image_points, image_size,
        result.camera_matrix, result.dist_coeffs, result.rvecs, result.tvecs);
    if (!std::isfinite(result.reprojection_error) ||
        !cv::checkRange(result.camera_matrix) || !cv::checkRange(result.dist_coeffs) ||
        result.camera_matrix.at<double>(0, 0) <= 0.0 || result.camera_matrix.at<double>(1, 1) <= 0.0) {
        return false;
    }
    result.view_errors.reserve(image_points.size());
    for (std::size_t i = 0; i < image_points.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(board_points, result.rvecs[i], result.tvecs[i],
                          result.camera_matrix, result.dist_coeffs, projected);
        result.view_errors.push_back(
            cv::norm(projected, image_points[i], cv::NORM_L2) /
            std::sqrt(static_cast<double>(image_points[i].size())));
    }
    result.used_sample_count = static_cast<int>(image_points.size());
    result.valid = true;
    return true;
}

}  // namespace

MonoCalibrator::MonoCalibrator(cv::Size board_size, float square_size)
    : board_size_(board_size), square_size_(square_size) {}

void MonoCalibrator::addSample(const std::vector<cv::Point2f>& corners) {
    if (validBoardGeometry(board_size_, square_size_) && validCalibrationCorners(corners, board_size_)) {
        image_points_.push_back(corners);
    }
}

bool MonoCalibrator::calibrate(cv::Size image_size, MonoCalibrationResult& result) const try {
    result = {};
    if (!validCalibrationSamples(image_points_, board_size_, square_size_, image_size)) return false;
    MonoCalibrationResult initial;
    const std::vector<cv::Point3f> board_points = makeObjectPoints();
    if (!calibrateMonoPoints(image_points_, board_points, image_size, initial)) return false;

    const double error_median = median(initial.view_errors);
    const double outlier_limit = std::max(0.8, error_median * 2.5);
    std::vector<std::vector<cv::Point2f>> filtered;
    for (std::size_t i = 0; i < image_points_.size(); ++i) {
        if (initial.view_errors[i] <= outlier_limit) filtered.push_back(image_points_[i]);
    }
    if (filtered.size() >= 8 && filtered.size() < image_points_.size()) {
        return calibrateMonoPoints(filtered, board_points, image_size, result);
    }
    result = initial;
    return true;
} catch (const cv::Exception&) {
    result = {};
    return false;
}

bool MonoCalibrator::save(const std::string& path, const MonoCalibrationResult& result) const {
    if (!result.valid) return false;
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "calibration_type" << "monocular";
    fs << "image_width" << result.image_size.width;
    fs << "image_height" << result.image_size.height;
    fs << "board_cols" << board_size_.width;
    fs << "board_rows" << board_size_.height;
    fs << "square_size_mm" << square_size_;
    fs << "sample_count" << result.used_sample_count;
    fs << "reprojection_error" << result.reprojection_error;
    fs << "view_errors" << "[";
    for (double error : result.view_errors) fs << error;
    fs << "]";
    fs << "camera_matrix" << result.camera_matrix;
    fs << "dist_coeffs" << result.dist_coeffs;
    return true;
}

std::size_t MonoCalibrator::sampleCount() const {
    return image_points_.size();
}

std::vector<cv::Point3f> MonoCalibrator::makeObjectPoints() const {
    return makeBoardObjectPoints(board_size_, square_size_);
}

FisheyeCalibrator::FisheyeCalibrator(cv::Size board_size, float square_size)
    : board_size_(board_size), square_size_(square_size) {}

void FisheyeCalibrator::addSample(const std::vector<cv::Point2f>& corners) {
    if (validBoardGeometry(board_size_, square_size_) && validCalibrationCorners(corners, board_size_)) {
        image_points_.push_back(corners);
    }
}

bool FisheyeCalibrator::calibrate(cv::Size image_size, FisheyeCalibrationResult& result) const try {
    result = {};
    if (!validCalibrationSamples(image_points_, board_size_, square_size_, image_size)) return false;

    const auto board_points = makeObjectPoints();
    std::vector<std::vector<cv::Point3f>> object_points(image_points_.size(), board_points);
    result = FisheyeCalibrationResult{};
    result.image_size = image_size;
    result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    result.dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    const int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
    result.reprojection_error = cv::fisheye::calibrate(
        object_points, image_points_, image_size,
        result.camera_matrix, result.dist_coeffs, result.rvecs, result.tvecs,
        flags, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));
    if (!std::isfinite(result.reprojection_error) ||
        !cv::checkRange(result.camera_matrix) || !cv::checkRange(result.dist_coeffs) ||
        result.camera_matrix.at<double>(0, 0) <= 0.0 ||
        result.camera_matrix.at<double>(1, 1) <= 0.0 || result.dist_coeffs.total() != 4) {
        return false;
    }

    result.view_errors.reserve(image_points_.size());
    for (std::size_t i = 0; i < image_points_.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::fisheye::projectPoints(board_points, projected, result.rvecs[i], result.tvecs[i],
                                   result.camera_matrix, result.dist_coeffs);
        result.view_errors.push_back(
            cv::norm(projected, image_points_[i], cv::NORM_L2) /
            std::sqrt(static_cast<double>(image_points_[i].size())));
    }

    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        result.camera_matrix, result.dist_coeffs, image_size, cv::Mat::eye(3, 3, CV_64F),
        result.new_camera_matrix, 0.0, image_size);
    cv::fisheye::initUndistortRectifyMap(
        result.camera_matrix, result.dist_coeffs, cv::Mat::eye(3, 3, CV_64F),
        result.new_camera_matrix, image_size, CV_32FC1, result.map_x, result.map_y);

    result.used_sample_count = static_cast<int>(image_points_.size());
    result.valid = true;
    return true;
} catch (const cv::Exception&) {
    result = {};
    return false;
}

bool FisheyeCalibrator::save(const std::string& path, const FisheyeCalibrationResult& result) const {
    if (!result.valid || result.dist_coeffs.total() != 4) return false;
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "calibration_type" << "fisheye";
    fs << "camera_model" << "opencv_fisheye_4";
    fs << "image_width" << result.image_size.width;
    fs << "image_height" << result.image_size.height;
    fs << "board_cols" << board_size_.width;
    fs << "board_rows" << board_size_.height;
    fs << "square_size_mm" << square_size_;
    fs << "sample_count" << result.used_sample_count;
    fs << "reprojection_error" << result.reprojection_error;
    fs << "view_errors" << "[";
    for (double error : result.view_errors) fs << error;
    fs << "]";
    fs << "camera_matrix" << result.camera_matrix;
    fs << "dist_coeffs" << result.dist_coeffs;
    fs << "new_camera_matrix" << result.new_camera_matrix;
    return true;
}

std::size_t FisheyeCalibrator::sampleCount() const {
    return image_points_.size();
}

std::vector<cv::Point3f> FisheyeCalibrator::makeObjectPoints() const {
    return makeBoardObjectPoints(board_size_, square_size_);
}

StereoCalibrator::StereoCalibrator(cv::Size board_size, float square_size)
    : board_size_(board_size), square_size_(square_size) {}

bool StereoCalibrator::detectCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners) const {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    corners.clear();

    const int sb_flags = cv::CALIB_CB_NORMALIZE_IMAGE;
    if (cv::findChessboardCornersSB(gray, board_size_, corners, sb_flags)) {
        if (cornerRefinementRadius(corners, 11) == 0) {
            corners.clear();
            return false;
        }
        return true;
    }

    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (!cv::findChessboardCorners(gray, board_size_, corners, flags)) {
        corners.clear();
        return false;
    }

    const int radius = cornerRefinementRadius(corners, 11);
    if (radius == 0) {
        corners.clear();
        return false;
    }
    cv::cornerSubPix(
        gray,
        corners,
        cv::Size(radius, radius),
        cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    return true;
}

void StereoCalibrator::addSample(const std::vector<cv::Point2f>& left_corners,
                                 const std::vector<cv::Point2f>& right_corners) {
    if (!validBoardGeometry(board_size_, square_size_) ||
        !validCalibrationCorners(left_corners, board_size_) ||
        !validCalibrationCorners(right_corners, board_size_)) return;
    left_points_.push_back(left_corners);
    right_points_.push_back(right_corners);
}

bool StereoCalibrator::calibrate(cv::Size image_size, StereoCalibrationResult& result) const try {
    result = {};
    if (!validCalibrationSamples(left_points_, board_size_, square_size_, image_size) ||
        !validCalibrationSamples(right_points_, board_size_, square_size_, image_size) ||
        left_points_.size() != right_points_.size()) {
        std::cerr << "Need at least 8 valid stereo samples before calibration.\n";
        return false;
    }

    result = StereoCalibrationResult{};

    std::vector<std::vector<cv::Point3f>> object_points(left_points_.size(), makeObjectPoints());

    result.image_size = image_size;
    result.left_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    result.right_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    result.left_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    result.right_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    const double left_rms = cv::calibrateCamera(
        object_points,
        left_points_,
        image_size,
        result.left_camera_matrix,
        result.left_dist_coeffs,
        rvecs,
        tvecs);
    const double right_rms = cv::calibrateCamera(
        object_points,
        right_points_,
        image_size,
        result.right_camera_matrix,
        result.right_dist_coeffs,
        rvecs,
        tvecs);
    if (!std::isfinite(left_rms) || !std::isfinite(right_rms) ||
        !cv::checkRange(result.left_camera_matrix) ||
        !cv::checkRange(result.right_camera_matrix) ||
        !cv::checkRange(result.left_dist_coeffs) ||
        !cv::checkRange(result.right_dist_coeffs)) {
        return false;
    }

    const int flags = cv::CALIB_FIX_INTRINSIC;
    result.reprojection_error = cv::stereoCalibrate(
        object_points,
        left_points_,
        right_points_,
        result.left_camera_matrix,
        result.left_dist_coeffs,
        result.right_camera_matrix,
        result.right_dist_coeffs,
        image_size,
        result.R,
        result.T,
        result.E,
        result.F,
        flags,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-5));
    if (!std::isfinite(result.reprojection_error) || result.reprojection_error > 2.0 ||
        result.T.empty() ||
        !cv::checkRange(result.R) || !cv::checkRange(result.T) ||
        cv::norm(result.T) <= 1e-6) {
        return false;
    }

    cv::stereoRectify(
        result.left_camera_matrix,
        result.left_dist_coeffs,
        result.right_camera_matrix,
        result.right_dist_coeffs,
        image_size,
        result.R,
        result.T,
        result.R1,
        result.R2,
        result.P1,
        result.P2,
        result.Q,
        cv::CALIB_ZERO_DISPARITY,
        0,
        image_size);

    cv::initUndistortRectifyMap(
        result.left_camera_matrix,
        result.left_dist_coeffs,
        result.R1,
        result.P1,
        image_size,
        CV_32FC1,
        result.left_map_x,
        result.left_map_y);
    cv::initUndistortRectifyMap(
        result.right_camera_matrix,
        result.right_dist_coeffs,
        result.R2,
        result.P2,
        image_size,
        CV_32FC1,
        result.right_map_x,
        result.right_map_y);

    result.valid = true;
    return true;
} catch (const cv::Exception&) {
    result = {};
    return false;
}

bool StereoCalibrator::calibrateWithIntrinsics(cv::Size image_size,
                                               const MonoCalibrationResult& left,
                                               const MonoCalibrationResult& right,
                                               StereoCalibrationResult& result) const try {
    result = {};
    if (!validCalibrationSamples(left_points_, board_size_, square_size_, image_size) ||
        !validCalibrationSamples(right_points_, board_size_, square_size_, image_size) ||
        left_points_.size() != right_points_.size() || !left.valid || !right.valid ||
        left.image_size != image_size || right.image_size != image_size ||
        !validCameraIntrinsics(left.camera_matrix, left.dist_coeffs, false) ||
        !validCameraIntrinsics(right.camera_matrix, right.dist_coeffs, false)) {
        std::cerr << "Need valid monocular parameters and at least 8 stereo samples.\n";
        return false;
    }

    result = StereoCalibrationResult{};
    result.image_size = image_size;
    result.left_camera_matrix = left.camera_matrix.clone();
    result.right_camera_matrix = right.camera_matrix.clone();
    result.left_dist_coeffs = left.dist_coeffs.clone();
    result.right_dist_coeffs = right.dist_coeffs.clone();
    std::vector<std::vector<cv::Point3f>> object_points(left_points_.size(), makeObjectPoints());

    result.reprojection_error = cv::stereoCalibrate(
        object_points, left_points_, right_points_,
        result.left_camera_matrix, result.left_dist_coeffs,
        result.right_camera_matrix, result.right_dist_coeffs,
        image_size, result.R, result.T, result.E, result.F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-5));
    if (!std::isfinite(result.reprojection_error) || result.reprojection_error > 2.0 ||
        result.R.empty() || result.T.empty() ||
        !cv::checkRange(result.R) || !cv::checkRange(result.T) || cv::norm(result.T) <= 1e-6) {
        return false;
    }

    cv::stereoRectify(result.left_camera_matrix, result.left_dist_coeffs,
                      result.right_camera_matrix, result.right_dist_coeffs,
                      image_size, result.R, result.T, result.R1, result.R2,
                      result.P1, result.P2, result.Q, cv::CALIB_ZERO_DISPARITY,
                      0, image_size);
    cv::initUndistortRectifyMap(result.left_camera_matrix, result.left_dist_coeffs,
                                result.R1, result.P1, image_size, CV_32FC1,
                                result.left_map_x, result.left_map_y);
    cv::initUndistortRectifyMap(result.right_camera_matrix, result.right_dist_coeffs,
                                result.R2, result.P2, image_size, CV_32FC1,
                                result.right_map_x, result.right_map_y);
    result.valid = true;
    return true;
} catch (const cv::Exception&) {
    result = {};
    return false;
}

bool StereoCalibrator::save(const std::string& path, const StereoCalibrationResult& result) const {
    if (!result.valid) {
        return false;
    }

    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        return false;
    }

    fs << "image_width" << result.image_size.width;
    fs << "image_height" << result.image_size.height;
    fs << "board_cols" << board_size_.width;
    fs << "board_rows" << board_size_.height;
    fs << "calibration_type" << "stereo_extrinsics";
    fs << "square_size_mm" << square_size_;
    fs << "sample_count" << static_cast<int>(sampleCount());
    fs << "reprojection_error" << result.reprojection_error;
    fs << "left_camera_matrix" << result.left_camera_matrix;
    fs << "right_camera_matrix" << result.right_camera_matrix;
    fs << "left_dist_coeffs" << result.left_dist_coeffs;
    fs << "right_dist_coeffs" << result.right_dist_coeffs;
    fs << "R" << result.R;
    fs << "T" << result.T;
    fs << "E" << result.E;
    fs << "F" << result.F;
    fs << "R1" << result.R1;
    fs << "R2" << result.R2;
    fs << "P1" << result.P1;
    fs << "P2" << result.P2;
    fs << "Q" << result.Q;
    fs << "baseline_mm" << cv::norm(result.T);
    fs << "left_map_x" << result.left_map_x;
    fs << "left_map_y" << result.left_map_y;
    fs << "right_map_x" << result.right_map_x;
    fs << "right_map_y" << result.right_map_y;
    return true;
}

std::size_t StereoCalibrator::sampleCount() const {
    return left_points_.size();
}

cv::Size StereoCalibrator::boardSize() const {
    return board_size_;
}

std::vector<cv::Point3f> StereoCalibrator::makeObjectPoints() const {
    return makeBoardObjectPoints(board_size_, square_size_);
}

FisheyeStereoCalibrator::FisheyeStereoCalibrator(cv::Size board_size, float square_size)
    : board_size_(board_size), square_size_(square_size) {}

void FisheyeStereoCalibrator::addSample(const std::vector<cv::Point2f>& left_corners,
                                        const std::vector<cv::Point2f>& right_corners) {
    if (!validBoardGeometry(board_size_, square_size_) ||
        !validCalibrationCorners(left_corners, board_size_) ||
        !validCalibrationCorners(right_corners, board_size_)) return;
    left_points_.push_back(left_corners);
    right_points_.push_back(right_corners);
}

bool FisheyeStereoCalibrator::calibrateWithIntrinsics(
    cv::Size image_size,
    const FisheyeCalibrationResult& left,
    const FisheyeCalibrationResult& right,
    StereoCalibrationResult& result) const try {
    result = {};
    if (!validCalibrationSamples(left_points_, board_size_, square_size_, image_size) ||
        !validCalibrationSamples(right_points_, board_size_, square_size_, image_size) ||
        left_points_.size() != right_points_.size() || !left.valid || !right.valid ||
        left.image_size != image_size || right.image_size != image_size ||
        !validCameraIntrinsics(left.camera_matrix, left.dist_coeffs, true) ||
        !validCameraIntrinsics(right.camera_matrix, right.dist_coeffs, true)) {
        std::cerr << "Need matching fisheye intrinsics and at least 8 stereo samples.\n";
        return false;
    }

    result = StereoCalibrationResult{};
    result.image_size = image_size;
    result.left_camera_matrix = left.camera_matrix.clone();
    result.right_camera_matrix = right.camera_matrix.clone();
    result.left_dist_coeffs = left.dist_coeffs.clone();
    result.right_dist_coeffs = right.dist_coeffs.clone();
    const std::vector<cv::Point3f> board_points = makeObjectPoints();
    std::vector<std::vector<cv::Point3f>> object_points(left_points_.size(), board_points);

    result.reprojection_error = cv::fisheye::stereoCalibrate(
        object_points, left_points_, right_points_,
        result.left_camera_matrix, result.left_dist_coeffs,
        result.right_camera_matrix, result.right_dist_coeffs,
        image_size, result.R, result.T, cv::fisheye::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));
    if (!std::isfinite(result.reprojection_error) || result.reprojection_error > 2.0 ||
        result.R.empty() || result.T.empty() ||
        !cv::checkRange(result.R) || !cv::checkRange(result.T) || cv::norm(result.T) <= 1e-6) {
        return false;
    }

    cv::fisheye::stereoRectify(
        result.left_camera_matrix, result.left_dist_coeffs,
        result.right_camera_matrix, result.right_dist_coeffs,
        image_size, result.R, result.T, result.R1, result.R2,
        result.P1, result.P2, result.Q, cv::fisheye::CALIB_ZERO_DISPARITY,
        image_size, 0.0, 1.0);
    cv::fisheye::initUndistortRectifyMap(
        result.left_camera_matrix, result.left_dist_coeffs, result.R1, result.P1,
        image_size, CV_32FC1, result.left_map_x, result.left_map_y);
    cv::fisheye::initUndistortRectifyMap(
        result.right_camera_matrix, result.right_dist_coeffs, result.R2, result.P2,
        image_size, CV_32FC1, result.right_map_x, result.right_map_y);

    result.valid = true;
    return true;
} catch (const cv::Exception&) {
    result = {};
    return false;
}

bool FisheyeStereoCalibrator::save(const std::string& path,
                                   const StereoCalibrationResult& result) const {
    if (!result.valid || result.left_dist_coeffs.total() != 4 ||
        result.right_dist_coeffs.total() != 4) return false;
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "calibration_type" << "stereo_fisheye_extrinsics";
    fs << "camera_model" << "opencv_fisheye_4";
    fs << "image_width" << result.image_size.width;
    fs << "image_height" << result.image_size.height;
    fs << "board_cols" << board_size_.width;
    fs << "board_rows" << board_size_.height;
    fs << "square_size_mm" << square_size_;
    fs << "sample_count" << static_cast<int>(sampleCount());
    fs << "reprojection_error" << result.reprojection_error;
    fs << "left_camera_matrix" << result.left_camera_matrix;
    fs << "right_camera_matrix" << result.right_camera_matrix;
    fs << "left_dist_coeffs" << result.left_dist_coeffs;
    fs << "right_dist_coeffs" << result.right_dist_coeffs;
    fs << "R" << result.R;
    fs << "T" << result.T;
    fs << "R1" << result.R1;
    fs << "R2" << result.R2;
    fs << "P1" << result.P1;
    fs << "P2" << result.P2;
    fs << "Q" << result.Q;
    fs << "baseline_mm" << cv::norm(result.T);
    return true;
}

std::size_t FisheyeStereoCalibrator::sampleCount() const {
    return left_points_.size();
}

std::vector<cv::Point3f> FisheyeStereoCalibrator::makeObjectPoints() const {
    return makeBoardObjectPoints(board_size_, square_size_);
}
