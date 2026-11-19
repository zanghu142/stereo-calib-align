#pragma once

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

struct MonoCalibrationResult {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<double> view_errors;
    double reprojection_error = 0.0;
    cv::Size image_size;
    int used_sample_count = 0;
    bool valid = false;
};

class MonoCalibrator {
public:
    MonoCalibrator(cv::Size board_size, float square_size);

    void addSample(const std::vector<cv::Point2f>& corners);
    bool calibrate(cv::Size image_size, MonoCalibrationResult& result) const;
    bool save(const std::string& path, const MonoCalibrationResult& result) const;
    std::size_t sampleCount() const;

private:
    std::vector<cv::Point3f> makeObjectPoints() const;

    cv::Size board_size_;
    float square_size_;
    std::vector<std::vector<cv::Point2f>> image_points_;
};

struct FisheyeCalibrationResult {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<double> view_errors;
    cv::Mat new_camera_matrix;
    cv::Mat map_x;
    cv::Mat map_y;
    double reprojection_error = 0.0;
    cv::Size image_size;
    int used_sample_count = 0;
    bool valid = false;
};

class FisheyeCalibrator {
public:
    FisheyeCalibrator(cv::Size board_size, float square_size);

    void addSample(const std::vector<cv::Point2f>& corners);
    bool calibrate(cv::Size image_size, FisheyeCalibrationResult& result) const;
    bool save(const std::string& path, const FisheyeCalibrationResult& result) const;
    std::size_t sampleCount() const;

private:
    std::vector<cv::Point3f> makeObjectPoints() const;

    cv::Size board_size_;
    float square_size_;
    std::vector<std::vector<cv::Point2f>> image_points_;
};

struct StereoCalibrationResult {
    cv::Mat left_camera_matrix;
    cv::Mat right_camera_matrix;
    cv::Mat left_dist_coeffs;
    cv::Mat right_dist_coeffs;
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
    cv::Mat left_map_x;
    cv::Mat left_map_y;
    cv::Mat right_map_x;
    cv::Mat right_map_y;
    double reprojection_error = 0.0;
    cv::Size image_size;
    bool valid = false;
};

class StereoCalibrator {
public:
    StereoCalibrator(cv::Size board_size, float square_size);

    bool detectCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners) const;
    void addSample(const std::vector<cv::Point2f>& left_corners,
                   const std::vector<cv::Point2f>& right_corners);
    bool calibrate(cv::Size image_size, StereoCalibrationResult& result) const;
    bool calibrateWithIntrinsics(cv::Size image_size,
                                 const MonoCalibrationResult& left,
                                 const MonoCalibrationResult& right,
                                 StereoCalibrationResult& result) const;
    bool save(const std::string& path, const StereoCalibrationResult& result) const;
    std::size_t sampleCount() const;
    cv::Size boardSize() const;

private:
    std::vector<cv::Point3f> makeObjectPoints() const;

    cv::Size board_size_;
    float square_size_;
    std::vector<std::vector<cv::Point2f>> left_points_;
    std::vector<std::vector<cv::Point2f>> right_points_;
};

class FisheyeStereoCalibrator {
public:
    FisheyeStereoCalibrator(cv::Size board_size, float square_size);

    void addSample(const std::vector<cv::Point2f>& left_corners,
                   const std::vector<cv::Point2f>& right_corners);
    bool calibrateWithIntrinsics(cv::Size image_size,
                                 const FisheyeCalibrationResult& left,
                                 const FisheyeCalibrationResult& right,
                                 StereoCalibrationResult& result) const;
    bool save(const std::string& path, const StereoCalibrationResult& result) const;
    std::size_t sampleCount() const;

private:
    std::vector<cv::Point3f> makeObjectPoints() const;

    cv::Size board_size_;
    float square_size_;
    std::vector<std::vector<cv::Point2f>> left_points_;
    std::vector<std::vector<cv::Point2f>> right_points_;
};
