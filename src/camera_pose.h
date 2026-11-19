#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

struct CameraPose {
    bool calibrated = false;
    double rms_px = 0.0;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Vec3d x_axis;
    cv::Vec3d y_axis;
    cv::Vec3d z_axis;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double roll_deg = 0.0;
    bool valid = false;
};

double opticalHeadingDeg(const cv::Vec3d& optical_axis);
double opticalPitchDeg(const cv::Vec3d& optical_axis);
double imageRollDeg(const cv::Vec3d& image_x_axis);

// Fisheye mode requires calibrated four-coefficient intrinsics. Pose RMS is
// always measured in original image pixels using the selected camera model.
bool estimateCameraPose(const std::vector<cv::Point2f>& corners,
                        const std::vector<cv::Point3f>& object_points,
                        cv::Size image_size,
                        const cv::Mat& calibrated_camera_matrix,
                        const cv::Mat& calibrated_dist_coeffs,
                        CameraPose& pose,
                        bool fisheye = false);