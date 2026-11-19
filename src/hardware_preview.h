#pragma once

#include <opencv2/opencv.hpp>

#include <string>

class HardwarePreview {
public:
    HardwarePreview(cv::Size left_size, cv::Size right_size, int fps);
    ~HardwarePreview();

    bool start();
    void stop();
    bool available() const;
    void push(const cv::Mat& left,
              const cv::Mat& right,
              const std::string& status_top,
              const std::string& status_sub);

private:
    cv::Size left_size_;
    cv::Size right_size_;
    int fps_ = 30;
    bool available_ = false;

#ifdef HAVE_GSTREAMER_PREVIEW
    struct Impl;
    Impl* impl_ = nullptr;
#endif
};
