#pragma once

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>

#include <string>

struct GstStereoConfig {
    std::string left_device = "/dev/video0";
    std::string right_device = "/dev/video1";
    int width = 1920;
    int height = 1080;
    int fps = 30;
};

class GstStereoSource {
public:
    explicit GstStereoSource(const GstStereoConfig& config);
    ~GstStereoSource();

    bool start();
    bool grabLatest(cv::Mat& left, cv::Mat& right);
    void stop();
    std::string statusText() const;

private:
    static std::string makePipeline(const GstStereoConfig& config);
    static cv::Mat sampleToMat(GstSample* sample);

    GstStereoConfig config_;
    GstElement* pipeline_ = nullptr;
    GstElement* left_sink_ = nullptr;
    GstElement* right_sink_ = nullptr;
    std::string status_text_;
};
