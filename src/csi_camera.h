#pragma once

#include <string>

struct CsiCameraConfig {
    int sensor_id = 0;
    int width = 1920;
    int height = 1080;
    int fps = 30;
    int flip_method = 0;
    std::string device;
};

std::string buildJetsonCsiPipeline(const CsiCameraConfig& config);
