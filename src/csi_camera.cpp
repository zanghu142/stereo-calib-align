#include "csi_camera.h"

#include <sstream>

std::string buildJetsonCsiPipeline(const CsiCameraConfig& config) {
    std::ostringstream pipeline;
    if (!config.device.empty()) {
        pipeline
            << "v4l2src device=" << config.device << " ! "
            << "video/x-raw, format=UYVY, width=" << config.width
            << ", height=" << config.height
            << ", framerate=" << config.fps << "/1 ! "
            << "queue max-size-buffers=1 leaky=downstream ! "
            << "videoconvert ! "
            << "video/x-raw, format=BGR ! "
            << "appsink drop=1 max-buffers=1 sync=false";
    } else {
        pipeline
            << "nvarguscamerasrc sensor-id=" << config.sensor_id << " ! "
            << "video/x-raw(memory:NVMM), width=" << config.width
            << ", height=" << config.height
            << ", framerate=" << config.fps << "/1 ! "
            << "nvvidconv flip-method=" << config.flip_method << " ! "
            << "video/x-raw, format=BGRx ! "
            << "videoconvert ! "
            << "video/x-raw, format=BGR ! "
            << "appsink drop=1 sync=false";
    }
    return pipeline.str();
}
