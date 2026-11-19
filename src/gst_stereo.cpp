#include "gst_stereo.h"

#include <sstream>

GstStereoSource::GstStereoSource(const GstStereoConfig& config)
    : config_(config) {}

GstStereoSource::~GstStereoSource() {
    stop();
}

std::string GstStereoSource::makePipeline(const GstStereoConfig& config) {
    const int out_w = config.width * 2;
    const int out_h = config.height;

    std::ostringstream ss;
    ss
        << "v4l2src device=" << config.left_device << " ! "
        << "video/x-raw,format=UYVY,width=" << config.width << ",height=" << config.height << ",framerate=" << config.fps << "/1 ! "
        << "tee name=t0 "
        << "t0. ! queue max-size-buffers=1 leaky=downstream ! "
        << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! comp.sink_0 "
        << "t0. ! queue max-size-buffers=1 leaky=downstream ! "
        << "videoconvert ! video/x-raw,format=BGR ! appsink name=leftsink drop=true max-buffers=1 sync=false emit-signals=false "
        << "v4l2src device=" << config.right_device << " ! "
        << "video/x-raw,format=UYVY,width=" << config.width << ",height=" << config.height << ",framerate=" << config.fps << "/1 ! "
        << "tee name=t1 "
        << "t1. ! queue max-size-buffers=1 leaky=downstream ! "
        << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! comp.sink_1 "
        << "t1. ! queue max-size-buffers=1 leaky=downstream ! "
        << "videoconvert ! video/x-raw,format=BGR ! appsink name=rightsink drop=true max-buffers=1 sync=false emit-signals=false "
        << "nvcompositor name=comp sink_0::xpos=0 sink_0::ypos=0 sink_0::width=" << config.width
        << " sink_0::height=" << config.height
        << " sink_1::xpos=" << config.width
        << " sink_1::ypos=0 sink_1::width=" << config.width
        << " sink_1::height=" << config.height
        << " ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=" << out_w << ",height=" << out_h
        << " ! nv3dsink sync=false";
        ;
    return ss.str();
}

cv::Mat GstStereoSource::sampleToMat(GstSample* sample) {
    if (!sample) {
        return {};
    }
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        return {};
    }

    GstStructure* s = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        return {};
    }

    cv::Mat frame(height, width, CV_8UC3, const_cast<guint8*>(map.data));
    cv::Mat out = frame.clone();
    gst_buffer_unmap(buffer, &map);
    return out;
}

bool GstStereoSource::start() {
    stop();

    status_text_ = "starting";
    GError* error = nullptr;
    const std::string pipeline_str = makePipeline(config_);
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline_ || error) {
        if (error) {
            status_text_ = error->message;
            g_error_free(error);
        }
        return false;
    }

    left_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "leftsink");
    right_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "rightsink");
    if (!left_sink_ || !right_sink_) {
        status_text_ = "missing appsinks";
        stop();
        return false;
    }

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    status_text_ = "playing";
    return true;
}

bool GstStereoSource::grabLatest(cv::Mat& left, cv::Mat& right) {
    if (!pipeline_ || !left_sink_ || !right_sink_) {
        return false;
    }

    GstSample* left_sample = gst_app_sink_try_pull_sample(GST_APP_SINK(left_sink_), GST_SECOND / 30);
    GstSample* right_sample = gst_app_sink_try_pull_sample(GST_APP_SINK(right_sink_), GST_SECOND / 30);
    if (!left_sample || !right_sample) {
        if (left_sample) gst_sample_unref(left_sample);
        if (right_sample) gst_sample_unref(right_sample);
        return false;
    }

    left = sampleToMat(left_sample);
    right = sampleToMat(right_sample);
    gst_sample_unref(left_sample);
    gst_sample_unref(right_sample);
    return !left.empty() && !right.empty();
}

void GstStereoSource::stop() {
    if (left_sink_) {
        gst_object_unref(left_sink_);
        left_sink_ = nullptr;
    }
    if (right_sink_) {
        gst_object_unref(right_sink_);
        right_sink_ = nullptr;
    }
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

std::string GstStereoSource::statusText() const {
    return status_text_;
}
