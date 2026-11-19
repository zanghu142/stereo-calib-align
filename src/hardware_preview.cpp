#include "hardware_preview.h"

#include <algorithm>
#include <sstream>

#ifdef HAVE_GSTREAMER_PREVIEW
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

struct HardwarePreview::Impl {
    GstElement* pipeline = nullptr;
    GstElement* left_src = nullptr;
    GstElement* right_src = nullptr;
    GstElement* overlay = nullptr;
    guint64 frame_index = 0;
    std::string last_top;
    std::string last_sub;
};

static std::string makePipeline(const cv::Size& left_size, const cv::Size& right_size, int fps) {
    const int out_w = left_size.width + right_size.width;
    const int out_h = std::max(left_size.height, right_size.height);
    std::ostringstream ss;
    ss
        << "appsrc name=leftsrc is-live=true do-timestamp=true format=time block=false "
        << "caps=video/x-raw,format=BGR,width=" << left_size.width
        << ",height=" << left_size.height
        << ",framerate=" << fps << "/1 ! "
        << "queue max-size-buffers=1 leaky=downstream ! "
        << "videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! comp.sink_0 "
        << "appsrc name=rightsrc is-live=true do-timestamp=true format=time block=false "
        << "caps=video/x-raw,format=BGR,width=" << right_size.width
        << ",height=" << right_size.height
        << ",framerate=" << fps << "/1 ! "
        << "queue max-size-buffers=1 leaky=downstream ! "
        << "videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! comp.sink_1 "
        << "nvcompositor name=comp "
        << "sink_0::xpos=0 sink_0::ypos=0 sink_0::width=" << left_size.width
        << " sink_0::height=" << out_h
        << " sink_1::xpos=" << left_size.width
        << " sink_1::ypos=0 sink_1::width=" << right_size.width
        << " sink_1::height=" << out_h
        << " ! textoverlay name=statusoverlay shaded-background=true "
        << "font-desc=\"Sans, 18\" halignment=right valignment=top xpad=24 ypad=16 ! "
        << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width=" << out_w
        << ",height=" << out_h
        << " ! nvegltransform ! nveglglessink sync=false fullscreen=true force-aspect-ratio=false";
    return ss.str();
}

HardwarePreview::HardwarePreview(cv::Size left_size, cv::Size right_size, int fps)
    : left_size_(left_size), right_size_(right_size), fps_(fps) {}

HardwarePreview::~HardwarePreview() {
    stop();
}

bool HardwarePreview::start() {
    stop();

    if (!gst_is_initialized()) {
        int argc = 0;
        char** argv = nullptr;
        gst_init(&argc, &argv);
    }

    GError* error = nullptr;
    const std::string pipeline_str = makePipeline(left_size_, right_size_, fps_);
    impl_ = new Impl();
    impl_->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!impl_->pipeline || error) {
        if (error) {
            g_error_free(error);
        }
        stop();
        return false;
    }

    impl_->left_src = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "leftsrc");
    impl_->right_src = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "rightsrc");
    impl_->overlay = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "statusoverlay");
    if (!impl_->left_src || !impl_->right_src || !impl_->overlay) {
        stop();
        return false;
    }

    const GstStateChangeReturn state_ret = gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING);
    if (state_ret == GST_STATE_CHANGE_FAILURE) {
        stop();
        return false;
    }
    available_ = true;
    return true;
}

void HardwarePreview::stop() {
#ifdef HAVE_GSTREAMER_PREVIEW
    if (impl_) {
        if (impl_->pipeline) {
            gst_element_set_state(impl_->pipeline, GST_STATE_NULL);
        }
        if (impl_->left_src) gst_object_unref(impl_->left_src);
        if (impl_->right_src) gst_object_unref(impl_->right_src);
        if (impl_->overlay) gst_object_unref(impl_->overlay);
        if (impl_->pipeline) gst_object_unref(impl_->pipeline);
        delete impl_;
        impl_ = nullptr;
    }
#endif
    available_ = false;
}

bool HardwarePreview::available() const {
    return available_;
}

void HardwarePreview::push(const cv::Mat& left,
                           const cv::Mat& right,
                           const std::string& status_top,
                           const std::string& status_sub) {
#ifdef HAVE_GSTREAMER_PREVIEW
    if (!available_ || !impl_ || !impl_->left_src || !impl_->right_src) {
        return;
    }

    if (impl_->overlay &&
        (impl_->last_top != status_top || impl_->last_sub != status_sub)) {
        std::ostringstream text;
        text << status_top << "\n" << status_sub;
        g_object_set(impl_->overlay, "text", text.str().c_str(), nullptr);
        impl_->last_top = status_top;
        impl_->last_sub = status_sub;
    }

    auto pushFrame = [](GstElement* src, const cv::Mat& frame, guint64 frame_index, int fps) {
        if (!src || frame.empty()) return;
        cv::Mat contiguous = frame.isContinuous() ? frame : frame.clone();
        const std::size_t bytes = static_cast<std::size_t>(contiguous.total() * contiguous.elemSize());
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
        if (!buffer) return;
        gst_buffer_fill(buffer, 0, contiguous.data, bytes);
        const guint64 duration = GST_SECOND / static_cast<guint64>(fps);
        GST_BUFFER_PTS(buffer) = frame_index * duration;
        GST_BUFFER_DURATION(buffer) = duration;
        gst_app_src_push_buffer(GST_APP_SRC(src), buffer);
    };

    pushFrame(impl_->left_src, left, impl_->frame_index, fps_);
    pushFrame(impl_->right_src, right, impl_->frame_index, fps_);
    ++impl_->frame_index;
#else
    (void)left;
    (void)right;
    (void)status_top;
    (void)status_sub;
#endif
}

#else

HardwarePreview::HardwarePreview(cv::Size left_size, cv::Size right_size, int fps)
    : left_size_(left_size), right_size_(right_size), fps_(fps) {}

HardwarePreview::~HardwarePreview() = default;

bool HardwarePreview::start() {
    available_ = false;
    return false;
}

void HardwarePreview::stop() {}

bool HardwarePreview::available() const {
    return available_;
}

void HardwarePreview::push(const cv::Mat&,
                           const cv::Mat&,
                           const std::string&,
                           const std::string&) {}

#endif
