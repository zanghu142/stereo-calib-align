#include "calibrator.h"
#include "camera_pose.h"
#include "raw_alignment.h"
#include "frame_timing.h"
#include "corner_refinement.h"
#include "calibration_validation.h"
#include "hardware_preview.h"

#include <opencv2/opencv.hpp>
#ifdef HAVE_FREETYPE
#include <ft2build.h>
#include FT_FREETYPE_H
#endif

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <queue>
#include <memory>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef HAVE_EGL_OPENGL_VIEW
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#endif

#ifdef HAVE_X11
#include <X11/Xlib.h>
#endif

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

struct AppConfig {
    std::string mode = "stereo";
    std::string left_device = "/dev/video0";
    std::string right_device = "/dev/video1";
    int width = 1920;
    int height = 1080;
    int fps = 30;
    // CTGD-12x9-60-45-5 横放：12x9 printed squares -> 11x8 inner corners, 50 mm squares.
    int board_cols = 11;
    int board_rows = 8;
    float square_size = 50.0F;
    double process_scale = 0.25;
    std::string output;
    std::string capture_dir = "captures";
    std::string font_path;
    std::string left_mono_file = "left_camera.yml";
    std::string right_mono_file = "right_camera.yml";
    std::string left_fisheye_file = "left_fisheye.yml";
    std::string right_fisheye_file = "right_fisheye.yml";
    bool use_hardware_preview = true;
    int mono_samples = 15;
    double mono_max_rms = 1.0;
    double mono_min_sharpness = 75.0;
    double mono_min_coverage = 3.0;
    double mono_max_coverage = 55.0;
};

static std::atomic<bool> g_quit{false};

static void onSignal(int) {
    g_quit.store(true);
}

static int parseIntArg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    return std::stoi(argv[++i]);
}

static float parseFloatArg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    return std::stof(argv[++i]);
}

static std::string parseStringArg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    return argv[++i];
}

static AppConfig parseArgs(int argc, char** argv) {
    AppConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--mode") config.mode = parseStringArg(argc, argv, i);
        else if (arg == "--left-device") config.left_device = parseStringArg(argc, argv, i);
        else if (arg == "--right-device") config.right_device = parseStringArg(argc, argv, i);
        else if (arg == "--width") config.width = parseIntArg(argc, argv, i);
        else if (arg == "--height") config.height = parseIntArg(argc, argv, i);
        else if (arg == "--fps") config.fps = parseIntArg(argc, argv, i);
        else if (arg == "--board-cols") config.board_cols = parseIntArg(argc, argv, i);
        else if (arg == "--board-rows") config.board_rows = parseIntArg(argc, argv, i);
        else if (arg == "--square-size") config.square_size = parseFloatArg(argc, argv, i);
        else if (arg == "--process-scale") config.process_scale = std::stod(parseStringArg(argc, argv, i));
        else if (arg == "--output") config.output = parseStringArg(argc, argv, i);
        else if (arg == "--capture-dir") config.capture_dir = parseStringArg(argc, argv, i);
        else if (arg == "--font") config.font_path = parseStringArg(argc, argv, i);
        else if (arg == "--left-mono") config.left_mono_file = parseStringArg(argc, argv, i);
        else if (arg == "--right-mono") config.right_mono_file = parseStringArg(argc, argv, i);
        else if (arg == "--left-fisheye") config.left_fisheye_file = parseStringArg(argc, argv, i);
        else if (arg == "--right-fisheye") config.right_fisheye_file = parseStringArg(argc, argv, i);
        else if (arg == "--no-hw-preview") config.use_hardware_preview = false;
        else if (arg == "--mono-samples") config.mono_samples = parseIntArg(argc, argv, i);
        else if (arg == "--mono-max-rms") config.mono_max_rms = std::stod(parseStringArg(argc, argv, i));
        else if (arg == "--mono-min-sharpness") config.mono_min_sharpness = std::stod(parseStringArg(argc, argv, i));
        else if (arg == "--mono-min-coverage") config.mono_min_coverage = std::stod(parseStringArg(argc, argv, i));
        else if (arg == "--mono-max-coverage") config.mono_max_coverage = std::stod(parseStringArg(argc, argv, i));
        else if (arg == "--help") {
            std::cout << "Usage: stereo_csi_calibrator [options]\n"
                      << "  --mode physical|physical-fisheye|stereo|stereo-fisheye|mono-left|mono-right|fisheye-left|fisheye-right\n"
                      << "  --left-device /dev/video0 --right-device /dev/video1\n"
                      << "  --width 1920 --height 1080 --fps 30\n"
                      << "  --board-cols 11 --board-rows 8 --square-size 50  (CTGD-12x9-60-45-5 horizontal: 12x9 squares, mm)\n"
                      << "  Vertical board: --board-cols 8 --board-rows 11 --square-size 50\n"
                      << "  --process-scale 0.25\n"
                      << "  --output <path> --capture-dir captures\n"
                      << "  --left-mono left_camera.yml --right-mono right_camera.yml\n"
                      << "  --left-fisheye left_fisheye.yml --right-fisheye right_fisheye.yml\n"
                      << "  --no-hw-preview  (use OpenCV window instead of Jetson hardware fullscreen preview)\n"
                      << "  --font /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc\n"
                      << "  --mono-samples 15 --mono-max-rms 1.0\n"
                      << "  --mono-min-sharpness 75 --mono-min-coverage 3 --mono-max-coverage 55\n"
                      << "Keys: a alignment, v view, s stereo sample, c calibrate, r rectified, w write, q quit\n"
                      << "Physical modes: four-panel leveling from left/right mono intrinsics; physical-fisheye uses --left-fisheye/--right-fisheye.\n"
                      << "Physical modes show current-frame baseline and pose estimates; invalid/stale frames disable the readout.\n"
                      << "Stereo modes compute extrinsics from synchronized samples.\n"
                      << "3D: j/l yaw, i/k pitch, b/n roll, u/o zoom, 0 reset; "
                         "x axes, h reference, e errors. OpenCV window: drag to rotate, Shift+drag roll, middle-drag pan, wheel zoom.\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    if (config.mode != "physical" && config.mode != "physical-fisheye" &&
        config.mode != "stereo" && config.mode != "stereo-fisheye" &&
        config.mode != "mono-left" && config.mode != "mono-right" &&
        config.mode != "fisheye-left" && config.mode != "fisheye-right") {
        throw std::runtime_error("Invalid --mode; use physical, physical-fisheye, stereo, stereo-fisheye, mono-left/right, or fisheye-left/right");
    }
    if (!validBoardGeometry({config.board_cols,config.board_rows}, config.square_size)) {
        throw std::runtime_error("Invalid chessboard dimensions or square size");
    }
    if (config.width <= 0 || config.height <= 0 || config.fps <= 0 ||
        !std::isfinite(config.process_scale) || config.process_scale <= 0.0 || config.process_scale > 1.0)
        throw std::runtime_error("Invalid dimensions/fps or --process-scale (must be in (0,1])");
    if (config.mono_samples < 8) config.mono_samples = 8;
    if (!std::isfinite(config.mono_max_rms) || config.mono_max_rms <= 0.0)
        throw std::runtime_error("--mono-max-rms must be finite and positive");
    if (!std::isfinite(config.mono_min_sharpness) || !std::isfinite(config.mono_min_coverage) ||
        !std::isfinite(config.mono_max_coverage) ||
        config.mono_min_sharpness < 0.0 || config.mono_min_coverage <= 0.0 ||
        config.mono_max_coverage <= config.mono_min_coverage || config.mono_max_coverage > 100.0) {
        throw std::runtime_error("Invalid monocular image quality thresholds");
    }
    if (config.output.empty()) {
        if (config.mode == "mono-left") config.output = "left_camera.yml";
        else if (config.mode == "mono-right") config.output = "right_camera.yml";
        else if (config.mode == "stereo") config.output = "stereo_extrinsics.yml";
        else if (config.mode == "fisheye-left") config.output = "left_fisheye.yml";
        else if (config.mode == "fisheye-right") config.output = "right_fisheye.yml";
        else if (config.mode == "stereo-fisheye") config.output = "stereo_fisheye_extrinsics.yml";
        else config.output = "";
    }
    return config;
}

class LatestFrameGrabber {
public:
    LatestFrameGrabber(std::string device, int width, int height, int fps)
        : device_(std::move(device)), width_(width), height_(height), fps_(fps) {}

    bool start() {
        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) return false;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        running_.store(true);
        worker_ = std::thread(&LatestFrameGrabber::loop, this);
        return true;
    }

    void stop() {
        running_.store(false);
        if (worker_.joinable()) worker_.join();
        cap_.release();
    }

    bool get(cv::Mat& frame, std::uint64_t& seq, FrameClock::time_point* received = nullptr) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_.empty()) return false;
        latest_.copyTo(frame);
        seq = seq_;
        if (received) *received = received_;
        return true;
    }

    FrameClock::time_point receivedTime() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return received_;
    }

private:
    void loop() {
        cv::Mat frame;
        while (running_.load()) {
            if (!cap_.read(frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            const auto received = FrameClock::now();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                frame.copyTo(latest_);
                ++seq_;
                received_ = received;
            }
        }
    }

    std::string device_;
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;
    cv::VideoCapture cap_;
    std::thread worker_;
    std::atomic<bool> running_{false};
    mutable std::mutex mutex_;
    cv::Mat latest_;
    std::uint64_t seq_ = 0;
    FrameClock::time_point received_{};
};

static bool detectScaled(const StereoCalibrator& calibrator,
                         const cv::Mat& image,
                         double scale,
                         std::vector<cv::Point2f>& corners) {
    if (scale <= 0.0 || scale >= 0.999) {
        return calibrator.detectCorners(image, corners);
    }
    cv::Mat small;
    cv::resize(image, small, cv::Size(), scale, scale, cv::INTER_AREA);
    bool found = calibrator.detectCorners(small, corners);
    if (!found) return false;
    if (refineScaledBoardCorners(image, small.size(), scale, corners)) return true;
    corners.clear();
    return false;
}

static std::string sampleName(const std::string& dir, const std::string& side, std::size_t index) {
    std::ostringstream name;
    name << dir << "/" << side << "_" << std::setw(4) << std::setfill('0') << index << ".png";
    return name.str();
}

static bool loadMonoCalibration(const std::string& path,
                                cv::Size expected_image_size,
                                cv::Size expected_board_size,
                                float expected_square_size,
                                MonoCalibrationResult& result,
                                std::string& error) try {
    result = {};
    error.clear();
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        error = "无法打开单目标定文件：" + path;
        return false;
    }
    const int width = static_cast<int>(fs["image_width"]);
    const int height = static_cast<int>(fs["image_height"]);
    const int cols = static_cast<int>(fs["board_cols"]);
    const int rows = static_cast<int>(fs["board_rows"]);
    const double square_size = static_cast<double>(fs["square_size_mm"]);
    std::string calibration_type;
    fs["calibration_type"] >> calibration_type;
    if (calibration_type != "monocular") {
        error = "Not a pinhole monocular calibration: " + path;
        return false;
    }
    fs["camera_matrix"] >> result.camera_matrix;
    fs["dist_coeffs"] >> result.dist_coeffs;
    result.image_size = {width, height};
    result.used_sample_count = static_cast<int>(fs["sample_count"]);
    result.reprojection_error = static_cast<double>(fs["reprojection_error"]);
    if (result.camera_matrix.empty() || result.dist_coeffs.empty()) {
        error = "单目标定文件缺少 camera_matrix 或 dist_coeffs：" + path;
        return false;
    }
    if (!validBoardGeometry(expected_board_size, expected_square_size) ||
        !std::isfinite(square_size) || square_size <= 0.0 || width <= 0 || height <= 0 ||
        result.image_size != expected_image_size ||
        cv::Size(cols, rows) != expected_board_size ||
        std::abs(square_size - expected_square_size) > 1e-4) {
        std::ostringstream ss;
        ss << "单目标定文件规格不匹配：" << path
           << "（文件 " << width << "x" << height << ", " << cols << "x" << rows
           << ", " << square_size << " mm；当前 " << expected_image_size.width << "x"
           << expected_image_size.height << ", " << expected_board_size.width << "x"
           << expected_board_size.height << ", " << expected_square_size << " mm）";
        error = ss.str();
        result.valid = false;
        return false;
    }
    if (!validCameraIntrinsics(result.camera_matrix, result.dist_coeffs, false)) {
        error = "单目标定文件中的内参或畸变参数无效：" + path;
        result.valid = false;
        return false;
    }
    if (fs["reprojection_error"].empty() || !std::isfinite(result.reprojection_error) ||
        result.reprojection_error < 0.0 || result.reprojection_error > 2.0) {
        std::ostringstream ss;
        ss << "单目标定文件重投影误差过大：" << path << " ("
           << result.reprojection_error << " px)";
        error = ss.str();
        result.valid = false;
        return false;
    }
    result.camera_matrix.convertTo(result.camera_matrix, CV_64F);
    result.dist_coeffs.convertTo(result.dist_coeffs, CV_64F);
    result.valid = true;
    return true;
} catch (const cv::Exception&) {
    result = {};
    error = "Invalid pinhole calibration file: " + path;
    return false;
}

static bool loadFisheyeCalibration(const std::string& path,
                                   cv::Size expected_image_size,
                                   cv::Size expected_board_size,
                                   float expected_square_size,
                                   FisheyeCalibrationResult& result,
                                   std::string& error) try {
    result = {};
    error.clear();
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        error = "无法打开鱼眼标定文件：" + path;
        return false;
    }
    std::string calibration_type;
    std::string camera_model;
    fs["calibration_type"] >> calibration_type;
    fs["camera_model"] >> camera_model;
    const int width = static_cast<int>(fs["image_width"]);
    const int height = static_cast<int>(fs["image_height"]);
    const int cols = static_cast<int>(fs["board_cols"]);
    const int rows = static_cast<int>(fs["board_rows"]);
    const double square_size = static_cast<double>(fs["square_size_mm"]);
    fs["camera_matrix"] >> result.camera_matrix;
    fs["dist_coeffs"] >> result.dist_coeffs;
    result.image_size = {width, height};
    result.used_sample_count = static_cast<int>(fs["sample_count"]);
    result.reprojection_error = static_cast<double>(fs["reprojection_error"]);
    if (calibration_type != "fisheye" || camera_model != "opencv_fisheye_4") {
        error = "标定文件不是 OpenCV 四参数鱼眼模型：" + path;
        return false;
    }
    if (result.camera_matrix.channels() != 1 || result.dist_coeffs.channels() != 1 ||
        result.camera_matrix.rows != 3 || result.camera_matrix.cols != 3 ||
        result.dist_coeffs.total() != 4 ||
        (result.dist_coeffs.rows != 1 && result.dist_coeffs.cols != 1)) {
        error = "Invalid fisheye matrix dimensions: " + path;
        return false;
    }
    result.camera_matrix.convertTo(result.camera_matrix, CV_64F);
    result.dist_coeffs.convertTo(result.dist_coeffs, CV_64F);
    if (!cv::checkRange(result.camera_matrix) ||
        !cv::checkRange(result.dist_coeffs) || result.camera_matrix.at<double>(0, 0) <= 0.0 ||
        result.camera_matrix.at<double>(1, 1) <= 0.0 ||
        std::abs(result.camera_matrix.at<double>(0, 1)) > 1e-9 ||
        std::abs(result.camera_matrix.at<double>(1, 0)) > 1e-9 ||
        std::abs(result.camera_matrix.at<double>(2, 0)) > 1e-9 ||
        std::abs(result.camera_matrix.at<double>(2, 1)) > 1e-9 ||
        std::abs(result.camera_matrix.at<double>(2, 2) - 1.0) > 1e-9) {
        error = "鱼眼文件缺少有效 camera_matrix 或四个 dist_coeffs：" + path;
        return false;
    }
    if (!validBoardGeometry(expected_board_size, expected_square_size) ||
        !std::isfinite(square_size) || square_size <= 0.0 || width <= 0 || height <= 0 ||
        result.image_size != expected_image_size || cv::Size(cols, rows) != expected_board_size ||
        std::abs(square_size - expected_square_size) > 1e-4) {
        error = "鱼眼标定文件的分辨率或棋盘规格与当前设置不匹配：" + path;
        return false;
    }
    if (fs["reprojection_error"].empty() || !std::isfinite(result.reprojection_error) ||
        result.reprojection_error < 0.0 || result.reprojection_error > 2.0) {
        error = "鱼眼标定文件 RMS 无效或超过 2 px：" + path;
        return false;
    }
    result.valid = true;
    return true;
} catch (const cv::Exception&) {
    result = {};
    error = "鱼眼标定文件格式无效：" + path;
    return false;
}

class TextRenderer {
public:
    ~TextRenderer() {
#ifdef HAVE_FREETYPE
        if (face_) FT_Done_Face(face_);
        if (library_) FT_Done_FreeType(library_);
#endif
    }

    bool start(const std::string& font_path) {
#ifdef HAVE_FREETYPE
        const std::vector<std::string> candidates = font_path.empty() ? defaultFontPaths()
                                                                       : std::vector<std::string>{font_path};
        if (FT_Init_FreeType(&library_) != 0) {
            return false;
        }
        for (const auto& path : candidates) {
            if (!std::filesystem::exists(path)) continue;
            FT_Face candidate_face = nullptr;
            if (FT_New_Face(library_, path.c_str(), 0, &candidate_face) != 0) {
                continue;
            }
            face_ = candidate_face;
            font_path_ = path;
            return true;
        }
#else
        (void)font_path;
#endif
        return false;
    }

    bool available() const {
#ifdef HAVE_FREETYPE
        return face_ != nullptr;
#else
        return false;
#endif
    }

    const std::string& fontPath() const {
        return font_path_;
    }

    cv::Size textSize(const std::string& text, double scale, int thickness = 1) const {
#ifdef HAVE_FREETYPE
        if (face_) {
            const int pixels=std::max(12,static_cast<int>(std::round(scale*32.0)));
            FT_Set_Pixel_Sizes(face_,0,pixels);
            int width=0;
            for(const auto cp:decodeUtf8(text)) {
                if(FT_Load_Char(face_,cp,FT_LOAD_DEFAULT)==0) width+=static_cast<int>(face_->glyph->advance.x>>6);
                else width+=pixels;
            }
            return {width+pixels/8,pixels};
        }
#endif
        int baseline=0;
        const cv::Size size=cv::getTextSize(text,cv::FONT_HERSHEY_SIMPLEX,scale,thickness,&baseline);
        return {size.width,size.height+baseline};
    }

    void put(cv::Mat& image,
             const std::string& text,
             cv::Point origin,
             double scale,
             const cv::Scalar& color,
             int thickness = 1) const {
#ifdef HAVE_FREETYPE
        if (face_) {
            const int pixel_height = std::max(12, static_cast<int>(std::round(scale * 32.0)));
            drawUtf8(image, text, origin + cv::Point(1, 1), pixel_height, cv::Scalar(0, 0, 0));
            drawUtf8(image, text, origin, pixel_height, color);
            return;
        }
#endif
        cv::putText(image, text, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, scale,
                    cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
        cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
    }

private:
    static std::vector<std::uint32_t> decodeUtf8(const std::string& text) {
        std::vector<std::uint32_t> out;
        for (std::size_t i = 0; i < text.size();) {
            const unsigned char c = static_cast<unsigned char>(text[i]);
            if (c < 0x80) {
                out.push_back(c);
                ++i;
            } else if ((c >> 5) == 0x6 && i + 1 < text.size()) {
                out.push_back(((c & 0x1F) << 6) |
                              (static_cast<unsigned char>(text[i + 1]) & 0x3F));
                i += 2;
            } else if ((c >> 4) == 0xE && i + 2 < text.size()) {
                out.push_back(((c & 0x0F) << 12) |
                              ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 6) |
                              (static_cast<unsigned char>(text[i + 2]) & 0x3F));
                i += 3;
            } else if ((c >> 3) == 0x1E && i + 3 < text.size()) {
                out.push_back(((c & 0x07) << 18) |
                              ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 12) |
                              ((static_cast<unsigned char>(text[i + 2]) & 0x3F) << 6) |
                              (static_cast<unsigned char>(text[i + 3]) & 0x3F));
                i += 4;
            } else {
                out.push_back('?');
                ++i;
            }
        }
        return out;
    }

#ifdef HAVE_FREETYPE
    void drawUtf8(cv::Mat& image,
                  const std::string& text,
                  cv::Point origin,
                  int pixel_height,
                  const cv::Scalar& color) const {
        FT_Set_Pixel_Sizes(face_, 0, pixel_height);
        int pen_x = origin.x;
        const int baseline_y = origin.y;
        for (const std::uint32_t codepoint : decodeUtf8(text)) {
            if (codepoint == '\n') {
                pen_x = origin.x;
                continue;
            }
            if (FT_Load_Char(face_, codepoint, FT_LOAD_RENDER) != 0) {
                pen_x += pixel_height / 2;
                continue;
            }
            const FT_GlyphSlot glyph = face_->glyph;
            const FT_Bitmap& bitmap = glyph->bitmap;
            const int x0 = pen_x + glyph->bitmap_left;
            const int y0 = baseline_y - glyph->bitmap_top;
            for (int row = 0; row < static_cast<int>(bitmap.rows); ++row) {
                const int y = y0 + row;
                if (y < 0 || y >= image.rows) continue;
                for (int col = 0; col < static_cast<int>(bitmap.width); ++col) {
                    const int x = x0 + col;
                    if (x < 0 || x >= image.cols) continue;
                    const unsigned char alpha = bitmap.buffer[row * bitmap.pitch + col];
                    if (alpha == 0) continue;
                    cv::Vec3b& dst = image.at<cv::Vec3b>(y, x);
                    for (int channel = 0; channel < 3; ++channel) {
                        const int blended = (dst[channel] * (255 - alpha) +
                                             static_cast<int>(color[channel]) * alpha) / 255;
                        dst[channel] = static_cast<unsigned char>(std::clamp(blended, 0, 255));
                    }
                }
            }
            pen_x += static_cast<int>(glyph->advance.x >> 6);
        }
    }
#endif

    static std::vector<std::string> defaultFontPaths() {
        return {
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
            "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.otf",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/noto/NotoSansCJKsc-Regular.otf",
            "/usr/share/fonts/truetype/arphic/ukai.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        };
    }

    std::string font_path_;
#ifdef HAVE_FREETYPE
    FT_Library library_ = nullptr;
    FT_Face face_ = nullptr;
#endif
};

static TextRenderer g_text_renderer;

struct GuideLine {
    cv::Point2f a;
    cv::Point2f b;
    bool valid = false;
};

struct BoardGuide {
    GuideLine horizontal[3];
    GuideLine vertical;
    double horizontal_angle_deg[3] = {0.0, 0.0, 0.0};
    bool valid = false;
};

struct StereoGuide {
    BoardGuide left;
    BoardGuide right;
    cv::Point2f left_center;
    cv::Point2f right_center;
    bool centers_valid = false;
    double stereo_y_diff = -1.0;
    double signed_stereo_y_diff = 0.0;
    double board_to_screen_center_y = -1.0;
    double signed_roll_diff_deg = 0.0;
    double avg_corner_y_diff = -1.0;
    double horizontal_offset = -1.0;
    double roll_diff_deg = -1.0;
    bool valid = false;
};

struct StereoPose {
    bool frame_pair_valid = true;
    bool require_raw_alignment = false;
    bool raw_resolution_valid = true;
    RawAlignment raw_alignment;
    double cy_diff_px = 0.0;
    bool reliable = false;
    int stable_frames = 0;
    double orientation_error_deg = 0.0;
    double optical_error_deg = 0.0;
    CameraPose left;
    CameraPose right;
    cv::Vec3d left_center;
    cv::Vec3d right_center;
    double baseline_parallel_error_deg = -1.0;
    double height_diff_mm = 0.0;
    double pitch_diff_deg = 0.0;
    double yaw_diff_deg = 0.0;
    double roll_diff_deg = 0.0;
    bool valid = false;
};

struct OrbitViewState {
    // The initial composition keeps the complete board on the left and the
    // elevated stereo cameras on the right, matching the installation sketch.
    // Axonometric startup view: external oblique elevated observation of the
    // complete installation, with no perspective distortion.
    double yaw_deg = 45.0;
    double pitch_deg = -35.0;
    double roll_deg = 0.0;
    double zoom = 1.0;
    double pan_x = 0.0;
    double pan_y = 0.0;
    bool show_axes = true;
    bool show_reference = true;
    bool show_error = true;
};

static OrbitViewState g_orbit_view;

struct SceneMouseState {
    cv::Rect area;
    cv::Point last;
    bool dragging = false;
    std::atomic<int>* mode = nullptr;
};
static void sceneMouse(int event,int x,int y,int flags,void* data) {
    auto& mouse=*static_cast<SceneMouseState*>(data);
    const cv::Point p(x,y);
    if(event==cv::EVENT_LBUTTONUP || event==cv::EVENT_MBUTTONUP) mouse.dragging=false;
    if(!mouse.mode || (!mouse.area.contains(p) && !mouse.dragging)) {
        mouse.dragging=false; return;
    }
    if(event==cv::EVENT_LBUTTONDOWN || event==cv::EVENT_MBUTTONDOWN) {
        mouse.mode->store(3);
        mouse.dragging=true; mouse.last=p;
    }
    if(event==cv::EVENT_MOUSEWHEEL) {
        mouse.mode->store(3);
        g_orbit_view.zoom=std::clamp(g_orbit_view.zoom*std::pow(0.9,cv::getMouseWheelDelta(flags)/120.0),0.35,3.0);
    }
    if(mouse.mode->load()!=3) return;
    if(event==cv::EVENT_MOUSEMOVE && mouse.dragging) {
        const cv::Point d=p-mouse.last; mouse.last=p;
        if(flags&cv::EVENT_FLAG_MBUTTON) {
            g_orbit_view.pan_x+=double(d.x)/std::max(1,mouse.area.width);
            g_orbit_view.pan_y+=double(d.y)/std::max(1,mouse.area.height);
        } else if(flags&cv::EVENT_FLAG_LBUTTON) {
            if(flags&cv::EVENT_FLAG_SHIFTKEY) g_orbit_view.roll_deg+=d.x*0.4;
            else {g_orbit_view.yaw_deg+=d.x*0.4;g_orbit_view.pitch_deg+=d.y*0.4;}
        }
    }
}

static cv::Matx33d sceneRotation() {
    cv::Matx33d rx, ry, rz;
    cv::Rodrigues(cv::Vec3d(g_orbit_view.pitch_deg*CV_PI/180,0,0),rx);
    cv::Rodrigues(cv::Vec3d(0,g_orbit_view.yaw_deg*CV_PI/180,0),ry);
    cv::Rodrigues(cv::Vec3d(0,0,g_orbit_view.roll_deg*CV_PI/180),rz);
    return rz*rx*ry;
}

#ifdef HAVE_EGL_OPENGL_VIEW
class GlStereoRenderer {
public:
    ~GlStereoRenderer() {
        destroy();
    }

    bool render(cv::Mat& panel,
                const StereoPose& pose,
                cv::Size2f board_world_size,
                const OrbitViewState& view) {
        if (panel.empty() || panel.type() != CV_8UC3) return false;
        if (!ensure(panel.cols, panel.rows)) return false;

        eglMakeCurrent(display_, surface_, surface_, context_);
        glViewport(0, 0, width_, height_);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glClearColor(0.08F, 0.09F, 0.10F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const float board_w = std::max(1.0F, board_world_size.width);
        const float board_h = std::max(1.0F, board_world_size.height);
        const Vec3 board_origin{0.0F, 0.0F, 0.0F};
        const Vec3 board_center{board_origin.x + board_w * 0.5F,
                                board_origin.y + board_h * 0.5F,
                                0.0F};
        const bool measured_pose = pose.valid && isFinite(toGl(pose.left_center)) &&
                                   isFinite(toGl(pose.right_center));
        const Vec3 left = measured_pose ? toGl(pose.left_center)
                                        : Vec3{board_w * 0.75F, board_h * 0.30F, board_w * 2.35F};
        const Vec3 right = measured_pose ? toGl(pose.right_center)
                                         : Vec3{board_w * 1.45F, board_h * 0.30F, board_w * 2.20F};

        Vec3 left_x;
        Vec3 left_y;
        Vec3 left_z;
        Vec3 right_x;
        Vec3 right_y;
        Vec3 right_z;
        if (measured_pose) {
            left_x = normalize(toGl(pose.left.x_axis));
            left_y = normalize(toGl(pose.left.y_axis));
            left_z = normalize(toGl(pose.left.z_axis));
            right_x = normalize(toGl(pose.right.x_axis));
            right_y = normalize(toGl(pose.right.y_axis));
            right_z = normalize(toGl(pose.right.z_axis));
        } else {
            makeCameraBasis(normalize(sub(board_center, left)), left_x, left_y, left_z);
            makeCameraBasis(normalize(sub(board_center, right)), right_x, right_y, right_z);
        }

        const float board_span = std::max(board_w, board_h);
        const float layout_span = std::max({board_span,
                                            norm(sub(left, board_center)),
                                            norm(sub(right, board_center)),
                                            norm(sub(right, left))});
        const float camera_size = std::clamp(layout_span * 0.055F,
                                             board_span * 0.10F,
                                             board_span * 0.70F);
        const float camera_axis_length = std::max(camera_size * 2.2F, board_span * 0.24F);
        const float world_axis_length = board_span * 0.32F;

        std::vector<Vec3> scene_points = {
            board_origin,
            {board_w, 0.0F, 0.0F},
            {board_w, board_h, 0.0F},
            {0.0F, board_h, 0.0F},
            board_center,
            left,
            right,
            add(left, mul(left_x, camera_axis_length)),
            add(left, mul(left_y, camera_axis_length)),
            add(left, mul(left_z, camera_axis_length)),
            add(right, mul(right_x, camera_axis_length)),
            add(right, mul(right_y, camera_axis_length)),
            add(right, mul(right_z, camera_axis_length)),
            add(board_origin, {world_axis_length, 0.0F, 0.0F}),
            add(board_origin, {0.0F, world_axis_length, 0.0F}),
            add(board_origin, {0.0F, 0.0F, -world_axis_length})
        };
        appendCameraFrustumPoints(scene_points, left, left_x, left_y, left_z, camera_size);
        appendCameraFrustumPoints(scene_points, right, right_x, right_y, right_z, camera_size);

        Mat4 view_rotation = identity();
        const cv::Matx33d rotation = sceneRotation();
        for(int r=0;r<3;++r) for(int c=0;c<3;++c)
            view_rotation.data[c*4+r]=static_cast<float>(rotation(r,c));
        Bounds3 view_bounds;
        for (const Vec3& point : scene_points) {
            view_bounds.include(transformPoint(view_rotation, point));
        }

        const float aspect = static_cast<float>(width_) / std::max(1, height_);
        const float zoom = static_cast<float>(std::clamp(view.zoom, 0.35, 3.0));
        const Vec3 view_center = view_bounds.center();
        const Vec3 view_size = view_bounds.size();
        float half_width = std::max(view_size.x * 0.58F, board_span * 0.45F) * zoom;
        float half_height = std::max(view_size.y * 0.58F, board_span * 0.45F) * zoom;
        if (half_width / half_height < aspect) {
            half_width = half_height * aspect;
        } else {
            half_height = half_width / aspect;
        }

        const float depth_gap = std::max({view_size.x, view_size.y, view_size.z, board_span});
        const float camera_distance = view_size.z * 0.5F + depth_gap;
        const Mat4 model = multiply(translate(-view_center.x,
                                              -view_center.y,
                                              -view_center.z - camera_distance),
                                    view_rotation);
        Mat4 projection = orthographic(-half_width,
                                             half_width,
                                             -half_height,
                                             half_height,
                                             std::max(0.01F, depth_gap * 0.01F),
                                             camera_distance + view_size.z * 0.5F + depth_gap);
        projection.data[12]+=static_cast<float>(view.pan_x*2);
        projection.data[13]-=static_cast<float>(view.pan_y*2);
        const Mat4 mvp = multiply(projection, model);
        glUseProgram(program_);
        glUniformMatrix4fv(mvp_location_, 1, GL_FALSE, mvp.data.data());

        drawBoardPlane(board_w, board_h, board_origin);
        drawLine(left, right, {0.42F, 0.78F, 1.0F}, 4.0F);
        drawLine(left, board_center, {1.0F, 0.82F, 0.42F}, 2.0F);
        drawLine(right, board_center, {1.0F, 0.82F, 0.42F}, 2.0F);
        drawCameraFrustum(left, left_x, left_y, left_z, camera_size, {1.0F, 0.42F, 0.12F});
        drawCameraFrustum(right, right_x, right_y, right_z, camera_size, {0.12F, 0.72F, 1.0F});
        drawCameraAxes(left, left_x, left_y, left_z, camera_axis_length);
        drawCameraAxes(right, right_x, right_y, right_z, camera_axis_length);
        if (view.show_axes) drawWorldAxes(board_w, board_h, board_origin);
        if (view.show_reference) drawReferenceLine(board_w, board_h, board_origin);
        if (view.show_error && measured_pose) drawErrorArrows(pose, left, right, board_w);

        std::vector<unsigned char> rgba(static_cast<std::size_t>(width_ * height_ * 4));
        glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        cv::Mat rgba_image(height_, width_, CV_8UC4, rgba.data());
        cv::Mat bgr;
        cv::cvtColor(rgba_image, bgr, cv::COLOR_RGBA2BGR);
        cv::flip(bgr, panel, 0);
        drawSceneLabels(panel,
                        mvp,
                        board_center,
                        left,
                        right,
                        left_x,
                        left_y,
                        left_z,
                        right_x,
                        right_y,
                        right_z,
                        camera_axis_length);
        return true;
    }

private:
    struct Vec3 {
        float x;
        float y;
        float z;
    };

    struct Mat4 {
        std::array<float, 16> data{};
    };

    struct Bounds3 {
        Vec3 min{};
        Vec3 max{};
        bool valid = false;

        void include(const Vec3& point) {
            if (!isFinite(point)) return;
            if (!valid) {
                min = point;
                max = point;
                valid = true;
                return;
            }
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            min.z = std::min(min.z, point.z);
            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
            max.z = std::max(max.z, point.z);
        }

        Vec3 center() const {
            return {(min.x + max.x) * 0.5F,
                    (min.y + max.y) * 0.5F,
                    (min.z + max.z) * 0.5F};
        }

        Vec3 size() const {
            return {std::max(1.0F, max.x - min.x),
                    std::max(1.0F, max.y - min.y),
                    std::max(1.0F, max.z - min.z)};
        }
    };

    bool ensure(int width, int height) {
        if (display_ == EGL_NO_DISPLAY && !initEgl()) return false;
        if (width == width_ && height == height_ && surface_ != EGL_NO_SURFACE) return true;
        destroySurface();
        width_ = width;
        height_ = height;
        const EGLint attrs[] = {EGL_WIDTH, width_, EGL_HEIGHT, height_, EGL_NONE};
        surface_ = eglCreatePbufferSurface(display_, config_, attrs);
        if (surface_ == EGL_NO_SURFACE) return false;
        if (!eglMakeCurrent(display_, surface_, surface_, context_)) return false;
        return program_ || initGl();
    }

    bool initEgl() {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display_ == EGL_NO_DISPLAY) return false;
        if (!eglInitialize(display_, nullptr, nullptr)) return false;
        if (!eglBindAPI(EGL_OPENGL_ES_API)) return false;
        const EGLint config_attrs[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 16,
            EGL_NONE
        };
        EGLint count = 0;
        if (!eglChooseConfig(display_, config_attrs, &config_, 1, &count) || count < 1) return false;
        const EGLint context_attrs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
        context_ = eglCreateContext(display_, config_, EGL_NO_CONTEXT, context_attrs);
        return context_ != EGL_NO_CONTEXT;
    }

    bool initGl() {
        const char* vertex_src =
            "attribute vec3 a_pos;\n"
            "attribute vec3 a_color;\n"
            "uniform mat4 u_mvp;\n"
            "varying vec3 v_color;\n"
            "void main(){ gl_Position = u_mvp * vec4(a_pos, 1.0); v_color = a_color; }\n";
        const char* fragment_src =
            "precision mediump float;\n"
            "varying vec3 v_color;\n"
            "void main(){ gl_FragColor = vec4(v_color, 1.0); }\n";
        const GLuint vs = compileShader(GL_VERTEX_SHADER, vertex_src);
        const GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragment_src);
        if (!vs || !fs) return false;
        program_ = glCreateProgram();
        glAttachShader(program_, vs);
        glAttachShader(program_, fs);
        glBindAttribLocation(program_, 0, "a_pos");
        glBindAttribLocation(program_, 1, "a_color");
        glLinkProgram(program_);
        GLint linked = 0;
        glGetProgramiv(program_, GL_LINK_STATUS, &linked);
        glDeleteShader(vs);
        glDeleteShader(fs);
        if (!linked) return false;
        mvp_location_ = glGetUniformLocation(program_, "u_mvp");
        glGenBuffers(1, &vbo_);
        return mvp_location_ >= 0 && vbo_ != 0;
    }

    static GLuint compileShader(GLenum type, const char* source) {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (!compiled) {
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }

    static Vec3 toGl(const cv::Vec3d& point) {
        return {static_cast<float>(point[0]), static_cast<float>(point[1]), static_cast<float>(-point[2])};
    }

    static bool isFinite(const Vec3& point) {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    // Build a right-handed camera frame whose Z axis points along the supplied
    // optical direction.  The world Y axis is used as the up reference unless
    // it is nearly parallel to the optical direction.
    static void makeCameraBasis(const Vec3& optical_direction,
                                Vec3& x_axis,
                                Vec3& y_axis,
                                Vec3& z_axis) {
        z_axis = normalize(optical_direction);
        Vec3 up{0.0F, 1.0F, 0.0F};
        if (std::abs(dot(up, z_axis)) > 0.92F) {
            up = {1.0F, 0.0F, 0.0F};
        }
        x_axis = normalize(cross(up, z_axis));
        y_axis = normalize(cross(z_axis, x_axis));
    }

    static Vec3 transformPoint(const Mat4& matrix, const Vec3& point) {
        const float x = matrix.data[0] * point.x + matrix.data[4] * point.y +
                        matrix.data[8] * point.z + matrix.data[12];
        const float y = matrix.data[1] * point.x + matrix.data[5] * point.y +
                        matrix.data[9] * point.z + matrix.data[13];
        const float z = matrix.data[2] * point.x + matrix.data[6] * point.y +
                        matrix.data[10] * point.z + matrix.data[14];
        const float w = matrix.data[3] * point.x + matrix.data[7] * point.y +
                        matrix.data[11] * point.z + matrix.data[15];
        if (std::abs(w) > 1e-6F && std::abs(w - 1.0F) > 1e-6F) {
            return {x / w, y / w, z / w};
        }
        return {x, y, z};
    }

    static void appendCameraFrustumPoints(std::vector<Vec3>& points,
                                          const Vec3& center,
                                          const Vec3& x_axis,
                                          const Vec3& y_axis,
                                          const Vec3& optical_axis,
                                          float size) {
        const Vec3 x = normalize(x_axis);
        const Vec3 y = normalize(y_axis);
        const Vec3 z = normalize(optical_axis);
        const float depth = std::max(size * 2.4F, 1.0F);
        const Vec3 plane_center = add(center, mul(z, depth));
        const Vec3 dx = mul(x, size);
        const Vec3 dy = mul(y, size);
        points.push_back(plane_center);
        points.push_back(add(add(plane_center, dx), dy));
        points.push_back(add(add(plane_center, dx), mul(dy, -1.0F)));
        points.push_back(add(add(plane_center, mul(dx, -1.0F)), mul(dy, -1.0F)));
        points.push_back(add(add(plane_center, mul(dx, -1.0F)), dy));
    }

    void drawBoardPlane(float board_w, float board_h, const Vec3& origin) {
        const Vec3 p00{origin.x, origin.y, origin.z};
        const Vec3 p10{origin.x + board_w, origin.y, origin.z};
        const Vec3 p11{origin.x + board_w, origin.y + board_h, origin.z};
        const Vec3 p01{origin.x, origin.y + board_h, origin.z};
        auto boardPoint = [&](float x, float y) {
            return Vec3{origin.x + x, origin.y + y, origin.z};
        };
        drawTriangles({p00, p10, p11, p00, p11, p01},
                      {0.08F, 0.20F, 0.19F});
        const Vec3 c{0.15F, 0.95F, 0.75F};
        drawLine(p00, p10, c, 3.0F);
        drawLine(p10, p11, c, 3.0F);
        drawLine(p11, p01, c, 3.0F);
        drawLine(p01, p00, c, 3.0F);
        for (int i = 1; i < 8; ++i) {
            const float x = board_w * i / 8.0F;
            drawLine(boardPoint(x, 0.0F), boardPoint(x, board_h), {0.24F, 0.33F, 0.34F}, 1.0F);
        }
        for (int i = 1; i < 6; ++i) {
            const float y = board_h * i / 6.0F;
            drawLine(boardPoint(0.0F, y), boardPoint(board_w, y), {0.24F, 0.33F, 0.34F}, 1.0F);
        }
    }

    void drawCameraFrustum(const Vec3& center,
                           const Vec3& x_axis,
                           const Vec3& y_axis,
                           const Vec3& optical_axis,
                           float size,
                           const Vec3& color) {
        const Vec3 x = normalize(x_axis);
        const Vec3 y = normalize(y_axis);
        const Vec3 z = normalize(optical_axis);
        const float depth = std::max(size * 2.4F, 1.0F);
        const Vec3 plane_center = add(center, mul(z, depth));
        const Vec3 dx = mul(x, size);
        const Vec3 dy = mul(y, size);
        const Vec3 c0 = add(add(plane_center, dx), dy);
        const Vec3 c1 = add(add(plane_center, dx), mul(dy, -1.0F));
        const Vec3 c2 = add(add(plane_center, mul(dx, -1.0F)), mul(dy, -1.0F));
        const Vec3 c3 = add(add(plane_center, mul(dx, -1.0F)), dy);
        const std::vector<Vec3> lines = {
            center, c0, center, c1, center, c2, center, c3,
            c0, c1, c1, c2, c2, c3, c3, c0
        };
        drawLines(lines, color, 3.0F);
    }

    void drawCameraAxes(const Vec3& center,
                        const Vec3& x_axis,
                        const Vec3& y_axis,
                        const Vec3& z_axis,
                        float length) {
        drawLine(center, add(center, mul(normalize(x_axis), length)), {1.0F, 0.15F, 0.10F}, 2.0F);
        drawLine(center, add(center, mul(normalize(y_axis), length)), {0.15F, 0.90F, 0.25F}, 2.0F);
        drawLine(center, add(center, mul(normalize(z_axis), length)), {0.20F, 0.45F, 1.0F}, 2.0F);
    }

    void drawSceneLabels(cv::Mat& panel,
                         const Mat4& mvp,
                         const Vec3& board_center,
                         const Vec3& left,
                         const Vec3& right,
                         const Vec3& left_x,
                         const Vec3& left_y,
                         const Vec3& left_z,
                         const Vec3& right_x,
                         const Vec3& right_y,
                         const Vec3& right_z,
                         float camera_axis_length) {
        const float ui = std::clamp(std::sqrt(static_cast<float>(panel.rows) / 720.0F) * 1.15F,
                                     0.75F,
                                     1.8F);
        auto project = [&](const Vec3& point, cv::Point& result) {
            const float x = mvp.data[0] * point.x + mvp.data[4] * point.y +
                             mvp.data[8] * point.z + mvp.data[12];
            const float y = mvp.data[1] * point.x + mvp.data[5] * point.y +
                             mvp.data[9] * point.z + mvp.data[13];
            const float w = mvp.data[3] * point.x + mvp.data[7] * point.y +
                            mvp.data[11] * point.z + mvp.data[15];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) ||
                std::abs(w) < 1e-6F) {
                return false;
            }
            const float ndc_x = x / w;
            const float ndc_y = y / w;
            if (!std::isfinite(ndc_x) || !std::isfinite(ndc_y) ||
                ndc_x < -1.15F || ndc_x > 1.15F || ndc_y < -1.15F || ndc_y > 1.15F) {
                return false;
            }
            result.x = static_cast<int>(std::lround((ndc_x * 0.5F + 0.5F) * panel.cols));
            result.y = static_cast<int>(std::lround((1.0F - (ndc_y * 0.5F + 0.5F)) * panel.rows));
            return result.x >= -panel.cols / 5 && result.x <= panel.cols * 6 / 5 &&
                   result.y >= -panel.rows / 5 && result.y <= panel.rows * 6 / 5;
        };
        auto label = [&](const Vec3& point,
                         const std::string& text,
                         const cv::Scalar& color,
                         const cv::Point& offset) {
            cv::Point position;
            if (!project(point, position)) return;
            position += cv::Point(static_cast<int>(std::lround(offset.x * ui)),
                                  static_cast<int>(std::lround(offset.y * ui)));
            g_text_renderer.put(panel, text, position, 0.58 * ui, color, 2);
        };

        label(board_center, g_text_renderer.available() ? "标定板" : "BOARD", cv::Scalar(170, 255, 220), {10, -12});
        label(left, g_text_renderer.available() ? "左" : "L", cv::Scalar(40, 150, 255), {-18, -14});
        label(right, g_text_renderer.available() ? "右" : "R", cv::Scalar(255, 180, 40), {10, -14});
        const cv::Scalar x_color(50, 80, 255);
        const cv::Scalar y_color(70, 220, 70);
        const cv::Scalar z_color(255, 130, 50);
        label(add(left, mul(left_x, camera_axis_length)), "X", x_color, {8, -8});
        label(add(left, mul(left_y, camera_axis_length)), "Y", y_color, {8, -8});
        label(add(left, mul(left_z, camera_axis_length)), "Z", z_color, {8, -8});
        label(add(right, mul(right_x, camera_axis_length)), "X", x_color, {8, -8});
        label(add(right, mul(right_y, camera_axis_length)), "Y", y_color, {8, -8});
        label(add(right, mul(right_z, camera_axis_length)), "Z", z_color, {8, -8});
    }

    void drawWorldAxes(float board_w, float board_h, const Vec3& board_origin) {
        const float length = std::max(board_w, board_h) * 0.32F;
        const Vec3 origin{board_origin.x - board_w * 0.06F,
                          board_origin.y - board_h * 0.06F,
                          board_origin.z + 0.02F};
        drawLine(origin, add(origin, {length, 0.0F, 0.0F}), {1.0F, 0.15F, 0.10F}, 3.0F);
        drawLine(origin, add(origin, {0.0F, length, 0.0F}), {0.15F, 0.90F, 0.25F}, 3.0F);
        drawLine(origin, add(origin, {0.0F, 0.0F, -length}), {0.20F, 0.45F, 1.0F}, 3.0F);
    }

    void drawReferenceLine(float board_w, float board_h, const Vec3& board_origin) {
        const float y = board_origin.y + board_h * 0.5F;
        drawLine({board_origin.x - board_w * 0.18F, y, board_origin.z + 0.02F},
                 {board_origin.x + board_w * 1.18F, y, board_origin.z + 0.02F},
                 {0.85F, 0.90F, 0.55F}, 2.0F);
    }

    void drawErrorArrows(const StereoPose& pose, const Vec3& left, const Vec3& right, float board_w) {
        const float scale = board_w * 0.20F;
        if (std::abs(pose.height_diff_mm) > 0.5) {
            const Vec3 dir = pose.height_diff_mm > 0.0 ? Vec3{0.0F, -1.0F, 0.0F} : Vec3{0.0F, 1.0F, 0.0F};
            drawArrow(right, add(right, mul(dir, scale)), {1.0F, 0.9F, 0.1F}, 3.0F);
        }
        if (std::abs(pose.yaw_diff_deg) > 0.10) {
            const Vec3 dir = normalize(sub(left, right));
            drawArrow(right, add(right, mul(dir, scale * 0.7F)), {1.0F, 0.35F, 0.25F}, 3.0F);
        }
    }

    void drawArrow(const Vec3& a, const Vec3& b, const Vec3& color, float width) {
        drawLine(a, b, color, width);
        const Vec3 dir = normalize(sub(b, a));
        const Vec3 side = normalize(cross(dir, {0.0F, 0.0F, 1.0F}));
        const float head = norm(sub(b, a)) * 0.18F;
        drawLine(b, add(add(b, mul(dir, -head)), mul(side, head * 0.45F)), color, width);
        drawLine(b, add(add(b, mul(dir, -head)), mul(side, -head * 0.45F)), color, width);
    }

    void drawLine(const Vec3& a, const Vec3& b, const Vec3& color, float width) {
        drawLines({a, b}, color, width);
    }

    void drawLines(const std::vector<Vec3>& points, const Vec3& color, float width) {
        if (points.empty()) return;
        std::vector<float> vertices;
        vertices.reserve(points.size() * 6);
        for (const auto& p : points) {
            vertices.push_back(p.x);
            vertices.push_back(p.y);
            vertices.push_back(p.z);
            vertices.push_back(color.x);
            vertices.push_back(color.y);
            vertices.push_back(color.z);
        }
        glLineWidth(width);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                     vertices.data(),
                     GL_STREAM_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(points.size()));
    }

    void drawTriangles(const std::vector<Vec3>& points, const Vec3& color) {
        if (points.empty()) return;
        std::vector<float> vertices;
        vertices.reserve(points.size() * 6);
        for (const auto& p : points) {
            vertices.push_back(p.x);
            vertices.push_back(p.y);
            vertices.push_back(p.z);
            vertices.push_back(color.x);
            vertices.push_back(color.y);
            vertices.push_back(color.z);
        }
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                     vertices.data(),
                     GL_STREAM_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(points.size()));
    }

    static float dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static float norm(const Vec3& a) {
        return std::sqrt(std::max(0.0F, dot(a, a)));
    }

    static Vec3 normalize(const Vec3& a) {
        const float n = norm(a);
        if (n < 1e-5F) return {0.0F, 0.0F, 1.0F};
        return {a.x / n, a.y / n, a.z / n};
    }

    static Vec3 add(const Vec3& a, const Vec3& b) {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    static Vec3 sub(const Vec3& a, const Vec3& b) {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    static Vec3 mul(const Vec3& a, float s) {
        return {a.x * s, a.y * s, a.z * s};
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    static Vec3 opticalAxisTowardBoard(const Vec3& camera, const Vec3& axis, const Vec3& board_center) {
        Vec3 dir = normalize(axis);
        if (dot(dir, sub(board_center, camera)) < 0.0F) {
            dir = mul(dir, -1.0F);
        }
        return dir;
    }

    static Mat4 identity() {
        Mat4 m;
        m.data = {1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1};
        return m;
    }

    static Mat4 multiply(const Mat4& a, const Mat4& b) {
        Mat4 out;
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                float v = 0.0F;
                for (int k = 0; k < 4; ++k) {
                    v += a.data[k * 4 + row] * b.data[col * 4 + k];
                }
                out.data[col * 4 + row] = v;
            }
        }
        return out;
    }

    static Mat4 translate(float x, float y, float z) {
        Mat4 m = identity();
        m.data[12] = x;
        m.data[13] = y;
        m.data[14] = z;
        return m;
    }

    static Mat4 rotateX(float deg) {
        Mat4 m = identity();
        const float r = deg * static_cast<float>(CV_PI / 180.0);
        const float c = std::cos(r);
        const float s = std::sin(r);
        m.data[5] = c;
        m.data[6] = s;
        m.data[9] = -s;
        m.data[10] = c;
        return m;
    }

    static Mat4 rotateY(float deg) {
        Mat4 m = identity();
        const float r = deg * static_cast<float>(CV_PI / 180.0);
        const float c = std::cos(r);
        const float s = std::sin(r);
        m.data[0] = c;
        m.data[2] = -s;
        m.data[8] = s;
        m.data[10] = c;
        return m;
    }

    static Mat4 orthographic(float left, float right, float bottom, float top, float near_z, float far_z) {
        Mat4 m = identity();
        m.data[0] = 2.0F / (right - left);
        m.data[5] = 2.0F / (top - bottom);
        m.data[10] = -2.0F / (far_z - near_z);
        m.data[12] = -(right + left) / (right - left);
        m.data[13] = -(top + bottom) / (top - bottom);
        m.data[14] = -(far_z + near_z) / (far_z - near_z);
        return m;
    }

    void destroySurface() {
        if (display_ != EGL_NO_DISPLAY && surface_ != EGL_NO_SURFACE) {
            eglDestroySurface(display_, surface_);
            surface_ = EGL_NO_SURFACE;
        }
    }

    void destroy() {
        if (display_ != EGL_NO_DISPLAY) {
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (vbo_) {
                glDeleteBuffers(1, &vbo_);
                vbo_ = 0;
            }
            if (program_) {
                glDeleteProgram(program_);
                program_ = 0;
            }
            destroySurface();
            if (context_ != EGL_NO_CONTEXT) {
                eglDestroyContext(display_, context_);
                context_ = EGL_NO_CONTEXT;
            }
            eglTerminate(display_);
            display_ = EGL_NO_DISPLAY;
        }
    }

    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLConfig config_ = nullptr;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    GLuint program_ = 0;
    GLuint vbo_ = 0;
    GLint mvp_location_ = -1;
    int width_ = 0;
    int height_ = 0;
};

static GlStereoRenderer g_gl_stereo_renderer;
#endif

static std::vector<cv::Point3f> makeBoardObjectPoints(int cols, int rows, float square_size) {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(cols * rows));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            points.emplace_back(static_cast<float>(x) * square_size,
                                static_cast<float>(y) * square_size,
                                0.0F);
        }
    }
    return points;
}

static bool autoSampleIsDifferent(const std::vector<cv::Point2f>& previous,
                                  const std::vector<cv::Point2f>& current) {
    if (previous.size() != current.size() || current.empty()) return true;
    cv::Rect2f prev_box = cv::boundingRect(previous);
    cv::Rect2f curr_box = cv::boundingRect(current);
    const cv::Point2f prev_center = (prev_box.tl() + prev_box.br()) * 0.5F;
    const cv::Point2f curr_center = (curr_box.tl() + curr_box.br()) * 0.5F;
    const float center_shift = cv::norm(curr_center - prev_center);
    const float prev_scale = std::max(prev_box.width, prev_box.height);
    const float curr_scale = std::max(curr_box.width, curr_box.height);
    const float scale_change = prev_scale > 1.0F ? std::abs(curr_scale / prev_scale - 1.0F) : 1.0F;
    double mean_corner_shift = 0.0;
    for (std::size_t i = 0; i < previous.size(); ++i) {
        mean_corner_shift += cv::norm(current[i] - previous[i]);
    }
    mean_corner_shift /= static_cast<double>(previous.size());
    return center_shift > 40.0F || scale_change > 0.08F || mean_corner_shift > 18.0;
}

static StereoPose makeStereoPose(const CameraPose& left, const CameraPose& right) {
    StereoPose pose;
    pose.left = left;
    pose.right = right;
    pose.valid = left.valid && right.valid;
    if (!pose.valid) return pose;
    for (const auto& camera : {left,right}) {
        for (int axis=0; axis<3; ++axis) {
            if (!std::isfinite(camera.rvec[axis]) || !std::isfinite(camera.tvec[axis]) ||
                !std::isfinite(camera.x_axis[axis]) || !std::isfinite(camera.y_axis[axis]) ||
                !std::isfinite(camera.z_axis[axis])) {
                pose.valid=false;
                return pose;
            }
        }
    }

    cv::Mat left_r;
    cv::Mat right_r;
    cv::Rodrigues(left.rvec, left_r);
    cv::Rodrigues(right.rvec, right_r);
    cv::Mat left_center = -left_r.t() * cv::Mat(left.tvec);
    cv::Mat right_center = -right_r.t() * cv::Mat(right.tvec);
    pose.left_center = {left_center.at<double>(0), left_center.at<double>(1), left_center.at<double>(2)};
    pose.right_center = {right_center.at<double>(0), right_center.at<double>(1), right_center.at<double>(2)};
    const cv::Vec3d baseline = pose.right_center - pose.left_center;
    const double baseline_norm = cv::norm(baseline);
    if (baseline_norm > 1e-6) {
        const double z_component = std::abs(baseline[2]) / baseline_norm;
        pose.baseline_parallel_error_deg = std::asin(std::clamp(z_component, 0.0, 1.0)) * 180.0 / CV_PI;
    }
    pose.height_diff_mm = pose.left_center[1] - pose.right_center[1]; // Board Y points down.

    // The total orientation error comes from the camera-to-camera rotation.
    // Adjustment readouts use each camera's board-frame angles so a physical
    // heading change remains visible and directly maps to the adjustment.
    const cv::Mat relative_rotation = left_r * right_r.t(); // Right frame expressed in left frame.
    pose.orientation_error_deg = std::acos(std::clamp((cv::trace(relative_rotation)[0] - 1.0) * 0.5, -1.0, 1.0)) * 180.0 / CV_PI;
    pose.optical_error_deg = std::acos(std::clamp(left.z_axis.dot(right.z_axis), -1.0, 1.0)) * 180.0 / CV_PI;
    pose.reliable = left.calibrated && right.calibrated &&
        std::isfinite(left.rms_px) && std::isfinite(right.rms_px) &&
        left.rms_px >= 0.0 && right.rms_px >= 0.0 && left.rms_px <= 2.0 && right.rms_px <= 2.0;
    pose.pitch_diff_deg = std::remainder(pose.right.pitch_deg - pose.left.pitch_deg, 360.0);
    pose.yaw_diff_deg = std::remainder(pose.right.yaw_deg - pose.left.yaw_deg, 360.0);
    pose.roll_diff_deg = std::remainder(pose.right.roll_deg - pose.left.roll_deg, 360.0);
    return pose;
}

static void applyCalibratedStereoGeometry(StereoPose& pose,
                                           const cv::Mat& stereo_rotation,
                                           const cv::Mat& stereo_translation) {
    if (!pose.valid || stereo_rotation.rows != 3 || stereo_rotation.cols != 3 ||
        stereo_translation.total() != 3) {
        return;
    }
    cv::Mat left_rotation;
    cv::Rodrigues(pose.left.rvec, left_rotation);
    cv::Mat right_rotation = stereo_rotation * left_rotation;
    cv::Mat right_translation = stereo_rotation * cv::Mat(pose.left.tvec) + stereo_translation;
    cv::Vec3d right_rvec;
    cv::Rodrigues(right_rotation, right_rvec);

    CameraPose calibrated_right = pose.right;
    calibrated_right.calibrated = true;
    calibrated_right.rvec = right_rvec;
    calibrated_right.tvec = {right_translation.at<double>(0),
                             right_translation.at<double>(1),
                             right_translation.at<double>(2)};
    const cv::Mat right_camera_to_board = right_rotation.t();
    calibrated_right.x_axis = {right_camera_to_board.at<double>(0, 0),
                               right_camera_to_board.at<double>(1, 0),
                               right_camera_to_board.at<double>(2, 0)};
    calibrated_right.y_axis = {right_camera_to_board.at<double>(0, 1),
                               right_camera_to_board.at<double>(1, 1),
                               right_camera_to_board.at<double>(2, 1)};
    calibrated_right.z_axis = {right_camera_to_board.at<double>(0, 2),
                               right_camera_to_board.at<double>(1, 2),
                               right_camera_to_board.at<double>(2, 2)};
    calibrated_right.pitch_deg = opticalPitchDeg(calibrated_right.z_axis);
    calibrated_right.yaw_deg = opticalHeadingDeg(calibrated_right.z_axis);
    calibrated_right.roll_deg = imageRollDeg(calibrated_right.x_axis);

    const int stable_frames = pose.stable_frames;
    StereoPose corrected = makeStereoPose(pose.left, calibrated_right);
    corrected.stable_frames = stable_frames;
    pose = corrected;
}

static std::string buildAlignmentHint(const StereoGuide& guide) {
    if (!guide.valid) {
        return "hint=keep the chessboard fully visible in both cameras";
    }
    if (std::abs(guide.signed_roll_diff_deg) > 0.5) {
        if (guide.signed_roll_diff_deg > 0.0) {
            return "hint=left side is more clockwise; rotate the left camera CCW or the right camera CW";
        }
        return "hint=right side is more clockwise; rotate the right camera CCW or the left camera CW";
    }
    if (guide.stereo_y_diff > 2.0 || guide.avg_corner_y_diff > 2.0) {
        if (guide.signed_stereo_y_diff > 0.0) {
            return "hint=the left image sits lower; raise the left side or lower the right side";
        }
        if (guide.signed_stereo_y_diff < 0.0) {
            return "hint=the right image sits lower; raise the right side or lower the left side";
        }
        return "hint=match the two camera heights";
    }
    if (guide.board_to_screen_center_y > 90.0) {
        return "hint=move the chessboard near the screen center for the final check";
    }
    if (guide.horizontal_offset > 120.0) {
        return "hint=reduce left-right framing offset before collecting samples";
    }
    return "hint=level looks good";
}

static cv::Point2f boardPoint(const std::vector<cv::Point2f>& corners,
                              int cols,
                              int row,
                              int col) {
    return corners[static_cast<std::size_t>(row * cols + col)];
}

static GuideLine fittedLineSegment(const std::vector<cv::Point2f>& points, cv::Size image_size) {
    GuideLine segment;
    if (points.size() < 2 || image_size.width <= 0 || image_size.height <= 0) {
        return segment;
    }

    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
    const double vx = line[0];
    const double vy = line[1];
    const double x0 = line[2];
    const double y0 = line[3];
    if (std::abs(vx) >= std::abs(vy)) {
        const double y_left = y0 + (0.0 - x0) * vy / vx;
        const double y_right = y0 + (static_cast<double>(image_size.width - 1) - x0) * vy / vx;
        segment.a = cv::Point2f(0.0F, static_cast<float>(y_left));
        segment.b = cv::Point2f(static_cast<float>(image_size.width - 1), static_cast<float>(y_right));
    } else {
        const double x_top = x0 + (0.0 - y0) * vx / vy;
        const double x_bottom = x0 + (static_cast<double>(image_size.height - 1) - y0) * vx / vy;
        segment.a = cv::Point2f(static_cast<float>(x_top), 0.0F);
        segment.b = cv::Point2f(static_cast<float>(x_bottom), static_cast<float>(image_size.height - 1));
    }
    segment.valid = true;
    return segment;
}

static double lineAngleDeg(const GuideLine& line) {
    if (!line.valid) return 0.0;
    const double dx = line.b.x - line.a.x;
    const double dy = line.b.y - line.a.y;
    return std::atan2(dy, dx) * 180.0 / CV_PI;
}

static double lineYAtX(const GuideLine& line, double x) {
    if (!line.valid) return -1.0;
    const double dx = line.b.x - line.a.x;
    if (std::abs(dx) < 1e-6) return -1.0;
    const double t = (x - line.a.x) / dx;
    return line.a.y + t * (line.b.y - line.a.y);
}

static BoardGuide makeBoardGuide(const std::vector<cv::Point2f>& corners,
                                 int cols,
                                 int rows,
                                 cv::Size image_size) {
    BoardGuide guide;
    if (cols <= 1 || rows <= 1 || corners.size() != static_cast<std::size_t>(cols * rows)) {
        return guide;
    }

    const int row_indices[3] = {0, rows / 2, rows - 1};
    for (int i = 0; i < 3; ++i) {
        std::vector<cv::Point2f> row_points;
        row_points.reserve(static_cast<std::size_t>(cols));
        for (int col = 0; col < cols; ++col) {
            row_points.push_back(boardPoint(corners, cols, row_indices[i], col));
        }
        guide.horizontal[i] = fittedLineSegment(row_points, image_size);
        guide.horizontal_angle_deg[i] = lineAngleDeg(guide.horizontal[i]);
    }

    const int mid_col = cols / 2;
    std::vector<cv::Point2f> col_points;
    col_points.reserve(static_cast<std::size_t>(rows));
    for (int row = 0; row < rows; ++row) {
        col_points.push_back(boardPoint(corners, cols, row, mid_col));
    }
    guide.vertical = fittedLineSegment(col_points, image_size);

    guide.valid = guide.horizontal[0].valid && guide.horizontal[1].valid &&
                  guide.horizontal[2].valid && guide.vertical.valid;
    return guide;
}

static cv::Size queryScreenSize() {
#ifdef HAVE_X11
    std::unique_ptr<Display, decltype(&XCloseDisplay)> display(XOpenDisplay(nullptr), &XCloseDisplay);
    if (!display) {
        return {};
    }
    const int screen = DefaultScreen(display.get());
    return {DisplayWidth(display.get(), screen), DisplayHeight(display.get(), screen)};
#else
    return {};
#endif
}

static void drawAlignmentGuides(cv::Mat& left,
                                cv::Mat& right,
                                const StereoGuide& guide,
                                bool align_ok) {
    const cv::Scalar screen_h_color(0, 255, 255);         // 高亮黄：固定水平基准线
    const cv::Scalar board_parallel_color(0, 255, 0);     // 高亮绿：棋盘横向拟合线
    const cv::Scalar center_color = align_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 80, 255);
    const cv::Scalar shadow_color(0, 0, 0);

    auto drawScreenLine = [](cv::Mat& image, double y, const cv::Scalar& color, int thickness, bool dashed) {
        if (image.empty()) return;
        const int yy = std::clamp(static_cast<int>(std::round(y)), 0, image.rows - 1);
        if (!dashed) {
            cv::line(image, cv::Point(0, yy + 1), cv::Point(image.cols - 1, yy + 1), cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
            cv::line(image, cv::Point(0, yy), cv::Point(image.cols - 1, yy), color, thickness, cv::LINE_AA);
            return;
        }
        constexpr int dash = 28;
        constexpr int gap = 16;
        for (int x = 0; x < image.cols; x += dash + gap) {
            cv::line(image,
                     cv::Point(x, yy + 1),
                     cv::Point(std::min(x + dash, image.cols - 1), yy + 1),
                     cv::Scalar(0, 0, 0),
                     thickness + 2,
                     cv::LINE_AA);
            cv::line(image,
                     cv::Point(x, yy),
                     cv::Point(std::min(x + dash, image.cols - 1), yy),
                     color,
                     thickness,
                     cv::LINE_AA);
        }
    };

    auto drawSegment = [](cv::Mat& image, const GuideLine& line, const cv::Scalar& color, int thickness) {
        if (!line.valid) return;
        cv::line(image, line.a + cv::Point2f(1.0F, 1.0F), line.b + cv::Point2f(1.0F, 1.0F),
                 cv::Scalar(0, 0, 0), thickness + 2, cv::LINE_AA);
        cv::line(image, line.a, line.b, color, thickness, cv::LINE_AA);
    };

    auto drawCrosshair = [&](cv::Mat& image, cv::Point2f point, const cv::Scalar& color) {
        const cv::Point p(static_cast<int>(std::round(point.x)), static_cast<int>(std::round(point.y)));
        cv::line(image, cv::Point(p.x - 24, p.y + 1), cv::Point(p.x + 24, p.y + 1), shadow_color, 5, cv::LINE_AA);
        cv::line(image, cv::Point(p.x + 1, p.y - 24), cv::Point(p.x + 1, p.y + 24), shadow_color, 5, cv::LINE_AA);
        cv::line(image, cv::Point(p.x - 24, p.y), cv::Point(p.x + 24, p.y), color, 3, cv::LINE_AA);
        cv::line(image, cv::Point(p.x, p.y - 24), cv::Point(p.x, p.y + 24), color, 3, cv::LINE_AA);
        cv::circle(image, p, 8, shadow_color, cv::FILLED, cv::LINE_AA);
        cv::circle(image, p, 5, color, cv::FILLED, cv::LINE_AA);
    };

    auto drawReferenceSet = [&](cv::Mat& image) {
        if (image.empty()) return;
        drawScreenLine(image, image.rows * 0.50, screen_h_color, 4, false);
    };

    drawReferenceSet(left);
    drawReferenceSet(right);

    if (!guide.valid) {
        return;
    }

    if (guide.centers_valid) {
        drawSegment(left, guide.left.horizontal[1], board_parallel_color, 5);
        drawSegment(right, guide.right.horizontal[1], board_parallel_color, 5);
        drawCrosshair(left, guide.left_center, center_color);
        drawCrosshair(right, guide.right_center, center_color);

    }
}

static std::string formatDeg(const char* label, double value) {
    std::ostringstream ss;
    ss << label << ": " << std::fixed << std::setprecision(1) << value << " deg";
    return ss.str();
}

static std::string formatDegCn(const char* label, double value) {
    std::ostringstream ss;
    ss << label << "：" << std::fixed << std::setprecision(1) << value << "°";
    return ss.str();
}

static void putPanelText(cv::Mat& image,
                         const std::string& text,
                         cv::Point origin,
                         double scale,
                         const cv::Scalar& color,
                         int thickness = 1) {
    g_text_renderer.put(image, text, origin, scale, color, thickness);
}

static const char* viewModeName(int view_mode) {
    const int mode = ((view_mode % 4) + 4) % 4;
    if (g_text_renderer.available()) {
        switch (mode) {
            case 0: return "俯视图";
            case 1: return "侧视图";
            case 2: return "正视图";
            default: return "立体图";
        }
    }
    switch (mode) {
        case 0: return "Top View";
        case 1: return "Side View";
        case 2: return "Front View";
        default: return "3D View";
    }
}

static cv::Point2d projectWorldPoint(const cv::Vec3d& point, int view_mode) {
    switch (((view_mode % 4) + 4) % 4) {
        case 0: // Top: along board Y, screen X/Z.
            return {point[0], -point[2]};
        case 1: // Side: along board X, screen Z/Y.
            return {point[2], -point[1]};
        case 2: // Front: normal to board, screen X/Y.
            return {point[0], -point[1]};
        default:
            // The board is Z=0 and the cameras are on the negative-Z side.
            // Negating Z keeps cameras visually above the board in the 3D view.
            const cv::Vec3d rotated=sceneRotation()*cv::Vec3d(point[0],point[1],-point[2]);
            return {rotated[0],rotated[1]};
    }
}

static bool isSpace3DView(int view_mode) {
    return (((view_mode % 4) + 4) % 4) == 3;
}

static std::string formatMm(const char* label, double value) {
    std::ostringstream ss;
    ss << label << ": " << std::fixed << std::setprecision(1) << value << " mm";
    return ss.str();
}

static std::string formatMmCn(const char* label, double value) {
    std::ostringstream ss;
    ss << label << "：" << std::fixed << std::setprecision(1) << value << " mm";
    return ss.str();
}

static std::string formatSignedDeg(const char* label, double value) {
    std::ostringstream ss;
    ss << label << ": " << (value >= 0.0 ? "+" : "") << std::fixed << std::setprecision(2) << value << "°";
    return ss.str();
}

static std::string formatSignedDegCn(const char* label, double value) {
    std::ostringstream ss;
    ss << label << "：" << (value >= 0.0 ? "+" : "") << std::fixed << std::setprecision(2) << value << "°";
    return ss.str();
}

static std::string formatAngleDegCn(const char* label, double value) {
    std::ostringstream ss;
    ss << label << "：" << (value >= 0.0 ? "+" : "") << std::fixed << std::setprecision(2) << value << "°";
    return ss.str();
}

static std::string formatAngleDeg(const char* label, double value) {
    std::ostringstream ss;
    ss << label << ": " << (value >= 0.0 ? "+" : "") << std::fixed << std::setprecision(2) << value << "°";
    return ss.str();
}

static cv::Point worldToPanel(const cv::Vec3d& point,
                              int view_mode,
                              const cv::Point2d& center,
                              double scale,
                              const cv::Point& panel_center);

static void drawBoardAxis(cv::Mat& panel,
                          const cv::Vec3d& direction,
                          const std::string& label,
                          const cv::Scalar& color,
                          int view_mode,
                          const cv::Point2d& projected_center,
                          double scale,
                          const cv::Point& panel_center,
                          double axis_length,
                          double ui) {
    const cv::Vec3d origin(0.0, 0.0, 0.0);
    const cv::Point p1 = worldToPanel(origin, view_mode, projected_center, scale, panel_center);
    const cv::Point p2 = worldToPanel(direction * axis_length, view_mode, projected_center, scale, panel_center);
    cv::arrowedLine(panel, p1, p2, color, std::max(2, static_cast<int>(4 * ui)), cv::LINE_AA, 0, 0.22);
    putPanelText(panel, label, p2 + cv::Point(static_cast<int>(8 * ui), static_cast<int>(-8 * ui)),
                 0.50 * ui, color, 2);
}

static cv::Point worldToPanel(const cv::Vec3d& point,
                              int view_mode,
                              const cv::Point2d& center,
                              double scale,
                              const cv::Point& panel_center) {
    const cv::Point2d p = projectWorldPoint(point, view_mode);
    return {panel_center.x + static_cast<int>(std::round((p.x - center.x) * scale)),
            panel_center.y - static_cast<int>(std::round((p.y - center.y) * scale))};
}

static void drawCameraAxes(cv::Mat& panel,
                           const cv::Vec3d& camera_center,
                           const CameraPose& camera,
                           int view_mode,
                           const cv::Point2d& projected_center,
                           double scale,
                           const cv::Point& panel_center,
                           double axis_length,
                           double ui) {
    if (!camera.valid) return;
    auto drawAxis = [&](const cv::Vec3d& axis,
                        const cv::Scalar& color,
                        const std::string& label) {
        const cv::Vec3d end = camera_center + axis * axis_length;
        const cv::Point p1 = worldToPanel(camera_center, view_mode, projected_center, scale, panel_center);
        const cv::Point p2 = worldToPanel(end, view_mode, projected_center, scale, panel_center);
        cv::arrowedLine(panel, p1, p2, color, std::max(2, static_cast<int>(4 * ui)), cv::LINE_AA, 0, 0.22);
        putPanelText(panel, label, p2 + cv::Point(static_cast<int>(8 * ui), static_cast<int>(-8 * ui)),
                     0.52 * ui, color, 2);
    };
    const cv::Point origin = worldToPanel(camera_center, view_mode, projected_center, scale, panel_center);
    cv::circle(panel, origin, std::max(4, static_cast<int>(7 * ui)), cv::Scalar(240, 240, 240), cv::FILLED, cv::LINE_AA);
    drawAxis(camera.x_axis, cv::Scalar(40, 80, 255), "X");
    drawAxis(camera.y_axis, cv::Scalar(60, 220, 80), "Y");
    drawAxis(camera.z_axis, cv::Scalar(255, 120, 40), "Z");
    putPanelText(panel,
                 g_text_renderer.available() ? "相机坐标" : "Camera Frame",
                 origin + cv::Point(static_cast<int>(10 * ui), static_cast<int>(-12 * ui)),
                 0.48 * ui,
                 cv::Scalar(235, 235, 235),
                 2);
}


// Board coordinates: X right, Y down on an upright board, Z its normal.
// These are geometric references; gravity is known only if the board is levelled.
static bool withinPhysicalTolerance(const StereoPose& pose) {
    for (int axis = 0; axis < 3; ++axis)
        if (!std::isfinite(pose.left_center[axis]) || !std::isfinite(pose.right_center[axis])) return false;
    return pose.valid && pose.reliable && pose.frame_pair_valid && pose.raw_resolution_valid &&
           std::isfinite(pose.orientation_error_deg) && pose.orientation_error_deg >= 0.0 &&
           std::abs(pose.height_diff_mm) <= 2.0 &&
           std::abs(pose.right_center[2] - pose.left_center[2]) <= 2.0 &&
           pose.orientation_error_deg <= 0.5;
}

static bool withinLevelTolerance(const StereoPose& pose) {
    const bool physical = withinPhysicalTolerance(pose);
    return pose.require_raw_alignment
        ? jointFisheyeLevelPasses(physical && pose.raw_resolution_valid, pose.raw_alignment)
        : physical;
}

// Never substitute a cached stereo model for the current measured centers.
static double liveBaselineMm(const StereoPose& pose) {
    if (!pose.valid || !pose.reliable || !pose.frame_pair_valid || !pose.raw_resolution_valid)
        return -1.0;
    for (int axis = 0; axis < 3; ++axis)
        if (!std::isfinite(pose.left_center[axis]) || !std::isfinite(pose.right_center[axis]))
            return -1.0;
    const double baseline = cv::norm(pose.right_center - pose.left_center);
    return std::isfinite(baseline) ? baseline : -1.0;
}

static void drawRelativeSpacePanel(cv::Mat& panel, const StereoPose& pose,
                                   int view_mode, cv::Size2f board_world_size) {
    panel.setTo(cv::Scalar(28, 32, 34));
    const int mode = ((view_mode % 4) + 4) % 4;
    const double ui = std::max(0.35, panel.rows / 720.0);
    const int header = static_cast<int>(110 * ui);
    const int footer = static_cast<int>(150 * ui);
    const cv::Rect area(0, header, panel.cols, std::max(1, panel.rows - header - footer));
    cv::Mat scene = panel(area);
    const double w = std::max(1.0F, board_world_size.width);
    const double h = std::max(1.0F, board_world_size.height);
    bool rendered = false;
#ifdef HAVE_EGL_OPENGL_VIEW
    if (mode == 3) rendered = g_gl_stereo_renderer.render(scene, pose, board_world_size, g_orbit_view);
#endif
    if (!rendered) {
        const cv::Vec3d board_center(w/2, h/2, 0);
        const cv::Vec3d centers[] = {
            pose.valid ? pose.left_center : cv::Vec3d(0, h/2, -w*2),
            pose.valid ? pose.right_center : cv::Vec3d(w, h/2, -w*2)
        };
        const double length = std::max(w,h)*0.35;
        struct Edge { cv::Vec3d a, b; cv::Scalar color; };
        std::vector<Edge> edges;
        auto edge = [&](cv::Vec3d a, cv::Vec3d b, cv::Scalar color) { edges.push_back({a,b,color}); };
        const cv::Scalar board_color(160,200,160), link_color(80,180,230);
        // The same board geometry is projected in every view, including edge-on views.
        for (int i=0;i<=8;++i) edge({w*i/8,0,0},{w*i/8,h,0},board_color);
        for (int i=0;i<=6;++i) edge({0,h*i/6,0},{w,h*i/6,0},board_color);
        edge(centers[0],centers[1],cv::Scalar(255,210,110));
        const cv::Scalar colors[] = {cv::Scalar(50,80,255),cv::Scalar(60,220,60),cv::Scalar(255,120,40)};
        for (int c=0;c<2;++c) {
            edge(centers[c],board_center,link_color); // Target link, not an optical axis.
            const CameraPose& camera = c==0 ? pose.left : pose.right;
            const cv::Vec3d axes[] = {camera.x_axis,camera.y_axis,camera.z_axis};
            if (!pose.valid) continue;
            for (int a=0;a<3;++a) edge(centers[c],centers[c]+axes[a]*length,colors[a]);
            const cv::Vec3d plane = centers[c]+camera.z_axis*length*0.6;
            cv::Vec3d corners[4];
            for(int i=0;i<4;++i) {
                corners[i]=plane+camera.x_axis*(length*0.20*(i<2?1:-1))+
                    camera.y_axis*(length*0.15*((i==0||i==3)?1:-1));
            }
            for(int i=0;i<4;++i) {
                edge(centers[c],corners[i],c==0?cv::Scalar(50,160,255):cv::Scalar(255,190,50));
                edge(corners[i],corners[(i+1)%4],c==0?cv::Scalar(50,160,255):cv::Scalar(255,190,50));
            }
        }
        if(g_orbit_view.show_axes) {
            edge({0,0,0},{length,0,0},colors[0]);
            edge({0,0,0},{0,length,0},colors[1]);
            edge({0,0,0},{0,0,length},colors[2]);
        }
        if (g_orbit_view.show_reference && pose.valid) {
            // A true board-horizontal reference through the left camera.
            edge(centers[0], {centers[1][0],centers[0][1],centers[0][2]}, cv::Scalar(120,160,120));
        }
        double minx=1e30,maxx=-1e30,miny=1e30,maxy=-1e30;
        for(const auto& e:edges) for(const auto& p:{e.a,e.b}) {
            const auto q=projectWorldPoint(p,mode);
            minx=std::min(minx,q.x);maxx=std::max(maxx,q.x);
            miny=std::min(miny,q.y);maxy=std::max(maxy,q.y);
        }
        const double scale=std::min(scene.cols*0.78/std::max(1.0,maxx-minx),
                                    scene.rows*0.72/std::max(1.0,maxy-miny)) /
                                    (mode==3?g_orbit_view.zoom:1.0);
        auto pixel = [&](const cv::Vec3d& p) {
            const auto q=projectWorldPoint(p,mode);
            return cv::Point(cvRound(scene.cols*(0.5+(mode==3?g_orbit_view.pan_x:0))+(q.x-(minx+maxx)*0.5)*scale),
                             cvRound(scene.rows*(0.5+(mode==3?g_orbit_view.pan_y:0))-(q.y-(miny+maxy)*0.5)*scale));
        };
        for(const auto& e:edges) cv::line(scene,pixel(e.a),pixel(e.b),e.color,std::max(1,cvRound(ui)),cv::LINE_AA);
        for(int c=0;c<2;++c) {
            const cv::Point p=pixel(centers[c]);
            cv::circle(scene,p,std::max(2,cvRound(4*ui)),cv::Scalar(250,250,250),cv::FILLED,cv::LINE_AA);
            putPanelText(scene,g_text_renderer.available()?(c==0?"左":"右"):(c==0?"L":"R"),p+cv::Point(cvRound((c==0?-22:10)*ui),cvRound(-8*ui)),0.6*ui,cv::Scalar(255,255,255),1);
            if(pose.valid) {
                const CameraPose& camera=c==0?pose.left:pose.right;
                const cv::Vec3d axes[]={camera.x_axis,camera.y_axis,camera.z_axis};
                for(int a=0;a<3;++a) putPanelText(scene,std::string(1,"XYZ"[a]),pixel(centers[c]+axes[a]*length)+cv::Point(4,-4),0.45*ui,colors[a],1);
            }
        }
    }
    const char* axes_cn[]={"俯视：X / Z（宽度 / 深度）", "侧视：Z / Y（深度 / 高度）", "正视：X / Y（标定板平面）", "立体：J/L旋转 I/K俯仰 B/N滚转 U/O缩放 0复位"};
    const char* axes_en[]={"Top: X / Z (width / depth)", "Side: Z / Y (depth / height)", "Front: X / Y (board plane)", "3D: J/L yaw I/K pitch B/N roll U/O zoom 0 reset"};
    putPanelText(panel,viewModeName(mode),{cvRound(20*ui),cvRound(32*ui)},0.85*ui,{255,255,255},2);
    putPanelText(panel,g_text_renderer.available()?axes_cn[mode]:axes_en[mode],{cvRound(20*ui),cvRound(62*ui)},0.5*ui,{220,220,220},1);
    putPanelText(panel,g_text_renderer.available()?"标定板坐标：X向右、Y向下、Z为法向 | V切换 | 黄色线指向标定板中心":
                 "Board: X right, Y down, Z normal | V: switch | target links point to board center",
                 {cvRound(20*ui),cvRound(90*ui)},0.38*ui,{170,190,190},1);
    int y=panel.rows-footer+cvRound(28*ui);
    auto row=[&](const std::string& text,cv::Scalar color) {
        putPanelText(panel,text,{cvRound(20*ui),y},0.48*ui,color,1);y+=cvRound(28*ui);
    };
    if(!pose.valid) {
        row(g_text_renderer.available()?"没有有效实时位姿：请让两幅画面同时看到棋盘":
            "未检测到双相机位姿：请让两幅画面同时看到棋盘",{0,180,255});
        return;
    }
    std::ostringstream ss;
    ss<<std::fixed<<std::setprecision(2)<<"高度差（-Y）："<<pose.height_diff_mm
      <<" mm | 深度差（Z）："<<pose.right_center[2]-pose.left_center[2]<<" mm";
    row(ss.str(),{240,240,240});
    ss.str("");ss<<"姿态总差："<<pose.orientation_error_deg<<"° | 光轴夹角："<<pose.optical_error_deg<<"°";
    row(ss.str(),{240,240,240});
    ss.str("");ss<<"PnP重投影误差 左/右："<<pose.left.rms_px<<" / "<<pose.right.rms_px<<" 像素 | 稳定帧："<<pose.stable_frames<<"/10";
    row(ss.str(),{210,210,210});
    const bool pass=withinLevelTolerance(pose)&&pose.stable_frames>=10;
    row(!pose.reliable?"仅供参考：正在自动获取物理尺度或棋盘检测误差过大":
        pass?(pose.require_raw_alignment?"联合合格：物理姿态及当前棋盘原图纵差均达标":"连续稳定且误差合格：可作为调平依据"):
             (pose.require_raw_alignment?"联合验收未通过：还需所有原图角点纵差≤2 px，连续10帧":"需要调整：高度/深度≤2 mm，姿态≤0.5°，需连续10帧"),
        pass?cv::Scalar(80,255,100):cv::Scalar(0,190,255));
}
static void addSuggestion(std::vector<std::string>& lines,
                          const std::string& camera,
                          double value,
                          double threshold,
                          const std::string& positive_action,
                          const std::string& negative_action) {
    if (std::abs(value) <= threshold) return;
    std::ostringstream ss;
    ss << camera << (g_text_renderer.available() ? "：" : ": ")
       << (value > 0.0 ? positive_action : negative_action)
       << " " << std::fixed << std::setprecision(1) << std::abs(value)
       << (g_text_renderer.available() ? "°" : " deg");
    lines.push_back(ss.str());
}

static void drawTextPanel(cv::Mat& panel,
                          const StereoPose& pose,
                          const std::string& status_top,
                          const std::string& status_sub,
                          int view_mode) {
    (void)status_sub;
    panel.setTo(cv::Scalar(24, 26, 30));
    const bool cn = g_text_renderer.available();
    // The panel is an operator readout, so use most of its height for values
    // while keeping a compact footer for the current view and controls.
    const double ui = std::clamp(std::min(std::sqrt(panel.rows / 720.0) * 2.55,
                                          panel.rows / 400.0), 1.55, 3.55);
    putPanelText(panel, cn ? "数值与调平" : "Values and Leveling", cv::Point(44, static_cast<int>(42 * ui)), 1.18 * ui, cv::Scalar(255, 255, 255), 3);
    putPanelText(panel, cn ? "参考：右相机相对左相机" : "Reference: right camera relative to left", cv::Point(46, static_cast<int>(76 * ui)), 0.62 * ui, cv::Scalar(210, 220, 220), 2);
    const double live_baseline = liveBaselineMm(pose);
    if (live_baseline >= 0.0) {
        const cv::Vec3d baseline = pose.right_center - pose.left_center;
        std::ostringstream reference;
        const int reference_limit = panel.cols - 72;
        std::ostringstream spacing;
        spacing << (cn ? "实时眼距基线（估计）：" : "Live baseline (estimated): ")
                << std::fixed << std::setprecision(1) << live_baseline << " mm";
        double spacing_scale = 0.78 * ui;
        const int spacing_width = g_text_renderer.textSize(spacing.str(), spacing_scale, 3).width;
        if (spacing_width > reference_limit) spacing_scale *= static_cast<double>(reference_limit) / spacing_width;
        putPanelText(panel, spacing.str(), cv::Point(46, static_cast<int>(111 * ui)), spacing_scale, cv::Scalar(100, 240, 255), 3);
        if (pose.reliable) {
            std::ostringstream reference;
            reference << (cn ? "实时估计中心差：" : "Live estimated center delta: ")
                      << std::fixed << std::setprecision(1)
                      << "X=" << baseline[0] << "  Y=" << baseline[1] << "  Z=" << baseline[2] << " mm";
            double components_scale = 0.48 * ui;
            const int components_width = g_text_renderer.textSize(reference.str(), components_scale, 2).width;
            if (components_width > reference_limit) components_scale *= static_cast<double>(reference_limit) / components_width;
            putPanelText(panel, reference.str(), cv::Point(46, static_cast<int>(139 * ui)), components_scale, cv::Scalar(100, 220, 240), 2);
        }
    } else {
        const std::string pending = cn ? "实时眼距基线：--（等待有效画面及姿态）"
                                       : "Live baseline: -- (waiting for valid frames and poses)";
        double pending_scale = 0.68 * ui;
        const int pending_width = g_text_renderer.textSize(pending, pending_scale, 3).width;
        if (pending_width > panel.cols - 72)
            pending_scale *= static_cast<double>(panel.cols - 72) / pending_width;
        putPanelText(panel, pending, cv::Point(46, static_cast<int>(111 * ui)), pending_scale, cv::Scalar(0, 190, 255), 3);
    }

    const int left_x = 46;
    const int right_x = panel.cols / 2 + 34;
    const int content_top = static_cast<int>(175 * ui);
    const int content_bottom = panel.rows - static_cast<int>(54 * ui);
    const int line_step = std::max(38, std::min(static_cast<int>(29 * ui),
                                                (content_bottom - content_top) / 10));
    auto writeAt = [&](int x, int& y, const std::string& text,
                       const cv::Scalar& color = cv::Scalar(230, 230, 230),
                       double scale = 0.64,
                       int thickness = 2) {
        double actual_scale = scale * ui;
        const int available_width = std::max(120, (x < right_x ? panel.cols / 2 : panel.cols) - x - 28);
        const int measured_width = g_text_renderer.textSize(text, actual_scale, thickness).width;
        if (measured_width > available_width) {
            actual_scale *= static_cast<double>(available_width) / measured_width;
        }
        putPanelText(panel, text, cv::Point(x, y), actual_scale, color, thickness);
        y += line_step;
    };

    if (!pose.valid) {
        int y = content_top;
        writeAt(left_x, y, !pose.frame_pair_valid
            ? (cn ? "视频过期或左右取帧时间差过大，禁止判定合格" : "Stale/skewed frame pair: PASS disabled")
            : !pose.raw_resolution_valid
            ? (cn ? "原始帧尺寸与标定尺寸不一致，禁止判定合格" : "Raw/calibration resolution mismatch: PASS disabled")
            : (cn ? "状态：等待两个相机同时检测到棋盘" : "Status: Waiting for both chessboards"), cv::Scalar(0, 190, 255), 0.68, 3);
        writeAt(left_x, y, cn ? "请让棋盘完整出现在两个画面中" : "Keep the board fully visible in both cameras", cv::Scalar(230, 230, 230), 0.62, 2);
        int view_y = panel.rows - static_cast<int>(54 * ui);
        std::ostringstream view_ss;
        view_ss << (cn ? "视角：" : "View: ") << viewModeName(view_mode) << (cn ? "  按 V 切换四视图" : "  (V switches 4 views)");
        writeAt(right_x, view_y, view_ss.str(), cv::Scalar(210, 220, 220), 0.36, 1);
        return;
    }

    int left_y = content_top;
    writeAt(left_x, left_y, cn ? "左相机" : "Left Camera", cv::Scalar(120, 210, 255), 0.78, 3);
    writeAt(left_x, left_y, cn ? formatAngleDegCn("俯仰角", pose.left.pitch_deg) : formatAngleDeg("Pitch", pose.left.pitch_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    writeAt(left_x, left_y, cn ? formatAngleDegCn("航向角", pose.left.yaw_deg) : formatAngleDeg("Yaw", pose.left.yaw_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    writeAt(left_x, left_y, cn ? formatAngleDegCn("横滚角", pose.left.roll_deg) : formatAngleDeg("Roll", pose.left.roll_deg), cv::Scalar(240, 240, 240), 0.62, 2);

    int right_y = content_top;
    writeAt(right_x, right_y, cn ? "右相机" : "Right Camera", cv::Scalar(120, 210, 255), 0.78, 3);
    writeAt(right_x, right_y, cn ? formatAngleDegCn("俯仰角", pose.right.pitch_deg) : formatAngleDeg("Pitch", pose.right.pitch_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    writeAt(right_x, right_y, cn ? formatAngleDegCn("航向角", pose.right.yaw_deg) : formatAngleDeg("Yaw", pose.right.yaw_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    writeAt(right_x, right_y, cn ? formatAngleDegCn("横滚角", pose.right.roll_deg) : formatAngleDeg("Roll", pose.right.roll_deg), cv::Scalar(240, 240, 240), 0.62, 2);

    int baseline_y = std::max(left_y, right_y) + static_cast<int>(12 * ui);
    std::ostringstream ss;
    writeAt(left_x, baseline_y, cn ? "差值" : "Difference", cv::Scalar(120, 210, 255), 0.78, 3);
    writeAt(left_x, baseline_y, cn ? formatSignedDegCn("俯仰差", pose.pitch_diff_deg) : formatSignedDeg("Pitch diff", pose.pitch_diff_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    writeAt(left_x, baseline_y, cn ? formatSignedDegCn("航向差", pose.yaw_diff_deg) : formatSignedDeg("Yaw diff", pose.yaw_diff_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    writeAt(left_x, baseline_y, cn ? formatSignedDegCn("横滚差", pose.roll_diff_deg) : formatSignedDeg("Roll diff", pose.roll_diff_deg), cv::Scalar(240, 240, 240), 0.62, 2);
    ss.str("");
    ss << (cn ? "高度差：" : "Height Diff: ")
       << (pose.height_diff_mm >= 0.0 ? "+" : "")
       << std::fixed << std::setprecision(1) << pose.height_diff_mm << " mm";
    writeAt(left_x, baseline_y, ss.str(), cv::Scalar(240, 240, 240), 0.62, 2);
    ss.str("");
    ss << (cn ? "深度差（右-左）：" : "Depth Diff (R-L): ")
       << (pose.right_center[2] - pose.left_center[2] >= 0.0 ? "+" : "")
       << std::fixed << std::setprecision(1)
       << (pose.right_center[2] - pose.left_center[2]) << " mm";
    writeAt(left_x, baseline_y, ss.str(), cv::Scalar(240, 240, 240), 0.62, 2);
    ss.str("");
    ss << (cn ? "基线方向误差：" : "Baseline direction error: ") << std::fixed << std::setprecision(2) << pose.baseline_parallel_error_deg << (cn ? "°" : " deg");
    writeAt(left_x, baseline_y, ss.str(), cv::Scalar(240, 240, 240), 0.62, 2);

    int suggestion_y = std::max(left_y, right_y) + static_cast<int>(12 * ui);

    std::vector<std::string> suggestions;
    addSuggestion(suggestions, "右相机俯仰", pose.pitch_diff_deg, 0.25, "向上微调", "向下微调");
    addSuggestion(suggestions, "右相机航向", pose.yaw_diff_deg, 0.25, "向左微调", "向右微调");
    addSuggestion(suggestions, "右相机横滚", pose.roll_diff_deg, 0.25, "逆时针微调", "顺时针微调");
    writeAt(right_x, suggestion_y, cn ? "建议" : "Suggestion", cv::Scalar(120, 210, 255), 0.78, 3);
    const bool qualified = withinLevelTolerance(pose) && pose.stable_frames >= 10;
    const bool physical_only = withinPhysicalTolerance(pose);
    writeAt(right_x, suggestion_y,
            qualified ? (pose.require_raw_alignment
                         ? (cn ? "物理及当前棋盘原图纵差：合格" : "Physical + current board raw dy: PASS")
                         : (cn ? "平行且同高：合格" : "Parallel and level: PASS"))
                      : pose.require_raw_alignment && physical_only && !rawAlignmentPasses(pose.raw_alignment)
                         ? (cn ? "物理达标，原图纵差未通过" : "Physical OK; raw vertical error: FAIL")
                      : withinLevelTolerance(pose)
                         ? (cn ? "条件达标，等待连续10帧确认" : "In tolerance; confirming 10 frames")
                      : (cn ? "状态：需要调整" : "Status: ADJUST"),
            qualified ? cv::Scalar(80, 255, 100) : cv::Scalar(0, 220, 255), 0.68, 3);
    if (pose.require_raw_alignment) {
        if (pose.raw_alignment.valid) {
            ss.str("");
            ss << (cn ? "原图纵差 最大/平均：" : "Raw |dy| max/mean: ") << std::fixed << std::setprecision(2)
               << pose.raw_alignment.max_abs_y_px << " / " << pose.raw_alignment.mean_abs_y_px << " px";
            writeAt(right_x, suggestion_y, ss.str(), cv::Scalar(0, 220, 255), 0.50, 2);
        } else writeAt(right_x, suggestion_y, cn ? "原图纵差无有效测量，不能合格" : "Raw vertical measurement invalid: cannot PASS", cv::Scalar(0, 220, 255), 0.50, 2);
    }
    if (qualified) {
        writeAt(right_x, suggestion_y, cn ? "全部误差已进入容差" : "All errors are within tolerance",
                cv::Scalar(120, 255, 120), 0.58, 2);
    } else if (pose.require_raw_alignment && physical_only) {
        if (!rawAlignmentPasses(pose.raw_alignment)) {
            ss.str("");
            ss << (cn ? "左右主点差 cy：" : "Principal point cy diff: ") << std::fixed << std::setprecision(2) << pose.cy_diff_px << " px";
            writeAt(right_x, suggestion_y, ss.str(), cv::Scalar(0, 220, 255), 0.50, 2);
            writeAt(right_x, suggestion_y, cn ? "机械调整可能无法同时达标；检查内参或做立体校正" : "Joint limits may be unreachable mechanically; check intrinsics/rectify", cv::Scalar(0, 220, 255), 0.42, 2);
        }
    } else {
        if (!pose.reliable) {
            writeAt(right_x, suggestion_y, cn ? "标定/重投影误差过大，当前结果仅供参考" : "Calibration/reprojection quality is insufficient", cv::Scalar(0, 170, 255), 0.46, 2);
        }
        if (std::abs(pose.height_diff_mm) > 2.0) {
            ss.str("");
            ss << (pose.height_diff_mm > 0.0 ? (cn ? "右相机偏高，向下调" : "Right camera is high, move down" ) :
                                               (cn ? "右相机偏低，向上调" : "Right camera is low, move up"))
               << " " << std::fixed << std::setprecision(1) << std::abs(pose.height_diff_mm) << " mm";
            suggestions.insert(suggestions.begin(), ss.str());
        }
        const double depth_diff = pose.right_center[2] - pose.left_center[2];
        if (std::abs(depth_diff) > 2.0) {
            ss.str("");
            ss << (depth_diff > 0.0 ? "右相机更靠近棋盘，向后调" : "右相机离棋盘更远，向前调")
               << " " << std::fixed << std::setprecision(1) << std::abs(depth_diff) << " mm";
            suggestions.insert(suggestions.begin(), ss.str());
        }
        int shown = 0;
        for (const auto& suggestion : suggestions) {
            if (shown >= 4 || suggestion_y > panel.rows - static_cast<int>(44 * ui)) break;
            writeAt(right_x, suggestion_y, suggestion, cv::Scalar(0, 220, 255), 0.58, 2);
            ++shown;
        }
    }

    int footer_y = panel.rows - static_cast<int>(40 * ui);
    std::ostringstream footer;
    footer << status_top << "  |  " << (cn ? "视角：" : "View: ") << viewModeName(view_mode) << (cn ? "  按 V 切换四视图" : "  (V switches 4 views)");
    if (view_mode == 3) {
        footer << (cn ? "  J/L水平 I/K俯仰 U/O缩放 0复位" : "  J/L yaw I/K pitch U/O zoom 0 reset");
    }
    putPanelText(panel, footer.str(), cv::Point(left_x, footer_y), 0.28 * ui, cv::Scalar(210, 220, 220), 1);
}

static cv::Mat makeDashboard(const cv::Mat& left,
                             const cv::Mat& right,
                             const StereoPose& pose,
                             const std::string& status_top,
                             const std::string& status_sub,
                             int view_mode,
                             cv::Size2f board_world_size) {
    const int tile_w = left.cols;
    const int tile_h = left.rows;
    cv::Mat dashboard(tile_h * 2, tile_w * 2, left.type(), cv::Scalar(0, 0, 0));
    cv::Mat left_top = dashboard(cv::Rect(0, 0, tile_w, tile_h));
    cv::Mat right_top = dashboard(cv::Rect(tile_w, 0, tile_w, tile_h));
    left.copyTo(left_top);
    right.copyTo(right_top);

    cv::Mat relative_panel = dashboard(cv::Rect(0, tile_h, tile_w, tile_h));
    cv::Mat text_panel = dashboard(cv::Rect(tile_w, tile_h, tile_w, tile_h));
    drawRelativeSpacePanel(relative_panel, pose, view_mode, board_world_size);
    drawTextPanel(text_panel, pose, status_top, status_sub, view_mode);

    cv::line(dashboard, cv::Point(tile_w, 0), cv::Point(tile_w, tile_h * 2 - 1), cv::Scalar(90, 90, 90), 2, cv::LINE_AA);
    cv::line(dashboard, cv::Point(0, tile_h), cv::Point(tile_w * 2 - 1, tile_h), cv::Scalar(90, 90, 90), 2, cv::LINE_AA);
    return dashboard;
}

static cv::Mat makeStereoCalibrationDashboard(const cv::Mat& left,
                                              const cv::Mat& right,
                                              const std::string& status_top,
                                              const std::string& status_sub,
                                              std::size_t sample_count,
                                              bool calibration_valid,
                                              double reprojection_error,
                                              double baseline_mm) {
    const int tile_w = left.cols;
    const int tile_h = left.rows;
    cv::Mat dashboard(tile_h * 2, tile_w * 2, left.type(), cv::Scalar(0, 0, 0));
    left.copyTo(dashboard(cv::Rect(0, 0, tile_w, tile_h)));
    right.copyTo(dashboard(cv::Rect(tile_w, 0, tile_w, tile_h)));
    cv::Mat panel = dashboard(cv::Rect(0, tile_h, tile_w * 2, tile_h));
    panel.setTo(cv::Scalar(24, 28, 32));
    const bool cn = g_text_renderer.available();
    const double ui = std::max(0.35, panel.rows / 720.0);
    auto row = [&](const std::string& text, int y, const cv::Scalar& color,
                   double scale = 0.62, int thickness = 2) {
        putPanelText(panel, text, cv::Point(cvRound(36 * ui), y), scale * ui, color, thickness);
    };
    std::ostringstream title;
    title << (cn ? "双目标定" : "Stereo Extrinsic Calibration")
          << "  |  " << (cn ? "左/右同步棋盘" : "Synchronized left/right board");
    row(title.str(), cvRound(46 * ui), cv::Scalar(255, 255, 255), 0.86, 3);

    std::ostringstream samples;
    samples << (cn ? "有效同步样本：" : "Synchronized samples: ") << sample_count
            << (sample_count < 8 ? (cn ? "（至少 8 组）" : " (need at least 8)") : "");
    row(samples.str(), cvRound(92 * ui), sample_count >= 8 ? cv::Scalar(80, 240, 120) : cv::Scalar(0, 210, 255));
    row(status_top, cvRound(132 * ui), cv::Scalar(210, 220, 220), 0.52, 1);
    row(status_sub, cvRound(164 * ui), cv::Scalar(210, 220, 220), 0.52, 1);

    std::ostringstream result;
    if (calibration_valid) {
        result << (cn ? "外参状态：已计算   RMS：" : "Extrinsics: computed   RMS: ")
               << std::fixed << std::setprecision(3) << reprojection_error << " px"
               << (cn ? "   基线：" : "   baseline: ")
               << std::fixed << std::setprecision(1) << baseline_mm << " mm";
    } else {
        result << (cn ? "外参状态：未计算" : "Extrinsics: not computed");
    }
    row(result.str(), cvRound(210 * ui), calibration_valid ? cv::Scalar(80, 240, 120) : cv::Scalar(0, 210, 255), 0.66, 2);
    row(cn ? "自动采样：稳定且有效时自动保存 | S 手动采样 | C 计算 | R 校正 | W 保存 | Q 退出"
           : "Auto-capture when stable and valid | S manual | C calibrate | R rectify | W write | Q quit",
        cvRound(258 * ui), cv::Scalar(255, 220, 130), 0.54, 2);
    row(cn ? "本界面独立于物理四宫格调平数据"
           : "This view is independent from the physical four-panel leveling view",
        panel.rows - cvRound(28 * ui), cv::Scalar(170, 180, 185), 0.42, 1);
    return dashboard;
}

class KeyReader {
public:
    bool start() {
#ifndef _WIN32
        if (running_) return true;
        if (tcgetattr(STDIN_FILENO, &old_term_) != 0) return false;
        termios raw = old_term_;
        raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (old_flags_ >= 0) {
            fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
        }
        running_ = true;
        started_ = true;
        worker_ = std::thread(&KeyReader::loop, this);
        return true;
#else
        return false;
#endif
    }

    void stop() {
#ifndef _WIN32
        if (!started_) return;
        running_ = false;
        if (worker_.joinable()) worker_.join();
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
        if (old_flags_ >= 0) {
            fcntl(STDIN_FILENO, F_SETFL, old_flags_);
        }
        started_ = false;
#endif
    }

    bool poll(int& key) {
#ifndef _WIN32
        std::lock_guard<std::mutex> lock(mutex_);
        if (keys_.empty()) return false;
        key = keys_.front();
        keys_.pop();
        return true;
#else
        (void)key;
        return false;
#endif
    }

    ~KeyReader() {
        stop();
    }

private:
#ifndef _WIN32
    void loop() {
        while (running_) {
            char ch = 0;
            const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                std::lock_guard<std::mutex> lock(mutex_);
                keys_.push(static_cast<int>(static_cast<unsigned char>(ch)));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    std::atomic<bool> running_{false};
    std::thread worker_;
    std::mutex mutex_;
    std::queue<int> keys_;
    termios old_term_{};
    int old_flags_ = -1;
    bool started_ = false;
#endif
};

static double meanCornerMotion(const std::vector<cv::Point2f>& a,
                               const std::vector<cv::Point2f>& b) {
    if (a.size() != b.size() || a.empty()) return 1e9;
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) sum += cv::norm(a[i] - b[i]);
    return sum / static_cast<double>(a.size());
}

struct MonoFrameQuality {
    double sharpness = 0.0;
    double coverage_percent = 0.0;
    double frame_rms = -1.0;
    bool sharpness_ok = false;
    bool coverage_ok = false;
    bool rms_ok = true;

    bool acceptable() const {
        return sharpness_ok && coverage_ok && rms_ok;
    }
};

static const cv::Mat& calibrationMapX(const MonoCalibrationResult&) {
    static const cv::Mat empty;
    return empty;
}

static const cv::Mat& calibrationMapY(const MonoCalibrationResult&) {
    static const cv::Mat empty;
    return empty;
}

static const cv::Mat& calibrationMapX(const FisheyeCalibrationResult& result) {
    return result.map_x;
}

static const cv::Mat& calibrationMapY(const FisheyeCalibrationResult& result) {
    return result.map_y;
}

template <typename CalibrationResult>
static MonoFrameQuality measureMonoFrameQuality(const cv::Mat& frame,
                                                const std::vector<cv::Point2f>& corners,
                                                const std::vector<cv::Point3f>& board_points,
                                                const CalibrationResult& calibration,
                                                double min_sharpness,
                                                double min_coverage,
                                                double max_coverage,
                                                double max_rms,
                                                bool evaluate_frame_rms = true) {
    MonoFrameQuality quality;
    if (frame.empty() || corners.empty()) return quality;
    cv::Mat gray;
    if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else gray = frame;
    const cv::Rect board_box = cv::boundingRect(corners) & cv::Rect(0, 0, gray.cols, gray.rows);
    if (board_box.width <= 0 || board_box.height <= 0) return quality;
    const cv::Mat roi = gray(board_box);
    cv::Mat laplacian;
    cv::Laplacian(roi, laplacian, CV_64F);
    cv::Scalar mean_value;
    cv::Scalar stddev_value;
    cv::meanStdDev(laplacian, mean_value, stddev_value);
    quality.sharpness = stddev_value[0] * stddev_value[0];
    quality.coverage_percent = 100.0 * static_cast<double>(board_box.area()) /
                               static_cast<double>(gray.cols * gray.rows);
    quality.sharpness_ok = quality.sharpness >= min_sharpness;
    quality.coverage_ok = quality.coverage_percent >= min_coverage &&
                          quality.coverage_percent <= max_coverage;
    if (evaluate_frame_rms && calibration.valid && calibration.camera_matrix.rows == 3 &&
        board_points.size() == corners.size()) {
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        if (cv::solvePnP(board_points, corners, calibration.camera_matrix,
                         calibration.dist_coeffs, rvec, tvec, false,
                         cv::SOLVEPNP_ITERATIVE)) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(board_points, rvec, tvec,
                          calibration.camera_matrix, calibration.dist_coeffs, projected);
        quality.frame_rms = cv::norm(projected, corners, cv::NORM_L2) /
                            std::sqrt(static_cast<double>(corners.size()));
        }
    }
    quality.rms_ok = quality.frame_rms < 0.0 || quality.frame_rms <= max_rms;
    return quality;
}

static bool isNovelMonoSample(const std::vector<std::vector<cv::Point2f>>& accepted,
                              const std::vector<cv::Point2f>& corners) {
    for (const auto& sample : accepted) {
        if (!autoSampleIsDifferent(sample, corners)) return false;
    }
    return true;
}

template <typename CalibrationResult>
static void drawMonoCalibrationPanel(cv::Mat& panel,
                                     const AppConfig& app,
                                     bool left_camera,
                                     bool fisheye,
                                     bool board_found,
                                     int stable_frames,
                                     const MonoFrameQuality& quality,
                                     std::size_t samples,
                                     const CalibrationResult& result,
                                     bool saved,
                                     const std::string& detail) {
    panel.setTo(cv::Scalar(24, 26, 30));
    const bool cn = g_text_renderer.available();
    const double ui = std::clamp(panel.rows / 1080.0, 0.75, 2.0);
    int y = cvRound(70 * ui);
    const int x = cvRound(54 * ui);
    auto row = [&](const std::string& text, double scale, cv::Scalar color, int thickness = 2) {
        putPanelText(panel, text, {x, y}, scale * ui, color, thickness);
        y += cvRound((scale * 48.0 + 24.0) * ui);
    };

    row(cn ? (fisheye ? (left_camera ? "左相机鱼眼标定" : "右相机鱼眼标定")
                      : (left_camera ? "左相机单目标定" : "右相机单目标定"))
           : (fisheye ? (left_camera ? "Left Fisheye Calibration" : "Right Fisheye Calibration")
                      : (left_camera ? "Left Monocular Calibration" : "Right Monocular Calibration")),
        1.35, {255, 255, 255}, 3);
    std::ostringstream ss;
    ss << (cn ? "标定板：" : "Board: ")
       << app.board_cols << "x" << app.board_rows
       << (cn ? " 内角点，格长 " : " inner corners, square size ")
       << app.square_size << " mm";
    row(ss.str(), 0.66, {210, 220, 220});
    ss.str("");
    ss << (cn ? "自动样本：" : "Automatic samples: ") << samples << "/" << app.mono_samples;
    row(ss.str(), 0.95, samples >= static_cast<std::size_t>(app.mono_samples)
                              ? cv::Scalar(80, 255, 100) : cv::Scalar(100, 240, 255), 3);
    ss.str("");
    ss << (cn ? "当前清晰度：" : "Current sharpness: ") << std::fixed << std::setprecision(1)
       << quality.sharpness << (cn ? "，门槛：" : ", minimum: ") << app.mono_min_sharpness
       << (quality.sharpness_ok ? "（合格）" : "（不合格）");
    row(ss.str(), 0.72, quality.sharpness_ok ? cv::Scalar(80, 255, 100) : cv::Scalar(0, 190, 255), 2);
    ss.str("");
    ss << (cn ? "棋盘覆盖率：" : "Board coverage: ") << std::fixed << std::setprecision(1)
       << quality.coverage_percent << "%" << (quality.coverage_ok ? "（合格）" : "（不合格）");
    row(ss.str(), 0.72, quality.coverage_ok ? cv::Scalar(80, 255, 100) : cv::Scalar(0, 190, 255), 2);
    ss.str("");
    if (quality.frame_rms >= 0.0) {
        ss << (cn ? "当前帧 RMS：" : "Current frame RMS: ") << std::fixed << std::setprecision(3)
           << quality.frame_rms << " px" << (quality.rms_ok ? "（合格）" : "（不合格）");
    } else if (fisheye) {
        ss << (cn ? "当前帧 RMS：鱼眼模式不使用此项筛选" : "Current frame RMS: not used for fisheye sampling");
    } else {
        ss << (cn ? "当前帧 RMS：等待至少8组样本完成初始计算" : "Current frame RMS: available after initial calibration");
    }
    row(ss.str(), 0.68, quality.rms_ok ? cv::Scalar(80, 255, 100) : cv::Scalar(0, 190, 255), 2);
    if (!board_found) {
        row(cn ? "状态：未检测到完整棋盘" : "Status: complete board not detected",
            0.72, {0, 190, 255}, 2);
    } else if (saved) {
        row(cn ? "状态：标定完成，参数已保存" : "Status: complete and saved",
            0.72, {80, 255, 100}, 3);
    } else if (stable_frames < 4) {
        row(cn ? "状态：已检测，请保持棋盘静止" : "Status: detected; hold the board still",
            0.72, {0, 220, 255}, 2);
    } else {
        row(cn ? "状态：等待新的位置、距离或倾角" : "Status: move to a new position, distance, or angle",
            0.72, {0, 220, 255}, 2);
    }
    if (!detail.empty()) row(detail, 0.60, {210, 220, 220}, 2);

    if (result.valid) {
        ss.str("");
        ss << (cn ? "重投影 RMS：" : "Reprojection RMS: ")
           << std::fixed << std::setprecision(3) << result.reprojection_error << " px";
        row(ss.str(), 0.82,
            result.reprojection_error <= app.mono_max_rms ? cv::Scalar(80, 255, 100)
                                                           : cv::Scalar(0, 190, 255), 3);
        ss.str("");
        ss << "fx=" << std::fixed << std::setprecision(2) << result.camera_matrix.template at<double>(0, 0)
           << "  fy=" << result.camera_matrix.template at<double>(1, 1);
        row(ss.str(), 0.68, {240, 240, 240});
        ss.str("");
        ss << "cx=" << result.camera_matrix.template at<double>(0, 2)
           << "  cy=" << result.camera_matrix.template at<double>(1, 2);
        row(ss.str(), 0.68, {240, 240, 240});
        ss.str("");
        ss << (cn ? "有效样本：" : "Used samples: ") << result.used_sample_count
           << (cn ? "，合格阈值：" : ", acceptance: ") << app.mono_max_rms << " px";
        row(ss.str(), 0.60, {190, 205, 210});
    }

    y = panel.rows - cvRound(55 * ui);
    row(cn ? (fisheye ? "S手动取样  C重新计算  R校正预览  W保存  Q退出" : "S手动取样  C重新计算  W保存  Q退出")
           : (fisheye ? "S capture  C calculate  R rectify  W save  Q quit" : "S capture  C calculate  W save  Q quit"),
        0.52, {175, 190, 195}, 1);
}

template <typename Calibrator, typename CalibrationResult>
static int runSingleCameraCalibration(const AppConfig& app,
                                      cv::Size image_size,
                                      bool left_camera,
                                      bool fisheye,
                                      Calibrator calibrator,
                                      CalibrationResult result) {
    const std::string device = left_camera ? app.left_device : app.right_device;
    const std::string side = left_camera ? "left" : "right";
    LatestFrameGrabber grabber(device, app.width, app.height, app.fps);
    if (!grabber.start()) {
        std::cerr << "Failed to open " << side << " camera: " << device << "\n";
        return 1;
    }
    cv::Mat startup_frame;
    std::uint64_t startup_seq = 0;
    const auto frame_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!g_quit.load() && std::chrono::steady_clock::now() < frame_deadline) {
        if (grabber.get(startup_frame, startup_seq) && !startup_frame.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (startup_frame.empty()) {
        std::cerr << "Camera opened but produced no video frames within 5 seconds: " << device << "\n"
                  << "Check the V4L2 device and capture format (for example: v4l2-ctl --all -d "
                  << device << ").\n";
        grabber.stop();
        return 1;
    }

    const std::filesystem::path output_path(app.output);
    if (!output_path.parent_path().empty()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::filesystem::create_directories(app.capture_dir);
    StereoCalibrator detector(cv::Size(app.board_cols, app.board_rows), app.square_size);
    std::vector<std::vector<cv::Point2f>> accepted_samples;
    std::vector<cv::Point2f> previous_corners;
    std::vector<cv::Point2f> corners;
    int stable_frames = 0;
    int cooldown = 0;
    std::size_t last_calibrated_count = 0;
    bool saved = false;
    std::string detail;
    bool show_rectified = false;

    HardwarePreview preview(image_size, image_size, app.fps);
    bool use_hw_preview = app.use_hardware_preview && preview.start();
    std::unique_ptr<KeyReader> key_reader;
    if (use_hw_preview) {
        key_reader = std::make_unique<KeyReader>();
        if (!key_reader->start()) {
            preview.stop();
            use_hw_preview = false;
        }
    }
    const std::string window_name = fisheye
        ? (left_camera ? "left fisheye calibration" : "right fisheye calibration")
        : (left_camera ? "left monocular calibration" : "right monocular calibration");
    if (!use_hw_preview) {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        std::cout << "Using OpenCV fallback preview.\n";
    } else {
        std::cout << "Using hardware preview.\n";
    }
    const cv::Size screen_size = queryScreenSize();

    auto capture = [&](const cv::Mat& frame, bool force) {
        if (corners.empty()) {
            detail = g_text_renderer.available() ? "无法取样：未检测到棋盘" : "Cannot capture: board not detected";
            return;
        }
        if (!force && !isNovelMonoSample(accepted_samples, corners)) {
            detail = g_text_renderer.available() ? "当前位置已采集，请明显移动或倾斜棋盘" : "Pose already captured; move or tilt the board";
            return;
        }
        calibrator.addSample(corners);
        accepted_samples.push_back(corners);
        saved = false;
        const std::size_t index = calibrator.sampleCount();
        cv::imwrite(sampleName(app.capture_dir, side + (fisheye ? "_fisheye" : "_mono"), index), frame);
        detail = g_text_renderer.available() ? "已采集第 " + std::to_string(index) + " 组"
                                              : "Captured sample " + std::to_string(index);
        std::cout << (fisheye ? "Fisheye " : "Monocular ") << side << " sample "
                  << index << "/" << app.mono_samples << " captured\n";
        cooldown = 10;
        stable_frames = 0;
    };

    auto calculateAndSave = [&](bool force_save, bool allow_save = true) {
        if (calibrator.sampleCount() < 8) {
            detail = g_text_renderer.available() ? "至少需要8组有效样本" : "At least 8 valid samples are required";
            return;
        }
        // Remember attempts as well as successes, so a failed solve is retried
        // only after another sample arrives or the user presses C/W.
        last_calibrated_count = calibrator.sampleCount();
        CalibrationResult candidate;
        try {
            if (!calibrator.calibrate(image_size, candidate)) {
                result = {};
                saved = false;
                show_rectified = false;
                detail = g_text_renderer.available() ? "标定计算失败，请增加不同姿态样本" : "Calibration failed; add diverse poses";
                std::cerr << "Calibration failed with " << calibrator.sampleCount() << " samples; add diverse poses.\n";
                return;
            }
        } catch (const cv::Exception& error) {
            result = {};
            saved = false;
            show_rectified = false;
            detail = g_text_renderer.available() ? "标定计算异常，请增加不同姿态样本" : error.what();
            std::cerr << "Calibration failed with " << calibrator.sampleCount() << " samples: " << error.what() << "\n";
            return;
        }
        result = candidate;
        if (!allow_save) {
            std::ostringstream ss;
            ss << (g_text_renderer.available() ? "初始计算完成，继续采样至目标组数；RMS："
                                               : "Initial calibration ready; continue to target samples; RMS: ")
               << std::fixed << std::setprecision(3) << result.reprojection_error << " px";
            detail = ss.str();
            saved = false;
            return;
        }
        if (!force_save && result.reprojection_error > app.mono_max_rms) {
            std::ostringstream ss;
            ss << (g_text_renderer.available() ? "RMS过大，请继续增加清晰且不同姿态的样本：" : "RMS too high; add clear diverse samples: ")
               << std::fixed << std::setprecision(3) << result.reprojection_error << " px";
            detail = ss.str();
            saved = false;
            return;
        }
        saved = calibrator.save(app.output, result);
        detail = saved ? (g_text_renderer.available() ? "参数已保存到：" : "Saved to: ") + app.output
                       : (g_text_renderer.available() ? "参数保存失败：" : "Failed to save: ") + app.output;
        if (saved) {
            std::cout << (fisheye ? "Fisheye calibration saved: " : "Monocular calibration saved: ") << app.output
                      << ", RMS=" << result.reprojection_error << " px\n";
        }
    };

    const std::vector<cv::Point3f> board_points =
        makeBoardObjectPoints(app.board_cols, app.board_rows, app.square_size);
    std::uint64_t previous_seq = 0;
    int exit_status = 0;
    while (!g_quit.load()) {
        cv::Mat frame;
        std::uint64_t seq = 0;
        if (!startup_frame.empty()) {
            frame = std::move(startup_frame);
            seq = startup_seq;
        } else if (!grabber.get(frame, seq)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (seq == previous_seq) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        previous_seq = seq;
        if (frame.size() != image_size) {
            std::cerr << "Capture resolution differs from requested calibration resolution; refusing resized samples.\n";
            exit_status = 2;
            break;
        }
        const bool found = detectScaled(detector, frame, app.process_scale, corners);
        if (found) {
            if (meanCornerMotion(previous_corners, corners) <= 1.5) ++stable_frames;
            else stable_frames = 0;
            previous_corners = corners;
        } else {
            corners.clear();
            previous_corners.clear();
            stable_frames = 0;
        }
        if (cooldown > 0) --cooldown;
        const MonoFrameQuality quality = measureMonoFrameQuality(frame, corners, board_points, result,
                                                                 app.mono_min_sharpness,
                                                                 app.mono_min_coverage,
                                                                 app.mono_max_coverage,
                                                                 app.mono_max_rms,
                                                                 !fisheye && result.reprojection_error <= app.mono_max_rms);
        if (!saved && found && stable_frames >= 4 && cooldown == 0 &&
            quality.acceptable() &&
            isNovelMonoSample(accepted_samples, corners)) {
            capture(frame, false);
        } else if (!saved && found && stable_frames >= 4 && cooldown == 0 && !quality.acceptable()) {
            if (!quality.sharpness_ok) detail = g_text_renderer.available() ? "未采样：画面清晰度不足" : "Not captured: image is not sharp enough";
            else if (!quality.coverage_ok) detail = g_text_renderer.available() ? "未采样：棋盘覆盖率不合适" : "Not captured: board coverage is unsuitable";
            else if (!quality.rms_ok) detail = g_text_renderer.available() ? "未采样：当前帧 RMS 超过阈值" : "Not captured: current frame RMS is too high";
        }
        if (!saved && calibrator.sampleCount() >= 8 &&
            calibrator.sampleCount() != last_calibrated_count) {
            calculateAndSave(false, calibrator.sampleCount() >= static_cast<std::size_t>(app.mono_samples));
        }

        cv::Mat annotated = frame.clone();
        if (found) cv::drawChessboardCorners(annotated, cv::Size(app.board_cols, app.board_rows), corners, true);
        if (fisheye && show_rectified && result.valid) {
            cv::remap(frame, annotated, calibrationMapX(result), calibrationMapY(result), cv::INTER_LINEAR);
        }
        cv::Mat panel(image_size, CV_8UC3);
        drawMonoCalibrationPanel(panel, app, left_camera, fisheye, found, stable_frames, quality,
                                 calibrator.sampleCount(), result, saved, detail);

        int key = -1;
        if (use_hw_preview) {
            preview.push(annotated, panel, "", "");
            while (key_reader && key_reader->poll(key)) {
                if (key != -1) break;
            }
        } else {
            cv::Mat combined;
            cv::hconcat(annotated, panel, combined);
            if (screen_size.width > 0 && screen_size.height > 0) {
                const double scale = std::min(static_cast<double>(screen_size.width) / combined.cols,
                                              static_cast<double>(screen_size.height) / combined.rows);
                if (scale < 1.0) cv::resize(combined, combined, cv::Size(), scale, scale, cv::INTER_AREA);
            }
            cv::imshow(window_name, combined);
            key = cv::waitKey(1) & 0xff;
        }
        if (key >= 'A' && key <= 'Z') key += 'a' - 'A';
        if (key == 'q' || key == 27) break;
        if (key == 's' && found) capture(frame, true);
        if (key == 'c') calculateAndSave(false);
        if (key == 'w') calculateAndSave(true);
        if (key == 'r' && fisheye && result.valid) show_rectified = !show_rectified;
    }

    preview.stop();
    if (key_reader) key_reader->stop();
    grabber.stop();
    if (!use_hw_preview) cv::destroyWindow(window_name);
    return exit_status;
}

static int runMonoCalibration(const AppConfig& app, cv::Size image_size) {
    const bool left_camera = app.mode == "mono-left";
    return runSingleCameraCalibration(app, image_size, left_camera, false,
        MonoCalibrator(cv::Size(app.board_cols, app.board_rows), app.square_size), MonoCalibrationResult{});
}

static int runFisheyeCalibration(const AppConfig& app, cv::Size image_size) {
    const bool left_camera = app.mode == "fisheye-left";
    return runSingleCameraCalibration(app, image_size, left_camera, true,
        FisheyeCalibrator(cv::Size(app.board_cols, app.board_rows), app.square_size), FisheyeCalibrationResult{});
}

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    AppConfig app;
    try {
        app = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 2;
    }

    const cv::Size image_size(app.width, app.height);
    const bool fisheye_stereo = app.mode == "stereo-fisheye";
    const bool fisheye_physical = app.mode == "physical-fisheye";
    const bool stereo_calibration_mode = app.mode == "stereo" || fisheye_stereo;
    StereoCalibrator calibrator(cv::Size(app.board_cols, app.board_rows), app.square_size);
    FisheyeStereoCalibrator fisheye_calibrator(cv::Size(app.board_cols, app.board_rows), app.square_size);
    StereoCalibrationResult calibration;
    MonoCalibrationResult left_mono;
    MonoCalibrationResult right_mono;
    FisheyeCalibrationResult left_fisheye;
    FisheyeCalibrationResult right_fisheye;
    bool mono_parameters_loaded = false;
    if (app.mode == "stereo" || app.mode == "physical") {
        std::string left_error;
        std::string right_error;
        const bool left_ok = loadMonoCalibration(app.left_mono_file, image_size,
                                                 cv::Size(app.board_cols, app.board_rows),
                                                 app.square_size, left_mono, left_error);
        const bool right_ok = loadMonoCalibration(app.right_mono_file, image_size,
                                                  cv::Size(app.board_cols, app.board_rows),
                                                  app.square_size, right_mono, right_error);
        if (!left_ok || !right_ok) {
            if (!left_ok) std::cerr << left_error << "\n";
            if (!right_ok) std::cerr << right_error << "\n";
            std::cerr << "物理调试/双目标定模式需要有效的左右单目标定文件；不会使用近似内参。\n";
            return 2;
        }
        mono_parameters_loaded = true;
        std::cout << "Loaded monocular parameters: left=" << app.left_mono_file
                  << ", right=" << app.right_mono_file << "\n";
    }
    if (fisheye_stereo || fisheye_physical) {
        std::string left_error;
        std::string right_error;
        const bool left_ok = loadFisheyeCalibration(app.left_fisheye_file, image_size,
                                                    cv::Size(app.board_cols, app.board_rows),
                                                    app.square_size, left_fisheye, left_error);
        const bool right_ok = loadFisheyeCalibration(app.right_fisheye_file, image_size,
                                                     cv::Size(app.board_cols, app.board_rows),
                                                     app.square_size, right_fisheye, right_error);
        if (!left_ok || !right_ok) {
            if (!left_ok) std::cerr << left_error << "\n";
            if (!right_ok) std::cerr << right_error << "\n";
            std::cerr << "Fisheye stereo/physical modes require valid left and right fisheye calibration files.\n";
            return 2;
        }
        std::cout << "Loaded fisheye parameters: left=" << app.left_fisheye_file
                  << ", right=" << app.right_fisheye_file << "\n";
        if (fisheye_physical) mono_parameters_loaded = true;
    }
    const std::vector<cv::Point3f> board_object_points =
        makeBoardObjectPoints(app.board_cols, app.board_rows, app.square_size);
    const cv::Size2f board_world_size((app.board_cols - 1) * app.square_size,
                                      (app.board_rows - 1) * app.square_size);
    // Stereo mode uses the two monocular files as fixed intrinsics and calibrates R/T.
    if (g_text_renderer.start(app.font_path)) {
        std::cout << "Using Chinese font: " << g_text_renderer.fontPath() << "\n";
    } else {
        std::cout << "Chinese font renderer unavailable; using ASCII fallback.\n";
    }

    if (app.mode == "mono-left" || app.mode == "mono-right") {
        return runMonoCalibration(app, image_size);
    }
    if (app.mode == "fisheye-left" || app.mode == "fisheye-right") {
        return runFisheyeCalibration(app, image_size);
    }

    LatestFrameGrabber left_grabber(app.left_device, app.width, app.height, app.fps);
    LatestFrameGrabber right_grabber(app.right_device, app.width, app.height, app.fps);
    if (!left_grabber.start()) {
        std::cerr << "Failed to open left camera: " << app.left_device << "\n";
        return 1;
    }
    if (!right_grabber.start()) {
        left_grabber.stop();
        std::cerr << "Failed to open right camera: " << app.right_device << "\n";
        return 1;
    }

    std::filesystem::create_directories(app.capture_dir);
    const cv::Size screen_size = queryScreenSize();

    HardwarePreview preview({app.width, app.height * 2}, {app.width, app.height * 2}, app.fps);
    bool use_hw_preview = app.use_hardware_preview && preview.start();
    std::unique_ptr<KeyReader> key_reader;
    if (use_hw_preview) {
        key_reader = std::make_unique<KeyReader>();
        if (!key_reader->start()) {
            preview.stop();
            use_hw_preview = false;
        } else {
            std::cout << "Using hardware preview.\n";
        }
    }
    if (!use_hw_preview) {
        cv::namedWindow("stereo calibration", cv::WINDOW_NORMAL);
        cv::setWindowProperty("stereo calibration", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        std::cout << "Using OpenCV fallback preview.\n";
    }

    std::atomic<bool> show_rectified{false};
    std::atomic<bool> align_mode{true};
    std::atomic<int> relative_view_mode{0};
    SceneMouseState scene_mouse;
    scene_mouse.mode=&relative_view_mode;
    if(!use_hw_preview) cv::setMouseCallback("stereo calibration",sceneMouse,&scene_mouse);
    std::vector<cv::Point2f> raw_left_corners;
    std::vector<cv::Point2f> raw_right_corners;

    std::atomic<bool> analysis_running{true};
    std::mutex model_mutex;
    std::mutex status_mutex;
    std::string status_line1 = "samples=0";
    std::string status_line2 = "left=no right=no mode=raw";
    StereoGuide latest_guide;
    StereoPose latest_pose;
    auto latest_pose_time = std::chrono::steady_clock::now();
    int stable_pose_count = 0; // Written only by the analysis thread.
    int stereo_stable_frames = 0;
    int stereo_sample_cooldown = 0;
    std::vector<cv::Point2f> previous_stereo_left;
    std::vector<cv::Point2f> previous_stereo_right;
    std::vector<cv::Point2f> last_auto_left;
    std::vector<cv::Point2f> last_auto_right;

    std::thread analysis_thread([&] {
        std::vector<cv::Point2f> left_corners;
        std::vector<cv::Point2f> right_corners;
        std::uint64_t last_left_seq = 0;
        std::uint64_t last_right_seq = 0;
        auto previous_analysis_time = std::chrono::steady_clock::now();
        while (analysis_running.load() && !g_quit.load()) {
            cv::Mat left;
            cv::Mat right;
            std::uint64_t left_seq = 0;
            std::uint64_t right_seq = 0;
            FrameClock::time_point left_received, right_received;
            if (!left_grabber.get(left, left_seq, &left_received) || !right_grabber.get(right, right_seq, &right_received)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (left_seq == last_left_seq || right_seq == last_right_seq) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            last_left_seq = left_seq;
            last_right_seq = right_seq;

            // Capture-mode changes may crop or bin the sensor. Resizing cannot
            // restore the field of view used to calibrate K.
            const bool raw_resolution_valid = left.size() == image_size && right.size() == image_size;

            const bool analyzed_left_found = raw_resolution_valid && detectScaled(calibrator, left, app.process_scale, left_corners);
            const bool analyzed_right_found = raw_resolution_valid && detectScaled(calibrator, right, app.process_scale, right_corners);
            StereoGuide guide;
            StereoPose pose;
            if (analyzed_left_found && analyzed_right_found) {
                const double left_motion = meanCornerMotion(previous_stereo_left, left_corners);
                const double right_motion = meanCornerMotion(previous_stereo_right, right_corners);
                if (left_motion <= 1.5 && right_motion <= 1.5) ++stereo_stable_frames;
                else stereo_stable_frames = 0;
                previous_stereo_left = left_corners;
                previous_stereo_right = right_corners;

                if (stereo_calibration_mode) {
                    const MonoFrameQuality left_quality = fisheye_stereo
                        ? measureMonoFrameQuality(left, left_corners, board_object_points, left_fisheye,
                              app.mono_min_sharpness, app.mono_min_coverage,
                              app.mono_max_coverage, app.mono_max_rms, false)
                        : measureMonoFrameQuality(left, left_corners, board_object_points, left_mono,
                              app.mono_min_sharpness, app.mono_min_coverage,
                              app.mono_max_coverage, app.mono_max_rms);
                    const MonoFrameQuality right_quality = fisheye_stereo
                        ? measureMonoFrameQuality(right, right_corners, board_object_points, right_fisheye,
                              app.mono_min_sharpness, app.mono_min_coverage,
                              app.mono_max_coverage, app.mono_max_rms, false)
                        : measureMonoFrameQuality(right, right_corners, board_object_points, right_mono,
                              app.mono_min_sharpness, app.mono_min_coverage,
                              app.mono_max_coverage, app.mono_max_rms);
                    if (stereo_sample_cooldown > 0) --stereo_sample_cooldown;
                    const bool novel = autoSampleIsDifferent(last_auto_left, left_corners) &&
                                       autoSampleIsDifferent(last_auto_right, right_corners);
                    if (freshFramePair(left_received, right_received, FrameClock::now()) &&
                        stereo_stable_frames >= 4 && stereo_sample_cooldown == 0 && novel &&
                        left_quality.acceptable() && right_quality.acceptable()) {
                    std::size_t next_index = 0;
                    {
                        std::lock_guard<std::mutex> lock(model_mutex);
                        next_index = fisheye_stereo ? fisheye_calibrator.sampleCount() + 1
                                                    : calibrator.sampleCount() + 1;
                        if (fisheye_stereo) fisheye_calibrator.addSample(left_corners, right_corners);
                        else calibrator.addSample(left_corners, right_corners);
                    }
                    cv::imwrite(sampleName(app.capture_dir, fisheye_stereo ? "left_fisheye" : "left", next_index), left);
                    cv::imwrite(sampleName(app.capture_dir, fisheye_stereo ? "right_fisheye" : "right", next_index), right);
                    last_auto_left = left_corners;
                    last_auto_right = right_corners;
                    stereo_sample_cooldown = 12;
                    stereo_stable_frames = 0;
                    std::cout << "Auto-captured " << (fisheye_stereo ? "stereo fisheye" : "stereo")
                              << " sample " << next_index << "\n";
                    }
                }
            } else {
                stereo_stable_frames = 0;
                previous_stereo_left.clear();
                previous_stereo_right.clear();
                if (stereo_calibration_mode && stereo_sample_cooldown > 0) --stereo_sample_cooldown;
            }
            if (analyzed_left_found && analyzed_right_found) {
                guide.left = makeBoardGuide(left_corners, app.board_cols, app.board_rows, image_size);
                guide.right = makeBoardGuide(right_corners, app.board_cols, app.board_rows, image_size);
                guide.valid = guide.left.valid && guide.right.valid;
                if (guide.valid) {
                    guide.left_center = boardPoint(left_corners, app.board_cols, app.board_rows / 2, app.board_cols / 2);
                    guide.right_center = boardPoint(right_corners, app.board_cols, app.board_rows / 2, app.board_cols / 2);
                    guide.centers_valid = true;
                    const double x_ref = image_size.width / 2.0;
                    const double y_ref = image_size.height / 2.0;
                    const double left_center_y = lineYAtX(guide.left.horizontal[1], x_ref);
                    const double right_center_y = lineYAtX(guide.right.horizontal[1], x_ref);
                    if (left_center_y >= 0.0 && right_center_y >= 0.0) {
                        guide.stereo_y_diff = std::abs(left_center_y - right_center_y);
                        guide.board_to_screen_center_y = 0.5 * (std::abs(left_center_y - y_ref) +
                                                                 std::abs(right_center_y - y_ref));
                    }
                    guide.signed_stereo_y_diff = guide.left_center.y - guide.right_center.y;
                    guide.stereo_y_diff = std::abs(guide.signed_stereo_y_diff);
                    guide.horizontal_offset = std::abs(guide.left_center.x - guide.right_center.x);
                    if (left_corners.size() == right_corners.size() && !left_corners.empty()) {
                        double sum_y_diff = 0.0;
                        for (std::size_t idx = 0; idx < left_corners.size(); ++idx) {
                            sum_y_diff += std::abs(left_corners[idx].y - right_corners[idx].y);
                        }
                        guide.avg_corner_y_diff = sum_y_diff / static_cast<double>(left_corners.size());
                    }
                    if (guide.left.vertical.valid && guide.right.vertical.valid) {
                        guide.signed_roll_diff_deg = lineAngleDeg(guide.left.vertical) - lineAngleDeg(guide.right.vertical);
                        guide.roll_diff_deg = std::abs(guide.signed_roll_diff_deg);
                    }
                }

                CameraPose left_pose;
                CameraPose right_pose;
                cv::Mat left_k, left_d, right_k, right_d;
                cv::Mat stereo_rotation, stereo_translation;
                {
                    std::lock_guard<std::mutex> lock(model_mutex);
                    if (fisheye_physical) {
                        left_k = left_fisheye.camera_matrix.clone();
                        left_d = left_fisheye.dist_coeffs.clone();
                        right_k = right_fisheye.camera_matrix.clone();
                        right_d = right_fisheye.dist_coeffs.clone();
                    } else if (mono_parameters_loaded) {
                        left_k = left_mono.camera_matrix.clone();
                        left_d = left_mono.dist_coeffs.clone();
                        right_k = right_mono.camera_matrix.clone();
                        right_d = right_mono.dist_coeffs.clone();
                    } else if(calibration.valid && calibration.image_size==image_size) {
                        left_k = calibration.left_camera_matrix.clone();
                        left_d = calibration.left_dist_coeffs.clone();
                        right_k = calibration.right_camera_matrix.clone();
                        right_d = calibration.right_dist_coeffs.clone();
                        stereo_rotation = calibration.R.clone();
                        stereo_translation = calibration.T.clone();
                    }
                }
                if (!fisheye_stereo && estimateCameraPose(left_corners, board_object_points, image_size, left_k, left_d, left_pose, fisheye_physical) &&
                    estimateCameraPose(right_corners, board_object_points, image_size, right_k, right_d, right_pose, fisheye_physical)) {
                    pose = makeStereoPose(left_pose, right_pose);
                    if (!mono_parameters_loaded) {
                        applyCalibratedStereoGeometry(pose, stereo_rotation, stereo_translation);
                    }
                }
            }

            pose.require_raw_alignment = fisheye_physical;
            pose.raw_resolution_valid = raw_resolution_valid;
            pose.frame_pair_valid = freshFramePair(left_received, right_received, FrameClock::now());
            if (!pose.frame_pair_valid) {
                pose.valid = false;
                pose.reliable = false;
                guide = {};
            }
            if (fisheye_physical) {
                pose.cy_diff_px = right_fisheye.camera_matrix.at<double>(1, 2) - left_fisheye.camera_matrix.at<double>(1, 2);
                if (analyzed_left_found && analyzed_right_found)
                    pose.raw_alignment = measureRawAlignment(left_corners, right_corners);
            }
            const auto analysis_time = std::chrono::steady_clock::now();
            if (analysis_time - previous_analysis_time > std::chrono::seconds(1)) stable_pose_count=0;
            previous_analysis_time = analysis_time;
            stable_pose_count=withinLevelTolerance(pose)?std::min(10,stable_pose_count+1):0;
            pose.stable_frames=stable_pose_count;

            std::ostringstream line1;
            line1 << (fisheye_physical
                          ? (g_text_renderer.available() ? "鱼眼物理四宫格调平模式" : "fisheye physical four-panel leveling")
                          : app.mode == "physical"
                          ? (g_text_renderer.available() ? "物理四宫格调试模式" : "physical four-panel debug mode")
                          : (fisheye_stereo
                                 ? (g_text_renderer.available() ? "双目鱼眼外参标定" : "stereo fisheye calibration")
                                 : (g_text_renderer.available() ? "单目标定参数已加载" : "monocular parameters loaded")));

            std::ostringstream line2;
            if (g_text_renderer.available()) {
                line2 << "左=" << (analyzed_left_found ? "已检测" : "未检测")
                      << " 右=" << (analyzed_right_found ? "已检测" : "未检测")
                      << " 模式=" << (show_rectified.load() ? "校正" : "原始");
            } else {
                line2 << "left=" << (analyzed_left_found ? "ok" : "no")
                      << " right=" << (analyzed_right_found ? "ok" : "no")
                      << " mode=" << (show_rectified.load() ? "rectified" : "raw");
            }
            line2 << (g_text_renderer.available() ? " 使用单目标定共同坐标系" : " mono-geometry");
            if (align_mode.load()) {
                const bool ok = !stereo_calibration_mode
                                ? (withinLevelTolerance(pose) && pose.stable_frames >= 10)
                                : guide.valid &&
                                guide.stereo_y_diff >= 0.0 && guide.stereo_y_diff <= 2.0 &&
                                guide.avg_corner_y_diff >= 0.0 && guide.avg_corner_y_diff <= 2.0 &&
                                guide.roll_diff_deg >= 0.0 && guide.roll_diff_deg <= 0.5;
                line2 << " align=" << (ok ? "ok" : "move");
                if (guide.valid && !fisheye_physical) {
                    line2 << " y=" << std::fixed << std::setprecision(1) << guide.stereo_y_diff
                          << " avg_y=" << std::fixed << std::setprecision(1) << guide.avg_corner_y_diff
                          << " x=" << std::fixed << std::setprecision(1) << guide.horizontal_offset
                          << " roll=" << std::fixed << std::setprecision(2) << guide.roll_diff_deg;
                }
            }
            if (guide.board_to_screen_center_y >= 0.0) {
                line2 << " center=" << std::fixed << std::setprecision(0) << guide.board_to_screen_center_y << "px";
            }
            if (stereo_calibration_mode) {
                line2 << " auto=on stable=" << stereo_stable_frames << "/4";
            }
            {
                std::lock_guard<std::mutex> lock(status_mutex);
                status_line1 = line1.str();
                status_line2 = line2.str();
                latest_guide = guide;
                latest_pose = pose;
                latest_pose_time=std::min(left_received, right_received);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(180));
        }
    });

    std::uint64_t prev_left_seq = 0;
    std::uint64_t prev_right_seq = 0;
    auto handleKey = [&](int key) {
        if (key >= 'A' && key <= 'Z') key += 'a' - 'A';
        if (key == 'q' || key == 27 || g_quit.load()) {
            g_quit.store(true);
            return false;
        }
        if (key == 'r') {
            if (!calibration.valid) {
                std::cout << "Rectification is unavailable until stereo calibration is computed.\n";
                return true;
            }
            show_rectified.store(!show_rectified.load());
        }
        if (key == 'a') {
            align_mode.store(!align_mode.load());
        }
        if (key == 'v') {
            relative_view_mode.store((relative_view_mode.load() + 1) % 4);
        }
        if (key == 'x') {
            g_orbit_view.show_axes = !g_orbit_view.show_axes;
        }
        if (key == 'h') {
            g_orbit_view.show_reference = !g_orbit_view.show_reference;
        }
        if (key == 'e') {
            g_orbit_view.show_error = !g_orbit_view.show_error;
        }
        // Rotate the OpenGL axonometric view from the keyboard.  This works
        // with both the X11 window and the Jetson hardware preview key reader.
        if (key == 'j') { relative_view_mode.store(3); g_orbit_view.yaw_deg -= 5.0; }
        if (key == 'l') { relative_view_mode.store(3); g_orbit_view.yaw_deg += 5.0; }
        if (key == 'i') { relative_view_mode.store(3); g_orbit_view.pitch_deg -= 5.0; }
        if (key == 'k') { relative_view_mode.store(3); g_orbit_view.pitch_deg += 5.0; }
        if (key == 'b') { relative_view_mode.store(3); g_orbit_view.roll_deg -= 5.0; }
        if (key == 'n') { relative_view_mode.store(3); g_orbit_view.roll_deg += 5.0; }
        if (key == 'u') { relative_view_mode.store(3); g_orbit_view.zoom = std::max(0.35, g_orbit_view.zoom * 0.90); }
        if (key == 'o') { relative_view_mode.store(3); g_orbit_view.zoom = std::min(3.0, g_orbit_view.zoom * 1.10); }
        if (key == '0') {
            relative_view_mode.store(3);
            g_orbit_view.yaw_deg = 45.0;
            g_orbit_view.pitch_deg = -35.0;
            g_orbit_view.roll_deg = 0.0;
            g_orbit_view.zoom = 1.0;
            g_orbit_view.pan_x = g_orbit_view.pan_y = 0.0;
        }
        if (key == 's') {
            if (!stereo_calibration_mode) return true;
            cv::Mat left;
            cv::Mat right;
            std::uint64_t left_seq = 0;
            std::uint64_t right_seq = 0;
            FrameClock::time_point left_received, right_received;
            if (!left_grabber.get(left, left_seq, &left_received) ||
                !right_grabber.get(right, right_seq, &right_received)) {
                return true;
            }
            if (left.size() != image_size || right.size() != image_size ||
                !freshFramePair(left_received, right_received, FrameClock::now())) {
                std::cout << "Cannot sample: capture resolution or frame timing does not match calibration.\n";
                return true;
            }
            const bool sample_left_found = detectScaled(calibrator, left, app.process_scale, raw_left_corners);
            const bool sample_right_found = detectScaled(calibrator, right, app.process_scale, raw_right_corners);
            if (sample_left_found && sample_right_found &&
                freshFramePair(left_received, right_received, FrameClock::now())) {
                std::size_t next_index = 0;
                {
                    std::lock_guard<std::mutex> lock(model_mutex);
                    next_index = fisheye_stereo ? fisheye_calibrator.sampleCount() + 1
                                                : calibrator.sampleCount() + 1;
                    if (fisheye_stereo) fisheye_calibrator.addSample(raw_left_corners, raw_right_corners);
                    else calibrator.addSample(raw_left_corners, raw_right_corners);
                }
                cv::imwrite(sampleName(app.capture_dir, fisheye_stereo ? "left_fisheye" : "left", next_index), left);
                cv::imwrite(sampleName(app.capture_dir, fisheye_stereo ? "right_fisheye" : "right", next_index), right);
                std::cout << "Saved sample " << next_index << "\n";
            } else {
                std::cout << "Both cameras must detect the chessboard before saving.\n";
            }
        }
        if (key == 'c') {
            if (!stereo_calibration_mode) return true;
            std::lock_guard<std::mutex> lock(model_mutex);
            const bool calibrated = fisheye_stereo
                ? fisheye_calibrator.calibrateWithIntrinsics(image_size, left_fisheye, right_fisheye, calibration)
                : (mono_parameters_loaded
                    ? calibrator.calibrateWithIntrinsics(image_size, left_mono, right_mono, calibration)
                    : calibrator.calibrate(image_size, calibration));
            if (calibrated) {
                std::cout << "Calibration RMS error: " << calibration.reprojection_error << "\n";
                show_rectified.store(true);
            } else {
                show_rectified.store(false);
                std::cout << "Stereo calibration failed; need at least 8 valid synchronized samples.\n";
            }
        }
        if (key == 'w') {
            if (!stereo_calibration_mode) return true;
            std::lock_guard<std::mutex> lock(model_mutex);
            if ((fisheye_stereo ? fisheye_calibrator.save(app.output, calibration)
                                : calibrator.save(app.output, calibration))) {
                std::cout << "Wrote " << app.output << "\n";
            } else {
                std::cout << "No valid calibration to write yet.\n";
            }
        }
        return true;
    };

    while (!g_quit.load()) {
        cv::Mat left;
        cv::Mat right;
        std::uint64_t left_seq = 0;
        std::uint64_t right_seq = 0;
        if (!left_grabber.get(left, left_seq) || !right_grabber.get(right, right_seq)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (left_seq == prev_left_seq && right_seq == prev_right_seq) {
            // Keep rendering so expired measurements lose PASS even if both
            // streams stop producing frames, and the operator can still quit.
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        prev_left_seq = left_seq;
        prev_right_seq = right_seq;

        if (left.size() != image_size) cv::resize(left, left, image_size);
        if (right.size() != image_size) cv::resize(right, right, image_size);

        cv::Mat left_view = left;
        cv::Mat right_view = right;
        if (show_rectified.load()) {
            std::lock_guard<std::mutex> lock(model_mutex);
            if (calibration.valid) {
                cv::remap(left, left_view, calibration.left_map_x, calibration.left_map_y, cv::INTER_LINEAR);
                cv::remap(right, right_view, calibration.right_map_x, calibration.right_map_y, cv::INTER_LINEAR);
            }
        }
        std::string line1;
        std::string line2;
        StereoGuide guide;
        StereoPose pose;
        {
            std::lock_guard<std::mutex> lock(status_mutex);
            line1 = status_line1;
            line2 = status_line2;
            guide = latest_guide;
            pose = latest_pose;
            const auto left_received = left_grabber.receivedTime();
            const auto right_received = right_grabber.receivedTime();
            if (!freshFramePair(left_received, right_received, FrameClock::now()) ||
                std::chrono::steady_clock::now()-latest_pose_time > std::chrono::seconds(1)) {
                pose={};
                pose.require_raw_alignment = fisheye_physical;
                pose.frame_pair_valid = false;
                guide={};
            }
        }

        if (!stereo_calibration_mode && align_mode.load()) {
            const bool qualified = withinLevelTolerance(pose) && pose.stable_frames >= 10;
            drawAlignmentGuides(left_view, right_view, guide, qualified);
        }
        cv::Mat dashboard;
        if (stereo_calibration_mode) {
            bool calibration_valid = false;
            double reprojection_error = 0.0;
            double baseline_mm = 0.0;
            std::size_t sample_count = 0;
            {
                std::lock_guard<std::mutex> lock(model_mutex);
                calibration_valid = calibration.valid;
                reprojection_error = calibration.reprojection_error;
                baseline_mm = calibration.T.empty() ? 0.0 : cv::norm(calibration.T);
                sample_count = fisheye_stereo ? fisheye_calibrator.sampleCount() : calibrator.sampleCount();
            }
            dashboard = makeStereoCalibrationDashboard(left_view, right_view, line1, line2,
                                                       sample_count, calibration_valid,
                                                       reprojection_error, baseline_mm);
        } else {
            dashboard = makeDashboard(left_view,
                                      right_view,
                                      pose,
                                      line1,
                                      line2,
                                      relative_view_mode.load(),
                                      board_world_size);
        }

        if (use_hw_preview) {
            cv::Mat left_column = dashboard(cv::Rect(0, 0, app.width, app.height * 2));
            cv::Mat right_column = dashboard(cv::Rect(app.width, 0, app.width, app.height * 2));
            preview.push(left_column, right_column, "", "");
            int key = -1;
            while (key_reader && key_reader->poll(key)) {
                if (!handleKey(key)) {
                    break;
                }
            }
            if (g_quit.load()) {
                break;
            }
        } else {
            cv::Mat combined = dashboard;
            double display_scale=1.0;
            int crop_x=0,crop_y=0;
            if (screen_size.width > 0 && screen_size.height > 0 &&
                (combined.cols != screen_size.width || combined.rows != screen_size.height)) {
                const double scale_x = static_cast<double>(screen_size.width) / combined.cols;
                const double scale_y = static_cast<double>(screen_size.height) / combined.rows;
                const double scale = std::max(scale_x, scale_y);
                display_scale=scale;
                const cv::Size fitted_size(
                    std::max(1, static_cast<int>(std::round(combined.cols * scale))),
                    std::max(1, static_cast<int>(std::round(combined.rows * scale))));
                cv::Mat fitted;
                cv::resize(combined, fitted, fitted_size, 0.0, 0.0, cv::INTER_LINEAR);
                const int offset_x = std::max(0, (fitted.cols - screen_size.width) / 2);
                const int offset_y = std::max(0, (fitted.rows - screen_size.height) / 2);
                crop_x=offset_x;crop_y=offset_y;
                combined = fitted(cv::Rect(offset_x, offset_y, screen_size.width, screen_size.height)).clone();
            }
            const double scene_ui=std::max(0.35,app.height/720.0);
            scene_mouse.area=cv::Rect(-crop_x,cvRound((app.height+110*scene_ui)*display_scale)-crop_y,
                cvRound(app.width*display_scale),std::max(1,cvRound((app.height-260*scene_ui)*display_scale)));
            cv::imshow("stereo calibration", combined);
            const int key = cv::waitKey(1) & 0xff;
            if (key != -1 && !handleKey(key)) {
                break;
            }
        }
    }

    analysis_running.store(false);
    if (analysis_thread.joinable()) analysis_thread.join();
    if (key_reader) key_reader->stop();
    preview.stop();
    left_grabber.stop();
    right_grabber.stop();
    cv::destroyAllWindows();
    return 0;
}
