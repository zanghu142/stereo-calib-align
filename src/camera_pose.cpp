#include "camera_pose.h"
#include "calibration_validation.h"

#include <cmath>

namespace {

// Optimize the measured pixels with K/D fixed. Undistortion magnifies corner
// noise nonuniformly, so minimizing undistorted error changes this objective.
class FisheyePoseResidual final : public cv::LMSolver::Callback {
public:
    FisheyePoseResidual(const std::vector<cv::Point3d>& board,
                        const std::vector<cv::Point2d>& pixels, const cv::Mat& k, const cv::Mat& d)
        : board_(board), pixels_(pixels), k_(k), d_(d) {}

    bool compute(cv::InputArray input, cv::OutputArray error, cv::OutputArray jacobian) const override {
        const cv::Mat parameters = input.getMat();
        if (!cv::checkRange(parameters)) return false;
        const cv::Vec3d r(parameters.at<double>(0),parameters.at<double>(1),parameters.at<double>(2));
        const cv::Vec3d t(parameters.at<double>(3),parameters.at<double>(4),parameters.at<double>(5));
        cv::Matx33d rotation;
        cv::Rodrigues(r,rotation);
        for (const auto& p : board_)
            if ((rotation*cv::Vec3d(p.x,p.y,p.z)+t)[2] <= 0.0) return false;
        std::vector<cv::Point2d> projected;
        cv::Mat full_jacobian;
        cv::fisheye::projectPoints(board_,projected,r,t,k_,d_,0.0,
                                    jacobian.needed() ? cv::OutputArray(full_jacobian) : cv::noArray());
        error.create(static_cast<int>(2*pixels_.size()),1,CV_64F);
        cv::Mat residual=error.getMat();
        for (std::size_t i=0;i<pixels_.size();++i) {
            residual.at<double>(static_cast<int>(2*i))=projected[i].x-pixels_[i].x;
            residual.at<double>(static_cast<int>(2*i+1))=projected[i].y-pixels_[i].y;
        }
        if (jacobian.needed()) {
            // OpenCV fisheye columns: fx,fy,cx,cy,k1..k4,rvec(3),tvec(3),alpha.
            full_jacobian.colRange(8,14).copyTo(jacobian);
            if (!cv::checkRange(jacobian.getMat())) return false;
        }
        return cv::checkRange(residual);
    }

private:
    std::vector<cv::Point3d> board_;
    std::vector<cv::Point2d> pixels_;
    cv::Mat k_,d_;
};

} // namespace

// Camera-axis angles in the board coordinate frame.
double opticalHeadingDeg(const cv::Vec3d& optical_axis) {
    // For the board convention X-right, Y-down, +Z points into the board;
    // a camera facing it looks along +Z in board coordinates.
    // Projecting onto the board XZ plane keeps heading responsive when pitch
    // or roll changes at the same time.
    return std::atan2(optical_axis[0], optical_axis[2]) * 180.0 / CV_PI;
}

double opticalPitchDeg(const cv::Vec3d& optical_axis) {
    const double horizontal = std::hypot(optical_axis[0], optical_axis[2]);
    return std::atan2(optical_axis[1], horizontal) * 180.0 / CV_PI;
}

double imageRollDeg(const cv::Vec3d& image_x_axis) {
    // Board +X is right and +Y is down, so positive image-X slope is clockwise.
    return std::atan2(image_x_axis[1], image_x_axis[0]) * 180.0 / CV_PI;
}

bool estimateCameraPose(const std::vector<cv::Point2f>& corners,
                        const std::vector<cv::Point3f>& object_points,
                        cv::Size image_size,
                        const cv::Mat& calibrated_camera_matrix,
                        const cv::Mat& calibrated_dist_coeffs,
                        CameraPose& pose,
                        bool fisheye) try {
    pose = {};
    if (image_size.width <= 0 || image_size.height <= 0 ||
        corners.size() != object_points.size() || corners.size() < 4) {
        return false;
    }
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const auto& p = corners[i];
        const auto& q = object_points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
            p.x >= image_size.width || p.y >= image_size.height ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) return false;
    }
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    if (!calibrated_camera_matrix.empty()) {
        if (calibrated_camera_matrix.rows != 3 || calibrated_camera_matrix.cols != 3 ||
            calibrated_camera_matrix.channels() != 1) return false;
        calibrated_camera_matrix.convertTo(camera_matrix, CV_64F);
        if (!calibrated_dist_coeffs.empty()) calibrated_dist_coeffs.convertTo(dist_coeffs, CV_64F);
    }
    if (camera_matrix.empty()) {
        if (fisheye) return false;
        camera_matrix = (cv::Mat_<double>(3, 3) << image_size.width, 0.0, image_size.width * 0.5,
                          0.0, image_size.width, image_size.height * 0.5,
                          0.0, 0.0, 1.0);
    }
    if (dist_coeffs.empty()) dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
    if (!validCameraIntrinsics(camera_matrix, dist_coeffs, fisheye)) return false;
    const std::vector<cv::Point3d> precise_board(object_points.begin(),object_points.end());
    const std::vector<cv::Point2d> measured_pixels(corners.begin(),corners.end());
    std::vector<cv::Point2d> pose_corners = measured_pixels;
    // MatExpr assignment (e.g. Mat::zeros below) may reuse its destination.
    // Own this buffer so zeroing PnP distortion cannot also zero fish-eye D.
    cv::Mat pose_dist_coeffs = dist_coeffs.clone();
    if (fisheye) {
        if (calibrated_dist_coeffs.total() != 4 || std::abs(camera_matrix.at<double>(0, 1)) > 1e-9 ||
            !cv::checkRange(camera_matrix) || !cv::checkRange(dist_coeffs) ||
            camera_matrix.at<double>(0, 0) <= 0.0 || camera_matrix.at<double>(1, 1) <= 0.0) return false;
        // P=K produces pinhole pixel coordinates. Fisheye coefficients must
        // never be passed to the pinhole solvePnP/refinement APIs.
        std::vector<cv::Point2d> precise_corners(corners.begin(), corners.end());
        cv::fisheye::undistortPoints(precise_corners, precise_corners, camera_matrix, dist_coeffs,
                                     cv::Mat::eye(3, 3, CV_64F), camera_matrix);
        std::vector<cv::Point3d> rays;
        for (const auto& p : precise_corners) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) >= 1e5 || std::abs(p.y) >= 1e5)
                return false;
            rays.emplace_back((p.x - camera_matrix.at<double>(0, 2)) / camera_matrix.at<double>(0, 0),
                              (p.y - camera_matrix.at<double>(1, 2)) / camera_matrix.at<double>(1, 1), 1.0);
        }
        std::vector<cv::Point2d> inverse_check;
        cv::fisheye::projectPoints(rays, inverse_check, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), camera_matrix, dist_coeffs);
        for (std::size_t i = 0; i < corners.size(); ++i)
            if (!std::isfinite(inverse_check[i].x) || !std::isfinite(inverse_check[i].y) ||
                cv::norm(inverse_check[i] - cv::Point2d(corners[i])) > 0.05) return false;
        pose_corners = precise_corners;
        for (const auto& p : pose_corners) {
            // OpenCV may return the finite -1e6 sentinel on inversion failure.
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                std::abs(p.x) >= 1e5F || std::abs(p.y) >= 1e5F) return false;
        }
        pose_dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
    }
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    // SQPNP supplies an initial pose, refined on undistorted points below.
    // A nearly frontal planar board can still have depth/tilt uncertainty:
    // low reprojection RMS does not by itself bound real-world pose error.
    bool pnp_ok = cv::solvePnP(precise_board, pose_corners, camera_matrix, pose_dist_coeffs,
                                rvec, tvec, false, cv::SOLVEPNP_SQPNP);
    if (!pnp_ok) {
        // Fall back when the initial solver cannot find a pose.
        pnp_ok = cv::solvePnP(precise_board, pose_corners, camera_matrix, pose_dist_coeffs,
                               rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }
    if (!pnp_ok) {
        pose = {};
        return false;
    }
    // Refine the PnP initial estimate; fisheye pixels are optimized separately below.
    const cv::Vec3d initial_rvec=rvec, initial_tvec=tvec;
    try {
        cv::solvePnPRefineLM(precise_board, pose_corners, camera_matrix, pose_dist_coeffs,
                             rvec, tvec,
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 1e-7));
    } catch (const cv::Exception&) {
        rvec=initial_rvec;
        tvec=initial_tvec;
    }
    if (fisheye) {
        // A failed refinement retains the PnP result; accept a candidate only
        // if its true pixel error is finite and no larger than the initial fit.
        try {
            auto residual=cv::makePtr<FisheyePoseResidual>(precise_board,measured_pixels,camera_matrix,dist_coeffs);
            cv::Mat parameters=(cv::Mat_<double>(6,1)<<rvec[0],rvec[1],rvec[2],tvec[0],tvec[1],tvec[2]);
            cv::Mat before,after;
            if (residual->compute(parameters,before,cv::noArray())) {
                cv::LMSolver::create(residual,40,1e-10)->run(parameters);
                if (residual->compute(parameters,after,cv::noArray()) && cv::norm(after)<=cv::norm(before)) {
                    rvec={parameters.at<double>(0),parameters.at<double>(1),parameters.at<double>(2)};
                    tvec={parameters.at<double>(3),parameters.at<double>(4),parameters.at<double>(5)};
                }
            }
        } catch (const cv::Exception&) {
            // Initial PnP pose remains available and is validated below.
        }
    }
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    if (!cv::checkRange(cv::Mat(rvec)) || !cv::checkRange(cv::Mat(tvec))) return false;
    for (const auto& p : object_points) {
        const double depth = rotation.at<double>(2, 0) * p.x + rotation.at<double>(2, 1) * p.y +
                             rotation.at<double>(2, 2) * p.z + tvec[2];
        if (!std::isfinite(depth) || depth <= 0.0) return false;
    }
    std::vector<cv::Point2d> projected;
    if (fisheye) {
        cv::fisheye::projectPoints(precise_board, projected, rvec, tvec, camera_matrix, dist_coeffs);
    } else {
        cv::projectPoints(precise_board, rvec, tvec, camera_matrix, dist_coeffs, projected);
    }
    pose.rms_px = cv::norm(projected, measured_pixels, cv::NORM_L2) / std::sqrt(static_cast<double>(corners.size()));
    if (!std::isfinite(pose.rms_px)) return false;
    pose.calibrated = !calibrated_camera_matrix.empty();
    pose.rvec = rvec;
    pose.tvec = tvec;
    const cv::Mat camera_to_board = rotation.t();
    pose.x_axis = {camera_to_board.at<double>(0, 0), camera_to_board.at<double>(1, 0), camera_to_board.at<double>(2, 0)};
    pose.y_axis = {camera_to_board.at<double>(0, 1), camera_to_board.at<double>(1, 1), camera_to_board.at<double>(2, 1)};
    pose.z_axis = {camera_to_board.at<double>(0, 2), camera_to_board.at<double>(1, 2), camera_to_board.at<double>(2, 2)};
    pose.pitch_deg = opticalPitchDeg(pose.z_axis);
    pose.yaw_deg = opticalHeadingDeg(pose.z_axis);
    pose.roll_deg = imageRollDeg(pose.x_axis);
    pose.valid = true;
    return true;
} catch (const cv::Exception&) {
    pose = {};
    return false;
}
