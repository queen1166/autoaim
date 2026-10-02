#include "modules/solver/coord_transform.h"
#include "modules/common/units.h"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace autoaim::solver {

namespace {
}

Eigen::Matrix3d R_cam2gimbal(const ExtrinsicConfig & cfg) {
  Eigen::Matrix3d R0;
  R0 <<  0,  0, 1,
        -1,  0, 0,
         0, -1, 0;

  const double yaw = cfg.cam_yaw_deg * kDeg2Rad;
  const double pitch = cfg.cam_pitch_deg * kDeg2Rad;
  const double roll = cfg.cam_roll_deg * kDeg2Rad;

  const Eigen::Matrix3d R_mount =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

  return R_mount * R0;
}

Eigen::Vector3d cameraToGimbal(const cv::Mat & tvec, const ExtrinsicConfig & cfg) {
  Eigen::Vector3d p_cam;
  if (tvec.total() == 3) {
    p_cam << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  } else {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector3d cam_origin(cfg.cam_x, cfg.cam_y, cfg.cam_z);

  return R_cam2gimbal(cfg) * p_cam + cam_origin;
}

Eigen::Quaterniond rvecToGimbalQuat(const cv::Mat & rvec, const ExtrinsicConfig & cfg) {
  cv::Mat R_cam;
  cv::Rodrigues(rvec, R_cam);

  Eigen::Matrix3d R_cam_eig;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_cam_eig(r, c) = R_cam.at<double>(r, c);
    }
  }

  const Eigen::Matrix3d R_gimbal = R_cam2gimbal(cfg) * R_cam_eig;
  return Eigen::Quaterniond(R_gimbal).normalized();
}

double yawFromQuat(const Eigen::Quaterniond & q) {
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

Eigen::Vector2d yawPitchFromGimbalPoint(const Eigen::Vector3d & p) {
  const double horiz = std::hypot(p.x(), p.y());
  return {std::atan2(p.y(), p.x()), std::atan2(p.z(), horiz)};
}

Eigen::Vector3d gimbalDirFromAngles(double yaw_rad, double pitch_rad) {
  const double cp = std::cos(pitch_rad);
  return {cp * std::cos(yaw_rad), cp * std::sin(yaw_rad), std::sin(pitch_rad)};
}

}
