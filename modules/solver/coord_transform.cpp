#include "modules/solver/coord_transform.h"
#include "modules/common/units.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
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

Eigen::Matrix3d R_gimbal2world(const Attitude & att) {
  // 约定与 tools/calib_extrinsic.py::r_gimbal_rad 一致：
  //   pitch 配置为正值代表"抬头"，而绕云台 +y 转 +φ 是低头，故取 -pitch。
  return Eigen::AngleAxisd(att.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(-att.pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(att.roll_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d R_cam2world(const ExtrinsicConfig & cfg, const Attitude & att) {
  return R_gimbal2world(att) * R_cam2gimbal(cfg);
}

Eigen::Quaterniond attitudeToQuat(const Attitude & att) {
  return Eigen::Quaterniond(R_gimbal2world(att)).normalized();
}

Attitude attitudeFromQuat(const Eigen::Quaterniond & q) {
  // 与 R_gimbal2world 的构造（Rz(yaw)·Ry(-pitch)·Rx(roll)）互为逆运算：
  // 展开后 R(2,0) = sin(pitch)、R(2,1) = cos(pitch)sin(roll)、
  // R(2,2) = cos(pitch)cos(roll)、R(1,0)/R(0,0) = sin(yaw)/cos(yaw)。
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  Attitude a;
  a.yaw_rad = std::atan2(R(1, 0), R(0, 0));
  a.pitch_rad = std::asin(std::clamp(R(2, 0), -1.0, 1.0));
  a.roll_rad = std::atan2(R(2, 1), R(2, 2));
  return a;
}

Eigen::Vector3d gimbalZeroToWorld(const Eigen::Vector3d & p_gimbal_zero,
                                  const Attitude & att) {
  return R_gimbal2world(att) * p_gimbal_zero;
}

Eigen::Vector3d cameraToWorld(const cv::Mat & tvec, const ExtrinsicConfig & cfg,
                              const Attitude & att) {
  return gimbalZeroToWorld(cameraToGimbal(tvec, cfg), att);
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

Eigen::Quaterniond rvecToWorldQuat(const cv::Mat & rvec, const ExtrinsicConfig & cfg,
                                   const Attitude & att) {
  cv::Mat R_cam;
  cv::Rodrigues(rvec, R_cam);

  Eigen::Matrix3d R_cam_eig;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R_cam_eig(r, c) = R_cam.at<double>(r, c);
    }
  }

  const Eigen::Matrix3d R_world = R_gimbal2world(att) * R_cam2gimbal(cfg) * R_cam_eig;
  return Eigen::Quaterniond(R_world).normalized();
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
