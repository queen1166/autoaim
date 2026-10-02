#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace autoaim::solver {

struct ExtrinsicConfig {
  double cam_yaw_deg = 0.0;
  double cam_pitch_deg = -15.0;
  double cam_roll_deg = 0.0;

  double cam_x = 0.0;
  double cam_y = 0.0;
  double cam_z = 0.0;
};

Eigen::Matrix3d R_cam2gimbal(const ExtrinsicConfig & cfg = {});

Eigen::Vector3d cameraToGimbal(const cv::Mat & tvec,
                               const ExtrinsicConfig & cfg = {});

Eigen::Quaterniond rvecToGimbalQuat(const cv::Mat & rvec,
                                    const ExtrinsicConfig & cfg = {});

double yawFromQuat(const Eigen::Quaterniond & q);

Eigen::Vector2d yawPitchFromGimbalPoint(const Eigen::Vector3d & p);

Eigen::Vector3d gimbalDirFromAngles(double yaw_rad, double pitch_rad);

}
