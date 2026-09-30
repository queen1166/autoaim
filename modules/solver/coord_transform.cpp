#include "modules/solver/coord_transform.h"
#include "modules/common/units.h"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace autoaim::solver {

namespace {
}

Eigen::Matrix3d R_cam2gimbal(const ExtrinsicConfig & cfg) {
  // 名义映射 R0：相机正对前方、无安装角时，光学系 → 云台系。
  //
  //   相机 z(前)  →  云台 +x(前)
  //   相机 x(右)  →  云台 -y   （云台 +y 是"左"，所以相机右 = 云台 -左）
  //   相机 y(下)  →  云台 -z   （云台 +z 是"上"）
  //
  // 写成矩阵就是这三条按列排开。det = +1，是正常旋转。
  Eigen::Matrix3d R0;
  R0 <<  0,  0, 1,
        -1,  0, 0,
         0, -1, 0;

  const double yaw = cfg.cam_yaw_deg * kDeg2Rad;
  const double pitch = cfg.cam_pitch_deg * kDeg2Rad;
  const double roll = cfg.cam_roll_deg * kDeg2Rad;

  // 在云台系下施加安装角。
  //
  // ⚠️ 符号：绕云台 +y（左）轴转 +φ 会让光轴【向下】俯 φ 度。
  //    而配置约定 cam_pitch_deg 负值 = 向下俯，所以这里用 -pitch。
  //    验证：pitch=-15 → Ry(+15) → 光轴 (cos15, 0, -sin15)，确实朝下。
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

  // 相机光心在云台系下的位置。首版通常全 0（见头文件说明）。
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
  // 标准 ZYX 欧拉角的 yaw 提取，等价于上游 tf2 的
  //     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw)
  //
  // ⚠️ 不要用 Eigen 的 q.toRotationMatrix().eulerAngles(2,1,0)(0) 代替。
  //    实测它在 |yaw| > π 时会跳到另一组**同样合法**的欧拉分解分支上，
  //    返回 yaw-π（差一整个 π！）。这是静默错误：不会编译报错、
  //    不会抛异常，只会让跟踪器每跨一次 ±π 就发散一次。
  //
  //    实测数据（绕 Z 轴的纯旋转）：
  //       真值 3.1400 → eulerAngles 给 3.140000   ✓
  //       真值 3.1500 → eulerAngles 给 0.008407   ✗（= 3.15 - π）
  //       真值 3.2000 → eulerAngles 给 0.058407   ✗
  //    而 atan2(R(1,0), R(0,0)) 在全程都对。
  //
  //    这个 bug 如果漏掉，现象是：靶板转到背面时跟踪器突然丢失目标，
  //    正面又恢复 —— 会被误判成检测问题，极难查。tests/test_coord.cpp
  //    用例 7b 把它锁住了。
  const Eigen::Matrix3d R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

Eigen::Vector2d yawPitchFromGimbalPoint(const Eigen::Vector3d & p) {
  // yaw   = atan2(左, 前)，目标在左 → yaw > 0
  // pitch = atan2(上, 水平距离)，目标在上方 → pitch > 0
  const double horiz = std::hypot(p.x(), p.y());
  return {std::atan2(p.y(), p.x()), std::atan2(p.z(), horiz)};
}

Eigen::Vector3d gimbalDirFromAngles(double yaw_rad, double pitch_rad) {
  const double cp = std::cos(pitch_rad);
  return {cp * std::cos(yaw_rad), cp * std::sin(yaw_rad), std::sin(pitch_rad)};
}

}  // namespace autoaim::solver
