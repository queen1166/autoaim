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

// 云台当前姿态（弧度），来自下位机反馈 ID 1 的 yaw/pitch/roll。
//
// 【为什么需要它】相机是焊在云台上的，跟着云台一起转。所以
//     p_零位系 = R_cam2gimbal · p_相机
// 只是把目标表达在"云台零位系"（云台 yaw=pitch=0 时那个朝向）里，
// 它随云台转动而转动。协议里下发的 yaw/pitch 是**世界系绝对角**，
// 必须再套一层云台当前朝向才能得到：
//     p_世界系 = R_gimbal2world(姿态) · p_零位系
// 漏掉这一步的现象：云台一旦离开 0° 就不再收敛，来回振荡
// （只有云台恰好停在 0° 附近时才自洽）。
//
// tools/calib_extrinsic.py 标定时用的就是含 R_gimbal 的完整模型，
// 这里必须与它一致，否则标定出来的外参在运行时用不了。
struct Attitude {
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double roll_rad = 0.0;
};

Eigen::Matrix3d R_cam2gimbal(const ExtrinsicConfig & cfg = {});

// 云台零位系 → 世界系：Rz(yaw) · Ry(-pitch) · Rx(roll)
// 与 tools/calib_extrinsic.py 的 r_gimbal_rad() 保持一致（roll=0 时完全等价）。
Eigen::Matrix3d R_gimbal2world(const Attitude & att);

// 相机系 → 世界系 的完整旋转：R_gimbal2world(att) · R_cam2gimbal(cfg)。
// PnP 降自由度优化要在世界系里做（模型法线、可见角、yaw 都是世界系约定），
// 所以那一层需要这个矩阵把候选旋转搬回相机系再重投影。
Eigen::Matrix3d R_cam2world(const ExtrinsicConfig & cfg, const Attitude & att);

Eigen::Quaterniond attitudeToQuat(const Attitude & att);

// attitudeToQuat 的逆运算（ZYX 欧拉角分解）。
// 姿态队列里存的是四元数（Slerp 需要），取出来要还原成 yaw/pitch/roll。
Attitude attitudeFromQuat(const Eigen::Quaterniond & q);

// 相机系 → 云台零位系（不含云台当前朝向）
Eigen::Vector3d cameraToGimbal(const cv::Mat & tvec,
                               const ExtrinsicConfig & cfg = {});

// 云台零位系 → 世界系
Eigen::Vector3d gimbalZeroToWorld(const Eigen::Vector3d & p_gimbal_zero,
                                  const Attitude & att);

// 相机系 → 世界系（= 上两步串联，Planner 与跟踪器使用的坐标）
Eigen::Vector3d cameraToWorld(const cv::Mat & tvec, const ExtrinsicConfig & cfg,
                              const Attitude & att);

Eigen::Quaterniond rvecToGimbalQuat(const cv::Mat & rvec,
                                    const ExtrinsicConfig & cfg = {});

Eigen::Quaterniond rvecToWorldQuat(const cv::Mat & rvec, const ExtrinsicConfig & cfg,
                                   const Attitude & att);

double yawFromQuat(const Eigen::Quaterniond & q);

Eigen::Vector2d yawPitchFromGimbalPoint(const Eigen::Vector3d & p);

Eigen::Vector3d gimbalDirFromAngles(double yaw_rad, double pitch_rad);

}
