#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace autoaim::solver {

struct ExtrinsicConfig {
  // 相机安装角，单位【度】。含义：
  //   cam_pitch_deg = -15 表示相机比云台正前方【向下俯】15 度
  //   cam_yaw_deg         相机相对云台正前方的水平偏转，正为向左
  //   cam_roll_deg        相机绕自身光轴的滚转
  double cam_yaw_deg = 0.0;
  double cam_pitch_deg = -15.0;
  double cam_roll_deg = 0.0;

  // 相机光心在【云台系】下的位置，单位 m。
  // 首版可以全填 0：5 m 处 0.05 m 的平移误差 ≈ 0.6 度，
  // 比安装角误差小一个量级。
  double cam_x = 0.0;
  double cam_y = 0.0;
  double cam_z = 0.0;
};

// 相机系 → 云台系的旋转矩阵。
//
// 构造方式：先把相机光轴按光学系的约定对齐到云台正前方（见 .cpp 里的 R0），
// 再在云台系下依次施加 Rz(yaw) * Ry(pitch) * Rx(roll)。
//
// 返回 3x3 正交旋转矩阵 R，满足 p_gimbal = R * p_cam。
// 调一次就会重算一遍（内部不做缓存），热路径上别每帧都调 ——
// 上层应该只在配置变化时调一次。
Eigen::Matrix3d R_cam2gimbal(const ExtrinsicConfig & cfg = {});

// 相机系下的位置（PnP 的 tvec，单位 m）→ 云台系
//
// tvec PnP 输出的平移向量（3x1 cv::Mat 或 3 元素），单位米
// 返回云台系下的三维坐标，单位米。相机光心的平移量（cfg.cam_x/y/z）
// 也在这里一并补偿掉。
Eigen::Vector3d cameraToGimbal(const cv::Mat & tvec,
                               const ExtrinsicConfig & cfg = {});

// 相机系下的朝向（PnP 的 rvec，Rodrigues）→ 云台系下的四元数
//
// rvec PnP 输出的旋转向量（Rodrigues 形式），3x1
// 返回云台系下的姿态四元数，直接可以塞进 ArmorMeasurement::orientation。
// 内部先 Rodrigues 转成旋转矩阵，再左乘 R_cam2gimbal。
Eigen::Quaterniond rvecToGimbalQuat(const cv::Mat & rvec,
                                    const ExtrinsicConfig & cfg = {});

// 把四元数转成绕竖直轴的偏航角（弧度）。
//
// 对于一块立在水平转轴上的装甲板，其法线水平，姿态四元数里
// 绕竖直轴的转角就是这块板"面朝哪"。这个值喂给跟踪器当观测量。
//
// q 云台系下的姿态四元数
// 返回 yaw 角（弧度），范围 ±π
//
// 注意 Eigen 的 eulerAngles(2,1,0) 返回的是 (yaw, pitch, roll)，
// yaw 在索引 [0]。写反了跟踪器会静默发散。
//
// ⚠️ 实现上【不能】用 eulerAngles —— 实测 |yaw| > π 时它会跳到
//    另一组合法分支返回 yaw - π。必须用 atan2(R(1,0), R(0,0))。
//    漏掉的现象是"靶板转到背面就丢目标"（回归测试 test_coord 用例 7b）。
double yawFromQuat(const Eigen::Quaterniond & q);

// 云台系下的方向向量 → 云台 yaw/pitch（弧度）。
// 约定：yaw = atan2(左, 前)，左为正；pitch = atan2(上, 水平距离)，上为正。
//
// p 云台系下的三维位置（或方向），单位米
// 返回 Vector2d(yaw_rad, pitch_rad)，就是协议里要发给下位机的两个角。
// 注意这里是【位置】不是单位向量 —— 高度用 p.z，水平距离用 hypot(p.x, p.y)。
Eigen::Vector2d yawPitchFromGimbalPoint(const Eigen::Vector3d & p);

// 上式的逆：云台角 → 单位方向向量。测试里做往返验证用。
//
// yaw_rad,pitch_rad 云台角（弧度）
// 返回云台系下的单位方向向量（模长 1）。
// 用法示例：验证 yawPitchFromGimbalPoint(gimbalDirFromAngles(y,p)) 能还原回 (y,p)。
Eigen::Vector3d gimbalDirFromAngles(double yaw_rad, double pitch_rad);

}  // namespace autoaim::solver
