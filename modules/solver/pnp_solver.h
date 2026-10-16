#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <array>
#include <vector>
#include "modules/detect/armor.h"
namespace autoaim::solver {
struct ArmorGeometry {
  float small_width = 132.0f;
  float small_height = 57.0f;
  float large_width = 223.0f;
  float large_height = 57.0f;
};

// PnP 降自由度优化（教程 §5A.4）。
//
// 【为什么要降】标准 solvePnP 同时估 6 个自由度。角点有像素误差时，
// 平移的误差会"泄漏"到 yaw、yaw 的误差又会反过来污染平移——4 个量互相分摊。
// 而装甲板的 pitch / roll 其实是机械常量（RM 制作规范决定），
// 于是把这两个固定住，只让最敏感的 yaw 自由：1D 搜索空间几乎不可能发散。
//
// 【为什么必须在世界系做】教程的 R(yaw) = Rz(yaw)·Ry(pitch)·Rx(roll) 是
// **世界系**约定：装甲板模型系的 +x 是法线、+z 是竖直，yaw 就是靶板绕世界
// 竖直轴的转角。放进相机系做会串味 —— 相机系里"正对相机"对应的姿态
// 根本不是 yaw=0，可见角窗口、搜索中心、输出语义全都要跟着错。
// 所以这里接收一个 R_cam2world，把候选姿态搬回相机系再重投影。
struct YawFitConfig {
  bool enable = true;

  // 装甲板自身的俯仰/翻滚（机械常量）。
  // RM 标准装甲板是 15° 仰角；本赛题靶板是否同样倾斜由现场实测确认。
  double armor_pitch_deg = 15.0;
  double armor_roll_deg = 0.0;

  // yaw 搜索窗口：以 PnP 解出的 yaw 为中心，左右各展开这么多度。
  // 要给"装甲板侧对 → 转过去"留余量，太窄会锁死在初值附近。
  double search_extend_deg = 60.0;

  // 可见角约束（§5A.4"单板观测"）：装甲板法线与视线的夹角 z_to_v 必须落在
  //   [π − must_not_see, π − must_see + angle_between_armors]  与
  //   [π + must_see − angle_between_armors, π + must_not_see]
  // 共同决定的窗口内。z_to_v ≈ π 表示靶板正对；偏离 π 越多越侧对。
  // 落在窗口外的候选直接加惩罚，避免优化跑到"背面"去。
  bool use_visibility_constraint = true;
  double must_see_angle_deg = 20.0;
  double must_not_see_angle_deg = 30.0;
  double angle_between_armors_deg = 90.0;  // 2π / 装甲板总数，四板车 = 90°

  int grid_samples = 41;      // 先粗扫一遍，避开三分法对单峰性的依赖
  int trisection_iters = 40;  // 再在最低点两侧二分细化
};

class PnPSolver {
 public:
  PnPSolver(const std::array<double, 9> & camera_matrix,
            const std::vector<double> & distortion_coefficients,
            const ArmorGeometry & geometry = ArmorGeometry{});

  bool solvePnP(const detect::Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const;
  double reprojectionError(const detect::Armor & armor, const cv::Mat & rvec,
                           const cv::Mat & tvec) const;
  float calculateDistanceToCenter(const cv::Point2f & image_point) const;

  // 固定 pitch/roll，用一维搜索求最优 yaw（世界系）。
  //
  // 【为什么要自己拿 PnP 的两个解】装甲板是一块平面（模型点都在 x=0 平面上），
  // 四点 PnP 存在"法线朝前 / 朝后"的两解歧义 —— 而且**两个解的旋转和平移都不同**，
  // 只把 yaw 加 π 而沿用同一份 tvec 是错的（重投影根本对不上）。
  // 所以这里用 solvePnPGeneric(SOLVEPNP_IPPE) 把平板的两个完整解都取出来，
  // 对每个解各自的 tvec 做一次 yaw 一维搜索，用可见角约束 + 重投影代价选优。
  // 被选中的那一支的 tvec 会一并返回 —— **位置必须和姿态取自同一个解**。
  //
  // R_cam2world = R_gimbal2world(云台姿态)·R_cam2gimbal(外参)。
  // 返回 false 表示搜索失败（此时调用方应退回普通 solvePnP 的结果）。
  //
  // yaw_reference_rad / has_reference：可选的先验。平面靶板在接近正对时
  // **两个分支的残差都接近 0**（真歧义），只看代价会在相邻帧之间来回翻，
  // 表现为瞄准 yaw 在几帧内跳几十度。给一个参考（跟踪器当前的 yaw 估计，
  // 或上一次被接受的精修结果）之后，代价相近时就选更靠近参考的那一支。
  bool fitYawSingle(const detect::Armor & armor,
                    const Eigen::Matrix3d & R_cam2world, double & yaw_out_rad,
                    cv::Mat & tvec_out, double * cost_out = nullptr,
                    double yaw_reference_rad = 0.0,
                    bool has_reference = false) const;

  // 双板联合：两块装甲板的 yaw 相差 angle_between_armors，用同一次搜索
  // 同时吃两板的角点，进一步压低 yaw 方差。
  bool fitYawDouble(const detect::Armor & armor_a, const cv::Mat & tvec_a,
                    const detect::Armor & armor_b, const cv::Mat & tvec_b,
                    const Eigen::Matrix3d & R_cam2world, double yaw_a_init_rad,
                    double & yaw_a_out_rad, double * cost_out = nullptr) const;

  const YawFitConfig & yaw_fit_config() const { return yaw_fit_; }

  void setYawFitConfig(const YawFitConfig & c) { yaw_fit_ = c; }

  const ArmorGeometry & geometry() const { return geometry_; }

 private:
  const std::vector<cv::Point3f> & objectPointsFor(detect::ArmorType type) const;

  // 候选 yaw 对应的"模型→相机"旋转（pitch/roll 固定）。
  Eigen::Matrix3d modelToCamera(double yaw_rad,
                                const Eigen::Matrix3d & R_cam2world) const;

  // 可见角约束：装甲板朝向云台的那一侧是否正对视线（教程 §5A.4）。
  bool visibleFromGimbal(const Eigen::Matrix3d & R_cam2world,
                         const Eigen::Vector3d & los_world, double yaw_rad) const;

  // 给定旋转，**闭式解出最优平移**。
  //
  // 这是本文件里最关键的一个小技巧：投影方程
  //     (Q+t)_x − a·(Q+t)_z = 0 ,  (Q+t)_y − b·(Q+t)_z = 0
  // （a=(u−cx)/fx、b=(v−cy)/fy、Q = R_model2cam·P_i）
  // 把 (Q+t) 展开之后，**对 t 是线性的**：
  //     t_x − a·t_z = a·Q_z − Q_x ,  t_y − b·t_z = b·Q_z − Q_y
  // 于是"已知姿态求平移"退化成一个 8×3 的线性最小二乘，闭式可解。
  //
  // 为什么非要它不可：装甲板是平面，四点的 PnP 有两个解，而且**两个解的
  // 姿态和平移都不一样**（不是简单地差一个 π）。如果沿用 PnP 给的那一份 tvec
  // 去搜 yaw，一旦 PnP 挑错了支，怎么搜都对不上（实测残差 25 px）。
  // 每个候选 yaw 都重解一次 tvec，就等于把"姿态 + 平移"一起优化了，
  // 平面歧义自然消失。
  bool solveTranslationForYaw(const std::vector<cv::Point3f> & obj,
                              const std::vector<cv::Point2f> & img,
                              const Eigen::Matrix3d & R_cam2world, double yaw_rad,
                              Eigen::Vector3d & t_out) const;

  // 以 center 为中心、±search_extend 为窗口做一次"粗扫 + 三分细化"。
  // 判据是"候选 yaw + 其最优平移"的总重投影误差。
  bool fitYawAround(const std::vector<cv::Point3f> & obj,
                    const std::vector<cv::Point2f> & img,
                    const Eigen::Matrix3d & R_cam2world,
                    const Eigen::Vector3d & los_world, double center_rad,
                    double & yaw_out_rad, Eigen::Vector3d & t_out,
                    double & cost_out) const;

  // 候选姿态下的重投影 RMS（内部先解最优平移）+ 可见角惩罚（1e6）。
  double costAtYaw(const std::vector<cv::Point3f> & obj,
                   const std::vector<cv::Point2f> & img,
                   const Eigen::Matrix3d & R_cam2world,
                   const Eigen::Vector3d & los_world, double yaw_rad,
                   Eigen::Vector3d * t_out = nullptr) const;

  // 精修结果的最大可接受重投影 RMS（像素）。超过它说明本窗口的解不可信
  // （典型场景：平面 PnP 挑错了支），调用方应退回普通 solvePnP。
  static constexpr double kMaxAcceptableCostPx = 2.0;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  ArmorGeometry geometry_;
  YawFitConfig yaw_fit_;

  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}
