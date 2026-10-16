
#pragma once

#include <Eigen/Dense>

namespace autoaim::aim {

// 弹道模型（教程 §7.4.3）。
enum class BallisticModel {
  // 解抛物线方程 a·t² + b·t + c = 0 得到发射角（含重力，精确）。
  QUADRATIC,
  // 直瞄 + 显式重力补偿：瞄点抬高一整个下坠量 0.5·g·t²。
  //
  // ⚠️ 注意符号：教程写的是 `pw(2,0) -= 0.5·g·fly_time²`，"从目标 Z 中扣除"。
  // 但要让弹丸在飞过 t 秒后正好落到目标上，枪口必须**抬高于**目标视线，
  // 也就是瞄点应该加上这个下坠量而不是减掉。减号会让弹着点系统性偏低。
  // 这里取物理上正确的 +，如需复现教程写法请改 gravity_drop_sign。
  GRAVITY_DROP,
};

struct AimConfig {
  // ── 延迟链（教程场景 2/3/6/7）──────────────────────────────────────
  // total_lead = fly_time + latency_s + gimbal_lag_s + process_latency_s
  //              + comm_latency_s
  double latency_s = 0.05;           // 从曝光到发出指令的处理耗时
  double gimbal_lag_s = 0.03;        // 下位机解包 → 云台转到位
  double process_latency_s = 0.0;    // 运行时每帧由 app 写入：now − 图像时间戳
  double comm_latency_s = 0.0;       // 兜底超参（教程的 comm_latency）

  double flight_coeff = 1.0;
  double pitch_offset_deg = 0.0;
  bool enable_ballistic = true;
  BallisticModel ballistic_model = BallisticModel::QUADRATIC;
  double gravity_drop_sign = 1.0;
  double gravity = 9.8;
  double min_bullet_speed = 1.0;

  double max_v_yaw = 10.0;
  double max_lead_s = 0.6;
  double min_aim_dist = 0.5;
  double max_aim_dist = 30.0;

  double max_jump_deg = 25.0;

  // fly_time 与目标位置互相依赖（鸡生蛋）：先用手头位置算一个初值，
  // 再用它外推目标位置、重算 fly_time，迭代几次就收敛。
  // 教程说 2~3 次即可，不迭代也能接受。
  int fly_time_iters = 3;

  // 发弹延迟（教程 §7.4.1）：它**不改变云台指向**，只改变"何时下 fire"。
  // 上位机的做法是把开火对准判定用的瞄准角整体前移这么多秒。
  double fire_lead_s = 0.0;
};

struct AimResult {
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double t_flight_s = 0.0;
  double t_lead_s = 0.0;
  Eigen::Vector3d aim_point = Eigen::Vector3d::Zero();
  bool valid = false;
};

class AimSolver {
 public:
  explicit AimSolver(const AimConfig & cfg = {}) : cfg_(cfg) {}

  // 给定世界系下的一个点，直接算绝对瞄准角（无提前量）。
  AimResult solveFromMeasurement(const Eigen::Vector3d & armor_pos_world,
                                 double bullet_speed) const;

  // 从 EKF 状态算瞄准角，含前向预测。
  //   extra_lead_s 会叠加到延迟链上 —— 用来生成"发弹时刻的瞄准角"（教程 §7.4.1）。
  AimResult solveFromState(const Eigen::VectorXd & target_state, double bullet_speed,
                           double extra_lead_s = 0.0) const;

  // 延迟链里除飞行时间以外的那一段（常数部分），供 Planner 复用。
  double fixedLeadSeconds() const;

  static double ballisticPitch(double d, double h, double v, double g);

  const AimConfig & config() const { return cfg_; }

  void setConfig(const AimConfig & c) { cfg_ = c; }

 private:
  AimResult finish(const Eigen::Vector3d & p, double bullet_speed,
                   double t_lead) const;

  AimConfig cfg_;
};

}
