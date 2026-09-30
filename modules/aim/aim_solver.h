// 瞄准层 —— rm_auto_aim 里【没有】这一层，是本次新写的，也是命中率的决定因素。
//
// 上游的输出止于 Target 消息（旋转中心、yaw、v_yaw、半径），
// tracker_node.cpp:333-353 那段循环是纯 RViz 可视化，从没算过瞄准点。
//
// 这里补上「目标位置 → 云台 yaw/pitch」以及提前量。为什么必须有提前量：
//
//   T_total = 曝光 + 图像传输排队 + 检测 + 串口 + 云台响应 + 弹丸飞行
//           ≈ 0.005 + 0.02~0.04 + 小 + 小 + 0.02~0.05 + 5/22 (=0.227)
//           ≈ 0.30 s
//
// 靶若 2 rad/s（约 3 秒一圈），0.3 s 转 0.6 rad ≈ 34°。不补偿必然脱靶。

#pragma once

#include <Eigen/Dense>

namespace autoaim::aim {

struct AimConfig {
  // 从"图像曝光时刻"到"现在"的固定延迟。用云台阶跃响应实测标定。
  double latency_s = 0.05;
  // 云台从收到指令到转到位的时间
  double gimbal_lag_s = 0.03;
  // 飞行时间系数：T_flight = flight_coeff * 距离 / 弹速。默认 1.0
  double flight_coeff = 1.0;
  // 经验俯仰偏置（度）。弹道模型不准时的兜底，只能实弹标定。
  double pitch_offset_deg = 0.0;
  // 是否用物理弹道模型算 pitch。
  // 建议先关掉、用 pitch_offset_deg 打出来，再开物理模型对照；
  // 两者差太多说明反馈帧里的 bullet_speed 没被真正注入（默认恒为 22.0）。
  bool enable_ballistic = true;
  double gravity = 9.8;
  // 弹速下限保护：反馈没注入或异常值时用它兜底
  double min_bullet_speed = 1.0;

  // 外推保护。EKF 收敛过程中 v_yaw 可能瞬间很大，
  // 乘上 0.3s 的提前量能把瞄准点甩到几弧度之外，云台直接飞出去。
  // 实测：转盘模式下 v_yaw 会在检测中断后冲到 55 rad/s（真值 2.0）。
  // 这是 EKF 在测量空窗后重新捕获时的瞬态，不是真实转速。
  // 10 rad/s ≈ 1.6 转/秒，远超任何校内赛靶板，留够了余量。
  double max_v_yaw = 10.0;   // rad/s
  double max_lead_s = 0.6;   // 秒
  // 瞄准点距离的合理区间（米）。超出说明解算炸了。
  double min_aim_dist = 0.5;
  double max_aim_dist = 30.0;

  // 单帧瞄准角最大变化（度）。超过就判为解算异常，由上层保持上一次角度。
  // 实测转盘重捕时 v_yaw 会冲到 55 rad/s，没有这道闸门云台会猛甩。
  double max_jump_deg = 25.0;
};

// 一次瞄准解算的产物。上层拿 yaw/pitch 打包发走，拿 valid 决定要不要用。
struct AimResult {
  double yaw_rad = 0.0;         // 云台 yaw 该指的角度（弧度）
  double pitch_rad = 0.0;       // 云台 pitch 该指的角度（弧度）
  double t_flight_s = 0.0;      // 估计的弹丸飞行时间（秒）
  double t_lead_s = 0.0;        // 总共外推了多久
  Eigen::Vector3d aim_point = Eigen::Vector3d::Zero();  // 云台系下的瞄准点
  bool valid = false;           // false 表示这次解算不可信，上层应保持上次角度
};

class AimSolver {
 public:
  // 构造：只存配置，无状态。这个类所有方法都是 const 的 ——
  // 它不持有任何跨帧记忆，每次调用都是独立的纯计算。
  explicit AimSolver(const AimConfig & cfg = {}) : cfg_(cfg) {}

  // 路径 A —— 无跟踪器（最小闭环用）。
  //
  // 直接用一次 PnP 测得的装甲板位置。没有速度信息，所以【没有提前量】，
  // 只做弹道补偿。对静止靶足够；对旋转靶会系统性落后约 0.3s 对应的角度。
  //
  // 同时用作跟踪器未收敛（LOST/DETECTING）时的兜底。
  //
  // armor_pos_gimbal 装甲板中心的云台系坐标（已做过 coord_transform），单位米
  // bullet_speed     弹速（m/s），来自下位机反馈；异常值会被
  //                  min_bullet_speed 兜底
  // 返回 AimResult。距离超出 [min_aim_dist, max_aim_dist] 时 valid = false。
  AimResult solveFromMeasurement(const Eigen::Vector3d & armor_pos_gimbal,
                                 double bullet_speed) const;

  // 路径 B —— 有跟踪器。外推整个 (t_flight + latency + gimbal_lag)。
  //
  // 这是主力路径：用 9 维状态里的 v_xc/v_yc/v_yaw 把目标推演到
  // "子弹真正到达的时刻"，再算角度。提前量就是这么做出来的。
  //
  // target_state  tracker 的 9 维状态（布局见 tracker/target_state.h）。
  //                维度不对会返回 valid = false，不会崩。
  // bullet_speed  弹速（m/s）
  // 返回 AimResult。v_yaw 超过 max_v_yaw 会被钳位，外推时间超过
  // max_lead_s 也会被截断 —— 都是为了挡住 EKF 重捕时的速度爆炸。
  AimResult solveFromState(const Eigen::VectorXd & target_state,
                           double bullet_speed) const;

  // 弹道：水平距离 d、高度差 h、弹速 v，返回所需的俯仰角（弧度）。
  // 解 h = d*tanθ - g*d²/(2v²cos²θ)，取低伸弹道（小根）。
  //
  // d 水平距离（米）
  // h 目标相对枪口的高度差（米），正 = 目标更高
  // v 弹速（米/秒），必须 > 0
  // g 重力加速度，传 9.8 即可（做成参数是为了测试能换值）
  //
  // 返回俯仰角（弧度），正 = 抬高。解不出时返回 0。
  // static 是因为它不依赖任何配置，测试可以直接调。
  static double ballisticPitch(double d, double h, double v, double g);

  // 当前生效的配置（只读）。
  const AimConfig & config() const { return cfg_; }

  // 换一套配置。用于运行期调整（比如现场调提前量）。
  void setConfig(const AimConfig & c) { cfg_ = c; }

 private:
  // 由云台系下的瞄准点算出最终角度。是上面两个 solve* 的公共收尾：
  // 转 yaw/pitch → 做距离合法性检查 → 组 AimResult。
  //
  // p            云台系下的瞄准点，单位米
  // bullet_speed 弹速，用于算 t_flight 和弹道俯仰
  // t_lead       这次一共外推了多久，原样记进结果的 t_lead_s 供调试
  //
  // 返回 AimResult。距离不在合理区间时 valid = false。
  AimResult finish(const Eigen::Vector3d & p, double bullet_speed,
                   double t_lead) const;

  AimConfig cfg_;
};

}  // namespace autoaim::aim
