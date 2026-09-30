// 跟踪器配置。
//
// 上游把这些参数散在 tracker_node.cpp 的 declare_parameter 里，
// 把 EKF 的六个 lambda 也写在节点里（tracker_node.cpp:34-125）。
// 非 ROS 环境没有参数服务器，所以集中到这里。

#pragma once

namespace autoaim::tracker {

// 靶板怎么转 —— 这决定了 EKF 的半径 r 该怎么处理。
//
// ⚠️ 现场才能确定是哪种，所以默认 AUTO。
//    判定方法：开 debug 打印 r。
//      稳定收敛到 0.15~0.5  → CAROUSEL（装甲板在圆周上跑）
//      持续趋近 0 且只有 yaw 在匀速变 → SELF_SPIN（位置不动，只是面朝方向在转）
//    或者更直接：站侧面看，位置在动就是转盘，位置不动就是自旋。
enum class RotationMode {
  // 装甲板绕一个中心公转（像旋转靶/风车）。r 有物理意义，上游模型直接适用。
  CAROUSEL,
  // 装甲板绕自身法线自转，位置固定。真值 r = 0。
  // 上游把 r 硬钳在 [0.12, 0.4]，这种情况下会把 xc 往一个物理上不存在的
  // 圆心拉，导致位置估计抖动、yaw 被牵连、预测全错。
  SELF_SPIN,
  // 前 N 帧按 CAROUSEL 跑，若 r 持续贴着下界就自动切 SELF_SPIN。
  AUTO,
};

struct TrackerConfig {
  // 靶板怎么转。默认 AUTO（前 N 帧按 CAROUSEL 跑，再自动判）。
  // 现场判定方法见上面 enum 的注释。
  RotationMode mode = RotationMode::AUTO;

  // ── 数据关联 ──────────────────────────────────────────────
  // 数据关联 = "这一帧观测到的这块板，还是上一帧那块吗"。
  // 两个条件都满足才算同一块，否则当作新目标走 CHANGE_TARGET。
  //
  // 预测位置与观测位置的差超过它就不算匹配（单位 m）
  double max_match_distance = 0.15;
  // yaw 差超过它就不算匹配（单位 rad）
  double max_match_yaw_diff = 1.0;

  // ── 状态机 ────────────────────────────────────────────────
  int tracking_thres = 5;       // DETECTING → TRACKING 需要连续匹配几帧
  double lost_time_thres = 0.3; // 丢失多久算 LOST（秒），会换算成帧数
  int change_thres = 20;        // CHANGE_TARGET 持续几帧

  // ── AUTO 模式判定 ─────────────────────────────────────────
  int auto_switch_frames = 60;  // 连续多少帧 r 贴下界就切 SELF_SPIN
  int auto_grace_frames = 120;  // 前多少帧不做判定（等滤波器收敛）

  // handleArmorJump 瞬移状态时协方差的放大倍数。
  // 上游只改状态不改协方差，跳变后会连续十几帧过度自信地发散。
  double jump_p_inflate = 20.0;

  // ── 半径约束（改成软约束，见 tracker.cpp）────────────────
  // r 的允许区间。上游是硬钳位（直接夹到边界），这里改成每帧朝它
  // 缓慢拉回 —— 硬钳位会让 r 卡在边界上，污染其他状态量的估计。
  double r_min = 0.12;
  double r_max = 0.40;
  double r_shrink = 0.95;  // SELF_SPIN：每帧乘这个数，向 0 收缩
  double r_pull = 0.05;    // CAROUSEL：每帧朝边界拉回的比例

  // ── EKF 噪声参数（取自上游 tracker_node.cpp:82-120）────────
  //
  // Q 是过程噪声（"我有多不信模型"），R 是观测噪声（"我有多不信测量"）。
  // Q 大 → 更信观测、跟得紧但抖；R 大 → 更信模型、平滑但滞后。
  //
  // s2q* 是 Q 的基准幅度，带 max/min 是因为 Q 是【自适应】的
  // （tracker.cpp 的 u_q）：目标动得越快，噪声放得越大 ——
  // 静止时用一个值，高速旋转时用另一个，用指数衰减在两者间插值。

  // 位置过程噪声幅度。max 对应"位置动得快"的情形。
  double s2qxyz_max = 0.1;
  double s2qxyz_min = 0.05;

  // 角度（yaw）过程噪声幅度。比位置的大 100 倍 ——
  // 因为转盘的 yaw 变化本来就剧烈得多。
  double s2qyaw_max = 10.0;
  double s2qyaw_min = 5.0;

  // 半径的过程噪声幅度。给得很大（80），意思是"允许 r 自由变化" ——
  // 这样才能让 AUTO 模式真的把 r 判出个结果来，而不是被噪声绑死。
  double s2qr = 80.0;

  // 观测噪声是【正比于观测绝对值】的（tracker.cpp 的 u_r）：
  // R_xyz = |r_xyz_factor * 测量位置|，远处测得不准，协方差就该大。
  // 这就是"远距离误差大"的建模方式。
  double r_xyz_factor = 0.05;

  // yaw 的观测噪声，固定值（不分远近，因为角度误差和距离关系不大）。
  double r_yaw = 0.02;
};

}  // namespace autoaim::tracker
