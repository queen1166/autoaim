#pragma once

namespace autoaim::tracker {

// 配置层给的操作模式（命令行 --rotation）。
//   CAROUSEL  装甲板挂在一段半径上公转（"旋转平移靶"的公转分量）
//   SELF_SPIN 原地转头，半径恒为 0
//   AUTO      按实测转速自动在两者间切换
enum class RotationMode {
  CAROUSEL,
  SELF_SPIN,
  AUTO,
};

// 运行期实际生效的模型（教程 §9.4.4 的双模型自适应）。
// 与 RotationMode 的区别：RotationMode 是"你告诉程序打的是什么靶"，
// ModelState 是"滤波器此刻实际在用哪套运动假设"。AUTO 模式下后者会随
// |v_yaw| 变化，带滞回，且可能停在中间的 BOTH。
enum class ModelState {
  ARMOR,  // 装甲板模型：|v_yaw| 小，半径被压到 0
  BOTH,   // 过渡带：半径自由估计
  WHOLE,  // 整车模型：高转速，半径受物理区间约束
};

struct TrackerConfig {
  RotationMode mode = RotationMode::AUTO;

  double max_match_distance = 0.15;
  double max_match_yaw_diff = 1.0;

  int tracking_thres = 5;
  double lost_time_thres = 0.3;

  // AUTO 模式的热身帧数：前这么多帧不切换模型，先让状态收敛。
  int auto_grace_frames = 120;

  double jump_p_inflate = 20.0;

  double r_min = 0.12;
  double r_max = 0.40;
  double r_shrink = 0.95;
  double r_pull = 0.05;

  // ── 过程噪声 Q：Piecewise White Noise（教程 §5B.3）────────────────────
  //
  // 物理假设：目标的加速度是分段常数，段内以白噪声随机变化，强度就是这里的方差。
  //   p_coord：平移加速度白噪声方差。sp25 取 100 → σ_a ≈ 10 m/s²（约 1g）
  //   p_yaw  ：角加速度白噪声方差。sp25 取 400 → σ_α ≈ 20 rad/s²
  //   p_r    ：旋转半径/高差的变化方差，机械上不会突变，取小值
  bool use_piecewise_q = false;
  double p_coord = 100.0;
  double p_yaw = 400.0;
  double p_r = 1.0;

  // 自适应版本的系数（use_piecewise_q = false 时生效，继承自 rm_auto_aim）：
  // 位置噪声 = exp(-|v_yaw|)·(max−min)+min，yaw 噪声 = exp(-|v_xy|)·(max−min)+min
  // —— 目标转得越快位置 Q 越小，平移越快 yaw Q 越小。
  double s2qxyz_max = 0.1;
  double s2qxyz_min = 0.05;
  double s2qyaw_max = 10.0;
  double s2qyaw_min = 5.0;
  double s2qr = 80.0;

  // ── 观测噪声 R：YPD 球坐标模型（教程 §5B.2）──────────────────────────
  //
  // 噪声天然在球坐标（Yaw-Pitch-Distance）下是对角、各通道独立的：
  // 像素抖动最直接影响方位角/俯仰角误差，PnP 的深度不确定度随距离线性增长。
  // 用雅可比把它们旋转到世界 XYZ（这里用径向/切向基向量外积等价实现），
  // 于是 R 是满阵、且每帧随目标位置重算。
  //
  // 取值依据：本赛题 5 m 处靶板，12–16 mm 镜头实测 PnP 距离中位误差 2.7–3.4 cm、
  // p95 7.7–11 cm → 横向角误差 ≈ 0.03/5 ≈ 0.006 rad。教程给的典型值 0.063 rad
  // 对应另一套镜头/像素抖动，直接照抄会过于不信任观测。现场按 --debug-dump
  // 看位置估计跟不跟得上再调。
  bool use_ypd_noise = true;
  double std_dev_azi_angle = 0.02;   // rad，方位角标准差
  double std_dev_ele_angle = 0.02;   // rad，俯仰角标准差
  double std_dev_dist_coeff = 0.05;  // 深度噪声系数（相对距离）
  double std_dev_tgt_yaw = 0.30;     // rad（≈17°），装甲板朝向角标准差

  // 旧的常数对角 R（use_ypd_noise = false 时生效），保留以便对比实验
  double r_xyz_factor = 0.05;
  double r_yaw = 0.02;

  // ── 匹配：马氏距离（教程 §5B.4）──────────────────────────────────────
  //
  // 欧氏距离把所有维度一视同仁，但深度方向的噪声远大于角度方向。
  // 马氏距离用预测协方差 S 归一化每个维度：
  //   d² = (z − z_pred)ᵀ S⁻¹ (z − z_pred)
  // 零假设下 d² 近似服从 χ²(4)，取 95% 置信界 ≈ 9.49 作为门限。
  bool use_mahalanobis = true;
  double mahalanobis_gate = 9.49;

  // ── 更新：IESEKF（教程 §5B.5）────────────────────────────────────────
  int iesekf_iters = 3;
  double iesekf_eps = 1e-4;

  // ── 装甲板朝向约定的 π 偏置 ─────────────────────────────────────────
  //
  // 状态里 yaw 与位置的关系是 xa = xc − r·cos(yaw)，而 yaw 是从装甲板姿态
  // 矩阵的第一列（模型 +x 的方向）取 atan2 得到的。两者的相对朝向差一个 π
  // 就会让 EKF 无法同时满足位置和朝向 —— 现象是 r 被硬撑到 r_max、
  // 瞄准 yaw 摆幅明显偏大（实测偏大约 33%）。
  //
  // 这个偏置取决于靶板实物"哪一面朝外"，现场标定：拿卷尺量旋转半径 r_true，
  // 用 --debug-dump 看收敛后的 r。若 r ≈ r_max 而不是 r_true，就加 ±180°。
  double armor_yaw_offset_rad = 0.0;

  // ── 装甲板切换检测（教程 §9.4.3）─────────────────────────────────────
  // 匹配成功后，若实测位置与预测位置相差超过这个距离，判定为"换了块板"
  // （同一辆车上的装甲板轮换，或旋转靶前后翻转），走跳变处理而不是普通更新。
  double same_position_threshold = 0.2;

  // ── 双模型自适应阈值（教程 §9.4.4）───────────────────────────────────
  double model_low_thres = 0.7;   // |v_yaw| ≥ 该值进入 BOTH
  double model_high_thres = 2.7;  // |v_yaw| ≥ 该值进入 WHOLE
  double model_hysteresis = 0.0;  // 退出阈值 = 进入阈值 − 该值（防阈值附近抖动）

  // 整车模型需要"看到旋转"才可信：连续观测到转速超阈值的帧数。
  int rotate_counter_min = 3;
  // 转速估算明显发散（|v_yaw| 超过该值）持续这么多帧 → 重置整车参数并降回。
  double whole_model_v_yaw_limit = 3.0;
  int whole_model_reset_frames = 30;
};

}
