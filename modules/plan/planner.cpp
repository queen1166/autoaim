#include "modules/plan/planner.h"

#include <algorithm>
#include <cmath>

#include "modules/common/units.h"
#include "modules/tracker/target_state.h"

namespace autoaim::plan {

namespace {
inline double shortestAngularDistance(double from, double to) {
  return std::remainder(to - from, kTwoPi);
}
}  // namespace

Planner::Planner(const PlannerConfig &cfg) { setConfig(cfg); }

void Planner::setConfig(const PlannerConfig &cfg) {
  cfg_ = cfg;
  yaw_mpc_.setup(cfg_.yaw_mpc);
  pitch_mpc_.setup(cfg_.pitch_mpc);
}

Eigen::VectorXd Planner::extrapolate(const Eigen::VectorXd &s, double t) const {
  if (s.size() != tracker::kStateDim) return Eigen::VectorXd::Zero(tracker::kStateDim);

  Eigen::VectorXd sp = s;
  sp(0) += s(1) * t;
  sp(2) += s(3) * t;
  sp(4) += s(5) * t;
  // yaw 角速度要夹取：重捕瞬间 EKF 会给出爆炸的转速，不夹的话参考轨迹会飞出去。
  sp(6) += std::clamp(s(7), -cfg_.max_ref_vel_rad, cfg_.max_ref_vel_rad) * t;
  return sp;
}

void Planner::buildReference(const Eigen::VectorXd &state, double bullet_speed,
                             double t_start_s, int horizon, Eigen::MatrixXd &ref_yaw,
                             Eigen::MatrixXd &ref_pitch) const {
  const double dt = cfg_.yaw_mpc.dt;

  ref_yaw.setZero(horizon, 2);
  ref_pitch.setZero(horizon, 2);

  double prev_yaw = 0.0;
  double prev_pitch = 0.0;

  for (int k = 0; k < horizon; ++k) {
    const double t = t_start_s + k * dt;
    const Eigen::VectorXd sp = extrapolate(state, t);
    const Eigen::Vector3d p = tracker::armorPositionFromState(sp);

    // 每一步都独立跑一次弹道解算：窗口里距离/高低差在变，
    // 整条轨迹共用一个俯仰补偿是不对的（远处需要的下坠补偿明显更大）。
    const aim::AimResult a = aim_.solveFromMeasurement(p, bullet_speed);
    if (!a.valid) {
      // 保持上一步，避免参考轨迹里冒出 NaN。
      ref_yaw(k, 0) = (k == 0) ? 0.0 : ref_yaw(k - 1, 0);
      ref_pitch(k, 0) = (k == 0) ? 0.0 : ref_pitch(k - 1, 0);
      continue;
    }

    // yaw 解卷绕到连续分支，否则跨 ±π 时 MPC 会看到一个 2π 的假跳变。
    const double raw_yaw = a.yaw_rad;
    const double yaw = (k == 0) ? raw_yaw
                                : prev_yaw + shortestAngularDistance(prev_yaw, raw_yaw);

    ref_yaw(k, 0) = yaw;
    ref_pitch(k, 0) = a.pitch_rad;
    prev_yaw = yaw;
    prev_pitch = a.pitch_rad;
  }

  // 参考角速度用前向差分算；最后一步沿用自身（差分退化为 0）。
  for (int k = 0; k < horizon; ++k) {
    const int k2 = std::min(k + 1, horizon - 1);
    const double dy = (k2 == k) ? 0.0 : (ref_yaw(k2, 0) - ref_yaw(k, 0)) / dt;
    const double dp = (k2 == k) ? 0.0 : (ref_pitch(k2, 0) - ref_pitch(k, 0)) / dt;
    ref_yaw(k, 1) = std::clamp(dy, -cfg_.max_ref_vel_rad, cfg_.max_ref_vel_rad);
    ref_pitch(k, 1) = std::clamp(dp, -cfg_.max_ref_vel_rad, cfg_.max_ref_vel_rad);
  }

  (void)prev_pitch;
}

Planner::Output Planner::planDirect(const Eigen::VectorXd &target_state,
                                    double bullet_speed,
                                    const timeutil::DelayChain &delay) {
  Output out;
  out.total_delay_s = delay.total();
  out.process_latency_s = delay.process_latency_s;
  out.fly_time_s = delay.fly_time_s;

  // 把延迟链里"除飞行时间外"的部分交给瞄准解算器（飞行时间由它自己迭代求解）。
  aim::AimConfig ac = aim_.config();
  ac.process_latency_s = delay.process_latency_s;
  ac.comm_latency_s = delay.comm_latency_s;
  aim_.setConfig(ac);

  const aim::AimResult a = aim_.solveFromState(target_state, bullet_speed);
  if (!a.valid) return out;

  out.yaw_rad = a.yaw_rad;
  out.pitch_rad = a.pitch_rad;
  out.valid = true;
  out.used_mpc = false;
  return out;
}

Planner::Output Planner::plan(const Eigen::VectorXd &target_state, double bullet_speed,
                              const timeutil::DelayChain &delay,
                              double gimbal_yaw_rad, double gimbal_pitch_rad) {
  if (!cfg_.enable || !yaw_mpc_.ready() || !pitch_mpc_.ready()) {
    return planDirect(target_state, bullet_speed, delay);
  }
  if (target_state.size() != tracker::kStateDim) return {};

  const int N = cfg_.yaw_mpc.horizon;
  const double dt = cfg_.yaw_mpc.dt;

  // 参考轨迹从"现在"开始铺（见头文件里对教程做法的说明）；
  // 指令取第 k_cmd 步 —— 那一步正好是"延迟链走完"的时刻。
  const int k_cmd = std::clamp(static_cast<int>(std::lround(delay.total() / dt)), 0,
                               N - 1);

  const aim::AimConfig saved = aim_.config();
  aim::AimConfig ac = saved;
  ac.process_latency_s = delay.process_latency_s;
  ac.comm_latency_s = delay.comm_latency_s;
  aim_.setConfig(ac);

  Eigen::MatrixXd ref_yaw, ref_pitch;
  buildReference(target_state, bullet_speed, 0.0, N, ref_yaw, ref_pitch);
  aim_.setConfig(saved);

  if (!ref_yaw.allFinite() || !ref_pitch.allFinite()) {
    return planDirect(target_state, bullet_speed, delay);
  }

  // 把 yaw 参考整体平移整数个 2π，让它和云台当前角度落在同一分支上。
  // 否则云台在 +179°、参考在 −179° 时，MPC 会命令它去转 358°。
  const double shift = std::round((gimbal_yaw_rad - ref_yaw(k_cmd, 0)) / kTwoPi) * kTwoPi;
  ref_yaw.col(0).array() += shift;

  const Mpc::Solution yaw_sol = yaw_mpc_.solve(gimbal_yaw_rad, 0.0, ref_yaw, k_cmd);
  const Mpc::Solution pitch_sol =
      pitch_mpc_.solve(gimbal_pitch_rad, 0.0, ref_pitch, k_cmd);

  if (!yaw_sol.ok || !pitch_sol.ok) {
    return planDirect(target_state, bullet_speed, delay);
  }

  Output out;
  out.yaw_rad = yaw_sol.angle;
  out.pitch_rad = pitch_sol.angle;
  out.yaw_vel_rad = yaw_sol.vel;
  out.pitch_vel_rad = pitch_sol.vel;
  out.yaw_acc_rad = yaw_sol.acc;
  out.pitch_acc_rad = pitch_sol.acc;
  out.mpc_iterations = std::max(yaw_sol.iterations, pitch_sol.iterations);
  out.total_delay_s = delay.total();
  out.process_latency_s = delay.process_latency_s;
  out.fly_time_s = delay.fly_time_s;
  out.valid = true;
  out.used_mpc = true;
  return out;
}

}
