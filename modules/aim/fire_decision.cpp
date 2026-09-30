#include "modules/aim/fire_decision.h"

#include <cmath>

#include "modules/aim/aim_solver.h"
#include "modules/common/units.h"

namespace autoaim::aim {

void FireDecision::reset() {
  converged_ = false;
  converge_since_ = 0.0;
  fire_until_ = 0.0;
  last_fire_end_ = -1e9;
  shots_requested_ = 0;
  last_yaw_err_deg_ = 0.0;
  last_pitch_err_deg_ = 0.0;
}

int FireDecision::update(const AimResult & aim, double gimbal_yaw_rad,
                         double gimbal_pitch_rad, bool tracking, double now_s,
                         double data_age_s) {
  // 云台反馈过期 → 不知道云台在哪，绝对不开火。
  // 串口断线时 feedback 会一直返回最后一帧，角度冻住，这里必须挡住。
  if (data_age_s > cfg_.max_data_age_s) {
    converged_ = false;
    return 0;
  }

  if (cfg_.require_tracking && !tracking) {
    converged_ = false;
    return 0;
  }

  if (!aim.valid) {
    converged_ = false;
    return 0;
  }

  if (shots_requested_ >= cfg_.max_shots) return 0;

  // ── 角误差 ──────────────────────────────────────────────────
  // yaw 必须用 remainder 归一化：云台反馈的 yaw 可能在 [-π,π]，
  // 而瞄准角是连续的，直接相减会得到 358° 这种假的大误差。
  const double yaw_err = std::remainder(aim.yaw_rad - gimbal_yaw_rad, kTwoPi);
  const double pitch_err = aim.pitch_rad - gimbal_pitch_rad;

  last_yaw_err_deg_ = yaw_err * kRad2Deg;
  last_pitch_err_deg_ = pitch_err * kRad2Deg;

  const bool aligned = std::abs(last_yaw_err_deg_) < cfg_.yaw_tol_deg &&
                       std::abs(last_pitch_err_deg_) < cfg_.pitch_tol_deg;

  // ── 收敛计时 ────────────────────────────────────────────────
  if (aligned) {
    if (!converged_) {
      converge_since_ = now_s;
      converged_ = true;
    }
  } else {
    converged_ = false;
  }

  // ── 请求开火 ────────────────────────────────────────────────
  // 已经在请求窗口里：继续拉高 fire_flag，让下位机走完供弹流程
  if (now_s < fire_until_) {
    return 1;
  }

  if (!converged_) return 0;
  if (now_s - converge_since_ < cfg_.min_converge_s) return 0;

  // 节流：协议没有发射反馈，只能靠时间间隔
  if (now_s - last_fire_end_ < cfg_.min_interval_s) return 0;

  fire_until_ = now_s + cfg_.fire_hold_s;
  last_fire_end_ = fire_until_;
  shots_requested_++;
  return 1;
}

}  // namespace autoaim::aim
