#include "modules/aim/aim_solver.h"
#include "modules/common/units.h"

#include <algorithm>
#include <cmath>

#include "modules/solver/coord_transform.h"
#include "modules/tracker/target_state.h"

namespace autoaim::aim {

namespace {
}  // namespace

double AimSolver::ballisticPitch(double d, double h, double v, double g) {
  // 弹道方程：h = d*tanθ - g*d²/(2*v²*cos²θ)
  // 用 1/cos²θ = 1 + tan²θ，令 u = tanθ，整理成一元二次：
  //     k*u² - d*u + (k + h) = 0,  其中 k = g*d²/(2v²)
  // 两个根对应高抛和低伸弹道，取小的那个（直射）。
  if (d <= 1e-6) return 0.0;

  const double k = g * d * d / (2.0 * v * v);
  if (k <= 1e-12) return std::atan2(h, d);  // 弹速无穷大 → 直线

  const double disc = d * d - 4.0 * k * (k + h);
  if (disc < 0.0) {
    // 这个距离/弹速组合打不到。退回直瞄，让上层去发现打不中。
    return std::atan2(h, d);
  }

  const double u = (d - std::sqrt(disc)) / (2.0 * k);  // 低伸弹道
  return std::atan(u);
}

AimResult AimSolver::finish(const Eigen::Vector3d & p, double bullet_speed,
                            double t_lead) const {
  AimResult out;
  out.aim_point = p;
  out.t_lead_s = t_lead;

  // 解算炸掉的兜底：距离离谱就当这一帧无效，让上层保持角度
  const double dist_check = p.norm();
  if (!p.allFinite() || dist_check < cfg_.min_aim_dist ||
      dist_check > cfg_.max_aim_dist) {
    out.valid = false;
    return out;
  }

  const double v = std::max(bullet_speed, cfg_.min_bullet_speed);
  const double dist = p.norm();
  out.t_flight_s = cfg_.flight_coeff * dist / v;

  // 云台角。与 coord_transform::yawPitchFromGimbalPoint 同一套约定：
  //   yaw   = atan2(左, 前)
  //   pitch = atan2(上, 水平距离)
  const auto yp = solver::yawPitchFromGimbalPoint(p);
  out.yaw_rad = yp.x();

  const double horiz = std::hypot(p.x(), p.y());
  if (cfg_.enable_ballistic) {
    out.pitch_rad = ballisticPitch(horiz, p.z(), v, cfg_.gravity);
  } else {
    out.pitch_rad = std::atan2(p.z(), horiz);
  }
  out.pitch_rad += cfg_.pitch_offset_deg * kDeg2Rad;

  out.valid = std::isfinite(out.yaw_rad) && std::isfinite(out.pitch_rad);
  return out;
}

AimResult AimSolver::solveFromMeasurement(const Eigen::Vector3d & armor_pos_gimbal,
                                          double bullet_speed) const {
  // 没有速度信息 → 不能外推，t_lead = 0。
  return finish(armor_pos_gimbal, bullet_speed, 0.0);
}

AimResult AimSolver::solveFromState(const Eigen::VectorXd & s,
                                    double bullet_speed) const {
  if (s.size() != tracker::kStateDim) return {};

  const Eigen::Vector3d p_now = tracker::armorPositionFromState(s);
  const double v = std::max(bullet_speed, cfg_.min_bullet_speed);

  // 先用当前距离估飞行时间，再把它和固定延迟一起作为外推量。
  // 距离在 0.3s 内的变化对 t_flight 的影响是二阶小量，一次迭代就够。
  const double t_flight = cfg_.flight_coeff * p_now.norm() / v;
  const double t_lead =
      std::min(t_flight + cfg_.latency_s + cfg_.gimbal_lag_s, cfg_.max_lead_s);

  // ★ 外推【整个 t_lead】，而不是只算 t_flight。
  //   上游 EKF 每帧只外推一个 dt_，这里是要预测弹丸出膛那一刻靶在哪。
  Eigen::VectorXd sp = s;
  sp(0) += s(1) * t_lead;  // xc
  sp(2) += s(3) * t_lead;  // yc
  sp(4) += s(5) * t_lead;  // za
  // v_yaw 先夹住再外推 —— 收敛过程中它可能瞬间很大
  sp(6) += std::clamp(s(7), -cfg_.max_v_yaw, cfg_.max_v_yaw) * t_lead;

  const Eigen::Vector3d p_pred = tracker::armorPositionFromState(sp);
  return finish(p_pred, bullet_speed, t_lead);
}

}  // namespace autoaim::aim
