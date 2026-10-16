#include "modules/aim/aim_solver.h"
#include "modules/common/units.h"

#include <algorithm>
#include <cmath>

#include "modules/solver/coord_transform.h"
#include "modules/tracker/target_state.h"

namespace autoaim::aim {

namespace {
}

double AimSolver::ballisticPitch(double d, double h, double v, double g) {
  if (d <= 1e-6) return 0.0;

  const double k = g * d * d / (2.0 * v * v);
  if (k <= 1e-12) return std::atan2(h, d);

  const double disc = d * d - 4.0 * k * (k + h);
  if (disc < 0.0) {
    return std::atan2(h, d);
  }

  const double u = (d - std::sqrt(disc)) / (2.0 * k);
  return std::atan(u);
}

AimResult AimSolver::finish(const Eigen::Vector3d & p, double bullet_speed,
                            double t_lead) const {
  AimResult out;
  out.aim_point = p;
  out.t_lead_s = t_lead;

  const double dist_check = p.norm();
  if (!p.allFinite() || dist_check < cfg_.min_aim_dist ||
      dist_check > cfg_.max_aim_dist) {
    out.valid = false;
    return out;
  }

  const double v = std::max(bullet_speed, cfg_.min_bullet_speed);
  const double dist = p.norm();
  out.t_flight_s = cfg_.flight_coeff * dist / v;

  const auto yp = solver::yawPitchFromGimbalPoint(p);
  out.yaw_rad = yp.x();

  const double horiz = std::hypot(p.x(), p.y());
  if (cfg_.enable_ballistic) {
    if (cfg_.ballistic_model == BallisticModel::QUADRATIC) {
      out.pitch_rad = ballisticPitch(horiz, p.z(), v, cfg_.gravity);
    } else {
      // 显式重力补偿：弹丸在 fly_time 内下落 0.5·g·t²，
      // 所以枪口要抬高同样的量（见 AimConfig::gravity_drop_sign 的说明）。
      const double drop =
          0.5 * cfg_.gravity * out.t_flight_s * out.t_flight_s * cfg_.gravity_drop_sign;
      out.pitch_rad = std::atan2(p.z() + drop, horiz);
    }
  } else {
    out.pitch_rad = std::atan2(p.z(), horiz);
  }
  out.pitch_rad += cfg_.pitch_offset_deg * kDeg2Rad;

  out.valid = std::isfinite(out.yaw_rad) && std::isfinite(out.pitch_rad);
  return out;
}

double AimSolver::fixedLeadSeconds() const {
  return cfg_.latency_s + cfg_.gimbal_lag_s + cfg_.process_latency_s +
         cfg_.comm_latency_s;
}

AimResult AimSolver::solveFromMeasurement(const Eigen::Vector3d & armor_pos_world,
                                          double bullet_speed) const {
  return finish(armor_pos_world, bullet_speed, 0.0);
}

AimResult AimSolver::solveFromState(const Eigen::VectorXd & s, double bullet_speed,
                                    double extra_lead_s) const {
  if (s.size() != tracker::kStateDim) return {};

  const double v = std::max(bullet_speed, cfg_.min_bullet_speed);
  const double fixed_lead = fixedLeadSeconds() + extra_lead_s;

  // fly_time 与目标位置互相依赖：目标越远 fly_time 越大，前推越远，
  // 前推后的距离又变了。迭代几次收敛（教程 §7.4.2）。
  const Eigen::Vector3d p_now = tracker::armorPositionFromState(s);
  double t_flight = cfg_.flight_coeff * p_now.norm() / v;
  double t_lead = 0.0;
  Eigen::Vector3d p_pred = p_now;

  const int iters = std::max(1, cfg_.fly_time_iters);
  for (int i = 0; i < iters; ++i) {
    t_lead = std::min(t_flight + fixed_lead, cfg_.max_lead_s);

    Eigen::VectorXd sp = s;
    sp(0) += s(1) * t_lead;
    sp(2) += s(3) * t_lead;
    sp(4) += s(5) * t_lead;
    // yaw 角速度也夹取：转盘重捕时 EKF 的速度估计会炸（实测冲到 55 rad/s），
    // 不夹的话会外推到离谱的角度。
    sp(6) += std::clamp(s(7), -cfg_.max_v_yaw, cfg_.max_v_yaw) * t_lead;

    p_pred = tracker::armorPositionFromState(sp);
    const double new_t_flight = cfg_.flight_coeff * p_pred.norm() / v;
    if (std::abs(new_t_flight - t_flight) < 1e-6) {
      t_flight = new_t_flight;
      break;
    }
    t_flight = new_t_flight;
  }

  return finish(p_pred, bullet_speed, t_lead);
}

}
