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
  return finish(armor_pos_gimbal, bullet_speed, 0.0);
}

AimResult AimSolver::solveFromState(const Eigen::VectorXd & s,
                                    double bullet_speed) const {
  if (s.size() != tracker::kStateDim) return {};

  const Eigen::Vector3d p_now = tracker::armorPositionFromState(s);
  const double v = std::max(bullet_speed, cfg_.min_bullet_speed);

  const double t_flight = cfg_.flight_coeff * p_now.norm() / v;
  const double t_lead =
      std::min(t_flight + cfg_.latency_s + cfg_.gimbal_lag_s, cfg_.max_lead_s);

  Eigen::VectorXd sp = s;
  sp(0) += s(1) * t_lead;
  sp(2) += s(3) * t_lead;
  sp(4) += s(5) * t_lead;
  sp(6) += std::clamp(s(7), -cfg_.max_v_yaw, cfg_.max_v_yaw) * t_lead;

  const Eigen::Vector3d p_pred = tracker::armorPositionFromState(sp);
  return finish(p_pred, bullet_speed, t_lead);
}

}
