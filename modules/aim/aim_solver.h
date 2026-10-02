
#pragma once

#include <Eigen/Dense>

namespace autoaim::aim {

struct AimConfig {
  double latency_s = 0.05;
  double gimbal_lag_s = 0.03;
  double flight_coeff = 1.0;
  double pitch_offset_deg = 0.0;
  bool enable_ballistic = true;
  double gravity = 9.8;
  double min_bullet_speed = 1.0;

  double max_v_yaw = 10.0;
  double max_lead_s = 0.6;
  double min_aim_dist = 0.5;
  double max_aim_dist = 30.0;

  double max_jump_deg = 25.0;
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

  AimResult solveFromMeasurement(const Eigen::Vector3d & armor_pos_gimbal,
                                 double bullet_speed) const;

  AimResult solveFromState(const Eigen::VectorXd & target_state,
                           double bullet_speed) const;

  static double ballisticPitch(double d, double h, double v, double g);

  const AimConfig & config() const { return cfg_; }

  void setConfig(const AimConfig & c) { cfg_ = c; }

 private:
  AimResult finish(const Eigen::Vector3d & p, double bullet_speed,
                   double t_lead) const;

  AimConfig cfg_;
};

}
