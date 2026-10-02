
#pragma once

namespace autoaim::aim {

struct AimResult;

struct FireConfig {
  double yaw_tol_deg = 1.5;
  double pitch_tol_deg = 1.5;

  double min_converge_s = 0.10;

  double fire_hold_s = 0.15;

  double min_interval_s = 0.9;

  int max_shots = 50;

  double max_data_age_s = 0.2;

  bool require_tracking = false;
};

class FireDecision {
 public:
  explicit FireDecision(const FireConfig & cfg = {}) : cfg_(cfg) {}

  int update(const AimResult & aim, double gimbal_yaw_rad, double gimbal_pitch_rad,
             bool tracking, double now_s, double data_age_s);

  void reset();

  int shotsRequested() const { return shots_requested_; }

  double lastYawErrDeg() const { return last_yaw_err_deg_; }

  double lastPitchErrDeg() const { return last_pitch_err_deg_; }

  FireConfig & config() { return cfg_; }
  const FireConfig & config() const { return cfg_; }

 private:
  FireConfig cfg_;

  bool converged_ = false;
  double converge_since_ = 0.0;
  double fire_until_ = 0.0;
  double last_fire_end_ = -1e9;
  int shots_requested_ = 0;

  double last_yaw_err_deg_ = 0.0;
  double last_pitch_err_deg_ = 0.0;
};

}
