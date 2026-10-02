
#include "modules/tracker/tracker.h"
#include "modules/common/units.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "modules/solver/coord_transform.h"

namespace autoaim::tracker {

namespace {
inline double shortestAngularDistance(double from, double to) {
  return std::remainder(to - from, kTwoPi);
}
}

Tracker::Tracker(const TrackerConfig & cfg)
    : cfg_(cfg), active_mode_(cfg.mode == RotationMode::AUTO ? RotationMode::CAROUSEL
                                                            : cfg.mode) {
  target_state = Eigen::VectorXd::Zero(kStateDim);
}

const char * Tracker::stateStr() const {
  switch (tracker_state) {
    case LOST: return "LOST";
    case DETECTING: return "DETECTING";
    case TRACKING: return "TRACKING";
    case TEMP_LOST: return "TEMP_LOST";
    case CHANGE_TARGET: return "CHANGE_TARGET";
  }
  return "?";
}

void Tracker::reset() {
  tracker_state = LOST;
  tracked_id.clear();
  target_state.setZero();
  detect_count_ = lost_count_ = change_count_ = 0;
  auto_frames_ = low_r_frames_ = 0;
  last_yaw_ = 0.0;
  active_mode_ = cfg_.mode == RotationMode::AUTO ? RotationMode::CAROUSEL : cfg_.mode;
}

void Tracker::initEkf(const ArmorMeasurement & a) {

  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) += x(7) * dt_;
    return x_new;
  };

  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd m(9, 9);
    m << 1, dt_, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, dt_, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, dt_, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, dt_, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1;
    return m;
  };

  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    const double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * std::cos(yaw);
    z(1) = yc - r * std::sin(yaw);
    z(2) = x(4);
    z(3) = x(6);
    return z;
  };

  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd m(4, 9);
    const double yaw = x(6), r = x(8);
    m << 1, 0, 0, 0, 0, 0,  r * std::sin(yaw), 0, -std::cos(yaw),
         0, 0, 1, 0, 0, 0, -r * std::cos(yaw), 0, -std::sin(yaw),
         0, 0, 0, 0, 1, 0,  0,                  0,  0,
         0, 0, 0, 0, 0, 0,  1,                  0,  0;
    return m;
  };

  auto u_q = [this](const Eigen::VectorXd & x_p) {
    const double vx = x_p(1), vy = x_p(3), v_yaw = x_p(7);
    const double dx = std::sqrt(vx * vx + vy * vy);
    const double dy = std::abs(v_yaw);

    const double x = std::exp(-dy) * (cfg_.s2qxyz_max - cfg_.s2qxyz_min) + cfg_.s2qxyz_min;
    const double y = std::exp(-dx) * (cfg_.s2qyaw_max - cfg_.s2qyaw_min) + cfg_.s2qyaw_min;

    const double t = dt_, r = cfg_.s2qr;
    const double q_x_x = std::pow(t, 4) / 4 * x;
    const double q_x_vx = std::pow(t, 3) / 2 * x;
    const double q_vx_vx = std::pow(t, 2) * x;
    const double q_y_y = std::pow(t, 4) / 4 * y;
    const double q_y_vy = std::pow(t, 3) / 2 * x;
    const double q_vy_vy = std::pow(t, 2) * y;
    const double q_r = std::pow(t, 4) / 4 * r;

    Eigen::MatrixXd q(9, 9);
    q << q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
         q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
         0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
         0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
         0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
         0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
         0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
         0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
         0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    return q;
  };

  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    const double x = cfg_.r_xyz_factor;
    r.diagonal() << std::abs(x * z[0]), std::abs(x * z[1]), std::abs(x * z[2]),
        cfg_.r_yaw;
    return r;
  };

  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();

  ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  last_yaw_ = 0.0;
  const double yaw = orientationToYaw(a.orientation);

  target_state = Eigen::VectorXd::Zero(kStateDim);

  if (active_mode_ == RotationMode::SELF_SPIN) {
    target_state << a.position.x(), 0, a.position.y(), 0, a.position.z(), 0, yaw, 0, 0.0;
  } else {
    const double r = 0.2;
    target_state << a.position.x() + r * std::cos(yaw), 0,
        a.position.y() + r * std::sin(yaw), 0, a.position.z(), 0, yaw, 0, r;
  }

  ekf.setState(target_state);
}

void Tracker::init(const std::vector<ArmorMeasurement> & armors) {
  if (armors.empty()) return;

  double min_distance = std::numeric_limits<double>::max();
  tracked_armor = armors[0];
  for (const auto & armor : armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEkf(tracked_armor);

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;
  detect_count_ = 0;
  change_count_ = 0;
  auto_frames_ = 0;
  low_r_frames_ = 0;
}

void Tracker::initChange(const ArmorMeasurement & a) {
  initEkf(a);
  tracked_id = a.number;
  tracker_state = DETECTING;
  detect_count_ = 0;
  change_count_ = 0;
}

double Tracker::orientationToYaw(const Eigen::Quaterniond & q) {
  double yaw = solver::yawFromQuat(q);

  yaw = last_yaw_ + shortestAngularDistance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

void Tracker::applyRadiusConstraint() {
  if (target_state.size() != kStateDim) return;

  if (active_mode_ == RotationMode::SELF_SPIN) {
    if (std::abs(target_state(8)) > 1e-4) {
      target_state(8) *= cfg_.r_shrink;
      ekf.setState(target_state);
    }
  } else {
    const double r = target_state(8);
    const double lim = std::clamp(r, cfg_.r_min, cfg_.r_max);
    if (std::abs(lim - r) > 1e-9) {
      target_state(8) += cfg_.r_pull * (lim - r);
      ekf.setState(target_state);
    }
  }
}

void Tracker::updateAutoMode() {
  if (cfg_.mode != RotationMode::AUTO) {
    active_mode_ = cfg_.mode;
    return;
  }
  if (active_mode_ == RotationMode::SELF_SPIN) return;

  ++auto_frames_;
  if (auto_frames_ < cfg_.auto_grace_frames) return;

  if (target_state(8) < cfg_.r_min * 1.5) {
    ++low_r_frames_;
  } else {
    low_r_frames_ = 0;
  }

  if (low_r_frames_ > cfg_.auto_switch_frames) {
    std::printf("[tracker] AUTO: r 持续贴近下界，切换到 SELF_SPIN\n");
    active_mode_ = RotationMode::SELF_SPIN;
    target_state(8) = 0.0;
    ekf.setState(target_state);
  }
}

void Tracker::update(const std::vector<ArmorMeasurement> & armors, double dt) {
  dt_ = std::clamp(dt, 1e-3, 0.2);
  lost_thres = std::clamp(
      static_cast<int>(cfg_.lost_time_thres / dt_), 1, 2000);

  const Eigen::VectorXd ekf_prediction = ekf.predict();
  target_state = ekf_prediction;

  bool matched = false;

  if (!armors.empty()) {
    ArmorMeasurement same_id_armor;
    int same_id_armors_count = 0;
    const Eigen::Vector3d predicted_position = armorPositionFromState(ekf_prediction);
    double min_position_diff = std::numeric_limits<double>::max();
    double diff_min_position_diff = std::numeric_limits<double>::max();
    double yaw_diff = std::numeric_limits<double>::max();
    int diff_count = 0;
    ArmorMeasurement diff_tracked_armor;

    for (const auto & armor : armors) {
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        const double position_diff = (predicted_position - armor.position).norm();
        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          yaw_diff = std::abs(orientationToYaw(armor.orientation) - ekf_prediction(6));
          tracked_armor = armor;
        }
      } else if (tracker_state == CHANGE_TARGET) {
        diff_count += 1;
        const double position_diff = (predicted_position - armor.position).norm();
        if (position_diff < diff_min_position_diff) {
          diff_min_position_diff = position_diff;
          diff_tracked_armor = armor;
        }
      }
    }

    if (diff_count != 0) {
      initChange(diff_tracked_armor);
      return;
    }

    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    if (min_position_diff < cfg_.max_match_distance &&
        yaw_diff < cfg_.max_match_yaw_diff) {
      matched = true;
      const double measured_yaw = orientationToYaw(tracked_armor.orientation);
      const Eigen::Vector4d measurement(tracked_armor.position.x(),
                                        tracked_armor.position.y(),
                                        tracked_armor.position.z(), measured_yaw);
      target_state = ekf.update(measurement);
    } else if (same_id_armors_count == 1 && yaw_diff > cfg_.max_match_yaw_diff) {
      handleArmorJump(same_id_armor);
    }
  }

  applyRadiusConstraint();
  updateAutoMode();

  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > cfg_.tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  } else if (tracker_state == CHANGE_TARGET) {
    if (change_count_ > cfg_.change_thres) {
      tracker_state = TRACKING;
      change_count_ = 0;
    } else {
      change_count_++;
    }
  }
}

void Tracker::handleArmorJump(const ArmorMeasurement & a) {
  const double yaw = orientationToYaw(a.orientation);

  if (active_mode_ == RotationMode::SELF_SPIN) {
    target_state(6) = yaw;
    ekf.setStateInflated(target_state, cfg_.jump_p_inflate);
    return;
  }

  target_state(6) = yaw;

  const Eigen::Vector3d p = a.position;
  const Eigen::Vector3d infer_p = armorPositionFromState(target_state);
  if ((p - infer_p).norm() > cfg_.max_match_distance) {
    const double r = target_state(8);
    target_state(0) = p.x() + r * std::cos(yaw);
    target_state(1) = 0;
    target_state(2) = p.y() + r * std::sin(yaw);
    target_state(3) = 0;
    target_state(4) = p.z();
    target_state(5) = 0;
  }
  ekf.setStateInflated(target_state, cfg_.jump_p_inflate);
}

}
