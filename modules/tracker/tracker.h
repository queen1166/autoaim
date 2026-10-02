
#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "modules/tracker/armor_measurement.h"
#include "modules/tracker/extended_kalman_filter.h"
#include "modules/tracker/target_state.h"
#include "modules/tracker/tracker_config.h"

namespace autoaim::tracker {

class Tracker {
 public:
  explicit Tracker(const TrackerConfig & cfg = {});

  void init(const std::vector<ArmorMeasurement> & armors);

  void update(const std::vector<ArmorMeasurement> & armors, double dt);

  void reset();

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    CHANGE_TARGET,
  };

  State state() const { return tracker_state; }

  const char * stateStr() const;

  bool tracking() const {
    return tracker_state == TRACKING || tracker_state == TEMP_LOST;
  }

  const Eigen::VectorXd & targetState() const { return target_state; }

  RotationMode activeMode() const { return active_mode_; }

  double radius() const { return target_state.size() == kStateDim ? target_state(8) : 0.0; }

  double vYaw() const { return target_state.size() == kStateDim ? target_state(7) : 0.0; }

  const ArmorMeasurement & trackedArmor() const { return tracked_armor; }

  const std::string & trackedId() const { return tracked_id; }

  double infoPositionDiff() const { return info_position_diff; }

  double infoYawDiff() const { return info_yaw_diff; }

  const TrackerConfig & config() const { return cfg_; }

  ExtendedKalmanFilter ekf;

 private:
  void initEkf(const ArmorMeasurement & a);

  void initChange(const ArmorMeasurement & a);

  void handleArmorJump(const ArmorMeasurement & a);

  double orientationToYaw(const Eigen::Quaterniond & q);

  void applyRadiusConstraint();

  void updateAutoMode();

  TrackerConfig cfg_;
  RotationMode active_mode_;

  State tracker_state = LOST;
  std::string tracked_id;
  ArmorMeasurement tracked_armor;

  Eigen::VectorXd target_state;

  double dt_ = 0.01;
  double last_yaw_ = 0.0;
  int lost_thres = 30;

  int detect_count_ = 0;
  int lost_count_ = 0;
  int change_count_ = 0;

  int auto_frames_ = 0;
  int low_r_frames_ = 0;

  double info_position_diff = 0.0;
  double info_yaw_diff = 0.0;
};

}
