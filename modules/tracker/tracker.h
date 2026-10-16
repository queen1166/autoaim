
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
  };

  State state() const { return tracker_state; }

  const char * stateStr() const;

  bool tracking() const {
    return tracker_state == TRACKING || tracker_state == TEMP_LOST;
  }

  const Eigen::VectorXd & targetState() const { return target_state; }

  RotationMode activeMode() const { return active_mode_; }

  // 当前实际生效的双模型状态（见 tracker_config.h 的 ModelState）。
  ModelState modelState() const { return model_state_; }

  const char * modelStateStr() const;

  // 整车模型是否已"看到足够的旋转"而可信（教程 §9.4.4）。
  bool wholeModelTrusted() const { return rotate_counter_ >= cfg_.rotate_counter_min; }

  double radius() const { return target_state.size() == kStateDim ? target_state(8) : 0.0; }

  double vYaw() const { return target_state.size() == kStateDim ? target_state(7) : 0.0; }

  const ArmorMeasurement & trackedArmor() const { return tracked_armor; }

  const std::string & trackedId() const { return tracked_id; }

  double infoPositionDiff() const { return info_position_diff; }

  double infoYawDiff() const { return info_yaw_diff; }

  // 上一次匹配的马氏距离平方与门限（教程 §5B.4），调试用。
  double infoMahalanobis() const { return info_mahalanobis_; }

  double infoPositionJump() const { return info_position_jump; }

  const TrackerConfig & config() const { return cfg_; }

  ExtendedKalmanFilter ekf;

 private:
  void initEkf(const ArmorMeasurement & a);

  void handleArmorJump(const ArmorMeasurement & a);

  double orientationToYaw(const Eigen::Quaterniond & q);

  void applyRadiusConstraint();

  void updateModelState();

  // 双模型切换：按 |v_yaw| 与滞回确定 ARMOR / BOTH / WHOLE。
  ModelState selectModelState(double v_yaw) const;

  // YPD 球坐标观测噪声 → 世界 XYZ 协方差（教程 §5B.2）。
  Eigen::Matrix<double, 4, 4> makeYpdNoise(const Eigen::VectorXd & z_meas) const;

  TrackerConfig cfg_;
  RotationMode active_mode_;
  ModelState model_state_ = ModelState::ARMOR;

  State tracker_state = LOST;
  std::string tracked_id;
  ArmorMeasurement tracked_armor;

  Eigen::VectorXd target_state;

  double dt_ = 0.01;
  double last_yaw_ = 0.0;
  int lost_thres = 30;

  int detect_count_ = 0;
  int lost_count_ = 0;

  int auto_frames_ = 0;

  // 整车模型可信度：连续观测到"在转"的帧数，以及转速发散持续的帧数。
  int rotate_counter_ = 0;
  int diverged_frames_ = 0;

  double info_position_diff = 0.0;
  double info_yaw_diff = 0.0;
  double info_mahalanobis_ = 0.0;
  double info_position_jump = 0.0;
};

}
