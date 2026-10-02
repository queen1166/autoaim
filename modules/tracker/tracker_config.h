
#pragma once

namespace autoaim::tracker {

enum class RotationMode {
  CAROUSEL,
  SELF_SPIN,
  AUTO,
};

struct TrackerConfig {
  RotationMode mode = RotationMode::AUTO;

  double max_match_distance = 0.15;
  double max_match_yaw_diff = 1.0;

  int tracking_thres = 5;
  double lost_time_thres = 0.3;
  int change_thres = 20;

  int auto_switch_frames = 60;
  int auto_grace_frames = 120;

  double jump_p_inflate = 20.0;

  double r_min = 0.12;
  double r_max = 0.40;
  double r_shrink = 0.95;
  double r_pull = 0.05;


  double s2qxyz_max = 0.1;
  double s2qxyz_min = 0.05;

  double s2qyaw_max = 10.0;
  double s2qyaw_min = 5.0;

  double s2qr = 80.0;

  double r_xyz_factor = 0.05;

  double r_yaw = 0.02;
};

}
