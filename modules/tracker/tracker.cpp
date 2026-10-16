
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
  }
  return "?";
}

const char * Tracker::modelStateStr() const {
  switch (model_state_) {
    case ModelState::ARMOR: return "ARMOR";
    case ModelState::BOTH: return "BOTH";
    case ModelState::WHOLE: return "WHOLE";
  }
  return "?";
}

void Tracker::reset() {
  tracker_state = LOST;
  tracked_id.clear();
  target_state.setZero();
  detect_count_ = lost_count_ = 0;
  auto_frames_ = 0;
  rotate_counter_ = diverged_frames_ = 0;
  info_mahalanobis_ = info_position_jump = 0.0;
  last_yaw_ = 0.0;
  active_mode_ = cfg_.mode == RotationMode::AUTO ? RotationMode::CAROUSEL : cfg_.mode;
  model_state_ = active_mode_ == RotationMode::SELF_SPIN ? ModelState::ARMOR
                                                         : ModelState::WHOLE;
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

  // 过程噪声 Q —— Piecewise White Noise（教程 §5B.3）。
  //
  // 物理假设：加速度是分段常数，段内以白噪声随机变化，强度就是方差参数。
  // 对状态 [x, vx] 这一对，白噪声加速度 a（方差 p）在 dt 内造成的协方差是
  //     [[dt⁴/4, dt³/2], [dt³/2, dt²]] · p
  // 与"匀速模型 + 未建模扰动"的推导一致。
  auto u_q = [this](const Eigen::VectorXd & x_p) {
    const double vx = x_p(1), vy = x_p(3), v_yaw = x_p(7);
    const double dx = std::sqrt(vx * vx + vy * vy);
    const double dy = std::abs(v_yaw);

    double x, y, r;
    if (cfg_.use_piecewise_q) {
      // 常数强度：p_coord 管平移（x/y/z 三轴共用），p_yaw 管角速度，p_r 管半径。
      x = cfg_.p_coord;
      y = cfg_.p_yaw;
      r = cfg_.p_r;
    } else {
      // 自适应强度（继承 rm_auto_aim）：转得越快位置 Q 越小，平移越快 yaw Q 越小。
      x = std::exp(-dy) * (cfg_.s2qxyz_max - cfg_.s2qxyz_min) + cfg_.s2qxyz_min;
      y = std::exp(-dx) * (cfg_.s2qyaw_max - cfg_.s2qyaw_min) + cfg_.s2qyaw_min;
      r = cfg_.s2qr;
    }

    const double t = dt_;
    const double q_x_x = std::pow(t, 4) / 4 * x;
    const double q_x_vx = std::pow(t, 3) / 2 * x;
    const double q_vx_vx = std::pow(t, 2) * x;
    const double q_y_y = std::pow(t, 4) / 4 * y;
    const double q_y_vy = std::pow(t, 3) / 2 * y;
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

  // 观测噪声 R —— YPD 球坐标模型（教程 §5B.2），见 makeYpdNoise。
  auto u_r = [this](const Eigen::VectorXd & z) {
    if (cfg_.use_ypd_noise) return makeYpdNoise(z);

    Eigen::Matrix<double, 4, 4> r = Eigen::Matrix<double, 4, 4>::Zero();
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
  auto_frames_ = 0;
  rotate_counter_ = 0;
  diverged_frames_ = 0;
  model_state_ = active_mode_ == RotationMode::SELF_SPIN
                     ? ModelState::ARMOR
                     : (active_mode_ == RotationMode::CAROUSEL ? ModelState::WHOLE
                                                               : ModelState::BOTH);
}

Eigen::Matrix<double, 4, 4> Tracker::makeYpdNoise(const Eigen::VectorXd & z) const {
  Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();

  const Eigen::Vector3d p(z[0], z[1], z[2]);
  const double dist = p.norm();
  const double var_yaw = cfg_.std_dev_tgt_yaw * cfg_.std_dev_tgt_yaw;
  if (!(dist > 1e-3)) {
    R.diagonal() << 1e-2, 1e-2, 1e-2, var_yaw;
    return R;
  }

  // 三个正交基：径向（深度）、方位切向、俯仰切向。
  const Eigen::Vector3d u_r = p / dist;

  Eigen::Vector3d u_azi = Eigen::Vector3d::UnitZ().cross(u_r);  // 水平面内 ⟂ 视线
  if (u_azi.norm() < 1e-6) u_azi = Eigen::Vector3d::UnitX();     // 视线近竖直时退化
  u_azi.normalize();
  const Eigen::Vector3d u_ele = u_r.cross(u_azi).normalized();

  // 角噪声按弧长放大：弧长 = 距离 × 角标准差。
  const double sigma_r = cfg_.std_dev_dist_coeff * dist;
  const double sigma_azi = cfg_.std_dev_azi_angle * dist;
  const double sigma_ele = cfg_.std_dev_ele_angle * dist;

  // 等价于 R_xyz = J · diag(σ_r², σ_azi², σ_ele²) · Jᵀ，
  // 其中 J 的三列就是这三个方向向量。
  R.topLeftCorner<3, 3>() = sigma_r * sigma_r * u_r * u_r.transpose() +
                           sigma_azi * sigma_azi * u_azi * u_azi.transpose() +
                           sigma_ele * sigma_ele * u_ele * u_ele.transpose();
  R(3, 3) = var_yaw;
  return R;
}

ModelState Tracker::selectModelState(double v_yaw) const {
  if (active_mode_ == RotationMode::SELF_SPIN) return ModelState::ARMOR;
  if (active_mode_ == RotationMode::CAROUSEL) return ModelState::WHOLE;

  const double a = std::abs(v_yaw);
  const double h = cfg_.model_hysteresis;

  // 带滞回的双向切换：升降用同一对阈值，退出阈值再往下挪 h，防止在阈值附近抖。
  switch (model_state_) {
    case ModelState::ARMOR:
      return (a >= cfg_.model_low_thres) ? ModelState::BOTH : ModelState::ARMOR;
    case ModelState::BOTH:
      if (a >= cfg_.model_high_thres) return ModelState::WHOLE;
      if (a < cfg_.model_low_thres - h) return ModelState::ARMOR;
      return ModelState::BOTH;
    case ModelState::WHOLE:
      if (a < cfg_.model_high_thres - h) return ModelState::BOTH;
      return ModelState::WHOLE;
  }
  return ModelState::ARMOR;
}

void Tracker::updateModelState() {
  ++auto_frames_;
  const double v_yaw = target_state.size() == kStateDim ? target_state(7) : 0.0;

  // 整车模型的可信度：连续观测到"确实在转"的帧数。
  if (std::abs(v_yaw) >= cfg_.model_low_thres) {
    rotate_counter_++;
  } else {
    rotate_counter_ = 0;
  }

  // 转速发散保护：估算炸到远超物理上限并持续若干帧 → 重置旋转参数、降回装甲板模型。
  if (std::abs(v_yaw) > cfg_.whole_model_v_yaw_limit) {
    if (++diverged_frames_ > cfg_.whole_model_reset_frames) {
      std::printf("[tracker] v_yaw=%.1f 持续 %d 帧发散，重置整车模型\n", v_yaw,
                  diverged_frames_);
      diverged_frames_ = 0;
      rotate_counter_ = 0;
      if (target_state.size() == kStateDim) {
        target_state(8) = 0.0;
        ekf.setState(target_state);
      }
      model_state_ = ModelState::ARMOR;
      return;
    }
  } else {
    diverged_frames_ = 0;
  }

  if (cfg_.mode == RotationMode::AUTO && auto_frames_ < cfg_.auto_grace_frames) {
    return;  // 热身期不动模型
  }
  model_state_ = selectModelState(v_yaw);
}

double Tracker::orientationToYaw(const Eigen::Quaterniond & q) {
  // π 偏置见 TrackerConfig::armor_yaw_offset_rad 的说明。
  double yaw = solver::yawFromQuat(q) + cfg_.armor_yaw_offset_rad;

  yaw = last_yaw_ + shortestAngularDistance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

void Tracker::applyRadiusConstraint() {
  if (target_state.size() != kStateDim) return;

  if (model_state_ == ModelState::ARMOR) {
    // 装甲板模型：半径应恒为 0（原地转头），每帧缩一点把它逼回去，
    // 而不是硬清零 —— 硬清零会丢掉已经积累的旋转信息。
    if (std::abs(target_state(8)) > 1e-4) {
      target_state(8) *= cfg_.r_shrink;
      ekf.setState(target_state);
    }
    return;
  }

  // BOTH / WHOLE：把半径软拉到物理可能的区间里。
  // 装甲板不可能挂在半径 3 m 的圆上，也不可能贴在旋转中心（那是 ARMOR 的假设）。
  // 说明：9 维状态下 BOTH 与 WHOLE 的半径约束相同，区别在于后者会置可信标志
  // （wholeModelTrusted()），供下游决定是否采信旋转参数。若是 4 装甲板整车，
  // 还需要 another_r / dz 两个自由度，本赛题单板靶用不到。
  const double r = target_state(8);
  const double lim = std::clamp(r, cfg_.r_min, cfg_.r_max);
  if (std::abs(lim - r) > 1e-9) {
    target_state(8) += cfg_.r_pull * (lim - r);
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
    const Eigen::Vector3d predicted_position = armorPositionFromState(ekf_prediction);
    const double predicted_yaw = ekf_prediction(6);

    // ── 目标匹配（教程 §5B.4）─────────────────────────────────────────
    // 用马氏距离而不是欧氏距离：深度方向噪声远大于角度方向，欧氏门限要么
    // 宽到放进"已经换目标"的观测，要么窄到把正常深度抖动也拒掉。
    // 马氏距离用预测协方差 S 归一化每个维度，等价于"观测点离误差椭球中心
    // 有几个椭球半径"，零假设下 d² ~ χ²(4)。
    int best_i = -1;
    double best_metric = std::numeric_limits<double>::max();
    double min_position_diff = std::numeric_limits<double>::max();
    double yaw_diff = std::numeric_limits<double>::max();
    double mahalanobis = std::numeric_limits<double>::max();

    for (size_t i = 0; i < armors.size(); ++i) {
      const auto & armor = armors[i];
      if (!armor.number.empty() && armor.number != tracked_id) continue;

      // 这里只做"选谁"的判断，不推进 last_yaw_（那个有状态，只在最终选中的
      // 那块板上调用一次）。
      const double raw_yaw = solver::yawFromQuat(armor.orientation);
      const double dyaw_abs =
          std::abs(shortestAngularDistance(predicted_yaw, raw_yaw));
      const double unwrapped = predicted_yaw + shortestAngularDistance(predicted_yaw, raw_yaw);
      const Eigen::Vector4d z(armor.position.x(), armor.position.y(),
                              armor.position.z(), unwrapped);

      double metric;
      if (cfg_.use_mahalanobis) {
        metric = ekf.mahalanobisSquared(z);
        // 马氏距离只衡量"像不像同一个目标"，yaw 超限仍按跳变处理，
        // 所以两者要同时满足；这里用组合指标挑最近的候选。
        if (dyaw_abs > cfg_.max_match_yaw_diff) metric += 1e3;
      } else {
        metric = (predicted_position - armor.position).norm();
      }

      if (metric < best_metric) {
        best_metric = metric;
        best_i = static_cast<int>(i);
        min_position_diff = (predicted_position - armor.position).norm();
        yaw_diff = dyaw_abs;
        mahalanobis = cfg_.use_mahalanobis ? ekf.mahalanobisSquared(z) : metric;
        tracked_armor = armor;
      }
    }

    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;
    info_mahalanobis_ = mahalanobis;

    if (best_i >= 0) {
      const double gate = cfg_.use_mahalanobis ? cfg_.mahalanobis_gate
                                               : cfg_.max_match_distance;
      const bool gate_ok = (best_metric < gate);

      // ── 装甲板切换检测（教程 §9.4.3）───────────────────────────────
      // 门内但位置跳了一大截 → 是同一辆车换了块板（或旋转靶翻面），
      // 不是"位置估计抖了"。走跳变处理（保留速度、放大协方差），
      // 而不是普通更新（会把跳变当成真实测量把位置拽过去）。
      info_position_jump = min_position_diff;

      if (gate_ok && min_position_diff > cfg_.same_position_threshold) {
        handleArmorJump(tracked_armor);
      } else if (gate_ok) {
        matched = true;
        const double measured_yaw = orientationToYaw(tracked_armor.orientation);
        const Eigen::Vector4d measurement(tracked_armor.position.x(),
                                          tracked_armor.position.y(),
                                          tracked_armor.position.z(), measured_yaw);
        // IESEKF：多迭代几次，在没有新矩阵分解的前提下把线性化点挪到真值附近。
        if (cfg_.iesekf_iters > 1) {
          target_state =
              ekf.updateIterated(measurement, cfg_.iesekf_iters, cfg_.iesekf_eps);
        } else {
          target_state = ekf.update(measurement);
        }
      } else if (yaw_diff > cfg_.max_match_yaw_diff) {
        // 只剩 yaw 对不上：典型的装甲板翻面/换块。
        handleArmorJump(tracked_armor);
      }
    }
  }

  applyRadiusConstraint();
  updateModelState();

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
  }
}

void Tracker::handleArmorJump(const ArmorMeasurement & a) {
  const double yaw = orientationToYaw(a.orientation);

  // 装甲板模型（半径恒 0）：只有朝向变了，直接把 yaw 换成观测值，
  // 再把协方差放大，告诉滤波器"接下来几帧多信观测"。
  if (model_state_ == ModelState::ARMOR) {
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
