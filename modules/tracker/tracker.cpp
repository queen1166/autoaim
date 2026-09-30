// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
//
// 派生自 rm_auto_aim/armor_tracker/src/tracker.cpp
// EKF 的六个 lambda 派生自 rm_auto_aim/armor_tracker/src/tracker_node.cpp:34-125
// （上游把模型写在 ROS 节点里，非 ROS 环境没有参数服务器，搬进 initEkf()）

#include "modules/tracker/tracker.h"
#include "modules/common/units.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "modules/solver/coord_transform.h"

namespace autoaim::tracker {

namespace {
// angles::shortest_angular_distance(from, to) 的等价实现。
// ⚠️ 参数顺序别反：上游语义是"从 from 到 to 的最短角差"。
inline double shortestAngularDistance(double from, double to) {
  return std::remainder(to - from, kTwoPi);
}
}  // namespace

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
  // ── EKF 模型（对应上游 tracker_node.cpp:34-125）──────────────────
  //
  // 状态 [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]
  // 观测 [xa, ya, za, yaw]

  // f —— 匀速积分
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) += x(7) * dt_;
    return x_new;
  };

  // J_f
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

  // h —— 装甲板在圆周上的位置。注意是【减】号。
  // 自旋模式 r→0 时 h 退化成 z=(xc,yc,za,yaw)，即位置固定、朝向自由 —— 正合适。
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    const double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * std::cos(yaw);  // xa
    z(1) = yc - r * std::sin(yaw);  // ya
    z(2) = x(4);                    // za
    z(3) = x(6);                    // yaw
    return z;
  };

  // J_h
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd m(4, 9);
    const double yaw = x(6), r = x(8);
    m << 1, 0, 0, 0, 0, 0,  r * std::sin(yaw), 0, -std::cos(yaw),
         0, 0, 1, 0, 0, 0, -r * std::cos(yaw), 0, -std::sin(yaw),
         0, 0, 0, 0, 1, 0,  0,                  0,  0,
         0, 0, 0, 0, 0, 0,  1,                  0,  0;
    return m;
  };

  // Q —— 自适应过程噪声：目标动得越快，噪声放得越大
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
    // ⚠️ 上游这里用的是 x 而不是 y：
    //      double q_y_vy = pow(t,3)/2*x
    //    从量纲看应该和 q_y_y / q_vy_vy 用同一个 y。这是上游的笔误，
    //    但它是【已知能工作】的行为，非 ROS 移植不该顺手改数值。
    //    保留原样；如果将来要改，必须配合离线回放数据对比再动。
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

  // R —— 量测噪声正比于观测值的绝对值（远处测得不准）
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

  // ── 初值 ─────────────────────────────────────────────────────
  last_yaw_ = 0.0;
  const double yaw = orientationToYaw(a.orientation);

  target_state = Eigen::VectorXd::Zero(kStateDim);

  if (active_mode_ == RotationMode::SELF_SPIN) {
    // 自旋：位置固定，旋转中心【就是】装甲板自己 → r = 0。
    // 上游无条件写 r = 0.2 且把 xc 挪到板子后面 0.2m，这里必须分开。
    target_state << a.position.x(), 0, a.position.y(), 0, a.position.z(), 0, yaw, 0, 0.0;
  } else {
    // 转盘：上游把初始圆心放在装甲板后方 0.2m 处
    const double r = 0.2;
    target_state << a.position.x() + r * std::cos(yaw), 0,
        a.position.y() + r * std::sin(yaw), 0, a.position.z(), 0, yaw, 0, r;
  }

  ekf.setState(target_state);
}

void Tracker::init(const std::vector<ArmorMeasurement> & armors) {
  if (armors.empty()) return;

  // 选离图像中心最近的那块（上游逻辑）
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
  // 校内赛只有一块靶板，没有数字可区分，恒为 1。
  // 上游这里靠 number 字符串判 4/2/3 块，会走到 NORMAL_4 分支去 swap
  // dz/another_r，污染状态 —— 显式短路掉。
}

void Tracker::initChange(const ArmorMeasurement & a) {
  initEkf(a);
  tracked_id = a.number;
  tracker_state = DETECTING;
  detect_count_ = 0;
  change_count_ = 0;
}

double Tracker::orientationToYaw(const Eigen::Quaterniond & q) {
  // 提取 yaw。⚠️ 约定见 coord_transform::yawFromQuat —— Eigen 的
  // eulerAngles(2,1,0) 返回 (yaw, pitch, roll)，yaw 在索引 [0]。
  double yaw = solver::yawFromQuat(q);

  // 展平成连续值（-π~π → -∞~∞）。
  // 自旋模式下这一步是关键：不展平的话 yaw 每半圈跳一次 2π，
  // v_yaw 会在正负之间来回翻，EKF 推不动。
  yaw = last_yaw_ + shortestAngularDistance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

void Tracker::applyRadiusConstraint() {
  if (target_state.size() != kStateDim) return;

  if (active_mode_ == RotationMode::SELF_SPIN) {
    // 自旋：让 r 自己收缩到 0。
    // r→0 时 j_h 的第 7 列（r*sin, -r*cos）也趋近 0，yaw 退化成
    // 直接观测 z(3) = x(6) —— 这正是"位置固定、朝向自由"想要的形式，
    // 所以不需要额外改 j_h。
    if (std::abs(target_state(8)) > 1e-4) {
      target_state(8) *= cfg_.r_shrink;
      ekf.setState(target_state);
    }
  } else {
    // 转盘：保留上游"把 r 限制在 [0.12, 0.4]"的意图，
    // 但改成软约束 —— 上游是硬赋值，每帧都把协方差丢掉一部分。
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
  if (active_mode_ == RotationMode::SELF_SPIN) return;  // 已切换，不回头

  ++auto_frames_;
  if (auto_frames_ < cfg_.auto_grace_frames) return;  // 等滤波器收敛

  // r 长期贴着下界 → 大概率是自旋（真值 r=0，被约束硬撑在 r_min 附近）
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
  // dt 夹在合理区间：
  //   太小 → lost_thres = lost_time_thres/dt 会变成上万帧，
  //          目标早没了 tracker 还赖在 TEMP_LOST 里报 tracking()=true，
  //          云台会去追一个幽灵。
  //   太大 → 单帧外推过头。
  dt_ = std::clamp(dt, 1e-3, 0.2);
  // static_cast<int> 一个超大 double 是 UB，先夹住再转
  lost_thres = std::clamp(
      static_cast<int>(cfg_.lost_time_thres / dt_), 1, 2000);

  // KF 预测
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
      // 没匹配上，但只有一块同 id 的板且 yaw 跳了 —— 判为自旋导致的换面
      handleArmorJump(same_id_armor);
    }
  }

  applyRadiusConstraint();
  updateAutoMode();

  // ── 状态机（与上游一致）────────────────────────────────────────
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
    // 自旋：位置没变，只是面朝方向跳了。【只更新 yaw】。
    // 不碰位置、不换 r，并且【保留 v_yaw】—— 它是跨越 ±π 的关键，
    // 清掉的话角速度信息就丢了，EKF 要重新收敛。
    target_state(6) = yaw;
    ekf.setStateInflated(target_state, cfg_.jump_p_inflate);
    return;
  }

  // 转盘：上游逻辑。
  target_state(6) = yaw;

  // 注意：上游在这里还会 swap(target_state(8), another_r) 并设 dz。
  // 那是为"一块板有 2 个半径 2 个高度"的 4 装甲板车型设计的。
  // 校内赛是单靶板，updateArmorsNum 已短路为 1 块，这里不再处理。
  const Eigen::Vector3d p = a.position;
  const Eigen::Vector3d infer_p = armorPositionFromState(target_state);
  if ((p - infer_p).norm() > cfg_.max_match_distance) {
    const double r = target_state(8);
    target_state(0) = p.x() + r * std::cos(yaw);  // xc
    target_state(1) = 0;                          // v_xc
    target_state(2) = p.y() + r * std::sin(yaw);  // yc
    target_state(3) = 0;                          // v_yc
    target_state(4) = p.z();                      // za
    target_state(5) = 0;                          // v_za
    // v_yaw 保留
  }
  // 状态被瞬移了，协方差必须跟着膨胀，否则接下来十几帧会过度自信地发散
  ekf.setStateInflated(target_state, cfg_.jump_p_inflate);
}

}  // namespace autoaim::tracker
