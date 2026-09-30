// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
//
// 派生自 rm_auto_aim/armor_tracker/include/armor_tracker/extended_kalman_filter.hpp
// 改动：仅命名空间 rm_auto_aim → autoaim::tracker。逻辑逐字未改。
//
// 这是一个通用 EKF：非线性函数 f/h 和它们的雅可比、Q/R 都用 std::function
// 传进来，所以模型本身不在这里 —— 在 tracker.cpp 的 initEkf() 里。

#pragma once

#include <Eigen/Dense>

#include <functional>

namespace autoaim::tracker {

class ExtendedKalmanFilter {
 public:
  // ── 六个函数类型别名 ──────────────────────────────────────
  // 模型本身不写在这个类里，而是以回调的形式从外面传进来
  // （实际由 tracker.cpp 的 initEkf() 提供）。所以这个 EKF 是通用的：
  // 换成别的被跟踪对象，只要换个模型就行，这个文件一个字不用改。

  // 输入状态向量、输出向量：状态转移 f(x) 和观测方程 h(x)
  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;

  // 输入状态向量、输出矩阵：雅可比 jacobian_f / jacobian_h
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

  // 无输入、输出矩阵：噪声协方差 Q / R（可能随状态变，所以也是函数）
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  // 默认构造：什么都不初始化，各矩阵都是空的。
  // 这个状态下调 predict()/update() 会崩 —— 必须用下面那个构造函数。
  // 存在只是为了让 Tracker 能把它当普通成员声明。
  ExtendedKalmanFilter() = default;

  // 构造并完成初始化：注册模型、按 P0 建立初始协方差、
  // 从 P0 的维度推断状态维数 n 并准备好单位阵 I。
  //
  // f, h          状态转移和观测方程
  // j_f, j_h      它们的雅可比（EKF 靠它做线性化）
  // u_q, u_r      Q/R 的生成函数（可以随状态变化，因为过程噪声
  //               往往和当前速度有关 —— 转得快的靶板需要更大的 Q）
  // P0            初始状态协方差。填得越大表示"我越不信初值"，
  //               收敛越快但前期抖动越大。
  ExtendedKalmanFilter(const VecVecFunc & f, const VecVecFunc & h,
                       const VecMatFunc & j_f, const VecMatFunc & j_h,
                       const VecMatFunc & u_q, const VecMatFunc & u_r,
                       const Eigen::MatrixXd & P0);

  // 直接写后验状态。半径约束用它强行纠正状态。
  //
  // 只改 x_post，【不动协方差】。所以它适合"小幅修正"；
  // 如果是大幅瞬移（比如跳变），要用下面的 setStateInflated()，
  // 否则滤波器会带着"我很确信"的错误协方差继续跑。
  void setState(const Eigen::VectorXd & x0);

  // 强行改状态【并且】放大协方差。
  //
  // handleArmorJump 会把状态整体瞬移（yaw 直接跳到测量值、速度清零），
  // 但 P_post 还停留在"已经收敛"的小值上 —— 滤波器会过度相信自己刚
  // 编出来的状态，接下来十几帧发散。跳变时必须配一次膨胀。
  //
  // x0       新的状态向量
  // p_factor 协方差放大倍数（P_post *= p_factor）。
  //          TrackerConfig::jump_p_inflate 默认 20.0。
  void setStateInflated(const Eigen::VectorXd & x0, double p_factor);

  // 预测步（时间更新）：用状态转移 f 把 x 外推到当前时刻，
  // 同时用雅可比线性化算协方差 P_pri = F*P_post*Fᵀ + Q。
  //
  // 返回先验状态 x_pri。dt 不在参数里 —— 它被闭包进了 f 和 jacobian_f，
  // 由 tracker.cpp 每次更新时重新注册（这是为了让模型能感知帧间隔）。
  Eigen::MatrixXd predict();

  // 更新步（测量更新）：用观测量 z 修正先验，算出卡尔曼增益 K，
  // 得到后验状态 x_post 和后验协方差 P_post。
  //
  // z 观测向量，维度必须等于 h(x) 的输出维度（本项目是 4 维：
  //   [xa, ya, za, yaw]，见 target_state.h）。
  //   维度不对 Eigen 会直接断言失败（debug 下 abort）。
  //
  // 返回后验状态 x_post，这才是"这一帧最终的估计"。
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

 private:
  VecVecFunc f;   // 状态转移
  VecVecFunc h;   // 观测方程
  VecMatFunc jacobian_f;
  Eigen::MatrixXd F;
  VecMatFunc jacobian_h;
  Eigen::MatrixXd H;
  VecMatFunc update_Q;
  Eigen::MatrixXd Q;
  VecMatFunc update_R;
  Eigen::MatrixXd R;

  // 协方差：P_pri 是先验（预测后、校正前），P_post 是后验（校正后）。
  // K 是卡尔曼增益，预测和观测之间的权重。
  Eigen::MatrixXd P_pri;
  Eigen::MatrixXd P_post;
  Eigen::MatrixXd K;

  int n;              // 状态维数，由构造时的 P0 推出来
  Eigen::MatrixXd I;  // n×n 单位阵，避免每次现构造

  // 状态向量：x_pri 是先验（预测后），x_post 是后验（校正后）。
  // 外界看到的"当前状态"永远是 x_post。
  Eigen::VectorXd x_pri;
  Eigen::VectorXd x_post;
};

}  // namespace autoaim::tracker
