
#pragma once

#include <Eigen/Dense>

#include <functional>

namespace autoaim::tracker {

class ExtendedKalmanFilter {
 public:

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;

  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(const VecVecFunc & f, const VecVecFunc & h,
                       const VecMatFunc & j_f, const VecMatFunc & j_h,
                       const VecMatFunc & u_q, const VecMatFunc & u_r,
                       const Eigen::MatrixXd & P0);

  void setState(const Eigen::VectorXd & x0);

  void setStateInflated(const Eigen::VectorXd & x0, double p_factor);

  Eigen::MatrixXd predict();

  Eigen::MatrixXd update(const Eigen::VectorXd & z);

  // ── IESEKF：迭代扩展卡尔曼滤波（教程 §5B.5）──────────────────────────
  //
  // 标准 EKF 只在预测值 x_pred 处对非线性观测函数做**一次**一阶泰勒展开。
  // 当 x_pred 离真值较远（初始化阶段、目标突然变速）时，这一次展开的近似
  // 误差可能很大，更新后的状态反而更偏。
  //
  // IEKF 的做法是在更新里多迭代几次：每次用**更新后**的状态重新算 H 和 K，
  // 收敛后等价于在真值附近线性化。开销只是多算几遍 H/K，
  // 相对整条算法链（推理 + PnP + 全部预测/更新）几乎可以忽略。
  //
  //   x^{k+1} = x_pred + K(x^k)·(z − h(x^k) − H(x^k)·(x_pred − x^k))
  Eigen::MatrixXd updateIterated(const Eigen::VectorXd & z, int max_iters,
                                 double eps);

  // 观测应满足的分布：S = H·P_pri·Hᵀ + R（在 x_pri 处线性化）。
  // 马氏距离匹配要用它当"误差椭球"的尺度。
  Eigen::MatrixXd innovationCovariance(const Eigen::VectorXd & z);

  // 马氏距离的平方：d² = (z − z_pred)ᵀ S⁻¹ (z − z_pred)，z_pred = h(x_pri)。
  double mahalanobisSquared(const Eigen::VectorXd & z);

  const Eigen::MatrixXd & posteriorCovariance() const { return P_post; }

 private:
  // 用给定线性化点算 H / S / K，供 update 与 updateIterated 共用。
  void computeGainAt(const Eigen::VectorXd & x_lin, const Eigen::VectorXd & z);

  VecVecFunc f;
  VecVecFunc h;
  VecMatFunc jacobian_f;
  Eigen::MatrixXd F;
  VecMatFunc jacobian_h;
  Eigen::MatrixXd H;
  VecMatFunc update_Q;
  Eigen::MatrixXd Q;
  VecMatFunc update_R;
  Eigen::MatrixXd R;

  Eigen::MatrixXd P_pri;
  Eigen::MatrixXd P_post;
  Eigen::MatrixXd K;

  int n;
  Eigen::MatrixXd I;

  Eigen::VectorXd x_pri;
  Eigen::VectorXd x_post;
};

}
