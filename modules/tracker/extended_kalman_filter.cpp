
#include "modules/tracker/extended_kalman_filter.h"

#include <algorithm>
#include <limits>

namespace autoaim::tracker {

ExtendedKalmanFilter::ExtendedKalmanFilter(const VecVecFunc & f, const VecVecFunc & h,
                                           const VecMatFunc & j_f, const VecMatFunc & j_h,
                                           const VecMatFunc & u_q, const VecMatFunc & u_r,
                                           const Eigen::MatrixXd & P0)
    : f(f),
      h(h),
      jacobian_f(j_f),
      jacobian_h(j_h),
      update_Q(u_q),
      update_R(u_r),
      P_post(P0),
      n(P0.rows()),
      I(Eigen::MatrixXd::Identity(n, n)),
      x_pri(n),
      x_post(n) {}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

void ExtendedKalmanFilter::setStateInflated(const Eigen::VectorXd & x0,
                                            double p_factor) {
  x_post = x0;
  if (p_factor > 1.0) {
    P_post *= p_factor;
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (i != j) P_post(i, j) *= 0.5;
      }
    }
  }
}

Eigen::MatrixXd ExtendedKalmanFilter::predict() {
  F = jacobian_f(x_post), Q = update_Q(x_post);

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

void ExtendedKalmanFilter::computeGainAt(const Eigen::VectorXd & x_lin,
                                         const Eigen::VectorXd & z) {
  H = jacobian_h(x_lin);
  R = update_R(z);
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z) {
  computeGainAt(x_pri, z);

  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;
}

Eigen::MatrixXd ExtendedKalmanFilter::updateIterated(const Eigen::VectorXd & z,
                                                     int max_iters,
                                                     double eps) {
  // 观测噪声 R 与观测 z 有关（YPD 模型按距离缩放），整轮迭代固定用同一个 R，
  // 否则每次重算 R 会把噪声模型也一起"迭代"，失去收敛意义。
  computeGainAt(x_pri, z);
  const Eigen::MatrixXd R_fixed = R;

  Eigen::VectorXd x = x_pri;
  for (int k = 0; k < std::max(1, max_iters); ++k) {
    // 在当前线性化点重算 H、K（R 保持不变）。
    H = jacobian_h(x);
    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R_fixed).inverse();

    const Eigen::VectorXd dx = K * (z - h(x) - H * (x_pri - x));
    const Eigen::VectorXd x_new = x_pri + dx;

    const double step = (x_new - x).norm();
    x = x_new;
    if (step < eps) break;
  }

  x_post = x;
  P_post = (I - K * H) * P_pri;
  return x_post;
}

Eigen::MatrixXd ExtendedKalmanFilter::innovationCovariance(const Eigen::VectorXd & z) {
  H = jacobian_h(x_pri);
  R = update_R(z);
  return H * P_pri * H.transpose() + R;
}

double ExtendedKalmanFilter::mahalanobisSquared(const Eigen::VectorXd & z) {
  const Eigen::MatrixXd S = innovationCovariance(z);
  const Eigen::VectorXd innovation = z - h(x_pri);
  // 用 LDLT 而不是直接求逆：S 是对称正定的，数值上更稳。
  const Eigen::VectorXd sol = S.ldlt().solve(innovation);
  const double d2 = innovation.dot(sol);
  return std::isfinite(d2) ? d2 : std::numeric_limits<double>::infinity();
}

}
