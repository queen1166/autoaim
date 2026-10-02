
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

 private:
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
