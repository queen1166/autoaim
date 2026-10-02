#pragma once
#include <Eigen/Dense>
namespace autoaim::tracker {

inline constexpr int kStateDim = 9;

inline constexpr int kMeasDim = 4;

inline Eigen::Vector3d armorPositionFromState(const Eigen::VectorXd & x) {
  const double xc = x(0), yc = x(2), za = x(4), yaw = x(6), r = x(8);
  return {xc - r * std::cos(yaw), yc - r * std::sin(yaw), za};
}

}
