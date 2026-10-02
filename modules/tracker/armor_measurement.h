#pragma once

#include <Eigen/Dense>

#include <string>

namespace autoaim::tracker {
struct ArmorMeasurement {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  double distance_to_image_center = 0.0;

  std::string number;
  std::string type;
};

}
