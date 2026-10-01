#pragma once

#include <Eigen/Dense>

#include <string>

namespace autoaim::tracker {
struct ArmorMeasurement {
  // 装甲板中心在【云台系】下的位置，单位 m
  Eigen::Vector3d position = Eigen::Vector3d::Zero();

  // 装甲板在【云台系】下的姿态。
  // 用 coord_transform::rvecToGimbalQuat 从 PnP 的 rvec 转过来了。
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  // 装甲板中心到图像主点的像素距离。
  // 用于多目标时挑"离准星最近的那个"；校内赛只有一块板，基本用不上，
  // 但保留它可以让 init() 的选板逻辑和上游一致。
  double distance_to_image_center = 0.0;

  // 分类器结果。校内赛默认不启用分类器，这里会是空串。
  // 单靶板场景下 tracked_id 恒为 ""，匹配逻辑照样工作。
  std::string number;
  std::string type;  // "small" / "large" / "single"
};

}  // namespace autoaim::tracker
