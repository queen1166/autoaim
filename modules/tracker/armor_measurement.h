// 替代上游的 auto_aim_interfaces::msg::Armor。
//
// 上游在 detector_node 里把 PnP 结果转成 tf2 四元数、塞进 ROS 消息、
// 再在 tracker_node 里用 tf2_buffer 变换到 odom 系。非 ROS 环境没有这条路。
//
// 这里改成：检测侧算完就把位置和姿态都放到【云台系】，直接传结构体。
// 坐标变换在模块边界之外完成（coord_transform），跟踪器只管跟踪。

#pragma once

#include <Eigen/Dense>

#include <string>

namespace autoaim::tracker {

// 一帧里的一块装甲板观测。检测模块（其实是 autoaim_app）填好后
// 喂给 Tracker::update()，跟踪器只消费它、不修改它。
//
// 注意这里已经是【云台系】了 —— 坐标变换在模块边界之外完成，
// 跟踪器不需要知道相机装在哪，只管跟踪。
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
