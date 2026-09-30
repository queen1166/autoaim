// EKF 状态向量的契约。
//
// 这个布局来自 rm_auto_aim 的 tracker_node.cpp:30-31（他们把模型写在 ROS 节点里）。
// 跟踪器和瞄准层都依赖它，所以单独拎出来，只有这一处定义。
//
// 状态（9 维）：目标绕竖直轴旋转
//
//   [0] xc      旋转中心的 x（云台系，前）
//   [1] v_xc    旋转中心速度
//   [2] yc      旋转中心的 y（云台系，左）
//   [3] v_yc
//   [4] za      装甲板中心的高度（云台系，上）
//   [5] v_za
//   [6] yaw     装甲板的角位置
//   [7] v_yaw   角速度
//   [8] r       旋转中心到装甲板的半径
//
// 观测量（4 维）：[xa, ya, za, yaw] —— 装甲板中心位置 + 它的朝向
//
// 几何关系（这是整个模型的核心，也是唯一容易写错符号的地方）：
//
//     xa = xc - r*cos(yaw)
//     ya = yc - r*sin(yaw)
//
// 注意是【减】号。反解就是 xc = xa + r*cos(yaw)，与 initEKF / handleArmorJump 一致。
// tests/test_state.cpp 里有一个往返测试把这个符号锁死。

#pragma once

#include <Eigen/Dense>

namespace autoaim::tracker {

// 状态向量维数。改这个数必须同步改 tracker.cpp 里的 Q/F/P0 矩阵 ——
// 那些矩阵是硬编码的 9x9。
inline constexpr int kStateDim = 9;

// 观测向量维数：[xa, ya, za, yaw]。
inline constexpr int kMeasDim = 4;

// 从状态算出装甲板在云台系下的位置。
//
// 用途：数据关联时拿它算"预测的装甲板位置 vs 观测位置"的差；
// 调试时也用它看滤波器现在认为板子在哪。
//
// x 9 维状态向量，布局见上面
// 返回云台系下的装甲板中心坐标，单位米
//
// 与 EKF 的 h(x) 前两维 + za 必须逐元素一致 —— 那里写错符号的话，
// 跟踪器不会报错，只会静默发散。
inline Eigen::Vector3d armorPositionFromState(const Eigen::VectorXd & x) {
  const double xc = x(0), yc = x(2), za = x(4), yaw = x(6), r = x(8);
  return {xc - r * std::cos(yaw), yc - r * std::sin(yaw), za};
}

}  // namespace autoaim::tracker
