#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "modules/solver/coord_transform.h"

namespace autoaim::timeutil {

// 姿态时间戳队列 —— 把下位机反馈的云台姿态对齐到图像帧的时间戳。
//
// 【为什么不能用"最新一包"】三个信息源物理上异步：
//   · 相机帧有自己的时间线（曝光 → 读出 → 传输，到达上位机时已经过去几毫秒）
//   · 姿态反馈以它自己的频率从串口进来
//   · 两者到达时间不同，缓冲区里那包反馈不一定是"这张图曝光那一刻"的姿态
// 于是在姿态角速度较大时（云台在追旋转靶），用最新姿态去解释几毫秒前的图像，
// 会带来与转速成正比的系统性角度偏差。
//
// 【做法】以相机帧为主时钟（data->time），在姿态队列里找到它两侧最近的
// 两个样本，用**四元数球面线性插值（Slerp）**求出 t_image 时刻的姿态。
//
// 【为什么是 Slerp 而不是线性插值】lerp 在两个四元数之间走"直线"，
// 结果会掉出单位四元数球面，不再是合法旋转（角度越大偏得越多）。
// Slerp 走球面最短弧，插值结果永远是合法单位四元数。
class AttitudeBuffer {
 public:
  explicit AttitudeBuffer(size_t capacity = 256) : capacity_(capacity) {}

  // 线程安全：rxLoop 写，planLoop/processLoop 读。
  void push(uint64_t t_ns, const Eigen::Quaterniond & q);

  // 取 t_ns 时刻的姿态（Slerp）。
  //   · t_ns 在样本区间内  → 插值
  //   · t_ns 比最新还新    → 返回最新（外推不外推姿态，宁可滞后）
  //   · t_ns 比最旧还旧    → 返回最旧
  // 队列为空返回 false。
  bool query(uint64_t t_ns, Eigen::Quaterniond & out) const;

  // 取最新一包（没有则 false），同时回传它的时间戳。
  bool latest(uint64_t & t_ns, Eigen::Quaterniond & q) const;

  // 落在 [t_lo, t_hi] 内的样本数，用来判断队列是否"有足够跨度可插值"。
  size_t size() const;

  void clear();

 private:
  struct Sample {
    uint64_t t_ns = 0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  };

  mutable std::mutex m_;
  std::deque<Sample> buf_;
  size_t capacity_;
};

}
