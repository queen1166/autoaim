#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "modules/detect/armor.h"

namespace autoaim::detect {

// 探测器抽象接口。
//
// 为什么要有它：教程 §5A.1 给了三条探测路线（纯传统 / 纯神经网络 / 神经网络+传统优化），
// 本仓库两条都实现：
//   · Detector            —— 传统视觉：灰度二值化 → 找灯条 → 配对
//   · YoloPoseDetector    —— 神经网络：YOLO-Pose 端到端出 4 个关键点
// 两者产出同一种 `Armor`（关键点会填进 left_light / right_light），
// 所以下游 PnP / 跟踪 / 瞄准一行都不用改。
//
// 图像约定：**输入是 RGB**（`bsp::Frame::image` 就是 RGB，不是 OpenCV 默认的 BGR）。
class ArmorDetectorBase {
 public:
  virtual ~ArmorDetectorBase() = default;

  virtual std::vector<Armor> detect(const cv::Mat & rgb) = 0;

  // 调试可视化。默认什么都不画，子类按需覆盖。
  virtual void drawResults(cv::Mat & /*img*/) {}

  virtual const char * name() const = 0;
};

}
