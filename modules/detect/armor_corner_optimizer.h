#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "modules/detect/armor.h"

namespace autoaim::detect {

// 角点精修 —— 教程 §5A.3（PCA 矫正灯条端点），也是 §5A.1 的"方案 C"。
//
// 【解决的问题】神经网络（尤其 int8 量化后）给出的角点有 1~3 像素偏差；
// 传统视觉用 minAreaRect 的顶点则会被二值化阈值带偏——阈值一变，
// 灯条轮廓胖瘦就变，端点位置跟着漂。而 PnP 对这 4 个点很敏感，
// 距离越远、杠杆放大越厉害。
//
// 【做法】只用神经网络的**大致位置**，把精度交给物理量：
//   1. 以给定的两个端点连线为轴，取出灯条附近的旋转 ROI
//   2. 在 ROI 内阈值化得到灯条像素，对它们做 PCA 求出灯条的**对称轴**
//      —— 主成分从整片点云里提取方向，不吃阈值的偶然性
//   3. 沿对称轴向两端搜索**亮度梯度最大处**（灯条边缘的明暗交界）
//      —— 明暗交界是物理固有的亮度跳变，不随阈值漂移
//   4. 用这两个交界点替换原来的端点
//
// 结果写回 Armor 的 left_light / right_light（top/bottom 与 center/length 一并更新），
// 并置 corner_refined = true。
struct CornerRefineConfig {
  bool enable = true;

  // 沿轴线搜索明暗交界时的采样与门限
  int search_extra_px = 6;        // 在端点外再往两端多搜几个像素
  double min_gradient = 8.0;      // 灰度梯度门限（0~255 尺度）
  int roi_margin_px = 4;          // 旋转 ROI 相对灯条宽度的外扩

  // 灯条前景阈值：灯条是过曝的 LED，灰度很高。0 = 用 OTSU 自适应。
  int light_thres = 0;

  // 精修结果与原端点偏差超过这个像素数就判定失败，保留原值（防止被杂光带跑）
  double max_shift_px = 12.0;
};

class ArmorCornerOptimizer {
 public:
  explicit ArmorCornerOptimizer(const CornerRefineConfig & cfg = {}) : cfg_(cfg) {}

  // 就地精修一块装甲板的两根灯条。rgb 必须是 RGB（与 Frame::image 一致）。
  // 返回 true 表示至少一根灯条被成功矫正。
  bool refine(const cv::Mat & rgb, Armor & armor) const;

  const CornerRefineConfig & config() const { return cfg_; }

  void setConfig(const CornerRefineConfig & c) { cfg_ = c; }

  // 单根灯条的精修，单独拆出来便于单测：给定端点 top/bottom（可能带偏），
  // 返回矫正后的 top/bottom。
  bool refineBar(const cv::Mat & rgb, const cv::Point2f & top,
                 const cv::Point2f & bottom, cv::Point2f & top_out,
                 cv::Point2f & bottom_out) const;

 private:
  CornerRefineConfig cfg_;
};

}
