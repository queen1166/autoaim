#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

#include "modules/detect/armor.h"
#include "modules/detect/armor_detector_base.h"

namespace autoaim::detect {

// YOLO-Pose 装甲板关键点探测器（教程 §5A.1 方案 B）。
//
// 为什么用它：它端到端地同时给出 "这里有装甲板"（bbox）和 4 个灯条角点，
// 光照/大角度鲁棒性远好于传统二值化；代价是需要模型文件和一点算力。
//
// 输出张量布局（按 ultralytics YOLOv8/v11-pose 的惯例）：
//     shape = [1, 4 + num_classes + 4 * kpt_dim, num_anchors]
//     row 0..3              : cx, cy, w, h        （letterbox 后的像素坐标）
//     row 4..4+nc-1         : 每个类别的分数
//     row 4+nc ..           : 4 个关键点 × kpt_dim，kpt_dim = 3 时是 (x, y, conf)
//
// 【关键点顺序不做假设】4 个关键点按"每根灯条两个端点"理解即可，顺序随意：
// 解码时按 x 坐标把 4 点分成左右两根灯条，灯条内按 y 分出上下端点，
// 再填进 Armor::left_light / right_light。这样模型训练时怎么排都能用。
struct YoloPoseConfig {
  std::string model_path;   // ONNX 文件；为空则探测器不可用

  int input_w = 640;
  int input_h = 640;

  float conf_thres = 0.25f;      // 检出置信度门限
  float iou_thres = 0.45f;       // NMS IoU 门限
  float kpt_conf_thres = 0.30f;  // 单个关键点置信度门限（4 个点全过才保留）

  int num_classes = 1;
  int kpt_dim = 3;               // 每个关键点的数值个数：3 = (x,y,conf)，2 = (x,y)

  // 输入图像是否为 RGB。`bsp::Frame::image` 是 RGB，所以默认不交换通道；
  // 若换成 OpenCV 惯常的 BGR 图像源，把这个置 true。
  bool input_is_bgr = false;

  // 大/小装甲板判定：bbox 宽高比 > 该阈值判为大装甲板。
  // （教程 §9.4.5 用 tag_id + 长宽比判定；本赛题只有一种靶板，按长宽比即可。）
  float large_aspect_ratio = 1.4f;
};

class YoloPoseDetector : public ArmorDetectorBase {
 public:
  explicit YoloPoseDetector(const YoloPoseConfig & cfg = {}) : cfg_(cfg) {}

  // 加载 ONNX。失败返回 false，调用方应回退到传统视觉。
  bool load();

  bool loaded() const { return loaded_; }

  std::vector<Armor> detect(const cv::Mat & rgb) override;

  void drawResults(cv::Mat & img) override;

  const char * name() const override { return "yolo-pose"; }

  const YoloPoseConfig & config() const { return cfg_; }

  void setConfig(const YoloPoseConfig & c) { cfg_ = c; }

  // ── 纯函数，单独拆出来是为了能脱离模型文件做单元测试 ──────────────────
  //
  // letterbox：等比缩放 + 居中补边到 (w, h)。ratio 是缩放系数，
  // pad_x / pad_y 是补边宽度，反变换要用它们。
  static cv::Mat letterbox(const cv::Mat & src, int w, int h, double & ratio,
                           int & pad_x, int & pad_y);

  // decode：把网络输出张量解成装甲板列表。
  static std::vector<Armor> decode(const cv::Mat & output, double ratio, int pad_x,
                                   int pad_y, int orig_w, int orig_h,
                                   const YoloPoseConfig & cfg);

  // 把 4 个关键点填进 Armor（按 x 分左右、按 y 分上下）。
  static bool buildArmorFromKeypoints(const cv::Point2f kpts[4], Armor & out);

 private:
  YoloPoseConfig cfg_;
  cv::dnn::Net net_;
  bool loaded_ = false;

  // 调试用：最近一帧的原始检出（画图时用）。
  std::vector<Armor> last_armors_;
};

}
