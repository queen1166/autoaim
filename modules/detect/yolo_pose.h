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
// 输出张量统一是 [1, 4 + num_classes + 4 * kpt_dim, num_anchors]，两种布局只在
// 前 4 个通道的含义上不同，见 HeadLayout。
//
// 【关键点顺序不做假设】4 个关键点按"每根灯条两个端点"理解即可，顺序随意：
// 解码时按 x 坐标把 4 点分成左右两根灯条，灯条内按 y 分出上下端点，
// 再填进 Armor::left_light / right_light。这样模型训练时怎么排都能用。
//
// 输出头布局。两种布局的通道总数公式相同（4 + 类别数 + 4×kpt_dim），
// 区别只在前 4 个通道装的是什么。
enum class HeadLayout {
  // [4 bbox(cx,cy,w,h)][num_classes 类别][4 × kpt_dim 关键点]
  // ultralytics YOLOv8/v11-pose 的标准导出。
  // 对应的惯例值：640×640 输入、1 类、kpt_dim 3、缩放 1/255。
  // --yolo-layout ultralytics 会把这套值一次设好。
  ULTRALYTICS,

  // [4 颜色][num_classes 类别][4 × 2 关键点]，**没有 bbox 通道**。
  // 用来打装甲板的深大 RobotPilots 26 赛季模型，它是从 OpenVINO 转出来的 21 通道头
  // （9 类：哨兵/英雄/工程/3~5号步兵/前哨站/基地底/基地顶）。
  // 对应的惯例值：640×480 输入、9 类、kpt_dim 2、缩放 1.0。这些也是本文件的默认值。
  //
  // 两个坑：
  //   1. 前 4 个通道是颜色分数（蓝/红/白/紫），**不是 cx,cy,w,h**。所以 bbox 只能
  //      从 4 个关键点的外接矩形算出来，直接读这 4 个通道会把所有框退化成 1px 小框。
  //   2. 颜色顺序是「0=蓝 1=红」，和 armor.h 的 RED=0 / BLUE=1 **刚好相反**，
  //      哪天要拿这个分支做敌我过滤，别直接赋值。
  COLOR_CLASS_KPT,
};

struct YoloPoseConfig {
  // 【本仓库的默认模型】下面的默认值都是按它填的，所以 ./build/autoaim 不带
  // --yolo 也会走神经网络这条路。
  //
  // 路径相对于**进程当前目录**，按 RUNBOOK 从仓库根目录跑 ./build/autoaim 即可；
  // 从别的目录跑会落空，load() 会打印它实际找的绝对路径。
  // 该文件被 .gitignore 排除（*.onnx），新 clone 的仓库里没有——属于预期，
  // 这时会打印提示并回退传统视觉（见 DetectConfig::yolo_fallback_to_traditional）。
  //
  // 换模型用 --yolo <路径>，换惯例（比如 ultralytics 那套）用 --yolo-layout，
  // 它会把输入尺寸/类别数/kpt_dim/缩放一起切好。
  std::string model_path = "models/Infantry-v8n-opencv.onnx";

  int input_w = 640;
  int input_h = 480;

  HeadLayout head_layout = HeadLayout::COLOR_CLASS_KPT;

  // 输入缩放系数。默认模型吃 0~255 原始像素值（除 255 之后输出全通道恒为 0，
  // 一帧都检不出来）；ultralytics 的导出用 1/255。
  // --yolo-layout 会顺带设好，--yolo-input-scale 可以覆盖。
  float input_scale = 1.0f;

  float conf_thres = 0.25f;      // 检出置信度门限
  float iou_thres = 0.45f;       // NMS IoU 门限
  float kpt_conf_thres = 0.30f;  // 单个关键点置信度门限（4 个点全过才保留；
                                 // 默认模型没有关键点置信度通道，用不上）

  int num_classes = 9;           // 默认模型的 9 类：哨兵/英雄/工程/3~5号步兵/前哨站/基地底/基地顶
  int kpt_dim = 2;               // 每个关键点的数值个数：3 = (x,y,conf)，2 = (x,y)

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
