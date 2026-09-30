// 装甲板数字分类器的抽象接口。
//
// 上游 rm_auto_aim 的 NumberClassifier 是具体类（cv::dnn + ONNX），
// 且 Detector 在所有路径上无条件解引用它（detector.cpp:36），
// 一旦没有赋值就是未定义行为。
//
// 这里改成抽象接口 + 可空指针，把 ONNX 依赖完全隔离到实现里。
// 校内赛只有一个 5 m 处的旋转靶板、没有数字，所以默认【不启用】——
// 省掉 ONNX 依赖，也省掉一类现场排查问题。
//
// 具体的 ONNX 实现在第 6 步补（参考 rm_auto_aim 的 number_classifier.cpp，
// 用 cv::dnn 而非 onnxruntime）。

#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace autoaim::detect {

struct Armor;

class NumberClassifier {
 public:
  // 虚析构：Detector 里持的是 unique_ptr<NumberClassifier>，
  // 析构时必须能调到具体实现（ONNX 那套）的析构，否则它的
  // cv::dnn::Net 等资源不释放。
  virtual ~NumberClassifier() = default;

  // 从图中抠出每块装甲板的数字 ROI，存进 armor.number_img。
  //
  // src    原始 RGB 图（不是 BGR）
  // armors 【原地修改】。对每块装甲板，按它的四个角点做透视变换，
  //        裁出一张正的、灰度化的小图，写回 armor.number_img。
  //
  // 这一步只裁图不推理，真正的分类在 classify() 里。
  // 拆成两步是因为裁剪可以批量做，推理必须整批一次喂给网络才快。
  virtual void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors) = 0;

  // 推理并填 armor.number / confidence / classfication_result，
  // 同时按置信度和 ignore_classes 过滤掉不合格的装甲板。
  //
  // armors 【原地修改，而且会变短】。置信度低于 threshold 或者命中
  //        ignore_classes 的装甲板会被直接 erase 掉。
  //
  // 注意：classify() 不只是打标签，它还会 erase 掉一批装甲板。
  // 关掉分类器 = 同时关掉这道过滤。
  // 所以"关掉数字识别"这个决定，实际影响的是"还要不要按数字筛板"。
  virtual void classify(std::vector<Armor> & armors) = 0;

  // 置信度阈值：低于它的装甲板会被 classify() 丢弃。
  // 0.7 是上游的默认值。校内赛没有数字，这个分类器默认根本不启用。
  float threshold = 0.7f;
};

}  // namespace autoaim::detect
