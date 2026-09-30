// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
//
// 派生自 rm_auto_aim/armor_detector/include/armor_detector/detector.hpp
// 改动：
//   1. 命名空间 → autoaim::detect
//   2. 删掉 auto_aim_interfaces 的 debug 消息成员（唯一的 ROS 耦合，纯调试用）
//   3. 补上 <memory>（上游靠 number_classifier.hpp 传递包含，很脆）
//   4. classifier 改为可空，detect() 里加保护
//   5. 新增 single_light_fallback / max_single_light_count

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "modules/detect/armor.h"
#include "modules/detect/number_classifier.h"

namespace autoaim::detect {

class Detector {
 public:
  // 一根灯条要满足的外形条件。不满足的直接在 findLights 里淘汰。
  struct LightParams {
    // 灯条外接矩形的 宽/高 比值范围。灯条是细长的，所以比值很小：
    // 太小说明是细线/噪点，太大说明是个方块不是灯条。
    double min_ratio = 0.1;
    double max_ratio = 0.4;
    // 灯条允许的最大倾斜角（度，偏离竖直方向）。
    // 装甲板斜着看时灯条会倾斜，但不会太夸张。
    double max_angle = 35.0;
    // 填充率下限 = 灯条轮廓面积 / 外接矩形面积。
    // 真实灯条是实心矩形，填充率接近 1；噪点形状不规则，填充率低。
    double min_fill_ratio = 0.8;
  };

  // 两根灯条要配成一块装甲板，需要满足的条件。
  struct ArmorParams {
    // 左右灯条的【长度比】下限。同一块装甲板上两根灯条应该差不多长，
    // 比值低于它就说明是两根不相干的灯条，不能配对。
    double min_light_ratio = 0.7;
    // 两灯条中心距 / 灯条长度 的比值范围，用来区分大小装甲板。
    // 小装甲板对应 [min_small, max_small]，大装甲板对应 [min_large, max_large]。
    // 落在这两个区间之外的配对直接丢弃。
    double min_small_center_distance = 0.8;
    double max_small_center_distance = 3.2;
    double min_large_center_distance = 3.2;
    double max_large_center_distance = 5.5;
    // 两灯条连线的最大水平夹角（度）。偏太多说明不是同一块板。
    double max_angle = 35.0;
  };

  // 构造检测器。所有参数都会被存下来，detect() 时使用。
  //
  // bin_thres 二值化阈值（"多亮才算灯条"，现场必须重调，见 README）
  // color     要找的颜色：RED 或 BLUE
  // l, a      灯条/装甲板的形状参数，见上面两个 struct
  Detector(int bin_thres, int color, const LightParams & l, const ArmorParams & a);

  // 主入口：一张图进去，一列表装甲板出来。
  //
  // 内部流程：preprocessImage（二值化）→ findLights（找灯条）
  //           → matchLights（两根配一块）→ 可选 classifier（认数字）
  // 一块都没找到时返回空 vector（不是错误）。若开启 single_light_fallback
  // 且只找到一个灯条，会返回一块 ArmorType::SINGLE 的降级结果。
  //
  // 输入必须是 RGB（不是 BGR）。preprocessImage 走 COLOR_RGB2GRAY，
  // 且 findLights 用 channel[0] 当 R、channel[2] 当 B。
  std::vector<Armor> detect(const cv::Mat & input);

  // 预处理：按 detect_color 提取颜色通道 → 灰度 → 二值化。
  //
  // 返回二值图（非零像素 = 疑似灯条）。结果同时存进成员 binary_img，
  // 供调试工具显示。纯红 (255,0,0) 的灰度只有 76，低于默认阈值 160 ——
  // 所以见过的现象是"画面上明明有灯条却检测不到"，先查这个阈值。
  cv::Mat preprocessImage(const cv::Mat & input);

  // 在二值图里找轮廓，逐个用 isLight() 筛选成候选灯条。
  //
  // 返回通过筛选的灯条列表，每根都带端点、长度、宽度、倾角。
  // 这个列表是 matchLights 的输入。
  std::vector<Light> findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img);

  // 把灯条两两配对成装甲板。遍历所有组合，用 isArmor() 判定，
  // 再调 containLight() 剔除"中间还夹着别的灯条"的误配对。
  //
  // 返回配对成功的装甲板列表（含类型 SMALL/LARGE）。可能为空。
  // 多个候选时会按到图像中心的距离排序 —— 近的排前面。
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // 调试用，无 ROS 依赖：把所有装甲板的数字 ROI 拼成一张大图返回，
  // 方便一次性看分类器到底抠出来了什么。没启用分类器时是空图。
  cv::Mat getAllNumbersImage();

  // 调试用：把当前检测结果（灯条框、装甲板框、中心点）画到 img 上。
  // img 是【原地修改】的，调用方传自己那份可写的图。
  void drawResults(cv::Mat & img);

  int binary_thres = 160;
  int detect_color = RED;
  LightParams l;
  ArmorParams a;

  // 可为空。为空时 detect() 跳过分类，armor.number 等字段保持默认值。
  std::unique_ptr<NumberClassifier> classifier;

  // 只找到一个合格灯条时的降级：产出一块 ArmorType::SINGLE。
  // 单灯条解不出距离和姿态，只能给方位角，仅用于开赛丢靶时让云台朝大致方向。
  bool single_light_fallback = true;
  // 超过这个数量的单灯条就不降级了（多半是噪声）
  int max_single_light_count = 3;

  // 调试：最近一帧的二值图
  cv::Mat binary_img;

 private:
  // 单个轮廓像不像一根灯条。逐条检查 LightParams 里的四个条件
  // （宽高比、倾角、填充率）。
  // 返回 true = 是灯条，可以进候选列表。
  bool isLight(const Light & possible_light);

  // 两根灯条之间是否夹着第三根灯条。
  //
  // 这是配对时最容易出错的地方：装甲板是竖直的，所以只要两块板
  // 在图上左右相邻，就可能被误配成"一块宽板"。判定办法是检查
  // light_1 和 light_2 连成的矩形框内，有没有别的灯条。
  //
  // 返回 true = 中间夹了东西，这两根不能配成一块板。
  bool containLight(const Light & light_1, const Light & light_2,
                    const std::vector<Light> & lights);

  // 两根灯条配成的这块板是 SMALL 还是 LARGE。
  //
  // 判据是"两灯条中心距 / 灯条长度"落在 ArmorParams 的哪个区间。
  // 返回 INVALID = 两个区间都不落，这对灯条不是装甲板，应丢弃。
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  // 最近一次 detect() 的中间结果。留作成员是为了让 drawResults()
  // 能画出来，以及方便调试工具查看，不参与任何计算逻辑。
  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace autoaim::detect
