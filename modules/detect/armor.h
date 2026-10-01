#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <string>

namespace autoaim::detect {

//颜色代号
//constexpr 编译期就定死的常量
constexpr int RED = 0;
constexpr int BLUE = 1;

// SINGLE 是校内赛新增的降级类型：只找到一个合格灯条时使用。
// 单灯条无法解算距离和姿态，只能给方位角。不要用它闭环开火。
//enum class(枚举),small/large:装甲板的大小
enum class ArmorType { SMALL, LARGE, SINGLE, INVALID };

inline const char * const ARMOR_TYPE_STR[] = {"small", "large", "single", "invalid"};

// 把枚举转成可读字符串（"small"/"large"/"single"/"invalid"），
// 打日志和 debug 显示用。传进来的值必须合法，不做越界保护。
inline const char * armorTypeStr(ArmorType t) {
  return ARMOR_TYPE_STR[static_cast<int>(t)];
}

// 一根灯条（装甲板上那两条竖直发光条之一）。
// 继承 cv::Rect 是为了顺带带上外接矩形 box（画图、算面积用）。
struct Light : public cv::Rect {
  Light() = default;

  // 由检测结果构造一根灯条，并把派生量一次算好。
  //
  // box        外接矩形（顺手传给基类 cv::Rect）
  // top,bottom 灯条的上下端点像素坐标
  // area       灯条像素面积，用来反推宽度（width = area / length）
  // tilt_angle 灯条的倾斜角，单位【度】。用来判断"是不是竖着的"
  //
  // 构造时自动算出：length（上下端点距离）、width（面积/长度）、
  // center（上下端点中点）。
  explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area,
                 float tilt_angle)
      : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle) {
    length = cv::norm(top - bottom);
    width = area / length;
    center = (top + bottom) / 2;
  }

  int color = RED;
  cv::Point2f top, bottom;
  cv::Point2f center;
  double length = 0;
  double width = 0;
  float tilt_angle = 0;
};

// 一块装甲板 = 左右两根灯条 + 它的中心 + 类型。
// 由 Detector::matchLights() 配对产出，是检测模块的最终输出。
struct Armor {

  Armor() = default;

  // 由左右两根灯条配成一块装甲板。
  //
  // 它会【自动按 x 坐标排序】—— 传入顺序无所谓，center.x 小的那根
  // 一定是 left_light。这一步很重要：PnP 的四个角点顺序依赖左右之分，
  // 靠调用方保证顺序迟早会出错。
  //
  // center 取两根灯条中心的中点。注意 type 不在这里定，
  // 由 Detector::isArmor() 根据两灯条的距离判定（大/小装甲板）。
  Armor(const Light & l1, const Light & l2) {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type = ArmorType::INVALID;

  // Number part（校内赛默认不启用分类器，这些字段保持默认值）
  cv::Mat number_img; //从装甲板上裁下来的数字小图
  std::string number;//识别的数字
  float confidence = 0.0f;//识别的置信度
  std::string classfication_result;  // 拼写沿用上游，勿改
};

}  // namespace autoaim::detect
