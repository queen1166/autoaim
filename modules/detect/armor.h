#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <string>

namespace autoaim::detect {

constexpr int RED = 0;
constexpr int BLUE = 1;

enum class ArmorType { SMALL, LARGE, SINGLE, INVALID };

inline const char * const ARMOR_TYPE_STR[] = {"small", "large", "single", "invalid"};

inline const char * armorTypeStr(ArmorType t) {
  return ARMOR_TYPE_STR[static_cast<int>(t)];
}

struct Light : public cv::Rect {
  Light() = default;

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

struct Armor {

  Armor() = default;

  Armor(const Light & l1, const Light & l2) {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type = ArmorType::INVALID;

  cv::Mat number_img;
  std::string number;
  float confidence = 0.0f;
  std::string classfication_result;
};

}
