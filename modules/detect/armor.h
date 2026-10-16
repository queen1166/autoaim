#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace autoaim::detect {

constexpr int RED = 0;
constexpr int BLUE = 1;

// 哨兵值：matchLights 见到它就不按颜色过滤，任何颜色的两根灯条都能配对。
// 单靶板赛题（只有一块板）用它免疫灯条颜色分类的抖动。
constexpr int ANY_COLOR = -1;

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

  // ── 关键点路径（YOLO-Pose）───────────────────────────────────────────
  // 规范顺序（与 PnP 的物体点一一对应，左/右按图像 x 排序）：
  //   0 = 左灯条上端  1 = 左灯条下端  2 = 右灯条上端  3 = 右灯条下端
  //
  // YOLO-Pose 检出后会用这四个点填好 left_light / right_light，
  // 使下游（PnP、跟踪、瞄准）完全不用区分探测器是传统视觉还是神经网络。
  cv::Point2f keypoints[4];
  bool from_keypoints = false;

  // 角点是否经过区域传统优化（PCA 精修）矫正过。
  bool corner_refined = false;
};

// 关键点在规范顺序里的下标。
enum KeypointIndex {
  KP_LEFT_TOP = 0,
  KP_LEFT_BOTTOM = 1,
  KP_RIGHT_TOP = 2,
  KP_RIGHT_BOTTOM = 3,
};

// PnP 需要的 4 个图像点，顺序必须与 pnp_solver 里的物体点一致：
//   [左灯条下, 左灯条上, 右灯条上, 右灯条下]
inline std::vector<cv::Point2f> armorImagePoints(const Armor & a) {
  return {a.left_light.bottom, a.left_light.top, a.right_light.top,
          a.right_light.bottom};
}

}
