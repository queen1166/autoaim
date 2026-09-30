// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
//
// 派生自 rm_auto_aim/armor_detector/src/detector.cpp
// 改动：
//   1. 命名空间 → autoaim::detect
//   2. 删掉 isLight/isArmor/matchLights 里的 debug 消息填充（纯 ROS 调试用）
//   3. detect() 里 classifier 加空指针保护（上游是无条件解引用）
//   4. 新增单灯条降级路径

#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include "modules/detect/armor_detector.h"

namespace autoaim::detect {

Detector::Detector(int bin_thres, int color, const LightParams & l,
                   const ArmorParams & a)
    : binary_thres(bin_thres), detect_color(color), l(l), a(a) {}

std::vector<Armor> Detector::detect(const cv::Mat & input) {
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  // 上游在这里无条件解引用 classifier，一旦没赋值就是 UB。
  if (classifier && !armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  // 降级：没能配出装甲板，但恰好找到少数几个合格灯条时，
  // 把它们各自当成一块 SINGLE 装甲板。
  // 单灯条解不出距离和姿态，仅用于让云台朝大致方位，不要用它闭环开火。
  if (armors_.empty() && single_light_fallback && !lights_.empty() &&
      static_cast<int>(lights_.size()) <= max_single_light_count) {
    for (const auto & light : lights_) {
      Armor armor(light, light);
      armor.type = ArmorType::SINGLE;
      armors_.emplace_back(armor);
    }
  }

  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img) {
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat bin_img;
  cv::threshold(gray_img, bin_img, binary_thres, 255, cv::THRESH_BINARY);

  return bin_img;
}

std::vector<Light> Detector::findLights(const cv::Mat & rgb_img,
                                        const cv::Mat & binary_img) {
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto b_rect = cv::boundingRect(contour);
    auto r_rect = cv::minAreaRect(contour);
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
    mask_contour.reserve(contour.size());
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
    }
    cv::fillPoly(mask, {mask_contour}, 255);
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // points / rotated rect area
    bool is_fill_rotated_rect =
        points.size() / (r_rect.size.width * r_rect.size.height) > l.min_fill_ratio;

    cv::Vec4f return_param;
    cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f top, bottom;
    double angle_k;
    if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
      top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
      angle_k = 0;
    } else {
      auto k = return_param[1] / return_param[0];
      auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      angle_k = std::atan(k) / CV_PI * 180 - 90;
      if (angle_k > 90) {
        angle_k = 180 - angle_k;
      }
    }
    auto light = Light(b_rect, top, bottom, static_cast<int>(points.size()), angle_k);

    if (isLight(light) && is_fill_rotated_rect) {
      auto rect = light;
      if (  // Avoid assertion failed
          0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols &&
          0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y),
                                     false) >= 0) {
              // 输入约定是 RGB，所以 [0] 是 R、[2] 是 B。喂 BGR 会红蓝颠倒。
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light) {
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  return ratio_ok && angle_ok;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights) {
  std::vector<Armor> armors;

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(const Light & light_1, const Light & light_2,
                            const std::vector<Light> & lights) {
  auto points =
      std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (bounding_rect.contains(test_light.top) ||
        bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2) {
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length
                                 ? light_1.length / light_2.length
                                 : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok =
      (a.min_small_center_distance <= center_distance &&
       center_distance < a.max_small_center_distance) ||
      (a.min_large_center_distance <= center_distance &&
       center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  if (!is_armor) return ArmorType::INVALID;
  return center_distance > a.min_large_center_distance ? ArmorType::LARGE
                                                       : ArmorType::SMALL;
}

cv::Mat Detector::getAllNumbersImage() {
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  }
  std::vector<cv::Mat> number_imgs;
  number_imgs.reserve(armors_.size());
  for (auto & armor : armors_) {
    number_imgs.emplace_back(armor.number_img);
  }
  cv::Mat all_num_img;
  cv::vconcat(number_imgs, all_num_img);
  return all_num_img;
}

void Detector::drawResults(cv::Mat & img) {
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    if (armor.type == ArmorType::SINGLE) {
      // 单灯条：只画灯条本身，没有对角连线
      cv::rectangle(img, armor.left_light, cv::Scalar(0, 165, 255), 2);
      cv::putText(img, "SINGLE", armor.left_light.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  cv::Scalar(0, 165, 255), 2);
      continue;
    }
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    if (armor.classfication_result.empty()) continue;
    cv::putText(img, armor.classfication_result, armor.left_light.top,
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace autoaim::detect
