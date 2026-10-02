
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
  struct LightParams {
    double min_ratio = 0.1;
    double max_ratio = 0.4;
    double max_angle = 35.0;
    double min_fill_ratio = 0.8;
  };

  struct ArmorParams {
    double min_light_ratio = 0.7;
    double min_small_center_distance = 0.8;
    double max_small_center_distance = 3.2;
    double min_large_center_distance = 3.2;
    double max_large_center_distance = 5.5;
    double max_angle = 35.0;
  };

  Detector(int bin_thres, int color, const LightParams & l, const ArmorParams & a);

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);

  std::vector<Light> findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img);

  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  cv::Mat getAllNumbersImage();

  void drawResults(cv::Mat & img);

  int binary_thres = 160;
  int detect_color = RED;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  bool single_light_fallback = true;
  int max_single_light_count = 3;

  cv::Mat binary_img;

 private:
  bool isLight(const Light & possible_light);

  bool containLight(const Light & light_1, const Light & light_2,
                    const std::vector<Light> & lights);

  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}
