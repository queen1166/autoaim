#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <vector>
#include "modules/detect/armor.h"
namespace autoaim::solver {
struct ArmorGeometry {
  float small_width = 132.0f;
  float small_height = 57.0f;
  float large_width = 223.0f;
  float large_height = 57.0f;
};

class PnPSolver {
 public:
  PnPSolver(const std::array<double, 9> & camera_matrix,
            const std::vector<double> & distortion_coefficients,
            const ArmorGeometry & geometry = ArmorGeometry{});

  bool solvePnP(const detect::Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const;
  double reprojectionError(const detect::Armor & armor, const cv::Mat & rvec,
                           const cv::Mat & tvec) const;
  float calculateDistanceToCenter(const cv::Point2f & image_point) const;

  const ArmorGeometry & geometry() const { return geometry_; }

 private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  ArmorGeometry geometry_;

  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}
