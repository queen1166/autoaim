
#include "modules/solver/pnp_solver.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

namespace autoaim::solver {

PnPSolver::PnPSolver(const std::array<double, 9> & camera_matrix,
                     const std::vector<double> & distortion_coefficients,
                     const ArmorGeometry & geometry)
    : camera_matrix_(cv::Mat(3, 3, CV_64F,
                             const_cast<double *>(camera_matrix.data()))
                         .clone()),
      dist_coeffs_(distortion_coefficients.empty()
                       ? cv::Mat::zeros(1, 5, CV_64F)
                       : cv::Mat(1, static_cast<int>(distortion_coefficients.size()),
                                 CV_64F,
                                 const_cast<double *>(distortion_coefficients.data()))
                             .clone()),
      geometry_(geometry) {
  constexpr double kMmToM = 1.0 / 1000.0;
  const double small_half_y = geometry_.small_width * kMmToM / 2.0;
  const double small_half_z = geometry_.small_height * kMmToM / 2.0;
  const double large_half_y = geometry_.large_width * kMmToM / 2.0;
  const double large_half_z = geometry_.large_height * kMmToM / 2.0;

  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const detect::Armor & armor, cv::Mat & rvec,
                         cv::Mat & tvec) const {
  if (armor.type == detect::ArmorType::SINGLE) return false;
  if (armor.type == detect::ArmorType::INVALID) return false;

  std::vector<cv::Point2f> image_armor_points;
  image_armor_points.reserve(4);
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  const auto & object_points = armor.type == detect::ArmorType::SMALL
                                   ? small_armor_points_
                                   : large_armor_points_;

  return cv::solvePnP(object_points, image_armor_points, camera_matrix_, dist_coeffs_,
                      rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
}

double PnPSolver::reprojectionError(const detect::Armor & armor, const cv::Mat & rvec,
                                    const cv::Mat & tvec) const {
  if (armor.type == detect::ArmorType::SINGLE ||
      armor.type == detect::ArmorType::INVALID) {
    return -1.0;
  }

  const std::vector<cv::Point2f> image_armor_points = {
      armor.left_light.bottom, armor.left_light.top,
      armor.right_light.top, armor.right_light.bottom};

  const auto & object_points = armor.type == detect::ArmorType::SMALL
                                   ? small_armor_points_
                                   : large_armor_points_;

  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, reprojected);
  if (reprojected.size() != image_armor_points.size()) return -1.0;

  double sum_sq = 0;
  for (size_t i = 0; i < reprojected.size(); ++i) {
    const cv::Point2f d = reprojected[i] - image_armor_points[i];
    sum_sq += d.x * d.x + d.y * d.y;
  }
  return std::sqrt(sum_sq / static_cast<double>(reprojected.size()));
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point) const {
  const float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
  const float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}
