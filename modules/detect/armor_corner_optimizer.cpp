#include "modules/detect/armor_corner_optimizer.h"

#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoaim::detect {

namespace {

// 双线性采样；越界返回 -1 表示无效。
double sampleBilinear(const cv::Mat & gray, double x, double y) {
  if (x < 0 || y < 0 || x > gray.cols - 1 || y > gray.rows - 1) return -1.0;
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, gray.cols - 1);
  const int y1 = std::min(y0 + 1, gray.rows - 1);
  const double fx = x - x0, fy = y - y0;

  const double v00 = gray.at<uint8_t>(y0, x0);
  const double v01 = gray.at<uint8_t>(y0, x1);
  const double v10 = gray.at<uint8_t>(y1, x0);
  const double v11 = gray.at<uint8_t>(y1, x1);

  return (1 - fy) * ((1 - fx) * v00 + fx * v01) + fy * ((1 - fx) * v10 + fx * v11);
}

}  // namespace

bool ArmorCornerOptimizer::refineBar(const cv::Mat & rgb, const cv::Point2f & top,
                                     const cv::Point2f & bottom, cv::Point2f & top_out,
                                     cv::Point2f & bottom_out) const {
  if (rgb.empty()) return false;
  if (rgb.channels() != 3) return false;

  const cv::Point2f axis = bottom - top;
  const double len = cv::norm(axis);
  if (len < 6.0) return false;  // 太短，PCA 没有意义

  const cv::Point2f u = axis / static_cast<float>(len);
  const cv::Point2f mid = (top + bottom) * 0.5f;

  // 灯条的等效宽度 ≈ 0.3 × 长度（RM 灯条经验长宽比），给足外扩余量。
  const double bar_w = std::clamp(0.35 * len, 4.0, 60.0);
  const int roi_w = static_cast<int>(std::ceil(bar_w)) + 2 * cfg_.roi_margin_px;
  const int roi_h =
      static_cast<int>(std::ceil(len)) + 2 * cfg_.search_extra_px;

  // 把灯条摆正成竖直 ROI：u → 图像 +y 方向，mid → ROI 中心。
  cv::Mat M = (cv::Mat_<double>(2, 3) << u.y, -u.x,
               roi_w * 0.5 - (u.y * mid.x - u.x * mid.y), u.x, u.y,
               roi_h * 0.5 - (u.x * mid.x + u.y * mid.y));

  cv::Mat roi;
  cv::warpAffine(rgb, roi, M, cv::Size(roi_w, roi_h), cv::INTER_LINEAR,
                 cv::BORDER_REPLICATE);

  cv::Mat gray;
  cv::cvtColor(roi, gray, cv::COLOR_RGB2GRAY);

  // 1) 阈值化取出灯条像素（灯条中心常过曝，OTSU 一般能把灯条和背景分开）。
  cv::Mat mask;
  if (cfg_.light_thres > 0) {
    cv::threshold(gray, mask, cfg_.light_thres, 255, cv::THRESH_BINARY);
  } else {
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  }

  std::vector<cv::Point2f> pts;
  pts.reserve(static_cast<size_t>(roi_w) * roi_h / 2);
  for (int y = 0; y < gray.rows; ++y) {
    const uint8_t * row = mask.ptr<uint8_t>(y);
    for (int x = 0; x < gray.cols; ++x) {
      if (row[x]) pts.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
  }
  if (pts.size() < 8) return false;

  // 2) PCA：主成分就是灯条的对称轴。从整片点云里提方向，不吃阈值的偶然性。
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto & p : pts) mean += Eigen::Vector2d(p.x, p.y);
  mean /= static_cast<double>(pts.size());

  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
  for (const auto & p : pts) {
    const Eigen::Vector2d d = Eigen::Vector2d(p.x, p.y) - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(pts.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  Eigen::Vector2d v = es.eigenvectors().col(1);  // 最大特征值对应的方向
  if (v.y() < 0) v = -v;                         // 统一朝下，便于取"上/下端点"

  // 3) 沿轴投影得到灯条的上下两端（阈值层面的端点）。
  double t_lo = 1e9, t_hi = -1e9;
  for (const auto & p : pts) {
    const double t = (Eigen::Vector2d(p.x, p.y) - mean).dot(v);
    t_lo = std::min(t_lo, t);
    t_hi = std::max(t_hi, t);
  }
  if (t_hi - t_lo < 4.0) return false;

  // 4) 在端点附近 ±search_extra 找亮度梯度最大处 = 明暗交界。
  //    这一步把端点从"阈值边界"挪到"物理边缘"，是精修的关键。
  auto findEdge = [&](double t_start, double dir) {
    double best_t = t_start;
    double best_g = cfg_.min_gradient;
    for (double s = -static_cast<double>(cfg_.search_extra_px);
         s <= static_cast<double>(cfg_.search_extra_px); s += 0.5) {
      const double t = t_start + dir * s;
      const Eigen::Vector2d a = mean + (t - 0.5) * v;
      const Eigen::Vector2d b = mean + (t + 0.5) * v;
      const double ga = sampleBilinear(gray, a.x(), a.y());
      const double gb = sampleBilinear(gray, b.x(), b.y());
      if (ga < 0 || gb < 0) continue;
      const double g = std::abs(gb - ga);
      if (g > best_g) {
        best_g = g;
        best_t = t;
      }
    }
    return best_t;
  };

  const double t_top = findEdge(t_lo, -1.0);
  const double t_bot = findEdge(t_hi, +1.0);
  if (t_bot - t_top < 4.0) return false;

  const Eigen::Vector2d p_top = mean + t_top * v;
  const Eigen::Vector2d p_bot = mean + t_bot * v;

  // 5) 从 ROI 坐标变回原图坐标。
  cv::Mat Minv;
  cv::invertAffineTransform(M, Minv);
  auto back = [&](const Eigen::Vector2d & p) {
    return cv::Point2f(static_cast<float>(Minv.at<double>(0, 0) * p.x() +
                                          Minv.at<double>(0, 1) * p.y() +
                                          Minv.at<double>(0, 2)),
                       static_cast<float>(Minv.at<double>(1, 0) * p.x() +
                                          Minv.at<double>(1, 1) * p.y() +
                                          Minv.at<double>(1, 2)));
  };

  const cv::Point2f new_top = back(p_top);
  const cv::Point2f new_bot = back(p_bot);

  // 6) 跳变保护：精修把人带跑偏时保留原值。
  const bool top_ok = cv::norm(new_top - top) <= cfg_.max_shift_px;
  const bool bot_ok = cv::norm(new_bot - bottom) <= cfg_.max_shift_px;
  if (!top_ok && !bot_ok) return false;

  top_out = top_ok ? new_top : top;
  bottom_out = bot_ok ? new_bot : bottom;
  return true;
}

bool ArmorCornerOptimizer::refine(const cv::Mat & rgb, Armor & armor) const {
  if (!cfg_.enable) return false;
  if (armor.type == ArmorType::SINGLE || armor.type == ArmorType::INVALID) {
    return false;
  }

  cv::Point2f lt, lb, rt, rb;
  const bool l_ok = refineBar(rgb, armor.left_light.top, armor.left_light.bottom, lt, lb);
  const bool r_ok =
      refineBar(rgb, armor.right_light.top, armor.right_light.bottom, rt, rb);
  if (!l_ok && !r_ok) return false;

  if (l_ok) {
    armor.left_light.top = lt;
    armor.left_light.bottom = lb;
  }
  if (r_ok) {
    armor.right_light.top = rt;
    armor.right_light.bottom = rb;
  }

  for (auto * light : {&armor.left_light, &armor.right_light}) {
    light->length = cv::norm(light->top - light->bottom);
    light->center = (light->top + light->bottom) * 0.5f;
  }
  armor.center = (armor.left_light.center + armor.right_light.center) * 0.5f;
  armor.corner_refined = true;
  return true;
}

}
