#pragma once
#include <opencv2/core.hpp>
#include <array>
#include <vector>
#include "modules/detect/armor.h"
namespace autoaim::solver {
// width  = 两根灯条【中心】之间的距离
// height = 灯条的【长度】
struct ArmorGeometry {
  float small_width = 132.0f;   // 小装甲板：两灯条中心距
  float small_height = 57.0f;   // 小装甲板：灯条长度
  float large_width = 223.0f;   // 大装甲板：两灯条中心距
  float large_height = 57.0f;   // 大装甲板：灯条长度
};

class PnPSolver {
 public:
  // 构造：把相机内参和装甲板几何存下来，并按几何预先算好
  // 大小装甲板的物体点坐标（只算一次，之后每帧复用）。
  //
  // camera_matrix 3x3 相机内参，行优先展平成 9 个 double
  // distortion_coefficients 畸变系数（k1 k2 p1 p2 k3）
  // geometry 大小装甲板的灯条几何尺寸（mm）
  //
  // 内参必须自己用棋盘格标定，默认值只是占位符。
  PnPSolver(const std::array<double, 9> & camera_matrix,
            const std::vector<double> & distortion_coefficients,
            const ArmorGeometry & geometry = ArmorGeometry{});

  // 解算装甲板位姿。rvec/tvec 在【相机光学系】下（x 右 / y 下 / z 前），
  // tvec 单位是米。
  //
  // 内部用 solvePnP_ITERATIVE（不用 IPPE —— 实测 IPPE 的 p95 误差差 10 倍）。
  //
  // armor    一块装甲板，用它四个角点当图像点，按 type 选对应的物体点
  // rvec     【输出】旋转向量（Rodrigues 形式），3x1
  // tvec     【输出】平移向量，3x1，单位米
  //
  // 返回 false 表示解算失败，原因有两种：
  //   · armor.type == INVALID（没配对成功的板子）
  //   · armor.type == SINGLE —— 单灯条是退化的，解不出位姿
  // 注意 rvec/tvec 在失败时不会被写入，调用方要先判返回值再用。
  bool solvePnP(const detect::Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const;
  // 把物体点重投影回图像，返回四个点的均方根像素误差。
  // 用来做质量闸门：误差过大说明检测的角点不可信，应丢弃该帧。
  //
  // armor    当时解算用的那块装甲板（提供图像点）
  // rvec/tvec solvePnP() 解出来的位姿
  //
  // 返回 RMS 像素误差，越小越好。调用方拿它和
  // SolverConfig::max_reprojection_error_px（默认 3.0）比，超了就丢这一帧。
  double reprojectionError(const detect::Armor & armor, const cv::Mat & rvec,
                           const cv::Mat & tvec) const;
  // 装甲板中心到图像主点的像素距离，用作瞄准优先级。
  // 多个候选时挑"离准星最近的那块"打。校内赛单靶板，基本用不上。
  //
  // image_point 装甲板中心在图像上的像素坐标
  // 返回像素距离，越小说明越靠近画面中心
  float calculateDistanceToCenter(const cv::Point2f & image_point) const;

  // 当前用的灯条几何参数（只读）。调试打印和反标定时要看。
  const ArmorGeometry & geometry() const { return geometry_; }

 private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  ArmorGeometry geometry_;

  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace autoaim::solver
