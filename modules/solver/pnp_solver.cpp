
#include "modules/solver/pnp_solver.h"

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "modules/common/units.h"

namespace autoaim::solver {

namespace {

// Eigen 默认列主序，直接 memcpy 成 cv::Mat 会转置。逐元素拷最省心。
cv::Mat eigenToCvMat(const Eigen::Matrix3d & R) {
  cv::Mat m(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) m.at<double>(r, c) = R(r, c);
  }
  return m;
}

}  // namespace

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

const std::vector<cv::Point3f> & PnPSolver::objectPointsFor(
    detect::ArmorType type) const {
  return type == detect::ArmorType::LARGE ? large_armor_points_ : small_armor_points_;
}

Eigen::Matrix3d PnPSolver::modelToCamera(double yaw_rad,
                                         const Eigen::Matrix3d & R_cam2world) const {
  const double pitch = yaw_fit_.armor_pitch_deg * kDeg2Rad;
  const double roll = yaw_fit_.armor_roll_deg * kDeg2Rad;

  // 装甲板模型系：+x 是法线、+y 是水平（两灯条连线方向）、+z 是竖直（灯条方向）。
  // 于是 yaw 是绕世界竖直轴转、pitch 绕 y 转、roll 绕 x 转 —— 教程 §5A.4 的约定。
  const Eigen::Matrix3d R_m2w =
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return R_cam2world.inverse() * R_m2w;
}

bool PnPSolver::solveTranslationForYaw(const std::vector<cv::Point3f> & obj,
                                       const std::vector<cv::Point2f> & img,
                                       const Eigen::Matrix3d & R_cam2world,
                                       double yaw_rad,
                                       Eigen::Vector3d & t_out) const {
  const Eigen::Matrix3d R_m2c = modelToCamera(yaw_rad, R_cam2world);

  const double fx = camera_matrix_.at<double>(0, 0);
  const double fy = camera_matrix_.at<double>(1, 1);
  const double cx = camera_matrix_.at<double>(0, 2);
  const double cy = camera_matrix_.at<double>(1, 2);
  if (std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9) return false;

  // 投影方程 (Q+t)_x − a·(Q+t)_z = 0、 (Q+t)_y − b·(Q+t)_z = 0
  // 展开后对 t **是线性的**：
  //     t_x − a·t_z = a·Q_z − Q_x
  //     t_y − b·t_z = b·Q_z − Q_y
  // 其中 a=(u−cx)/fx、b=(v−cy)/fy、Q = R_model2cam·P_i。
  //
  // 为什么必须每个候选姿态都重解一次 t：装甲板是平面，四点 PnP 有两个解，
  // 而且**两解的姿态和平移都不同**（不是简单差一个 π）。沿用 PnP 那一份 tvec
  // 去搜 yaw，一旦挑错支，怎么搜都对不上（实测残差能到 25 px）。
  const int n = static_cast<int>(obj.size());
  Eigen::MatrixXd A(2 * n, 3);
  Eigen::VectorXd rhs(2 * n);

  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3d Q =
        R_m2c * Eigen::Vector3d(obj[i].x, obj[i].y, obj[i].z);
    const double a = (img[i].x - cx) / fx;
    const double b = (img[i].y - cy) / fy;

    A(2 * i, 0) = 1.0;
    A(2 * i, 1) = 0.0;
    A(2 * i, 2) = -a;
    rhs(2 * i) = a * Q.z() - Q.x();

    A(2 * i + 1, 0) = 0.0;
    A(2 * i + 1, 1) = 1.0;
    A(2 * i + 1, 2) = -b;
    rhs(2 * i + 1) = b * Q.z() - Q.y();
  }

  const Eigen::Vector3d t = A.colPivHouseholderQr().solve(rhs);
  if (!t.allFinite()) return false;
  t_out = t;
  return true;
}

bool PnPSolver::visibleFromGimbal(const Eigen::Matrix3d & R_cam2world,
                                  const Eigen::Vector3d & los_world,
                                  double yaw_rad) const {
  if (!yaw_fit_.use_visibility_constraint) return true;

  const Eigen::Matrix3d R_m2c = modelToCamera(yaw_rad, R_cam2world);
  // 模型 +x（法线）在世界系下的方向
  const Eigen::Vector3d normal = R_cam2world * (R_m2c * Eigen::Vector3d::UnitX());

  // 可见角 z_to_v（教程 §5A.4"单板观测"）：装甲板法线与"云台→装甲板"视线的夹角。
  // 正对云台 → z_to_v = π，正是教程那个窗口的中心。
  //
  // ⚠️ 符号来自本工程的**左右标签约定**，不能凭直觉定：
  // 检测器按图像 x 排序标 left/right（`Armor` 的构造函数），而求解器把物体点
  // P0=(0,+hy,−hz) 分配给 left_light.bottom —— 于是"模型 +y"被**定义**成
  // "屏幕上朝左的那根灯条方向"。这块平面靶正对相机、且标签自洽时，
  // 模型 +x（法线）在状态约定里是**沿着视线指向外侧**的（z_to_v = π）。
  // 如果按直觉写成"法线指向相机"（z_to_v = 0），可见角窗口会把**所有**
  // 标签自洽的姿态全部否决掉 —— 现象是 fitYawSingle 永远返回 false。
  // 所以这里用 −n_h·los_h。
  //
  // 另外只在**水平面内**量这个角：靶板本身有固定的仰角（armor_pitch，RM 规范 15°），
  // 三维夹角直接比窗口的话正对时只有 π − 15°，够不着窗口。
  const Eigen::Vector3d los_h(los_world.x(), los_world.y(), 0.0);
  const Eigen::Vector3d n_h(normal.x(), normal.y(), 0.0);
  if (los_h.norm() < 1e-6 || n_h.norm() < 1e-6) return true;  // 退化，不拦

  const double cos_ang =
      std::clamp(-n_h.normalized().dot(los_h.normalized()), -1.0, 1.0);
  const double ang = std::acos(cos_ang);

  const double a_between = yaw_fit_.angle_between_armors_deg * kDeg2Rad;
  const double must_see = yaw_fit_.must_see_angle_deg * kDeg2Rad;
  const double must_not = yaw_fit_.must_not_see_angle_deg * kDeg2Rad;

  const double ang_min = std::max(kPi - must_not, kPi + must_see - a_between);
  const double ang_max = std::min(kPi + must_not, kPi - must_see + a_between);
  return ang >= ang_min && ang <= ang_max;
}

double PnPSolver::costAtYaw(const std::vector<cv::Point3f> & obj,
                            const std::vector<cv::Point2f> & img,
                            const Eigen::Matrix3d & R_cam2world,
                            const Eigen::Vector3d & los_world, double yaw_rad,
                            Eigen::Vector3d * t_out) const {
  if (!visibleFromGimbal(R_cam2world, los_world, yaw_rad)) {
    if (t_out) *t_out = Eigen::Vector3d::Zero();
    return 1e6;
  }

  Eigen::Vector3d t;
  if (!solveTranslationForYaw(obj, img, R_cam2world, yaw_rad, t)) return 1e9;
  if (t_out) *t_out = t;

  cv::Mat rvec;
  cv::Rodrigues(eigenToCvMat(modelToCamera(yaw_rad, R_cam2world)), rvec);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << t.x(), t.y(), t.z());

  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj, rvec, tvec, camera_matrix_, dist_coeffs_, proj);
  if (proj.size() != img.size()) return 1e9;

  double sum = 0.0;
  for (size_t i = 0; i < proj.size(); ++i) {
    const cv::Point2f d = proj[i] - img[i];
    sum += d.x * d.x + d.y * d.y;
  }
  return std::sqrt(sum / static_cast<double>(proj.size()));
}

bool PnPSolver::fitYawAround(const std::vector<cv::Point3f> & obj,
                             const std::vector<cv::Point2f> & img,
                             const Eigen::Matrix3d & R_cam2world,
                             const Eigen::Vector3d & los_world, double center_rad,
                             double & yaw_out_rad, Eigen::Vector3d & t_out,
                             double & cost_out) const {
  const double ext = yaw_fit_.search_extend_deg * kDeg2Rad;
  const double lo = center_rad - ext;
  const double hi = center_rad + ext;

  // 1) 粗扫，找到最低点所在的小区间 —— 避免三分细化被局部极小值困住。
  const int n = std::max(3, yaw_fit_.grid_samples);
  double best_yaw = center_rad;
  double best_cost = std::numeric_limits<double>::max();
  int best_i = 0;
  for (int i = 0; i < n; ++i) {
    const double y = lo + (hi - lo) * i / (n - 1);
    const double c = costAtYaw(obj, img, R_cam2world, los_world, y, nullptr);
    if (c < best_cost) {
      best_cost = c;
      best_yaw = y;
      best_i = i;
    }
  }
  if (best_cost > kMaxAcceptableCostPx) return false;  // 本窗口没有可用解

  // 2) 在最低点的左右邻格内三分细化。
  double a = lo + (hi - lo) * std::max(0, best_i - 1) / (n - 1);
  double b = lo + (hi - lo) * std::min(n - 1, best_i + 1) / (n - 1);
  for (int it = 0; it < yaw_fit_.trisection_iters && (b - a) > 1e-7; ++it) {
    const double m1 = a + (b - a) / 3.0;
    const double m2 = b - (b - a) / 3.0;
    if (costAtYaw(obj, img, R_cam2world, los_world, m1, nullptr) <
        costAtYaw(obj, img, R_cam2world, los_world, m2, nullptr)) {
      b = m2;
    } else {
      a = m1;
    }
  }

  const double y = 0.5 * (a + b);
  Eigen::Vector3d t;
  const double c = costAtYaw(obj, img, R_cam2world, los_world, y, &t);
  if (!std::isfinite(c) || c > best_cost) {
    // 细化反而更差 - 用粗扫的最低点
    yaw_out_rad = best_yaw;
    cost_out = best_cost;
    return solveTranslationForYaw(obj, img, R_cam2world, best_yaw, t_out);
  }

  yaw_out_rad = y;
  cost_out = c;
  t_out = t;
  return true;
}

bool PnPSolver::fitYawSingle(const detect::Armor & armor,
                             const Eigen::Matrix3d & R_cam2world, double & yaw_out_rad,
                             cv::Mat & tvec_out, double * cost_out,
                             double yaw_reference_rad, bool has_reference) const {
  if (armor.type == detect::ArmorType::SINGLE ||
      armor.type == detect::ArmorType::INVALID) {
    return false;
  }

  const auto & obj = objectPointsFor(armor.type);
  const std::vector<cv::Point2f> img = {armor.left_light.bottom, armor.left_light.top,
                                        armor.right_light.top, armor.right_light.bottom};

  // 搜索中心的初值：借一个普通 PnP 的 yaw 当提示即可，
  // 真正的搜索（每个候选都重解平移）不依赖提示值的正确性。
  double yaw_hint = 0.0;
  {
    std::vector<cv::Mat> rvecs, tvecs;
    int n_sol = 0;
    try {
      n_sol = cv::solvePnPGeneric(obj, img, camera_matrix_, dist_coeffs_, rvecs, tvecs,
                                  false, cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception &) {
      n_sol = 0;
    }
    if (n_sol <= 0) {
      cv::Mat rvec, tvec;
      if (cv::solvePnP(obj, img, camera_matrix_, dist_coeffs_, rvec, tvec, false,
                       cv::SOLVEPNP_ITERATIVE)) {
        rvecs = {rvec};
        tvecs = {tvec};
        n_sol = 1;
      }
    }
    if (n_sol <= 0) return false;

    cv::Mat R_cam;
    cv::Rodrigues(rvecs[0], R_cam);
    Eigen::Matrix3d R_cam_eig;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) R_cam_eig(r, c) = R_cam.at<double>(r, c);
    }
    const Eigen::Matrix3d R_w = R_cam2world * R_cam_eig;
    yaw_hint = std::atan2(R_w(1, 0), R_w(0, 0));
  }

  // 视线方向（世界系）：云台原点就是世界原点。
  // 它只用于可见角约束，用一个提示解算出来即可（方向对量级就够）。
  Eigen::Vector3d t_hint;
  if (!solveTranslationForYaw(obj, img, R_cam2world, yaw_hint, t_hint)) return false;
  const Eigen::Vector3d los_world = (R_cam2world * t_hint).normalized();
  if (!los_world.allFinite()) return false;

  // 两个候选中心：PnP 提示值，以及它加 π。
  // 平板 PnP 的法线朝前/朝后两解会让提示值整体偏 π，
  // 而搜索本身与提示值无关，所以两个都试一遍再取最优。
  struct Cand {
    double yaw = 0.0;
    double cost = 0.0;
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
  };
  std::vector<Cand> cands;

  for (int branch = 0; branch < 2; ++branch) {
    Cand c;
    if (!fitYawAround(obj, img, R_cam2world, los_world, yaw_hint + branch * kPi, c.yaw,
                      c.t, c.cost)) {
      continue;
    }
    cands.push_back(c);
  }
  if (cands.empty()) return false;

  double min_cost = std::numeric_limits<double>::max();
  for (const auto & c : cands) min_cost = std::min(min_cost, c.cost);

  // 代价相近的候选视为"歧义"。没有先验就取代价最低的；
  // 有先验（跟踪器当前的 yaw 估计）就取更靠近先验的那一支 ——
  // 否则平面靶板在正对时两解残差都接近 0，会在帧间来回翻。
  const double close_enough = std::max(1.25 * min_cost, min_cost + 0.3);

  const Cand * best = nullptr;
  for (const auto & c : cands) {
    if (c.cost > close_enough) continue;
    if (best == nullptr) {
      best = &c;
      continue;
    }
    if (!has_reference) {
      if (c.cost < best->cost) best = &c;
    } else {
      const double d_new =
          std::abs(std::remainder(c.yaw - yaw_reference_rad, kTwoPi));
      const double d_best =
          std::abs(std::remainder(best->yaw - yaw_reference_rad, kTwoPi));
      if (d_new < d_best) best = &c;
    }
  }
  if (best == nullptr) return false;

  const double best_cost = best->cost;
  const double best_yaw = best->yaw;
  const Eigen::Vector3d best_t = best->t;

  yaw_out_rad = best_yaw;
  tvec_out = (cv::Mat_<double>(3, 1) << best_t.x(), best_t.y(), best_t.z());
  if (cost_out) *cost_out = best_cost;
  return true;
}

bool PnPSolver::fitYawDouble(const detect::Armor & armor_a, const cv::Mat & tvec_a,
                             const detect::Armor & armor_b, const cv::Mat & tvec_b,
                             const Eigen::Matrix3d & R_cam2world,
                             double yaw_a_init_rad, double & yaw_a_out_rad,
                             double * cost_out) const {
  if (armor_a.type == detect::ArmorType::INVALID ||
      armor_b.type == detect::ArmorType::INVALID ||
      armor_a.type == detect::ArmorType::SINGLE ||
      armor_b.type == detect::ArmorType::SINGLE) {
    return false;
  }

  // 两块板的 yaw 相差 angle_between_armors。绕向未知，两种都试。
  const double delta = yaw_fit_.angle_between_armors_deg * kDeg2Rad;

  const auto & obj_a = objectPointsFor(armor_a.type);
  const auto & obj_b = objectPointsFor(armor_b.type);
  const std::vector<cv::Point2f> img_a = {
      armor_a.left_light.bottom, armor_a.left_light.top, armor_a.right_light.top,
      armor_a.right_light.bottom};
  const std::vector<cv::Point2f> img_b = {
      armor_b.left_light.bottom, armor_b.left_light.top, armor_b.right_light.top,
      armor_b.right_light.bottom};

  const Eigen::Vector3d los_a =
      (R_cam2world * Eigen::Vector3d(tvec_a.at<double>(0), tvec_a.at<double>(1),
                                     tvec_a.at<double>(2)))
          .normalized();
  const Eigen::Vector3d los_b =
      (R_cam2world * Eigen::Vector3d(tvec_b.at<double>(0), tvec_b.at<double>(1),
                                     tvec_b.at<double>(2)))
          .normalized();

  const double ext = yaw_fit_.search_extend_deg * kDeg2Rad;
  const double lo = yaw_a_init_rad - ext;
  const double hi = yaw_a_init_rad + ext;
  const int n = std::max(3, yaw_fit_.grid_samples);

  auto jointCost = [&](double yaw_a, int sign) {
    const double yaw_b = yaw_a + sign * delta;
    const double ca = costAtYaw(obj_a, img_a, R_cam2world, los_a, yaw_a, nullptr);
    const double cb = costAtYaw(obj_b, img_b, R_cam2world, los_b, yaw_b, nullptr);
    return std::max(ca, cb);  // 取较差的那块，防止一块被牺牲
  };

  double best_yaw = yaw_a_init_rad;
  double best_cost = std::numeric_limits<double>::max();
  int best_i = 0;
  int best_sign = +1;
  for (int sign : {+1, -1}) {
    for (int i = 0; i < n; ++i) {
      const double y = lo + (hi - lo) * i / (n - 1);
      const double c = jointCost(y, sign);
      if (c < best_cost) {
        best_cost = c;
        best_yaw = y;
        best_i = i;
        best_sign = sign;
      }
    }
  }
  if (best_cost > kMaxAcceptableCostPx) return false;

  double a = lo + (hi - lo) * std::max(0, best_i - 1) / (n - 1);
  double b = lo + (hi - lo) * std::min(n - 1, best_i + 1) / (n - 1);
  for (int it = 0; it < yaw_fit_.trisection_iters && (b - a) > 1e-7; ++it) {
    const double m1 = a + (b - a) / 3.0;
    const double m2 = b - (b - a) / 3.0;
    if (jointCost(m1, best_sign) < jointCost(m2, best_sign)) {
      b = m2;
    } else {
      a = m1;
    }
  }
  const double y = 0.5 * (a + b);
  const double c = jointCost(y, best_sign);
  if (c > best_cost) {
    yaw_a_out_rad = best_yaw;
    if (cost_out) *cost_out = best_cost;
  } else {
    yaw_a_out_rad = y;
    if (cost_out) *cost_out = c;
  }
  return true;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point) const {
  const float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
  const float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}
