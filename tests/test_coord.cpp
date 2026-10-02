
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>

#include "modules/solver/coord_transform.h"

using namespace autoaim::solver;

namespace {

int g_failed = 0;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

void checkNear(double got, double want, double tol, const char * what) {
  const bool ok = std::abs(got - want) < tol;
  std::printf("  %s %-34s got %+10.5f  want %+10.5f\n", ok ? "PASS" : "FAIL", what,
              got, want);
  if (!ok) g_failed++;
}

cv::Mat tvecOf(double x, double y, double z) {
  return (cv::Mat_<double>(3, 1) << x, y, z);
}

}

int main() {
  std::printf("用例 1：R_cam2gimbal 是正常旋转\n");
  {
    ExtrinsicConfig cfg;
    cfg.cam_pitch_deg = -15.0;
    cfg.cam_yaw_deg = 3.0;
    cfg.cam_roll_deg = 2.0;
    const Eigen::Matrix3d R = R_cam2gimbal(cfg);
    checkNear((R.transpose() * R - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12,
              "R^T R = I");
    checkNear(R.determinant(), 1.0, 1e-12, "det(R) = +1");
  }

  std::printf("用例 2：光轴方向 = cam_pitch_deg\n");
  {
    ExtrinsicConfig cfg;
    const Eigen::Vector3d axis = R_cam2gimbal(cfg) * Eigen::Vector3d(0, 0, 1);
    checkNear(axis.x(), std::cos(15 * kDeg2Rad), 1e-12, "光轴 x（前）");
    checkNear(axis.y(), 0.0, 1e-12, "光轴 y（左）");
    checkNear(axis.z(), -std::sin(15 * kDeg2Rad), 1e-12, "光轴 z（上，负数=下俯）");
  }

  std::printf("用例 3：相机 x(右) → 云台 -y\n");
  {
    ExtrinsicConfig cfg;
    cfg.cam_pitch_deg = 0.0;
    const Eigen::Vector3d right = R_cam2gimbal(cfg) * Eigen::Vector3d(1, 0, 0);
    checkNear(right.x(), 0.0, 1e-12, "相机右的 x 分量");
    checkNear(right.y(), -1.0, 1e-12, "相机右 → -y（云台 +y 是左）");
    checkNear(right.z(), 0.0, 1e-12, "相机右的 z 分量");

    const Eigen::Vector3d down = R_cam2gimbal(cfg) * Eigen::Vector3d(0, 1, 0);
    checkNear(down.z(), -1.0, 1e-12, "相机 y(下) → 云台 -z");
  }

  std::printf("用例 4：tvec=(0,0,5) 且相机下俯 15° → yaw=0°, pitch=-15°\n");
  {
    ExtrinsicConfig cfg;
    const Eigen::Vector3d p = cameraToGimbal(tvecOf(0, 0, 5.0), cfg);
    std::printf("       云台系坐标 = (%.4f, %.4f, %.4f)\n", p.x(), p.y(), p.z());
    checkNear(p.norm(), 5.0, 1e-9, "距离仍是 5m");

    const Eigen::Vector2d yp = yawPitchFromGimbalPoint(p);
    checkNear(yp.x() * kRad2Deg, 0.0, 1e-9, "yaw (deg)");
    checkNear(yp.y() * kRad2Deg, -15.0, 1e-9, "pitch (deg)");
  }

  std::printf("用例 5：相机光心偏移 5cm 会带来多大角度误差\n");
  {
    ExtrinsicConfig base;
    const Eigen::Vector3d p0 = cameraToGimbal(tvecOf(0, 0, 5.0), base);

    ExtrinsicConfig shifted = base;
    shifted.cam_y = 0.05;
    const Eigen::Vector3d p1 = cameraToGimbal(tvecOf(0, 0, 5.0), shifted);

    const Eigen::Vector2d a = yawPitchFromGimbalPoint(p0);
    const Eigen::Vector2d b = yawPitchFromGimbalPoint(p1);
    const double d_yaw = (b.x() - a.x()) * kRad2Deg;
    const double d_pitch = (b.y() - a.y()) * kRad2Deg;
    std::printf("       引起 yaw 偏移 %.3f°, pitch 偏移 %.3f°\n", d_yaw, d_pitch);
    std::printf("       → 5m 处 5cm 平移 ≈ 0.57°，与\"比安装角误差小一个量级\"一致\n");
    check(std::abs(d_yaw) < 1.0 && std::abs(d_pitch) < 1.0, "平移影响 < 1°");
  }

  std::printf("用例 6：gimbalDirFromAngles ↔ yawPitchFromGimbalPoint 往返\n");
  {
    const double yaws[] = {0.0, 30.0, -45.0, 179.0, -179.0};
    const double pitches[] = {0.0, 10.0, -15.0, -30.0};
    double worst = 0;
    for (double yd : yaws) {
      for (double pd : pitches) {
        const Eigen::Vector3d v = gimbalDirFromAngles(yd * kDeg2Rad, pd * kDeg2Rad);
        const Eigen::Vector2d yp = yawPitchFromGimbalPoint(v);
        const double e = std::max(std::abs(yp.x() - yd * kDeg2Rad),
                                  std::abs(yp.y() - pd * kDeg2Rad)) * kRad2Deg;
        worst = std::max(worst, e);
      }
    }
    std::printf("       最大往返误差 = %.2e 度\n", worst);
    check(worst < 1e-9, "往返一致");
  }

  {
    std::printf("用例 7：yawFromQuat 提取的是索引 [0]（yaw 不是 roll）\n");
    const Eigen::Quaterniond q(Eigen::AngleAxisd(30 * kDeg2Rad,
                                                  Eigen::Vector3d::UnitZ()));
    checkNear(yawFromQuat(q) * kRad2Deg, 30.0, 1e-9, "绕 z 转 30° → yaw = 30°");

    const Eigen::Quaterniond qx(Eigen::AngleAxisd(30 * kDeg2Rad,
                                                  Eigen::Vector3d::UnitX()));
    checkNear(yawFromQuat(qx) * kRad2Deg, 0.0, 1e-9, "绕 x 转 30° → yaw = 0°");
  }

  std::printf("用例 7b：yaw 全程扫描（含跨 ±π）\n");
  {
    double worst = 0;
    double worst_at = 0;
    for (double deg = -350.0; deg <= 350.0; deg += 5.0) {
      const double y = deg * kDeg2Rad;
      const Eigen::Quaterniond q(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));
      const double want = std::remainder(y, 2 * M_PI);
      const double got = yawFromQuat(q);
      const double err = std::abs(std::remainder(got - want, 2 * M_PI));
      if (err > worst) {
        worst = err;
        worst_at = deg;
      }
    }
    std::printf("       扫描 -350°~350°，最大误差 %.3e rad（出现在 %.0f°）\n", worst,
                worst_at);
    check(worst < 1e-9, "全程正确，没有在 ±π 处跳分支");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
