// PnP 离线测试：合成投影 → solvePnP → 断言位姿被还原。
//
// 用例 3 是本文件最重要的一个：它量化了"装甲板尺寸填错会怎样"。
// 这直接对应方案 §0(4)：物体点用的是【灯条几何】，不是板子外形尺寸。

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "modules/solver/pnp_solver.h"

using namespace autoaim;
using namespace autoaim::solver;

namespace {

int g_failed = 0;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

const std::array<double, 9> kK = {1000, 0, 720, 0, 1000, 540, 0, 0, 1};
const std::vector<double> kD = {0, 0, 0, 0, 0};

// 让装甲板正对相机的旋转：模型 x(法线) → 相机 z，模型 y(左) → 相机 -x，模型 z(上) → 相机 -y
cv::Matx33d headOnRotation() {
  return cv::Matx33d(0, -1, 0,
                     0,  0, -1,
                     1,  0,  0);
}

// 生成一块装甲板的四个投影点，并按 PnP 要求的顺序返回
// {left.bottom, left.top, right.top, right.bottom}
std::vector<cv::Point2f> projectArmor(float width_mm, float height_mm, double dist) {
  const double hy = width_mm / 2.0 / 1000.0;
  const double hz = height_mm / 2.0 / 1000.0;
  const std::vector<cv::Point3f> obj = {
      {0, static_cast<float>(hy),  static_cast<float>(-hz)},
      {0, static_cast<float>(hy),  static_cast<float>(hz)},
      {0, static_cast<float>(-hy), static_cast<float>(hz)},
      {0, static_cast<float>(-hy), static_cast<float>(-hz)}};

  const cv::Matx33d R = headOnRotation();
  cv::Mat rvec;
  cv::Rodrigues(cv::Mat(R), rvec);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, dist);

  std::vector<cv::Point2f> img;
  cv::projectPoints(obj, rvec, tvec, cv::Mat(3, 3, CV_64F,
                     const_cast<double *>(kK.data())), cv::Mat(1, 5, CV_64F,
                     const_cast<double *>(kD.data())), img);

  // 图像点顺序 = {left.bottom, left.top, right.top, right.bottom}
  return {img[0], img[1], img[2], img[3]};
}

detect::Armor makeArmor(const std::vector<cv::Point2f> & p, detect::ArmorType type) {
  detect::Light left(cv::Rect(0, 0, 12, 50), p[1], p[0], 600, 0.0f);    // top, bottom
  detect::Light right(cv::Rect(0, 0, 12, 50), p[2], p[3], 600, 0.0f);
  detect::Armor armor(left, right);
  armor.type = type;
  return armor;
}

}  // namespace

int main() {
  std::printf("相机 fx=fy=1000, cx=720, cy=540；靶板正对相机，放置在 5.00 m\n\n");

  // ── 用例 1：标准小装甲板 132x57，正对 5m ─────────────────────
  std::printf("用例 1：132x57mm @ 5.00m，尺寸填对\n");
  {
    auto pts = projectArmor(132, 57, 5.0);
    auto armor = makeArmor(pts, detect::ArmorType::SMALL);
    check(armor.left_light.center.x < armor.right_light.center.x, "左右顺序正确");

    PnPSolver solver(kK, kD);
    cv::Mat rvec, tvec;
    bool ok = solver.solvePnP(armor, rvec, tvec);
    check(ok, "solvePnP 成功");
    if (ok) {
      const double d = cv::norm(tvec);
      std::printf("       解出距离 = %.4f m（真值 5.0000）\n", d);
      check(std::abs(d - 5.0) < 0.05, "距离误差 < 5cm");
      const double err = solver.reprojectionError(armor, rvec, tvec);
      std::printf("       重投影 RMS = %.4f px\n", err);
      check(err < 0.01, "重投影误差 < 0.01px");
    }
  }

  // ── 用例 2：SINGLE 必须拒绝 ──────────────────────────────────
  std::printf("用例 2：SINGLE 类型应被拒绝\n");
  {
    auto pts = projectArmor(132, 57, 5.0);
    auto armor = makeArmor(pts, detect::ArmorType::SINGLE);
    PnPSolver solver(kK, kD);
    cv::Mat rvec, tvec;
    check(!solver.solvePnP(armor, rvec, tvec), "solvePnP 返回 false");
  }

  // ── 用例 3：尺寸填错会怎样（方案 §0(4) 的量化）────────────────
  // 把 140mm 当板宽填进去，但真值是 132mm 的灯条中心距
  std::printf("用例 3：几何参数填错 —— 这是最常见的致命错误\n");
  {
    auto pts = projectArmor(132, 57, 5.0);   // 真实几何是 132
    auto armor = makeArmor(pts, detect::ArmorType::SMALL);

    for (float w : {132.0f, 140.0f, 120.0f}) {
      ArmorGeometry g;
      g.small_width = w;
      PnPSolver solver(kK, kD, g);
      cv::Mat rvec, tvec;
      if (solver.solvePnP(armor, rvec, tvec)) {
        const double d = cv::norm(tvec);
        std::printf("       填 %.0fmm → 解出 %.4f m  误差 %+.2f%%\n",
                    w, d, (d - 5.0) / 5.0 * 100.0);
      }
    }
    std::printf("       → 尺寸错 6%%，距离就错 6%%。必须实测反标，不能猜。\n");
  }

  // ── 用例 4：距离变化时尺度是否线性 ────────────────────────────
  std::printf("用例 4：不同距离下的解算\n");
  {
    PnPSolver solver(kK, kD);
    for (double d_true : {1.0, 3.0, 5.0, 8.0}) {
      auto pts = projectArmor(132, 57, d_true);
      auto armor = makeArmor(pts, detect::ArmorType::SMALL);
      cv::Mat rvec, tvec;
      if (solver.solvePnP(armor, rvec, tvec)) {
        const double d = cv::norm(tvec);
        std::printf("       真值 %.1f m → 解出 %.4f m\n", d_true, d);
        if (std::abs(d - d_true) > 0.05 * d_true) g_failed++;
      } else {
        g_failed++;
      }
    }
    std::printf("       （任一行超出 5%% 会体现在最后的失败计数里）\n");
  }

  // ── 用例 5：calculateDistanceToCenter ────────────────────────
  std::printf("用例 5：calculateDistanceToCenter\n");
  {
    PnPSolver solver(kK, kD);
    const float d0 = solver.calculateDistanceToCenter(cv::Point2f(720, 540));
    const float d1 = solver.calculateDistanceToCenter(cv::Point2f(820, 540));
    std::printf("       主点处 = %.2f（应 0），右偏 100px = %.2f（应 100）\n", d0, d1);
    check(std::abs(d0) < 1e-3 && std::abs(d1 - 100.0f) < 1e-2, "距离计算正确");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
