// 检测链离线测试：合成图像 → Detector → 断言找到预期的装甲板。
//
// 不碰硬件，可以在任何有 OpenCV 的机器上跑。合成图象是确定性的，
// 所以这个测试能锁住参数改动和几个隐蔽的约定。
//
// 写这个测试的过程中踩到了三个真实的坑，全部写进下面的注释里 ——
// 它们同样会在真机上坑你。

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

#include "modules/detect/armor_detector.h"

using namespace autoaim::detect;

namespace {

int g_failed = 0;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

// ── 灯条颜色 ────────────────────────────────────────────────
// 坑 1：灰度必须 > binary_thres(160)，否则二值化后什么都没有。
//       纯红 (255,0,0) 的灰度只有 0.299*255 ≈ 76，远低于阈值 —— 抓不到。
//       真实装甲板灯条是过曝的 LED，接近白色但带色调，灰度很高。
//       这也说明 binary_thres 实际定义的是"多亮才算灯条"，
//       而这个和曝光、补光、环境光强相关 —— 必须在现场重调。
const cv::Scalar kRedBar(255, 180, 180);   // RGB，灰度 ≈ 202
const cv::Scalar kBlueBar(180, 180, 255);  // RGB，灰度 ≈ 189

constexpr float kBarW = 12.0f;
constexpr float kBarH = 50.0f;
constexpr float kBarAngleDeg = 3.0f;  // 见坑 3

// 在纯黑图上画灯条。必须是 RGB —— 检测器用 channel[0] 当 R，channel[2] 当 B。
//
// 坑 2：完美轴对齐矩形的 findContours 结果只有 4 个点，
//       会被上游 findLights 的 `contour.size() < 5` 过滤掉（那个过滤是挡退化轮廓的）。
//       真实相机有镜头模糊、抗锯齿、sensor 噪声，轮廓点远多于 5 个。
//
// 坑 3：只加高斯模糊还不够 —— 阈值化会把模糊边缘重新切成干净矩形，
//       仍然是 4 个点。必须让灯条【稍微转一点】，边界成为阶梯状，
//       轮廓点数才会上去（实测：轴对齐 4 点，转 3° 变 20 点）。
//       上游算法隐含要求灯条不是完美轴对齐的；现实中总是成立。
cv::Mat makeFrame(int w, int h, const std::vector<cv::Point2f> & centers,
                  const cv::Scalar & color = kRedBar) {
  cv::Mat img(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  for (const auto & c : centers) {
    const cv::RotatedRect rr(c, cv::Size2f(kBarW, kBarH), kBarAngleDeg);
    cv::Point2f corners[4];
    rr.points(corners);
    std::vector<cv::Point> poly(corners, corners + 4);
    cv::fillConvexPoly(img, poly, color);
  }
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0.8);
  return img;
}

bool hasPair(const std::vector<Armor> & armors) {
  for (const auto & a : armors) {
    if (a.type == ArmorType::SMALL || a.type == ArmorType::LARGE) return true;
  }
  return false;
}

}  // namespace

int main() {
  Detector::LightParams lp;  // 默认 0.1 / 0.4 / 35.0 / 0.8
  Detector::ArmorParams ap;  // 默认 0.7 / 0.8 / 3.2 / 3.2 / 5.5 / 35.0
  Detector detector(160, RED, lp, ap);

  // 灯条中心距 140，平均灯条长 50 → center_distance = 2.8
  // 落在 [0.8, 3.2) → 判为 SMALL
  const std::vector<cv::Point2f> twoBars = {{606, 525}, {746, 525}};

  // ── 用例 1：两块灯条应当配成一块 SMALL 装甲板 ──────────────
  std::printf("用例 1：两块灯条 → 一块 SMALL 装甲板\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, twoBars));
    check(armors.size() == 1, "找到 1 块装甲板");
    if (armors.size() == 1) {
      check(armors[0].type == ArmorType::SMALL, "类型是 SMALL");
      check(armors[0].left_light.center.x < armors[0].right_light.center.x,
            "左右灯条排序正确");
      check(std::abs(armors[0].left_light.length - kBarH) < 3.0,
            "灯条长度接近 50px");
      check(armors[0].left_light.color == RED, "颜色判为 RED");
    }
  }

  // ── 用例 2：只画一块灯条 → 走 SINGLE 降级 ────────────────────
  std::printf("用例 2：单块灯条 → SINGLE 降级\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, {{606, 525}}));
    check(armors.size() == 1, "找到 1 块");
    if (armors.size() == 1) {
      check(armors[0].type == ArmorType::SINGLE, "类型是 SINGLE");
    }
  }

  // ── 用例 3：关掉降级后，单灯条不应产出装甲板 ─────────────────
  std::printf("用例 3：关掉降级 → 单灯条不产出\n");
  {
    detector.single_light_fallback = false;
    auto armors = detector.detect(makeFrame(1440, 1080, {{606, 525}}));
    check(armors.empty(), "没有装甲板");
    detector.single_light_fallback = true;
  }

  // ── 用例 4：中心距过大 → 不应配成装甲板对 ────────────────────
  // 中心距 600，灯条长 50 → center_distance 12.0，超出 [0.8, 5.5]
  std::printf("用例 4：中心距 12 倍灯条长 → 配不出对\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, {{200, 525}, {800, 525}}));
    check(!hasPair(armors), "没有配成 SMALL/LARGE 对");
  }

  // ── 用例 5：蓝灯条在 detect_color=RED 时不应被选中 ───────────
  std::printf("用例 5：蓝色灯条 + detect_color=RED → 不选中\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, twoBars, kBlueBar));
    check(!hasPair(armors), "红蓝没写反");
  }

  // ── 用例 6：过暗的灯条应抓不到（阈值的实际含义）──────────────
  std::printf("用例 6：纯红 (255,0,0) 灰度仅 76 < 阈值 160 → 抓不到\n");
  {
    auto armors = detector.detect(
        makeFrame(1440, 1080, twoBars, cv::Scalar(255, 0, 0)));
    check(!hasPair(armors), "太暗 → 二值化后为空 → 无装甲板");
  }

  // ── 用例 7：中心距变了，类型判定要跟着变 ─────────────────────
  // 中心距 200，灯条长 50 → center_distance 4.0，落在 [3.2, 5.5) → LARGE
  std::printf("用例 7：中心距 4 倍灯条长 → 判为 LARGE\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, {{620, 525}, {820, 525}}));
    check(armors.size() == 1, "找到 1 块");
    if (armors.size() == 1) {
      check(armors[0].type == ArmorType::LARGE, "类型是 LARGE");
    }
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
