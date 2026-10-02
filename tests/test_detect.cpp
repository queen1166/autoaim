
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

const cv::Scalar kRedBar(255, 180, 180);
const cv::Scalar kBlueBar(180, 180, 255);

constexpr float kBarW = 12.0f;
constexpr float kBarH = 50.0f;
constexpr float kBarAngleDeg = 3.0f;

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

}

int main() {
  Detector::LightParams lp;
  Detector::ArmorParams ap;
  Detector detector(160, RED, lp, ap);

  const std::vector<cv::Point2f> twoBars = {{606, 525}, {746, 525}};

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

  std::printf("用例 2：单块灯条 → SINGLE 降级\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, {{606, 525}}));
    check(armors.size() == 1, "找到 1 块");
    if (armors.size() == 1) {
      check(armors[0].type == ArmorType::SINGLE, "类型是 SINGLE");
    }
  }

  std::printf("用例 3：关掉降级 → 单灯条不产出\n");
  {
    detector.single_light_fallback = false;
    auto armors = detector.detect(makeFrame(1440, 1080, {{606, 525}}));
    check(armors.empty(), "没有装甲板");
    detector.single_light_fallback = true;
  }

  std::printf("用例 4：中心距 12 倍灯条长 → 配不出对\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, {{200, 525}, {800, 525}}));
    check(!hasPair(armors), "没有配成 SMALL/LARGE 对");
  }

  std::printf("用例 5：蓝色灯条 + detect_color=RED → 不选中\n");
  {
    auto armors = detector.detect(makeFrame(1440, 1080, twoBars, kBlueBar));
    check(!hasPair(armors), "红蓝没写反");
  }

  std::printf("用例 6：纯红 (255,0,0) 灰度仅 76 < 阈值 160 → 抓不到\n");
  {
    auto armors = detector.detect(
        makeFrame(1440, 1080, twoBars, cv::Scalar(255, 0, 0)));
    check(!hasPair(armors), "太暗 → 二值化后为空 → 无装甲板");
  }

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
