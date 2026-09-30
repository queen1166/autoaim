// 检测调参工具：实时看检测结果，滑动条调二值化阈值。
//
// 现场光照和上游实验室完全不同，binary_thres 必须重调。
// 这个工具让你一边拖滑动条一边看角点，比改代码重编译快得多。
//
// 用法:
//   tool_detect_view <图像目录> [--no-loop]
//   tool_detect_view <单张图>
//
// 按键：q 退出，空格暂停/继续，[ ] 切换红蓝，s 保存当前帧

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdio>
#include <string>

#include "bsp/camera/camera_source.h"
#include "bsp/camera/replay_source.h"
#include "modules/detect/armor_detector.h"

using namespace autoaim;

namespace {

int g_thres = 160;
int g_color = 0;  // 0 = RED
bool g_paused = false;

// C++17 没有 std::string::ends_with
bool endsWith(const std::string & s, const std::string & suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

detect::Detector makeDetector() {
  detect::Detector::LightParams lp;
  detect::Detector::ArmorParams ap;
  return detect::Detector(g_thres, g_color, lp, ap);
}

}  // namespace

int main(int argc, char ** argv) {
  if (argc < 2) {
    std::printf("用法: %s <图像目录|单张图> [--no-loop]\n", argv[0]);
    return 1;
  }

  const std::string path = argv[1];
  bool loop = true;
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--no-loop") loop = false;
  }

  // 传的是单张图就直接读一次，传目录就走回放
  const bool single_image = endsWith(path, ".png") || endsWith(path, ".jpg") ||
                            endsWith(path, ".jpeg") || endsWith(path, ".bmp");

  if (single_image) {
    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
      std::fprintf(stderr, "读不到图: %s\n", path.c_str());
      return 1;
    }
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    cv::namedWindow("detect_view", cv::WINDOW_NORMAL);
    cv::createTrackbar("binary_thres", "detect_view", &g_thres, 255);
    cv::createTrackbar("color (0=R,1=B)", "detect_view", &g_color, 1);

    while (true) {
      auto detector = makeDetector();
      cv::Mat canvas_rgb = img.clone();
      auto armors = detector.detect(canvas_rgb);
      detector.drawResults(canvas_rgb);

      cv::Mat canvas;
      cv::cvtColor(canvas_rgb, canvas, cv::COLOR_RGB2BGR);

      // 左上角把二值图缩略图叠上去，直观看阈值效果
      if (!detector.binary_img.empty()) {
        cv::Mat small;
        cv::resize(detector.binary_img, small, cv::Size(320, 240));
        cv::Mat small_bgr;
        cv::cvtColor(small, small_bgr, cv::COLOR_GRAY2BGR);
        small_bgr.copyTo(canvas(cv::Rect(0, 0, small_bgr.cols, small_bgr.rows)));
      }

      char info[128];
      std::snprintf(info, sizeof(info), "thres=%d color=%d armors=%zu", g_thres,
                    g_color, armors.size());
      cv::putText(canvas, info, cv::Point(10, 270), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                  cv::Scalar(0, 255, 0), 2);

      cv::imshow("detect_view", canvas);
      const int key = cv::waitKey(30);
      if (key == 'q' || key == 27) break;
      if (key == 's') cv::imwrite("detect_view_snapshot.png", canvas);
    }
    return 0;
  }

  // 目录模式：回放
  bsp::ReplaySource::Config rc;
  rc.dir = path;
  rc.fps = 30.0;
  rc.loop = loop;
  rc.convert_bgr_to_rgb = true;

  bsp::ReplaySource cam(rc);
  if (!cam.open()) return 1;

  cv::namedWindow("detect_view", cv::WINDOW_NORMAL);
  cv::createTrackbar("binary_thres", "detect_view", &g_thres, 255);
  cv::createTrackbar("color (0=R,1=B)", "detect_view", &g_color, 1);

  size_t frame_no = 0;
  while (true) {
    auto frame = cam.grab(50);
    if (!frame) {
      if (!loop) {
        std::printf("回放结束，共 %zu 帧\n", frame_no);
        break;
      }
      continue;
    }
    frame_no++;

    if (g_paused) {
      cv::waitKey(10);
      continue;
    }

    auto detector = makeDetector();
    cv::Mat canvas_rgb = frame->image.clone();
    auto armors = detector.detect(canvas_rgb);
    detector.drawResults(canvas_rgb);

    cv::Mat canvas;
    cv::cvtColor(canvas_rgb, canvas, cv::COLOR_RGB2BGR);

    if (!detector.binary_img.empty()) {
      cv::Mat small, small_bgr;
      cv::resize(detector.binary_img, small, cv::Size(320, 240));
      cv::cvtColor(small, small_bgr, cv::COLOR_GRAY2BGR);
      small_bgr.copyTo(canvas(cv::Rect(0, 0, small_bgr.cols, small_bgr.rows)));
    }

    char info[160];
    std::snprintf(info, sizeof(info), "frame=%zu thres=%d color=%d armors=%zu%s",
                  frame_no, g_thres, g_color, armors.size(), g_paused ? " [PAUSED]" : "");
    cv::putText(canvas, info, cv::Point(10, 270), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);

    cv::imshow("detect_view", canvas);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
    if (key == ' ') g_paused = !g_paused;
    if (key == 's') {
      char name[64];
      std::snprintf(name, sizeof(name), "detect_snap_%06zu.png", frame_no);
      cv::imwrite(name, canvas);
      std::printf("已保存 %s\n", name);
    }
  }

  return 0;
}
