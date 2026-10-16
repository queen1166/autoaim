#include "modules/detect/yolo_pose.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace autoaim::detect {

cv::Mat YoloPoseDetector::letterbox(const cv::Mat & src, int w, int h, double & ratio,
                                    int & pad_x, int & pad_y) {
  ratio = std::min(static_cast<double>(w) / src.cols,
                   static_cast<double>(h) / src.rows);

  const int new_w = static_cast<int>(std::round(src.cols * ratio));
  const int new_h = static_cast<int>(std::round(src.rows * ratio));

  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  pad_x = (w - new_w) / 2;
  pad_y = (h - new_h) / 2;

  cv::Mat out;
  // 补边值 114 是 ultralytics 的惯例：和训练时的 letterbox 保持一致，
  // 否则网络会把灰边当成内容，角点会往外飘。
  cv::copyMakeBorder(resized, out, pad_y, h - new_h - pad_y, pad_x, w - new_w - pad_x,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return out;
}

bool YoloPoseDetector::buildArmorFromKeypoints(const cv::Point2f kpts[4], Armor & out) {
  // 按 x 排序，前两个归左灯条，后两个归右灯条。
  std::array<int, 4> idx = {0, 1, 2, 3};
  std::sort(idx.begin(), idx.end(),
            [&](int a, int b) { return kpts[a].x < kpts[b].x; });

  auto pair_of = [&](int i, int j, cv::Point2f & top, cv::Point2f & bottom) {
    if (kpts[i].y <= kpts[j].y) {
      top = kpts[i];
      bottom = kpts[j];
    } else {
      top = kpts[j];
      bottom = kpts[i];
    }
  };

  cv::Point2f lt, lb, rt, rb;
  pair_of(idx[0], idx[1], lt, lb);
  pair_of(idx[2], idx[3], rt, rb);

  Light left(cv::Rect(static_cast<int>(std::min(lt.x, lb.x)),
                      static_cast<int>(std::min(lt.y, lb.y)),
                      std::max(1, static_cast<int>(std::abs(lt.x - lb.x))),
                      std::max(1, static_cast<int>(std::abs(lt.y - lb.y)))),
             lt, lb, 1, 0.0f);
  Light right(cv::Rect(static_cast<int>(std::min(rt.x, rb.x)),
                       static_cast<int>(std::min(rt.y, rb.y)),
                       std::max(1, static_cast<int>(std::abs(rt.x - rb.x))),
                       std::max(1, static_cast<int>(std::abs(rt.y - rb.y)))),
              rt, rb, 1, 0.0f);

  out = Armor(left, right);
  out.keypoints[KP_LEFT_TOP] = lt;
  out.keypoints[KP_LEFT_BOTTOM] = lb;
  out.keypoints[KP_RIGHT_TOP] = rt;
  out.keypoints[KP_RIGHT_BOTTOM] = rb;
  out.from_keypoints = true;
  return true;
}

std::vector<Armor> YoloPoseDetector::decode(const cv::Mat & output, double ratio,
                                            int pad_x, int pad_y, int orig_w,
                                            int orig_h, const YoloPoseConfig & cfg) {
  std::vector<Armor> armors;
  if (output.empty()) return armors;

  // 统一摊平成 [C, N]（batch 恒为 1）。
  const int channels = output.size[output.dims - 2];
  const int anchors = output.size[output.dims - 1];
  const int expect = 4 + cfg.num_classes + 4 * cfg.kpt_dim;
  if (channels != expect) {
    std::fprintf(stderr,
                 "[yolo] 输出通道数 %d 与配置不符（%s 布局下期望 4 + %d 类 + 4×%d = %d）。\n"
                 "       检查 --yolo-classes / --yolo-kpt-dim / --yolo-layout 是否和模型匹配。\n",
                 channels,
                 cfg.head_layout == HeadLayout::COLOR_CLASS_KPT ? "color-class-kpt"
                                                                : "ultralytics",
                 cfg.num_classes, cfg.kpt_dim, expect);
    return armors;
  }

  cv::Mat out2d = output.reshape(1, channels);

  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  std::vector<cv::Point2f> kpt_store;  // 4 点 × 每个候选

  for (int n = 0; n < anchors; ++n) {
    float best_cls = 0.0f;
    for (int c = 0; c < cfg.num_classes; ++c) {
      best_cls = std::max(best_cls, out2d.at<float>(4 + c, n));
    }
    if (best_cls < cfg.conf_thres) continue;

    cv::Point2f kpts[4];
    bool ok = true;
    for (int k = 0; k < 4; ++k) {
      const int base = 4 + cfg.num_classes + k * cfg.kpt_dim;
      kpts[k] = cv::Point2f(out2d.at<float>(base, n), out2d.at<float>(base + 1, n));
      if (cfg.kpt_dim >= 3 && out2d.at<float>(base + 2, n) < cfg.kpt_conf_thres) {
        ok = false;
      }
    }
    if (!ok) continue;

    // 框要在 letterbox 坐标系里算好，下面那段统一反变换才同样适用。
    if (cfg.head_layout == HeadLayout::COLOR_CLASS_KPT) {
      // 这个头没有 bbox 通道，前 4 通道是颜色分数，只能拿关键点的外接矩形当框。
      // 直接读那 4 个分数会把所有框退化成原点附近 1px 的小框，NMS 互相压制。
      boxes.emplace_back(cv::boundingRect(std::vector<cv::Point2f>(kpts, kpts + 4)));
    } else {
      const double cx = out2d.at<float>(0, n);
      const double cy = out2d.at<float>(1, n);
      const double w = out2d.at<float>(2, n);
      const double h = out2d.at<float>(3, n);
      boxes.emplace_back(cx - w / 2.0, cy - h / 2.0, w, h);
    }
    scores.push_back(best_cls);

    // letterbox 反变换回原图坐标。
    auto inv = [&](const cv::Point2f & p) {
      return cv::Point2f(static_cast<float>((p.x - pad_x) / ratio),
                         static_cast<float>((p.y - pad_y) / ratio));
    };
    for (auto & p : kpts) p = inv(p);

    for (const auto & p : kpts) kpt_store.push_back(p);
  }

  if (boxes.empty()) return armors;

  // box 坐标也要反变换后再做 NMS，否则补边区域的框会挤在一起。
  for (auto & b : boxes) {
    b.x = (b.x - pad_x) / ratio;
    b.y = (b.y - pad_y) / ratio;
    b.width /= ratio;
    b.height /= ratio;
  }

  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, cfg.conf_thres, cfg.iou_thres, keep);

  for (int k : keep) {
    Armor a;
    if (!buildArmorFromKeypoints(&kpt_store[static_cast<size_t>(k) * 4], a)) continue;
    a.confidence = scores[static_cast<size_t>(k)];
    a.type = (boxes[static_cast<size_t>(k)].width /
                  std::max(1.0, boxes[static_cast<size_t>(k)].height) >
              cfg.large_aspect_ratio)
                 ? ArmorType::LARGE
                 : ArmorType::SMALL;
    armors.push_back(a);
  }

  (void)orig_w;
  (void)orig_h;
  return armors;
}

bool YoloPoseDetector::load() {
  loaded_ = false;
  if (cfg_.model_path.empty()) return false;

  // 先单独判存在性：readNetFromONNX 对"文件不存在"和"文件格式不对"抛的是同一类异常，
  // 分不出来。默认模型路径是相对的，从 build/ 里跑就会落空——这是最常见的失败原因，
  // 所以把绝对路径打出来，一眼能看出是当前目录不对。
  std::error_code ec;
  if (!std::filesystem::exists(cfg_.model_path, ec)) {
    std::fprintf(stderr,
                 "[yolo] 找不到模型文件: %s\n"
                 "       （绝对路径 %s）\n"
                 "       该文件被 .gitignore 排除，新 clone 的仓库里没有，见 "
                 "models/pretrained/README.md。\n",
                 cfg_.model_path.c_str(),
                 std::filesystem::absolute(cfg_.model_path, ec).string().c_str());
    return false;
  }

  try {
    net_ = cv::dnn::readNetFromONNX(cfg_.model_path);
  } catch (const cv::Exception & e) {
    std::fprintf(stderr, "[yolo] 加载模型失败: %s\n       %s\n", cfg_.model_path.c_str(),
                 e.what());
    return false;
  }
  if (net_.empty()) {
    std::fprintf(stderr, "[yolo] 模型为空: %s\n", cfg_.model_path.c_str());
    return false;
  }

  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  loaded_ = true;
  std::printf("[yolo] 已加载 %s（输入 %dx%d，%d 类，关键点 %d 维，%s 布局，输入缩放 %.6g）\n",
              cfg_.model_path.c_str(), cfg_.input_w, cfg_.input_h, cfg_.num_classes,
              cfg_.kpt_dim,
              cfg_.head_layout == HeadLayout::COLOR_CLASS_KPT ? "color-class-kpt"
                                                             : "ultralytics",
              static_cast<double>(cfg_.input_scale));
  return true;
}

std::vector<Armor> YoloPoseDetector::detect(const cv::Mat & rgb) {
  last_armors_.clear();
  if (!loaded_ || rgb.empty()) return last_armors_;

  double ratio = 1.0;
  int pad_x = 0, pad_y = 0;
  const cv::Mat input = letterbox(rgb, cfg_.input_w, cfg_.input_h, ratio, pad_x, pad_y);

  // Frame::image 是 RGB，所以默认不交换 R/B（见 YoloPoseConfig::input_is_bgr）。
  cv::Mat blob = cv::dnn::blobFromImage(input, cfg_.input_scale,
                                        cv::Size(cfg_.input_w, cfg_.input_h),
                                        cv::Scalar(), cfg_.input_is_bgr, false, CV_32F);

  net_.setInput(blob);
  cv::Mat output = net_.forward();

  last_armors_ = decode(output, ratio, pad_x, pad_y, rgb.cols, rgb.rows, cfg_);
  return last_armors_;
}

void YoloPoseDetector::drawResults(cv::Mat & img) {
  for (const auto & a : last_armors_) {
    const cv::Scalar color = a.type == ArmorType::LARGE ? cv::Scalar(0, 255, 0)
                                                        : cv::Scalar(255, 128, 0);
    cv::line(img, a.left_light.top, a.right_light.top, color, 2);
    cv::line(img, a.left_light.bottom, a.right_light.bottom, color, 2);
    cv::line(img, a.left_light.top, a.left_light.bottom, color, 2);
    cv::line(img, a.right_light.top, a.right_light.bottom, color, 2);
    for (int i = 0; i < 4; ++i) {
      cv::circle(img, a.keypoints[i], 3, cv::Scalar(0, 255, 255), -1);
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", a.confidence);
    cv::putText(img, buf, a.center, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 255), 1);
  }
}

}
