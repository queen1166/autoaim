#include "bsp/camera/replay_source.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <thread>

namespace autoaim::bsp {

namespace {

bool hasImageExt(const std::string & p) {
  const size_t dot = p.find_last_of('.');
  if (dot == std::string::npos) return false;
  std::string ext = p.substr(dot);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" ||
         ext == ".tif" || ext == ".tiff";
}

}  // namespace

std::string saveFrame(const std::string & dir, int index, const cv::Mat & rgb) {
  if (rgb.empty()) return {};
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);

  char name[64];
  std::snprintf(name, sizeof(name), "%06d.png", index);
  const std::string path = (std::filesystem::path(dir) / name).string();

  // cv::imwrite 按 BGR 解释写入，所以先把 RGB 转成 BGR，
  // 这样读回来再转一次就还原了。
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  if (!cv::imwrite(path, bgr)) return {};
  return path;
}

bool ReplaySource::open() {
  files_.clear();
  cursor_ = 0;

  std::error_code ec;
  if (!std::filesystem::is_directory(cfg_.dir, ec)) {
    std::fprintf(stderr, "[replay] 目录不存在: %s\n", cfg_.dir.c_str());
    return false;
  }

  for (const auto & entry : std::filesystem::directory_iterator(cfg_.dir, ec)) {
    if (!entry.is_regular_file()) continue;
    const std::string p = entry.path().string();
    if (hasImageExt(p)) files_.push_back(p);
  }

  // 按文件名排序 —— 保存时用的是 %06d 序号，所以字典序 == 时间序
  std::sort(files_.begin(), files_.end());

  if (files_.empty()) {
    std::fprintf(stderr, "[replay] 目录里没有图像: %s\n", cfg_.dir.c_str());
    return false;
  }

  opened_ = true;
  next_frame_ns_ = nowNs();
  std::printf("[replay] 载入 %zu 帧，回放帧率 %.1f fps\n", files_.size(), cfg_.fps);
  return true;
}

void ReplaySource::close() {
  files_.clear();
  cursor_ = 0;
  opened_ = false;
}

std::optional<Frame> ReplaySource::grab(int timeout_ms) {
  if (!opened_ || files_.empty()) return std::nullopt;

  // 按配置的帧率限速，让回放的时序和真相机一致（dt_ 才有意义）
  const uint64_t period_ns =
      cfg_.fps > 0.1 ? static_cast<uint64_t>(1e9 / cfg_.fps) : 0;

  if (period_ns > 0) {
    const uint64_t now = nowNs();
    if (now < next_frame_ns_) {
      const uint64_t wait_ns = next_frame_ns_ - now;
      if (wait_ns > static_cast<uint64_t>(timeout_ms) * 1000000ULL) {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
        return std::nullopt;
      }
      std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
    }
    next_frame_ns_ = std::max(nowNs(), next_frame_ns_) + period_ns;
  }

  if (cursor_ >= files_.size()) {
    if (!cfg_.loop) return std::nullopt;
    cursor_ = 0;
  }

  cv::Mat img = cv::imread(files_[cursor_], cv::IMREAD_COLOR);
  cursor_++;
  if (img.empty()) {
    // 读失败就跳过这一张，不要卡住整条流水线
    return std::nullopt;
  }

  if (cfg_.convert_bgr_to_rgb) {
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  }

  Frame f;
  f.image = img;
  f.timestamp_ns = nowNs();
  return f;
}

}  // namespace autoaim::bsp
