
#pragma once

#include <string>
#include <vector>

#include "bsp/camera/camera_source.h"

namespace autoaim::bsp {

class ReplaySource : public CameraSource {
 public:
  struct Config {
    std::string dir;
    double fps = 60.0;
    bool loop = true;
    bool convert_bgr_to_rgb = true;
  };

  explicit ReplaySource(const Config & cfg) : cfg_(cfg) {}

  bool open() override;

  void close() override;

  bool isOpen() const override { return opened_; }

  bool exhausted() const override {
    return opened_ && !cfg_.loop && cursor_ >= files_.size();
  }

  std::optional<Frame> grab(int timeout_ms = 100) override;

  const char * name() const override { return "ReplaySource"; }

  size_t frameCount() const { return files_.size(); }

  size_t cursor() const { return cursor_; }

 private:
  Config cfg_;
  std::vector<std::string> files_;
  size_t cursor_ = 0;
  bool opened_ = false;
  uint64_t next_frame_ns_ = 0;
};

std::string saveFrame(const std::string & dir, int index, const cv::Mat & rgb);

}
