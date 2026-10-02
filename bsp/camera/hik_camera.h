#pragma once

#include <string>
#include <vector>

#include "bsp/camera/camera_source.h"

namespace autoaim::bsp {

class HikCamera : public CameraSource {
 public:
  struct Config {
    float exposure_us = 5000.0f;
    float gain = 10.0f;
    bool auto_exposure = false;
    int grab_timeout_ms = 1000;
  };

  explicit HikCamera(const Config & cfg) : cfg_(cfg) {}

  ~HikCamera() override;

  bool open() override;

  void close() override;

  bool isOpen() const override { return handle_ != nullptr; }

  std::optional<Frame> grab(int timeout_ms = 100) override;

  const char * name() const override { return "HikCamera"; }

 private:
  Config cfg_;
  void * handle_ = nullptr;
  unsigned int payload_size_ = 0;
  int width_ = 0;
  int height_ = 0;
  bool is_rgb_native_ = false;
  std::vector<unsigned char> buffer_;
};

}
