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

    // 诊断用：>0 时在开流前把采集分辨率改成这个值；0 = 不动，用相机自己的。
    //
    // 为什么需要：Linux 内核给 USB 传输层的缓冲总量有上限（usbfs_memory_mb，
    // Ubuntu 默认 16 MB），而全分辨率 RGB8 一帧就要约 4.7 MB，SDK 取流时要同时
    // 囤好几帧，容易越界 → MV_CC_StartGrabbing 报 MV_E_RESOURCE(0x80000006)。
    // 压小分辨率**未必**能绕开（那 16 MB 未必按帧大小算），所以这是诊断开关，
    // 不是常规配置。真正的修法是让 root 调大 usbfs_memory_mb。
    int width = 0;
    int height = 0;
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
