
#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <optional>

namespace autoaim::bsp {

struct Frame {
  cv::Mat image;
  uint64_t timestamp_ns = 0;
};

class CameraSource {
 public:
  virtual ~CameraSource() = default;

  virtual bool open() = 0;

  virtual void close() = 0;

  virtual bool isOpen() const = 0;

  virtual std::optional<Frame> grab(int timeout_ms = 100) = 0;

  virtual bool exhausted() const { return false; }

  virtual const char * name() const = 0;
};

inline uint64_t nowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}
