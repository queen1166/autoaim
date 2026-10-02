
#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

namespace autoaim::msg {

template <typename T>
class Latest {
 public:
  Latest() = default;

  void set(const T & v) {
    std::lock_guard<std::mutex> lock(m_);
    value_ = v;
    has_ = true;
    ++seq_;
  }

  std::optional<T> tryGet() const {
    std::lock_guard<std::mutex> lock(m_);
    if (!has_) return std::nullopt;
    return value_;
  }

  uint64_t seq() const {
    std::lock_guard<std::mutex> lock(m_);
    return seq_;
  }

  bool has() const {
    std::lock_guard<std::mutex> lock(m_);
    return has_;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(m_);
    has_ = false;
    value_ = T{};
    ++seq_;
  }

 private:
  mutable std::mutex m_;

  T value_{};
  bool has_ = false;
  uint64_t seq_ = 0;
};

}
