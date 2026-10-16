#include "modules/time/attitude_buffer.h"

namespace autoaim::timeutil {

void AttitudeBuffer::push(uint64_t t_ns, const Eigen::Quaterniond & q) {
  std::lock_guard<std::mutex> lock(m_);

  // 乱序或重复时间戳直接丢弃，保持队列按时间单调。
  if (!buf_.empty() && t_ns <= buf_.back().t_ns) return;

  buf_.push_back(Sample{t_ns, q.normalized()});
  while (buf_.size() > capacity_) buf_.pop_front();
}

bool AttitudeBuffer::query(uint64_t t_ns, Eigen::Quaterniond & out) const {
  std::lock_guard<std::mutex> lock(m_);
  if (buf_.empty()) return false;

  if (t_ns <= buf_.front().t_ns) {
    out = buf_.front().q;
    return true;
  }
  if (t_ns >= buf_.back().t_ns) {
    out = buf_.back().q;
    return true;
  }

  // 找到第一个 t >= t_ns 的样本，它和它前一个是夹住 t_ns 的一对。
  size_t hi = 1;
  while (hi < buf_.size() && buf_[hi].t_ns < t_ns) ++hi;
  if (hi >= buf_.size()) {
    out = buf_.back().q;
    return true;
  }

  const Sample & a = buf_[hi - 1];
  const Sample & b = buf_[hi];
  const double span = static_cast<double>(b.t_ns - a.t_ns);
  const double alpha = span > 0.0 ? static_cast<double>(t_ns - a.t_ns) / span : 0.0;

  // Eigen 的 slerp 内部会处理"取最短弧"（dot < 0 时翻转符号）。
  out = a.q.slerp(alpha, b.q).normalized();
  return true;
}

bool AttitudeBuffer::latest(uint64_t & t_ns, Eigen::Quaterniond & q) const {
  std::lock_guard<std::mutex> lock(m_);
  if (buf_.empty()) return false;
  t_ns = buf_.back().t_ns;
  q = buf_.back().q;
  return true;
}

size_t AttitudeBuffer::size() const {
  std::lock_guard<std::mutex> lock(m_);
  return buf_.size();
}

void AttitudeBuffer::clear() {
  std::lock_guard<std::mutex> lock(m_);
  buf_.clear();
}

}
