// 覆盖式共享槽 —— 线程间传"控制量"用的容器。
//
// ── 为什么不能用队列 ──────────────────────────────────────────
//
//   detect     60 fps  → 每秒产出 60 个目标角
//   serial_tx 100 Hz   → 每秒要发 100 帧
//
// 发送线程平均要在【同一个目标角】上重复发 1.67 次。
// 所以 get() 必须是"读"，不是"消费"—— 消费语义会让发送线程有
// 一半时间拿不到值，只能发零，云台就会一顿一顿的。
//
// 同时也不能用队列：检测慢一拍时，你不想把 3 帧前的旧目标一个个
// 补发出去，你只想发现在最新的那个。
//
//   → 控制量要"最新"（本文件）；数据流（图像帧、日志）才用有界队列。

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

namespace autoaim::msg {

// 一个线程安全的"单格信箱"：写者覆盖，读者取最新，读不消费。
//
// 所有 public 方法都自带锁，调用方不需要（也不应该）在外面再加锁。
// T 需要可拷贝 —— tryGet() 返回的是副本，不是引用。
template <typename T>
class Latest {
 public:
  // 初始状态是"还没有任何值"，此时 has() 为 false、tryGet() 返回 nullopt。
  Latest() = default;

  // 写者覆盖。永远只保留一份。
  // 已有值直接丢弃（这就是"最新"的语义，没有队列积压的概念）。
  void set(const T & v) {
    std::lock_guard<std::mutex> lock(m_);
    value_ = v;
    has_ = true;
    ++seq_;
  }

  // 读者取最新。【不消费】—— 可以反复读同一个值。
  // 返回 nullopt 表示"从来没有 set() 过"（或刚被 clear() 清掉）。
  // 返回的是【副本】：拿到之后写者再改不会影响你手上这份。
  std::optional<T> tryGet() const {
    std::lock_guard<std::mutex> lock(m_);
    if (!has_) return std::nullopt;
    return value_;
  }

  // 版本号，每次 set()/clear() 都 +1。用来判断"自上次之后有没有更新过"。
  //
  // 典型用法：处理线程记住上次处理过的 seq，相等就说明没新帧，跳过。
  // 比"比较值本身"可靠 —— 值可能恰好相同，但确实是新来的。
  uint64_t seq() const {
    std::lock_guard<std::mutex> lock(m_);
    return seq_;
  }

  // 现在里面有没有值。等价于 tryGet() 有没有返回 nullopt，
  // 但不做一次 T 的拷贝，判断"有没有数据"时更便宜。
  bool has() const {
    std::lock_guard<std::mutex> lock(m_);
    return has_;
  }

  // 清空：值归零、has() 变 false，并【推进版本号】。
  // 推进是故意的 —— 让只盯着 seq() 的读者也能察觉到"这里发生了变更"。
  void clear() {
    std::lock_guard<std::mutex> lock(m_);
    has_ = false;
    value_ = T{};
    ++seq_;
  }

 private:
  // mutable：tryGet()/has()/seq() 是 const 方法，但也要加锁改内部状态。
  mutable std::mutex m_;

  // 被锁保护的三件套：值本身、有没有值、版本号。
  T value_{};
  bool has_ = false;
  uint64_t seq_ = 0;
};

}  // namespace autoaim::msg
