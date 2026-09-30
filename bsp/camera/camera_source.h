// 取帧抽象。
//
// 本设计里最重要的一个抽象：把"图像从哪来"和"图像怎么处理"彻底分开。
// 两个实现：
//   ReplaySource —— 读录好的图像序列，离线调参用
//   HikCamera    —— 海康 MVS SDK，真机用
//
// 为什么 Replay 是一等公民而不是附加功能：
//   NUC 是共用设备、车很难约。没有离线回放能力的话，每调一个检测参数
//   都要排队等车。先把"录包 + 回放"做出来，才有不占车的调参手段。

#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <optional>

namespace autoaim::bsp {

struct Frame {
  cv::Mat image;          // 必须是 RGB（不是 BGR）——检测器按 RGB 解释通道
  uint64_t timestamp_ns = 0;  // 单调时钟，取帧时刻
};

class CameraSource {
 public:
  // 虚析构：上层只持有 CameraSource*（比如 AutoAimApp::camera_），
  // 用基类指针 delete 时必须能调到派生类的析构（关相机句柄）。
  // 少了 virtual 就只调 ~CameraSource()，HikCamera 的 SDK 句柄永远不释放。
  virtual ~CameraSource() = default;

  // 打开数据源（真机是开相机流，回放是扫描目录）。
  //
  // 返回 true 成功。失败的原因会打到 stderr，具体见各实现的 open()。
  virtual bool open() = 0;

  // 关闭数据源。要求 open() 过，且可以重复调用。
  // 析构函数里会自动调，正常流程不用手动调。
  virtual void close() = 0;

  // 数据源当前是否可用。grab() 之前要先看它。
  virtual bool isOpen() const = 0;

  // 阻塞直到拿到一帧，或超时。超时返回 nullopt。
  //
  // timeout_ms 最多等多久（毫秒）。回放源不真的等，按 fps 算好时间戳直接给。
  // 返回 nullopt 有三种可能：超时 / 播完了 / 这一帧坏了 ——
  // 要区分前两种，看 exhausted()。
  virtual std::optional<Frame> grab(int timeout_ms = 100) = 0;

  // 数据源是否已经播完（回放不循环时用）。真相机永远返回 false。
  // 没有这个的话，调用方分不清 grab() 返回 nullopt 是"超时""播完了"
  // 还是"这一帧坏了"，很容易写成忙等空转。
  virtual bool exhausted() const { return false; }

  // 数据源的名字，调试用（比如日志里区分这次跑的是真机还是回放）。
  // 例："HikCamera" / "ReplaySource"。返回的一定是静态字符串，不用管释放。
  virtual const char * name() const = 0;
};

// 单调时钟，纳秒。所有时间戳统一用它。
inline uint64_t nowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace autoaim::bsp
