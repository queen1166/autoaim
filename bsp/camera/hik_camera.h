// 海康工业相机（MVS SDK）。
//
// ⚠️ 这个文件在本地【无法编译验证】—— 需要海康 MVS SDK。
//    CMake 检测到 SDK 才会把它加进构建（AUTOAIM_HAVE_MVS）。
//    第一次上机前，请对照官方示例核对 API：
//        /opt/MVS/Samples/64/C++/GrabImage/GrabImage.cpp
//    SDK 版本不同，个别函数签名有差异。
//
// ⚠️ 海康工业相机【不是 UVC 相机】。它不会出现在 /dev/video0，
//    cv::VideoCapture(0) 打不开。必须走 MVS SDK。

#pragma once

#include <string>
#include <vector>

#include "bsp/camera/camera_source.h"

namespace autoaim::bsp {

class HikCamera : public CameraSource {
 public:
  struct Config {
    // 曝光时间，单位【微秒】
    float exposure_us = 5000.0f;
    float gain = 10.0f;
    // ⭐ 必须关闭自动曝光。开着的话画面一有明暗变化它就重调，
    //    装甲板时亮时暗，检测会疯狂抖动。
    bool auto_exposure = false;
    // 取流超时
    int grab_timeout_ms = 1000;
  };

  // 构造：只记下配置，不碰硬件。要真开相机得调 open()。
  explicit HikCamera(const Config & cfg) : cfg_(cfg) {}

  // 析构：自动 close()，保证 MVS 的相机句柄一定被归还 ——
  // 不释放的话相机在进程退出后仍然被占用，下次要拔电源才能再用。
  ~HikCamera() override;

  // 枚举设备、按句柄打开、配好曝光/增益/像素格式、开始取流。
  // 返回 true 成功。任何一步失败都会清理已申请的资源并返回 false，
  // 失败原因打到 stderr。
  bool open() override;

  // 停流、关设备、销毁句柄。重复调用安全。
  void close() override;

  // 底层相机句柄是否还在。就是"open() 过且还没 close()"。
  bool isOpen() const override { return handle_ != nullptr; }

  // 取一帧。内部调 MV_CC_GetImageBuffer，把结果转成 RGB 存进 Frame。
  //
  // timeout_ms 单帧取流超时。实际生效的超时取 max(这个值, cfg_.grab_timeout_ms)。
  // 超时返回 nullopt。
  // 注意：相机原生给 BGR8 时会在这里做一次软件转换（见 is_rgb_native_）。
  std::optional<Frame> grab(int timeout_ms = 100) override;

  // 固定返回 "HikCamera"。日志里用来区分这次跑的是真机还是回放。
  const char * name() const override { return "HikCamera"; }

 private:
  Config cfg_;
  void * handle_ = nullptr;
  unsigned int payload_size_ = 0;
  int width_ = 0;
  int height_ = 0;
  // 相机是否原生给 RGB8。不支持时回退 BGR8 并在软件里转。
  bool is_rgb_native_ = false;
  std::vector<unsigned char> buffer_;
};

}  // namespace autoaim::bsp
