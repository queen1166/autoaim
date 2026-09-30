#include "bsp/camera/hik_camera.h"

#ifdef AUTOAIM_HAVE_MVS

// 海康 MVS SDK。头文件在 /opt/MVS/include，库在 /opt/MVS/lib/64。
#include "MvCameraControl.h"

#include <opencv2/core.hpp>

#include <cstdio>
#include <cstring>

namespace autoaim::bsp {

namespace {

// 把 SDK 的错误码打印成人能看的东西
void printError(const char * what, int ret) {
  std::fprintf(stderr, "[hik] %s 失败，错误码 0x%08X\n", what,
               static_cast<unsigned int>(ret));
}

bool selectFirstDevice(MV_CC_DEVICE_INFO_LIST & list) {
  const int ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &list);
  if (ret != MV_OK) {
    printError("MV_CC_EnumDevices", ret);
    return false;
  }
  if (list.nDeviceNum == 0) {
    std::fprintf(stderr, "[hik] 没找到任何设备。检查：USB3 口 / 网线 / 供电 / 驱动\n");
    return false;
  }
  std::printf("[hik] 发现 %u 个设备\n", list.nDeviceNum);
  return true;
}

}  // namespace

HikCamera::~HikCamera() { close(); }

bool HikCamera::open() {
  close();

  MV_CC_DEVICE_INFO_LIST device_list;
  std::memset(&device_list, 0, sizeof(device_list));
  if (!selectFirstDevice(device_list)) return false;

  int ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    printError("MV_CC_CreateHandle", ret);
    handle_ = nullptr;
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    printError("MV_CC_OpenDevice", ret);
    close();
    return false;
  }

  // 关触发：连续采集
  ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
  if (ret != MV_OK) printError("SetEnumValue(TriggerMode)", ret);

  // ⭐ 关自动曝光
  ret = MV_CC_SetEnumValue(handle_, "ExposureAuto", cfg_.auto_exposure ? 2 : 0);
  if (ret != MV_OK) printError("SetEnumValue(ExposureAuto)", ret);

  ret = MV_CC_SetFloatValue(handle_, "ExposureTime", cfg_.exposure_us);
  if (ret != MV_OK) printError("SetFloatValue(ExposureTime)", ret);

  ret = MV_CC_SetFloatValue(handle_, "Gain", cfg_.gain);
  if (ret != MV_OK) printError("SetFloatValue(Gain)", ret);

  // 直接要 RGB8，省掉一次转换 —— 检测器要求 RGB 输入。
  // 若相机不支持该格式，SetEnumValue 会返回错误，此时下面按 BGR 处理并转换。
  const int pixel_ret =
      MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed);
  is_rgb_native_ = (pixel_ret == MV_OK);
  if (!is_rgb_native_) {
    std::fprintf(stderr,
                 "[hik] 相机不支持 RGB8，回退到 BGR8 并在软件里转换\n");
    ret = MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    if (ret != MV_OK) printError("SetEnumValue(PixelFormat)", ret);
  }

  MVCC_INTVALUE_EX int_value;
  std::memset(&int_value, 0, sizeof(int_value));
  if (MV_CC_GetIntValueEx(handle_, "PayloadSize", &int_value) == MV_OK) {
    payload_size_ = int_value.nCurValue;
  }

  MVCC_INTVALUE_EX w, h;
  std::memset(&w, 0, sizeof(w));
  std::memset(&h, 0, sizeof(h));
  if (MV_CC_GetIntValueEx(handle_, "Width", &w) == MV_OK) width_ = w.nCurValue;
  if (MV_CC_GetIntValueEx(handle_, "Height", &h) == MV_OK) height_ = h.nCurValue;

  if (payload_size_ == 0 || width_ <= 0 || height_ <= 0) {
    std::fprintf(stderr, "[hik] 拿不到分辨率/载荷大小 (%d x %d, payload %u)\n",
                 width_, height_, payload_size_);
    close();
    return false;
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    printError("MV_CC_StartGrabbing", ret);
    close();
    return false;
  }

  std::printf("[hik] 已开启取流：%d x %d，曝光 %.0f us，增益 %.1f，%s\n", width_,
              height_, cfg_.exposure_us, cfg_.gain,
              is_rgb_native_ ? "RGB8" : "BGR8(软件转换)");
  return true;
}

void HikCamera::close() {
  if (handle_ == nullptr) return;
  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
  payload_size_ = 0;
  width_ = height_ = 0;
}

std::optional<Frame> HikCamera::grab(int timeout_ms) {
  if (handle_ == nullptr || payload_size_ == 0) return std::nullopt;

  // 缓冲区按最大载荷分配，够用
  if (buffer_.size() < payload_size_) buffer_.resize(payload_size_);

  MV_FRAME_OUT_INFO_EX info;
  std::memset(&info, 0, sizeof(info));

  const unsigned int wait_ms =
      static_cast<unsigned int>(timeout_ms > 0 ? timeout_ms : cfg_.grab_timeout_ms);

  const int ret = MV_CC_GetOneFrameTimeout(handle_, buffer_.data(), payload_size_,
                                           &info, wait_ms);
  if (ret != MV_OK) return std::nullopt;  // 超时或丢帧，交给上层重试

  if (info.nWidth == 0 || info.nHeight == 0) return std::nullopt;

  // SDK 的缓冲一行可能带 padding，用 nWidth*nHeight*3 精确切出有效像素
  const size_t need = static_cast<size_t>(info.nWidth) * info.nHeight * 3;
  if (need > buffer_.size()) return std::nullopt;

  cv::Mat wrapped(info.nHeight, info.nWidth, CV_8UC3, buffer_.data());
  Frame f;

  if (is_rgb_native_) {
    f.image = wrapped.clone();
  } else {
    cv::cvtColor(wrapped, f.image, cv::COLOR_BGR2RGB);
  }

  f.timestamp_ns = nowNs();
  return f;
}

}  // namespace autoaim::bsp

#endif  // AUTOAIM_HAVE_MVS
