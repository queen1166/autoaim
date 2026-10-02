
#pragma once

#include <memory>
#include <string>

#include "bsp/camera/camera_source.h"

namespace autoaim::bsp {

enum class CameraKind { REPLAY, HIK };

struct CameraSpec {
  CameraKind kind = CameraKind::REPLAY;

  std::string replay_dir;
  double replay_fps = 60.0;
  bool replay_loop = true;

  float exposure_us = 5000.0f;
  float gain = 10.0f;
  bool auto_exposure = false;

  // 海康相机诊断用：>0 时压小采集分辨率，用于绕开 usbfs_memory_mb 上限。
  // 理由与风险见 HikCamera::Config 里同名成员的注释。
  int width = 0;
  int height = 0;
};

std::unique_ptr<CameraSource> makeCamera(const CameraSpec & spec);

bool hasHikSupport();

}
