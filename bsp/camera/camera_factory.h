
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
};

std::unique_ptr<CameraSource> makeCamera(const CameraSpec & spec);

bool hasHikSupport();

}
