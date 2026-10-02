#include "bsp/camera/camera_factory.h"

#include <cstdio>

#include "bsp/camera/replay_source.h"

#ifdef AUTOAIM_HAVE_MVS
#include "bsp/camera/hik_camera.h"
#endif

namespace autoaim::bsp {

bool hasHikSupport() {
#ifdef AUTOAIM_HAVE_MVS
  return true;
#else
  return false;
#endif
}

std::unique_ptr<CameraSource> makeCamera(const CameraSpec & spec) {
  switch (spec.kind) {
    case CameraKind::REPLAY: {
      ReplaySource::Config rc;
      rc.dir = spec.replay_dir;
      rc.fps = spec.replay_fps;
      rc.loop = spec.replay_loop;
      rc.convert_bgr_to_rgb = true;
      return std::make_unique<ReplaySource>(rc);
    }

    case CameraKind::HIK: {
#ifdef AUTOAIM_HAVE_MVS
      HikCamera::Config hc;
      hc.exposure_us = spec.exposure_us;
      hc.gain = spec.gain;
      hc.auto_exposure = spec.auto_exposure;
      return std::make_unique<HikCamera>(hc);
#else
      std::fprintf(stderr,
                   "[camera] 本次构建不含海康支持（编译时没找到 MVS SDK）。\n"
                   "         用 --replay <目录> 走离线回放，或先装好 SDK 重新编译。\n"
                   "         SDK 默认在 /opt/MVS，装之前先问赛务——小电脑是共用设备。\n");
      return nullptr;
#endif
    }
  }
  return nullptr;
}

}
