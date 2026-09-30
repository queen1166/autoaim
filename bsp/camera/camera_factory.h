// 按配置造相机。把 #ifdef 关在这里，main.cpp 保持干净。
//
// 海康 SDK 缺失时，选 hik 会在运行时明确报错，而不是编译失败 ——
// 这样在没有 SDK 的机器上依然能跑 replay 模式做离线调参。

#pragma once

#include <memory>
#include <string>

#include "bsp/camera/camera_source.h"

namespace autoaim::bsp {

enum class CameraKind { REPLAY, HIK };

struct CameraSpec {
  CameraKind kind = CameraKind::REPLAY;

  // REPLAY
  std::string replay_dir;
  double replay_fps = 60.0;
  bool replay_loop = true;

  // HIK
  float exposure_us = 5000.0f;
  float gain = 10.0f;
  bool auto_exposure = false;
};

// 工厂：按 spec.kind 造出对应的相机对象。
//
// 返回的是抽象基类指针，调用方【不需要知道】背后是真机还是回放 ——
// 这正是 CameraSource 这层抽象存在的意义。
//
// 返回 nullptr 表示创建失败（会往 stderr 打印原因），比如：
//   spec.kind == HIK 但这个构建没带 MVS SDK（AUTOAIM_HAVE_MVS 为假）。
// 注意它只负责"造"，不负责"开" —— 拿到指针后还要自己调 open()。
std::unique_ptr<CameraSource> makeCamera(const CameraSpec & spec);

// 编译期是否带海康支持（即 CMake 有没有找到 MVS SDK）。
//
// 这个值在编译时就定死了，运行期不会变。main.cpp 用它来做启动检查：
// 用户选了 --hik 但构建里没有，就直接报错退出，而不是等到 open() 才失败 ——
// 报错信息能说得更清楚（"装好 SDK 后重新编译，装之前先问赛务"）。
bool hasHikSupport();

}  // namespace autoaim::bsp
