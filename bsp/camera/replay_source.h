// 离线回放：把录好的图像序列当成相机。
//
// 这是现场调参的主力手段。录包格式就是【一个目录里的图像文件】，
// 按文件名排序播放。简单到用 ls 就能看，出问题也好查。

#pragma once

#include <string>
#include <vector>

#include "bsp/camera/camera_source.h"

namespace autoaim::bsp {

class ReplaySource : public CameraSource {
 public:
  struct Config {
    std::string dir;           // 图像目录
    double fps = 60.0;         // 回放帧率（模拟相机帧率，会影响 dt_）
    bool loop = true;          // 播完是否循环
    // 图像是 BGR 还是 RGB。cv::imread 默认读成 BGR，
    // 而检测器要求 RGB，所以默认会做一次转换。
    bool convert_bgr_to_rgb = true;
  };

  // 构造：只记下配置，不碰磁盘。要真扫目录得调 open()。
  explicit ReplaySource(const Config & cfg) : cfg_(cfg) {}

  // 扫描 cfg_.dir 下的图像文件、按文件名排序、把游标归零。
  // 返回 true 成功；目录不存在或一张图都没有则返回 false。
  bool open() override;

  // 清空文件列表并把 opened_ 置回 false。重复调用安全。
  void close() override;

  // 是否已 open() 过。
  bool isOpen() const override { return opened_; }

  // 不循环 + 已经播到末尾 → 数据源枯竭
  bool exhausted() const override {
    return opened_ && !cfg_.loop && cursor_ >= files_.size();
  }

  // 取下一帧。返回 nullopt 表示：播完了（不循环）或图读不出来。
  //
  // 这个实现【不真的等待】—— 它按 cfg_.fps 算出这一帧"应该"的时间戳
  // 直接返回，所以回放跑得比真实相机快得多（离线调参要的就是这个）。
  // 时间戳仍然是单调递增的，保证下游的 dt 计算是对的。
  std::optional<Frame> grab(int timeout_ms = 100) override;

  // 固定返回 "ReplaySource"。
  const char * name() const override { return "ReplaySource"; }

  // 这次回放总共有多少帧（open() 之后才有意义）。
  // 用来估算跑完要多久，或者给 --no-loop 判断终点。
  size_t frameCount() const { return files_.size(); }

  // 已播到第几帧（循环时会回绕）。调参时想知道"现在在看哪一帧"。
  size_t cursor() const { return cursor_; }

 private:
  Config cfg_;
  std::vector<std::string> files_;
  size_t cursor_ = 0;
  bool opened_ = false;
  uint64_t next_frame_ns_ = 0;
};

// 把一块内存里的 RGB 图存成 PNG 到 dir，文件名按序号排。
// 配套的录制工具用。
//
// dir   目标目录（不存在会创建）
// index 文件序号，决定文件名（如 000042.png），决定播放顺序
// rgb   图像数据，必须是 RGB（不是 BGR）
//
// 返回写出的完整文件路径；写失败返回空串。
std::string saveFrame(const std::string & dir, int index, const cv::Mat & rgb);

}  // namespace autoaim::bsp
