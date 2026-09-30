#include "app/config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace autoaim::config {

namespace {

// 简单的 --key value 解析。够用，且不需要任何依赖。
bool wantValue(int i, int argc, char ** argv) {
  if (i + 1 >= argc) {
    std::fprintf(stderr, "参数 %s 缺少值\n", argv[i]);
    return false;
  }
  return true;
}

}  // namespace

void printHelp(const char * prog) {
  std::printf(
      "用法: %s [选项]\n"
      "\n"
      "相机:\n"
      "  --replay <目录>        用图像目录做离线回放（默认模式）\n"
      "  --replay-fps <fps>     回放帧率，默认 60\n"
      "  --no-loop              回放不循环\n"
      "  --hik                  用海康相机（需要 MVS SDK）\n"
      "  --exposure <us>        曝光时间，微秒，默认 5000\n"
      "  --gain <g>             增益，默认 10\n"
      "\n"
      "串口:\n"
      "  --device <路径>        串口设备，默认 /dev/ttyACM0\n"
      "                        建议用 /dev/serial/by-id/...\n"
      "  --tx-hz <hz>           发送频率，默认 100\n"
      "\n"
      "算法:\n"
      "  --thres <n>            二值化阈值，默认 160（现场必须重调）\n"
      "  --armor-w <mm>         两灯条中心距，默认 132（★ 必须实测反标）\n"
      "  --armor-h <mm>         灯条长度，默认 57（★ 必须实测反标）\n"
      "  --cam-pitch <deg>      相机俯仰安装角，负值向下，默认 -15\n"
      "  --cam-yaw <deg>        相机水平安装角，默认 0\n"
      "  --fx --fy --cx --cy    相机内参（★ 必须自己标定）\n"
      "  --enable-tracker       启用跟踪器（默认关）\n"
      "  --rotation <mode>      carousel | self_spin | auto（默认 auto）\n"
      "  --enable-fire          允许开火（默认关）\n"
      "\n"
      "调试:\n"
      "  --debug-view           显示图像窗口（按 q 退出）\n"
      "  --debug-dump           打印跟踪状态\n"
      "  -h, --help             显示本帮助\n"
      "\n"
      "示例:\n"
      "  # 离线回放调检测参数\n"
      "  %s --replay bags/0929 --debug-view --thres 180\n"
      "\n"
      "  # 真机：先只瞄准不开火\n"
      "  %s --hik --device /dev/serial/by-id/usb-XXX --debug-dump\n"
      "\n"
      "安全提示: 默认不开火。确认瞄得准之后再加 --enable-fire。\n",
      prog, prog, prog);
}

bool parseArgs(int argc, char ** argv, Config & cfg) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];

    if (a == "-h" || a == "--help") {
      printHelp(argv[0]);
      return false;
    } else if (a == "--replay") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.kind = CameraKind::REPLAY;
      cfg.camera.replay_dir = argv[++i];
    } else if (a == "--replay-fps") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.replay_fps = std::atof(argv[++i]);
    } else if (a == "--no-loop") {
      cfg.camera.replay_loop = false;
    } else if (a == "--hik") {
      cfg.camera.kind = CameraKind::HIK;
    } else if (a == "--exposure") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.exposure_us = static_cast<float>(std::atof(argv[++i]));
    } else if (a == "--gain") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.gain = static_cast<float>(std::atof(argv[++i]));
    } else if (a == "--device") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.serial.device = argv[++i];
    } else if (a == "--tx-hz") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.serial.tx_hz = std::atoi(argv[++i]);
      if (cfg.serial.tx_hz <= 0 || cfg.serial.tx_hz > 1000) {
        std::fprintf(stderr, "--tx-hz 要在 1~1000 之间\n");
        return false;
      }
    } else if (a == "--thres") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.binary_thres = std::atoi(argv[++i]);
    } else if (a == "--armor-w") {
      if (!wantValue(i, argc, argv)) return false;
      const float w = static_cast<float>(std::atof(argv[++i]));
      cfg.solver.geometry.small_width = w;
      cfg.solver.geometry.large_width = w;
    } else if (a == "--armor-h") {
      if (!wantValue(i, argc, argv)) return false;
      const float h = static_cast<float>(std::atof(argv[++i]));
      cfg.solver.geometry.small_height = h;
      cfg.solver.geometry.large_height = h;
    } else if (a == "--cam-pitch") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.extrinsic.cam_pitch_deg = std::atof(argv[++i]);
    } else if (a == "--cam-yaw") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.extrinsic.cam_yaw_deg = std::atof(argv[++i]);
    } else if (a == "--cam-roll") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.extrinsic.cam_roll_deg = std::atof(argv[++i]);
    } else if (a == "--fx") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[0] = std::atof(argv[++i]);
    } else if (a == "--fy") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[4] = std::atof(argv[++i]);
    } else if (a == "--cx") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[2] = std::atof(argv[++i]);
    } else if (a == "--cy") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[5] = std::atof(argv[++i]);
    } else if (a == "--enable-tracker") {
      cfg.enable_tracker = true;
    } else if (a == "--rotation") {
      if (!wantValue(i, argc, argv)) return false;
      const std::string m = argv[++i];
      if (m == "carousel") {
        cfg.tracker.mode = tracker::RotationMode::CAROUSEL;
      } else if (m == "self_spin") {
        cfg.tracker.mode = tracker::RotationMode::SELF_SPIN;
      } else if (m == "auto") {
        cfg.tracker.mode = tracker::RotationMode::AUTO;
      } else {
        std::fprintf(stderr, "--rotation 只能是 carousel / self_spin / auto\n");
        return false;
      }
    } else if (a == "--enable-fire") {
      cfg.enable_fire = true;
    } else if (a == "--debug-view") {
      cfg.debug_view = true;
    } else if (a == "--debug-dump") {
      cfg.debug_dump = true;
    } else {
      std::fprintf(stderr, "未知参数: %s\n\n", a.c_str());
      printHelp(argv[0]);
      return false;
    }
  }

  // 开了开火就必须有跟踪（或至少明确知道自己在做什么）
  if (cfg.enable_fire) {
    cfg.fire.require_tracking = cfg.enable_tracker;
  }
  return true;
}

}  // namespace autoaim::config
