
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <string>

#include "bsp/camera/camera_factory.h"
#include "bsp/camera/replay_source.h"
#include "bsp/serial/serial_port.h"
#include "modules/protocol/srm_protocol.h"

using namespace autoaim;

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) { g_running.store(false); }

struct Args {
  std::string out;
  std::string replay;
  std::string device;
  std::string angles_file;
  double secs = 20.0;
  double replay_fps = 60.0;
  int every = 1;
  int start = 0;
  int max_frames = 0;
  float exposure = 5000.0f;
  float gain = 10.0f;
  bool help = false;
};

void printHelp(const char * prog) {
  std::printf(
      "用法: %s --out <目录> [选项]\n"
      "\n"
      "输入（二选一）:\n"
      "  （默认）                用海康相机，需要 MVS SDK\n"
      "  --replay <目录>         用已有图像目录当输入（无相机时验证流程用）\n"
      "  --replay-fps <fps>      --replay 时的输入帧率，默认 60\n"
      "\n"
      "输出:\n"
      "  --out <目录>            输出目录（必填，已存在同名图像会拒绝覆盖）\n"
      "  --secs <秒>             录制时长，默认 20\n"
      "  --every <n>             每 n 帧存 1 张，默认 1。转盘靶用 1，\n"
      "                          静态标定用 3~5 就够\n"
      "  --start <n>             跳过前 n 帧再开始存，默认 0\n"
      "  --max-frames <n>        最多存 n 张，默认不限\n"
      "\n"
      "相机参数（只对海康生效）:\n"
      "  --exposure <us>         曝光时间，微秒，默认 5000\n"
      "  --gain <g>              增益，默认 10（尽量低，噪声会放大）\n"
      "\n"
      "同时记录云台角度（可选，给外参标定用）:\n"
      "  --device <路径>         串口设备，如 /dev/serial/by-id/usb-xxx\n"
      "  --angles <文件>         角度日志路径，默认 <out>/angles.csv\n"
      "\n"
      "  -h, --help              显示本帮助\n"
      "\n"
      "安全: 本工具只读串口，不发送任何字节，不会驱动云台、不会请求开火。\n",
      prog);
}

bool wantValue(int i, int argc, char ** argv) {
  if (i + 1 >= argc) {
    std::fprintf(stderr, "参数 %s 缺少值\n", argv[i]);
    return false;
  }
  return true;
}

bool parseArgs(int argc, char ** argv, Args & a) {
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    if (s == "-h" || s == "--help") {
      a.help = true;
      printHelp(argv[0]);
      return false;
    } else if (s == "--out") {
      if (!wantValue(i, argc, argv)) return false;
      a.out = argv[++i];
    } else if (s == "--replay") {
      if (!wantValue(i, argc, argv)) return false;
      a.replay = argv[++i];
    } else if (s == "--replay-fps") {
      if (!wantValue(i, argc, argv)) return false;
      a.replay_fps = std::atof(argv[++i]);
    } else if (s == "--device") {
      if (!wantValue(i, argc, argv)) return false;
      a.device = argv[++i];
    } else if (s == "--angles") {
      if (!wantValue(i, argc, argv)) return false;
      a.angles_file = argv[++i];
    } else if (s == "--secs") {
      if (!wantValue(i, argc, argv)) return false;
      a.secs = std::atof(argv[++i]);
    } else if (s == "--every") {
      if (!wantValue(i, argc, argv)) return false;
      a.every = std::atoi(argv[++i]);
      if (a.every < 1) {
        std::fprintf(stderr, "--every 至少是 1\n");
        return false;
      }
    } else if (s == "--start") {
      if (!wantValue(i, argc, argv)) return false;
      a.start = std::atoi(argv[++i]);
      if (a.start < 0) {
        std::fprintf(stderr, "--start 不能为负\n");
        return false;
      }
    } else if (s == "--max-frames") {
      if (!wantValue(i, argc, argv)) return false;
      a.max_frames = std::atoi(argv[++i]);
    } else if (s == "--exposure") {
      if (!wantValue(i, argc, argv)) return false;
      a.exposure = static_cast<float>(std::atof(argv[++i]));
    } else if (s == "--gain") {
      if (!wantValue(i, argc, argv)) return false;
      a.gain = static_cast<float>(std::atof(argv[++i]));
    } else {
      std::fprintf(stderr, "未知参数: %s\n\n", s.c_str());
      printHelp(argv[0]);
      return false;
    }
  }

  if (a.out.empty()) {
    std::fprintf(stderr, "必须给 --out <目录>\n\n");
    printHelp(argv[0]);
    return false;
  }
  if (a.angles_file.empty()) a.angles_file = a.out + "/angles.csv";
  return true;
}

bool dirHasImages(const std::string & dir) {
  for (const char * ext : {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}) {
    std::string p = dir + "/000000" + ext;
    if (std::FILE * f = std::fopen(p.c_str(), "rb")) {
      std::fclose(f);
      return true;
    }
  }
  return false;
}

}

int main(int argc, char ** argv) {
  Args a;
  if (!parseArgs(argc, argv, a)) return 1;

  bsp::CameraSpec spec;
  spec.kind = a.replay.empty() ? bsp::CameraKind::HIK : bsp::CameraKind::REPLAY;
  spec.replay_dir = a.replay;
  spec.replay_fps = a.replay_fps;
  spec.replay_loop = false;
  spec.exposure_us = a.exposure;
  spec.gain = a.gain;
  spec.auto_exposure = false;

  auto cam = bsp::makeCamera(spec);
  if (!cam || !cam->open()) {
    std::fprintf(stderr, "[record] 相机打不开，退出。\n");
    return 1;
  }

  if (dirHasImages(a.out)) {
    std::fprintf(stderr,
                 "[record] %s 里已经有录好的图像了，拒绝覆盖。\n"
                 "         换个 --out 目录，或者先把旧的移走。\n",
                 a.out.c_str());
    return 1;
  }

  bsp::SerialPort port;
  srm::FrameParser parser;
  bool have_serial = false;
  if (!a.device.empty()) {
    have_serial = port.open(a.device);
    if (!have_serial) {
      std::fprintf(stderr,
                   "[record] 串口打不开: %s —— 继续录图，但不记角度。\n"
                   "         排查: ls /dev/serial/by-id/ | groups | grep dialout\n",
                   a.device.c_str());
    }
  }

  std::ofstream angles;
  if (have_serial) {
    angles.open(a.angles_file);
    if (!angles) {
      std::fprintf(stderr, "[record] 角度日志写不了: %s\n", a.angles_file.c_str());
      return 1;
    }
    angles << "file,yaw_deg,pitch_deg,u,v\n";
    std::printf("[record] 云台角度写到 %s（串口只读，不发送）\n",
                a.angles_file.c_str());
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("[record] 输入=%s  输出=%s  时长 %.0fs  每 %d 帧存 1 张\n",
              cam->name(), a.out.c_str(), a.secs, a.every);
  if (have_serial) {
    std::printf("[record] ⚠️ 云台角度记录的是【存图那一刻读到的最近一帧反馈】，\n"
                "         最多滞后一帧。慢速扫动没问题；要更高精度就"
                "在每个角度停稳再录。\n");
  }
  std::printf("[record] Ctrl-C 提前结束。\n\n");

  const uint64_t t0 = bsp::nowNs();
  const uint64_t deadline = t0 + static_cast<uint64_t>(a.secs * 1e9);

  uint8_t rx[512];
  double fb_yaw = 0.0, fb_pitch = 0.0;
  bool have_angle = false;

  auto pumpSerial = [&]() {
    if (!have_serial) return;
    const ssize_t n = port.read_bytes(rx, sizeof(rx), 0);
    if (n <= 0) return;
    parser.feed(rx, static_cast<size_t>(n),
                [&](const srm::GimbalFeedback & fb) {
                  fb_yaw = fb.yaw_deg;
                  fb_pitch = fb.pitch_deg;
                  have_angle = true;
                },
                bsp::nowNs());
  };

  uint64_t total = 0;
  uint64_t kept = 0;
  uint64_t no_frame = 0;

  while (g_running.load()) {
    const uint64_t now = bsp::nowNs();
    if (now >= deadline) break;
    if (a.max_frames > 0 && static_cast<int>(kept) >= a.max_frames) break;

    const int budget_ms = static_cast<int>(
        std::min<uint64_t>((deadline - now) / 1000000ULL, 1000));
    auto frame = cam->grab(budget_ms);

    if (!frame) {
      if (cam->exhausted()) break;
      ++no_frame;
      pumpSerial();
      continue;
    }

    pumpSerial();
    ++total;

    if (static_cast<int>(total) <= a.start) continue;
    if ((static_cast<int>(total) - a.start - 1) % a.every != 0) continue;

    const std::string path =
        bsp::saveFrame(a.out, static_cast<int>(kept), frame->image);
    if (path.empty()) {
      std::fprintf(stderr, "[record] 写图失败: %s\n", a.out.c_str());
      break;
    }

    if (angles) {
      char name[64];
      std::snprintf(name, sizeof(name), "%06llu.png",
                    static_cast<unsigned long long>(kept));
      angles << name << ',' << std::fixed << std::setprecision(3)
             << (have_angle ? fb_yaw : 0.0) << ','
             << (have_angle ? fb_pitch : 0.0) << ",,\n";
    }
    ++kept;

    const double el = static_cast<double>(now - t0) / 1e9;
    if (have_serial && have_angle) {
      std::printf("\r  已存 %4llu 张  %5.1fs  云台 yaw=%+7.2f pitch=%+7.2f   ",
                  static_cast<unsigned long long>(kept), el, fb_yaw, fb_pitch);
    } else {
      std::printf("\r  已存 %4llu 张  %5.1fs                    ",
                  static_cast<unsigned long long>(kept), el);
    }
    std::fflush(stdout);
  }

  const double elapsed = static_cast<double>(bsp::nowNs() - t0) / 1e9;
  cam->close();
  if (angles) angles.close();
  if (have_serial) port.close();

  const double fps = elapsed > 0 ? static_cast<double>(total) / elapsed : 0.0;
  std::printf("\n\n──────── 录制结束 ────────\n");
  std::printf("  见到 %llu 帧 / %.1f s = %.1f fps\n",
              static_cast<unsigned long long>(total), elapsed, fps);
  std::printf("  存下 %llu 张 → %s\n",
              static_cast<unsigned long long>(kept), a.out.c_str());
  if (no_frame) {
    std::printf("  取帧超时 %llu 次\n",
                static_cast<unsigned long long>(no_frame));
  }
  if (have_serial) {
    std::printf("  角度日志 → %s（%llu 行）\n", a.angles_file.c_str(),
                static_cast<unsigned long long>(kept));
  }

  if (kept == 0) {
    std::printf("\n没有存下任何图。检查：--start 是不是太大 / 相机有没有出图。\n");
    return 1;
  }

  std::printf("\n回放时用：\n  --replay %s --replay-fps %.1f\n", a.out.c_str(), fps);
  if (have_serial) {
    std::printf("\n外参标定（先跑 calib_intrinsic.py 拿到内参）：\n"
                "  python3 tools/calib_extrinsic.py --images %s "
                "--angles %s --fx <fx> --fy <fy> --cx <cx> --cy <cy>\n",
                a.out.c_str(), a.angles_file.c_str());
  }
  std::printf("看检测效果：\n  ./build/tools/tool_detect_view %s\n", a.out.c_str());
  return 0;
}
