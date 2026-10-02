
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

#include "app/autoaim_app.h"
#include "app/config.h"
#include "bsp/camera/camera_factory.h"
#include "threads/threads.h"

namespace {

std::atomic<bool> g_running{true};

void onSignal(int) { g_running.store(false); }

void printStats(const autoaim::app::AutoAimApp & application, double elapsed_s) {
  const auto s = application.stats();
  const double fps = elapsed_s > 0 ? static_cast<double>(s.frames_processed) / elapsed_s
                                   : 0.0;
  std::printf(
      "\n──────── %.1f s ────────\n"
      "  取帧 %llu   处理 %llu (%.1f fps)\n"
      "  检出装甲板 %llu   PnP 成功 %llu   被闸门拒绝 %llu\n"
      "  串口 发送 %llu   接收 %llu   重同步 %llu\n"
      "  目标过期降级 %llu   跳变被拒 %llu   开火请求 %llu\n",
      elapsed_s,
      static_cast<unsigned long long>(s.frames_grabbed),
      static_cast<unsigned long long>(s.frames_processed), fps,
      static_cast<unsigned long long>(s.armors_found),
      static_cast<unsigned long long>(s.pnp_ok),
      static_cast<unsigned long long>(s.pnp_rejected),
      static_cast<unsigned long long>(s.tx_frames),
      static_cast<unsigned long long>(s.rx_frames),
      static_cast<unsigned long long>(s.rx_resync),
      static_cast<unsigned long long>(s.stale_cmds),
      static_cast<unsigned long long>(s.jump_rejected),
      static_cast<unsigned long long>(s.fire_requests));
}

}

int main(int argc, char ** argv) {
  autoaim::config::Config cfg;
  if (!autoaim::config::parseArgs(argc, argv, cfg)) {
    return 1;
  }

  if (cfg.camera.kind == autoaim::config::CameraKind::HIK &&
      !autoaim::bsp::hasHikSupport()) {
    std::fprintf(stderr,
                 "本次构建不含海康支持。用 --replay <目录> 走离线回放，\n"
                 "或装好 MVS SDK 后重新编译（装之前先问赛务）。\n");
    return 1;
  }

  autoaim::app::AutoAimApp application(std::move(cfg));
  if (!application.init()) return 1;

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto handles = autoaim::threads::start(application);

  const auto t0 = std::chrono::steady_clock::now();
  auto next_report = t0 + std::chrono::seconds(5);

  while (g_running.load() && application.running()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() >= next_report) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      printStats(application, elapsed);
      next_report += std::chrono::seconds(5);
    }
  }

  std::printf("\n正在停止…\n");
  application.stop();
  autoaim::threads::joinAll(handles);
  printStats(application,
             std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                 .count());
  std::printf("已退出。\n");
  return 0;
}
