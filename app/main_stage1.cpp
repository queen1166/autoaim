// ─────────────────────────────────────────────────────────────
// 阶段 1 最小测试程序
//
// 干的事：以 100 Hz 发目标帧让云台 yaw 缓慢扫动，
//         同时用另一个线程收 30 字节反馈帧并打印真实角度。
//
// 成功现象：云台跟着扫动，终端里的 yaw 数值跟着变。
//
// 云台不动？→ 按一下【鼠标右键】，进入视觉控制。
// 本程序 fire_flag 恒为 0，不会发射。
// ─────────────────────────────────────────────────────────────

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "bsp/serial/serial_port.h"
#include "modules/protocol/srm_protocol.h"

namespace {

std::atomic<bool> g_running{true};

void on_sigint(int) { g_running = false; }

uint64_t now_ns() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
            .count());
}

// 扫动参数：先给小幅度，确认符号方向之后再加大
constexpr double kSweepDeg    = 15.0;
constexpr double kSweepPeriod = 6.0;    // 一个来回几秒
constexpr int    kTxHz        = 100;

// ── 接收线程 ────────────────────────────────────────────────
void rx_loop(autoaim::bsp::SerialPort& port) {
    srm::FrameParser parser;
    uint8_t buf[256];

    while (g_running) {
        const ssize_t n = port.read_bytes(buf, sizeof(buf), 20);
        if (n <= 0) continue;

        parser.feed(
            buf, static_cast<size_t>(n),
            [](const srm::GimbalFeedback& fb) {
                // 注意：这里只打印。真实上位机里发送线程绝不能碰 IO。
                std::printf(
                    "\r反馈 yaw=%8.2f  pitch=%7.2f  roll=%7.2f  "
                    "弹速=%5.1f  mode=%d  color=%d      ",
                    fb.yaw_deg, fb.pitch_deg, fb.roll_deg,
                    fb.bullet_speed, fb.mode, fb.color);
                std::fflush(stdout);
            },
            now_ns());
    }
}

} // namespace

int main(int argc, char** argv) {
    // 建议改用 /dev/serial/by-id/usb-... ：重新插拔后编号可能变化
    const std::string dev = (argc > 1) ? argv[1] : "/dev/ttyACM0";

    autoaim::bsp::SerialPort port;
    if (!port.open(dev)) {
        std::fprintf(stderr, "打不开串口: %s\n\n", dev.c_str());
        std::fprintf(stderr, "排查顺序：\n");
        std::fprintf(stderr, "  1. ls /dev/serial/by-id/   看真实设备名\n");
        std::fprintf(stderr, "  2. sudo dmesg | tail        看内核有没有绑上 cdc_acm\n");
        std::fprintf(stderr, "  3. groups | grep dialout    看权限；不在就：\n");
        std::fprintf(stderr, "     sudo usermod -aG dialout $USER  然后【重新登录】\n");
        std::fprintf(stderr, "  4. lsof %s  看是不是被别的程序占用\n", dev.c_str());
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    std::printf("已打开 %s\n", dev.c_str());
    std::printf("云台 yaw 将在 ±%.0f 度之间扫动。Ctrl-C 退出。\n", kSweepDeg);
    std::printf("云台不动？→ 按一下鼠标右键，进入视觉控制。\n\n");

    std::thread rx(rx_loop, std::ref(port));

    const auto dt = std::chrono::microseconds(1000000 / kTxHz);
    const auto t0 = std::chrono::steady_clock::now();

    while (g_running) {
        // 记下本次循环的目标时刻，用于定频（sleep_until 比 sleep_for 准）
        const auto next = std::chrono::steady_clock::now() + dt;

        // 三角波：-A → +A → -A
        const double t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
                .count();
        const double phase = std::fmod(t, kSweepPeriod);
        const double half  = kSweepPeriod / 2.0;
        const double yaw   = (phase < half)
                               ? -kSweepDeg + 2.0 * kSweepDeg * (phase / half)
                               : kSweepDeg - 2.0 * kSweepDeg * ((phase - half) / half);

        srm::TargetCommand cmd;
        cmd.yaw_deg   = static_cast<float>(yaw);
        cmd.pitch_deg = 0.0f;
        cmd.fire_flag = 0;

        const auto frame = srm::pack_target(cmd);
        if (port.write_bytes(frame.data(), frame.size()) < 0) {
            std::fprintf(stderr, "\n写串口失败，退出。\n");
            break;
        }

        std::this_thread::sleep_until(next);
    }

    g_running = false;
    if (rx.joinable()) rx.join();
    port.close();
    std::printf("\n已退出。\n");
    return 0;
}
