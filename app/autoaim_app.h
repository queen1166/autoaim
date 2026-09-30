// 主状态机 —— 把各个模块串成一条流水线。
//
// 对应 control-2026 里 robot_cmd 的位置：它是唯一知道全局的地方，
// 其它模块只提供能力，不互相调用。
//
// 数据流：
//
//   cameraLoop  取帧 ──────────────┐
//                                  ▼  Latest<Frame>
//   processLoop                   检测 → PnP → 坐标变换 → 跟踪/瞄准 → 开火判定
//                                  │
//                                  ▼  Latest<TargetCommand>
//   txLoop                       100Hz 定频打包发出
//
//   rxLoop      收帧 → 组帧 → 解析 ──▶ Latest<GimbalFeedback>  ──▶ 开火判定用
//
// 硬约束：txLoop 里【只能】做"取最新值 → pack → write"。
// 打印、写文件、存图一律禁止 —— 它们会让 100Hz 定频抖动。

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "app/config.h"
#include "bsp/camera/camera_factory.h"
#include "bsp/serial/serial_port.h"
#include "modules/aim/aim_solver.h"
#include "modules/aim/fire_decision.h"
#include "modules/detect/armor_detector.h"
#include "modules/message_center/latest.h"
#include "modules/protocol/srm_protocol.h"
#include "modules/solver/pnp_solver.h"
#include "modules/tracker/tracker.h"

namespace autoaim::app {

class AutoAimApp {
 public:
  // 构造：存下配置、按配置造好各个模块对象（PnP、跟踪器等）。
  // 注意它【不碰硬件】—— 相机和串口都留给 init() 去开。
  explicit AutoAimApp(config::Config cfg);

  // 析构：停掉所有循环 → 关相机 → 关串口 → 销毁调试窗口。
  //
  // ⚠️ 前提是四个循环线程已经退出（threads::Handles 的析构负责 join）。
  //    所以 main.cpp 里 `handles` 必须声明在 `application` 【之后】——
  //    局部变量逆序析构，这样 handles 先销毁、join 完线程，
  //    application 才开始析构。反了的话线程会访问已析构的对象。
  ~AutoAimApp();

  // 禁止拷贝：它持有 unique_ptr 和 mutex 类成员，而且四个线程正持有
  // 它的 this 指针 —— 拷贝一份出来线程根本不知道。
  AutoAimApp(const AutoAimApp &) = delete;
  AutoAimApp & operator=(const AutoAimApp &) = delete;

  // 打开相机和串口。失败返回 false（调用方应该直接退出）。
  // 必须在起线程【之前】调用 —— 线程一起来就会开始用这两个资源。
  bool init();

  // ── 给 threads/ 调用的四个循环，各自跑在一个 std::thread 里 ──
  // 每个都是"启动 → while(running_) 干活 → 退出"的骨架，
  // 循环体内部自己检查 running_，收到 false 就收工（协作式退出）。

  // 相机线程（约 60 Hz）：取一帧 → 写进 latest_frame_。
  // 取帧会阻塞，所以单独一条线程，不能拖累别人。
  // 回放播完（exhausted）时会置 running_ = false 让整个程序收工。
  void cameraLoop();

  // 处理线程（有新帧就跑，最慢的一环）：检测 → 解算 → 跟踪 → 瞄准 → 开火。
  // 内部按 latest_frame_ 的 seq 判断"有没有新帧"，没有就小睡一会儿。
  // 绝不能被发送线程等 —— 这是它独立成线程的唯一理由。
  void processLoop();

  // 发送线程（100 Hz 定频，雷打不动）：取最新值 → 打包 → 写串口。
  // ⚠️ 这条线程里【只许】做这三件事 —— 打印、写文件、存图一律禁止，
  //    它们会让定频抖动，云台就跟着抖。用的是 sleep_until 而不是
  //    sleep_for，这样每一拍的误差不会累积。
  void txLoop();

  // 接收线程：读串口 → FrameParser 组帧 → 解析 → 写进 latest_feedback_。
  // 读会阻塞（带超时），所以单独一条线程。
  void rxLoop();

  // 让所有循环退出：把 running_ 置 false，各循环下一轮自己收工。
  // 不阻塞 —— 要等它们真结束得调 threads::joinAll()。
  void stop() { running_.store(false); }

  // 还有循环在跑吗。任何一个循环自己决定收工（比如回放播完、
  // 串口写失败）都会把 running_ 清掉，主循环必须能看见这件事，
  // 否则会一直等 Ctrl-C。
  bool running() const { return running_.load(); }

  // 运行统计。每 5 秒由 printStats（main.cpp）打印一次。
  // 排查问题时先看这里：每一环的损耗都能从两个相邻计数器的落差看出来。
  struct Stats {
    uint64_t frames_grabbed = 0;    // 相机线程成功取到的帧数
    uint64_t frames_processed = 0;  // 处理线程真正跑完的帧数（应略小于上一项）
    uint64_t armors_found = 0;      // 累计检出的装甲板【总块数】（不是帧数）
    uint64_t pnp_ok = 0;            // PnP 解算成功且通过重投影闸门的次数
    uint64_t pnp_rejected = 0;      // 重投影误差超闸门
    uint64_t tx_frames = 0;         // 发出的指令帧数（定频 100Hz，应稳定增长）
    uint64_t rx_frames = 0;         // 解析出的反馈帧数
    uint64_t rx_resync = 0;         // 丢帧重同步次数
    uint64_t stale_cmds = 0;        // 目标过期、降级为不发射的次数
    uint64_t jump_rejected = 0;     // 瞄准角单帧跳变过大被拒的次数
    uint64_t fire_requests = 0;     // 请求开火次数
  };

  // 取一份统计快照。各计数器都是原子的，逐个读不保证是同一时刻的一致值，
  // 但用于每 5 秒打一次的日志完全够了。
  Stats stats() const;

  // 当前配置（只读）。main.cpp 用它判断要不要显示调试窗口等。
  const config::Config & config() const { return cfg_; }

 private:
  // 一帧的完整处理：检测 → 解算 → 跟踪 → 瞄准 → 开火
  //
  // 这是全工程的骨架函数，九步依次是：
  //   ① 检测装甲板  ② 选离画面中心最近的一块  ③ PnP 解算位姿
  //   ④ 重投影误差闸门  ⑤ 相机系 → 云台系  ⑥ 跟踪器更新
  //   ⑦ 算提前量（有跟踪器走 solveFromState，否则走 solveFromMeasurement）
  //   ⑧ 单帧跳变闸门  ⑨ 开火判定 → 发布指令
  //
  // 只在 processLoop 线程里被调用（所以它的私有状态成员不用加锁）。
  void processFrame(const bsp::Frame & frame);

  // 目标过期时发的兜底指令：保持上次角度，但绝不请求开火
  //
  // "过期"= 跟踪器不可用 / 没有目标 / 这一帧解算失败。
  // 保持上次角度是为了让云台别乱动；强制不发射是因为此刻角度不可信。
  void publishHoldCommand();

  // 构造目标指令（不含开火位）
  //
  // yaw_rad, pitch_rad 云台角（弧度）。内部会转成角度制再打包 ——
  //                    协议要的是度，忘了转的话云台会以小得多的幅度乱动。
  // fire               开火标志，原样写进协议
  // stamp_ns           这一帧的时间戳，用于判断指令新鲜度
  // 结果写进 latest_cmd_，由 txLoop 取走。
  void publishCommand(double yaw_rad, double pitch_rad, int fire, uint64_t stamp_ns);

  // 调试显示：把检测结果画到图上，并叠加一行状态文字。
  // 只在 cfg_.debug_view 打开、且处理线程里调用（OpenCV 的窗口 API
  // 不是线程安全的，只能一个线程碰）。
  void drawDebug(const cv::Mat & rgb, const std::string & line);

  config::Config cfg_;

  // ── 各模块实例 ────────────────────────────────────────────
  // 用 unique_ptr 的三个（相机/PnP/跟踪器）是因为它们要么是多态的
  // （CameraSource 基类指针），要么可能整个不存在（跟踪器可关）。
  // 其余三个是具体的值成员，构造即创建。

  std::unique_ptr<bsp::CameraSource> camera_;  // 抽象接口 → 真机或回放
  bsp::SerialPort serial_;                     // 唯一的对外通信通道
  detect::Detector detector_;                  // 装甲板检测
  std::unique_ptr<solver::PnPSolver> pnp_;     // 位姿解算
  std::unique_ptr<tracker::Tracker> tracker_;  // 可为 nullptr（--enable-tracker 没开时）
  aim::AimSolver aim_;                         // 提前量 + 弹道
  aim::FireDecision fire_;                     // 开火判定

  // ── 三块"小黑板"：线程之间的全部通信手段 ──────────────────
  // 线程之间【不互相调用】，只往这三块黑板上写 / 从上面读。
  msg::Latest<bsp::Frame> latest_frame_;         // 相机线程 → 处理线程
  msg::Latest<srm::TargetCommand> latest_cmd_;   // 处理线程 → 发送线程
  msg::Latest<srm::GimbalFeedback> latest_feedback_;  // 接收线程 → 处理线程（反向）

  // 全局退出标志。主线程 stop() 写 false，四条工作线程各自读它决定收工。
  // 必须是原子的 —— 否则工作线程可能因为编译器优化永远看不到变化。
  std::atomic<bool> running_{true};

  // 统计。只有写者会 ++，读的时候不要求强一致。
  std::atomic<uint64_t> frames_grabbed_{0};
  std::atomic<uint64_t> frames_processed_{0};
  std::atomic<uint64_t> armors_found_{0};
  std::atomic<uint64_t> pnp_ok_{0};
  std::atomic<uint64_t> pnp_rejected_{0};
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> rx_resync_{0};
  std::atomic<uint64_t> stale_cmds_{0};
  std::atomic<uint64_t> jump_rejected_{0};
  std::atomic<uint64_t> fire_requests_{0};

  // ── 处理线程私有状态 ─────────────────────────────────────
  // 只有 processLoop 一条线程读写这些，所以【不需要加锁】。
  // 换成别的线程碰它们就是数据竞争。

  uint64_t last_frame_seq_ = 0;    // 上次处理的帧序号，用来判断有没有新帧
  uint64_t last_aim_ns_ = 0;       // 上一次成功瞄准的时刻，算跟踪器 dt 用
  double last_yaw_rad_ = 0.0;      // 上一次发出的 yaw（跳变闸门的基准）
  double last_pitch_rad_ = 0.0;    // 上一次发出的 pitch（同上）
  uint64_t last_tracker_update_ns_ = 0;  // 上次喂给跟踪器的时刻，用于算 dt

  // 是否已经有过一次有效瞄准。没有的话跳变闸门无从比较，
  // 第一帧必须无条件接受（也不能用 0 当基准 —— 云台可能本来就在别处）。
  bool have_last_aim_ = false;
};

}  // namespace autoaim::app
