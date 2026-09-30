#include "app/autoaim_app.h"
#include "modules/common/units.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "modules/solver/coord_transform.h"

namespace autoaim::app {

namespace {

constexpr uint64_t kNsPerSec = 1000000000ULL;

double toSeconds(uint64_t ns) { return static_cast<double>(ns) / 1e9; }

solver::ExtrinsicConfig toExtrinsic(const config::ExtrinsicCfg & c) {
  solver::ExtrinsicConfig e;
  e.cam_yaw_deg = c.cam_yaw_deg;
  e.cam_pitch_deg = c.cam_pitch_deg;
  e.cam_roll_deg = c.cam_roll_deg;
  e.cam_x = c.cam_x;
  e.cam_y = c.cam_y;
  e.cam_z = c.cam_z;
  return e;
}

}  // namespace

AutoAimApp::AutoAimApp(config::Config cfg)
    : cfg_(std::move(cfg)),
      detector_(cfg_.detect.binary_thres, cfg_.detect.detect_color,
                cfg_.detect.light, cfg_.detect.armor),
      aim_(cfg_.aim),
      fire_(cfg_.fire) {
  detector_.single_light_fallback = cfg_.detect.single_light_fallback;
  detector_.max_single_light_count = cfg_.detect.max_single_light_count;

  pnp_ = std::make_unique<solver::PnPSolver>(cfg_.solver.camera_matrix,
                                             cfg_.solver.dist_coeffs,
                                             cfg_.solver.geometry);
  if (cfg_.enable_tracker) {
    tracker_ = std::make_unique<tracker::Tracker>(cfg_.tracker);
  }
}

AutoAimApp::~AutoAimApp() {
  stop();
  if (camera_) camera_->close();
  serial_.close();
  if (cfg_.debug_view) cv::destroyAllWindows();
}

bool AutoAimApp::init() {
  // ── 相机 ────────────────────────────────────────────────
  bsp::CameraSpec spec;
  spec.kind = (cfg_.camera.kind == config::CameraKind::HIK) ? bsp::CameraKind::HIK
                                                           : bsp::CameraKind::REPLAY;
  spec.replay_dir = cfg_.camera.replay_dir;
  spec.replay_fps = cfg_.camera.replay_fps;
  spec.replay_loop = cfg_.camera.replay_loop;
  spec.exposure_us = cfg_.camera.exposure_us;
  spec.gain = cfg_.camera.gain;
  spec.auto_exposure = cfg_.camera.auto_exposure;

  camera_ = bsp::makeCamera(spec);
  if (!camera_ || !camera_->open()) {
    std::fprintf(stderr, "[app] 相机打不开\n");
    return false;
  }

  // ── 串口 ────────────────────────────────────────────────
  // USB CDC 虚拟串口，波特率由驱动忽略；填 115200 是为了兼容真 UART。
  if (!serial_.open(cfg_.serial.device, cfg_.serial.baud)) {
    std::fprintf(stderr, "[app] 串口打不开: %s\n", cfg_.serial.device.c_str());
    std::fprintf(stderr,
                 "  排查: ls /dev/serial/by-id/ | dmesg | tail | groups|grep dialout\n"
                 "  权限: sudo usermod -aG dialout $USER 后【重新登录】\n");
    return false;
  }

  std::printf("[app] 相机=%s  串口=%s @ %d Hz  跟踪器=%s  开火=%s\n",
              camera_->name(), cfg_.serial.device.c_str(), cfg_.serial.tx_hz,
              cfg_.enable_tracker ? "开" : "关", cfg_.enable_fire ? "开" : "关");
  if (cfg_.enable_fire) {
    std::printf("[app] ⚠️  已允许开火。fire_flag 只是视觉侧请求，\n"
                "      实际发射还需要鼠标左键的人工许可和安全员许可。\n");
  }
  return true;
}

AutoAimApp::Stats AutoAimApp::stats() const {
  Stats s;
  s.frames_grabbed = frames_grabbed_.load();
  s.frames_processed = frames_processed_.load();
  s.armors_found = armors_found_.load();
  s.pnp_ok = pnp_ok_.load();
  s.pnp_rejected = pnp_rejected_.load();
  s.tx_frames = tx_frames_.load();
  s.rx_frames = rx_frames_.load();
  s.rx_resync = rx_resync_.load();
  s.stale_cmds = stale_cmds_.load();
  s.jump_rejected = jump_rejected_.load();
  s.fire_requests = fire_requests_.load();
  return s;
}

// ─────────────────────────────────────────────────────────────
// 相机线程：只做取帧 + 打时间戳
// ─────────────────────────────────────────────────────────────
void AutoAimApp::cameraLoop() {
  while (running_.load()) {
    auto frame = camera_->grab(50);
    if (!frame) {
      // 回放播完（--no-loop）→ 收工。不 break 的话这里会变成
      // 每秒上千次 cv::imread 的忙等空转，白烧一个核。
      if (camera_->exhausted()) {
        std::printf("[camera] 回放结束\n");
        running_.store(false);
        break;
      }
      continue;   // 真相机的超时，接着等
    }
    frames_grabbed_.fetch_add(1);
    latest_frame_.set(*frame);
  }
}

// ─────────────────────────────────────────────────────────────
// 处理线程：检测 → 解算 → 跟踪 → 瞄准 → 开火
// ─────────────────────────────────────────────────────────────
void AutoAimApp::processFrame(const bsp::Frame & frame) {
  frames_processed_.fetch_add(1);

  // ── 1. 检测 ──────────────────────────────────────────────
  auto armors = detector_.detect(frame.image);
  if (armors.empty()) {
    publishHoldCommand();
    if (cfg_.debug_view) drawDebug(frame.image, "no armor");
    return;
  }
  armors_found_.fetch_add(1);

  // ── 2. 选板 ──────────────────────────────────────────────
  // 校内赛只有一块靶板，但噪声可能产生多块。挑离图像主点最近的。
  const auto best = std::min_element(
      armors.begin(), armors.end(), [this](const detect::Armor & a, const detect::Armor & b) {
        return pnp_->calculateDistanceToCenter(a.center) <
               pnp_->calculateDistanceToCenter(b.center);
      });

  if (best->type == detect::ArmorType::SINGLE) {
    // 单灯条是退化的：解不出距离和姿态。只用于开赛丢靶时让云台朝大致方向，
    // 不参与闭环，也不开火。
    publishHoldCommand();
    if (cfg_.debug_view) drawDebug(frame.image, "single light (degraded)");
    return;
  }

  // ── 3. PnP ───────────────────────────────────────────────
  cv::Mat rvec, tvec;
  if (!pnp_->solvePnP(*best, rvec, tvec)) {
    publishHoldCommand();
    return;
  }

  // 重投影误差闸门：挡住解分支翻转造成的离群（实测 IPVE 类解法有这个问题，
  // ITERATIVE 好很多，但多一道闸门很便宜）。
  const double err = pnp_->reprojectionError(*best, rvec, tvec);
  if (err < 0 || err > cfg_.solver.max_reprojection_error_px) {
    pnp_rejected_.fetch_add(1);
    publishHoldCommand();
    if (cfg_.debug_view) drawDebug(frame.image, "pnp rejected");
    return;
  }
  pnp_ok_.fetch_add(1);

  // ── 4. 相机系 → 云台系 ───────────────────────────────────
  const auto ext = toExtrinsic(cfg_.extrinsic);
  const Eigen::Vector3d pos_gimbal = solver::cameraToGimbal(tvec, ext);

  // 目标必须在云台前方。z 是"上"，位置在身后的话 x<0，直接丢。
  if (!pos_gimbal.allFinite() || pos_gimbal.x() <= 0.1) {
    publishHoldCommand();
    return;
  }

  // ── 5. 跟踪 / 直接瞄准 ───────────────────────────────────
  const uint64_t now = frame.timestamp_ns;
  aim::AimResult aim_result;
  bool tracking_ok = false;

  if (tracker_) {
    tracker::ArmorMeasurement m;
    m.position = pos_gimbal;
    m.orientation = solver::rvecToGimbalQuat(rvec, ext);
    m.distance_to_image_center = pnp_->calculateDistanceToCenter(best->center);
    m.number = best->number;
    m.type = detect::armorTypeStr(best->type);

    double dt = 0.01;
    if (last_tracker_update_ns_ != 0) {
      dt = toSeconds(now - last_tracker_update_ns_);
      if (dt <= 0 || dt > 1.0) dt = 0.01;  // 时间戳异常时兜底
    }
    last_tracker_update_ns_ = now;

    if (tracker_->state() == tracker::Tracker::LOST) {
      tracker_->init({m});
    } else {
      tracker_->update({m}, dt);
    }

    tracking_ok = tracker_->tracking();
    if (tracking_ok) {
      double bullet_speed = 22.0;
      if (auto fb = latest_feedback_.tryGet()) bullet_speed = fb->bullet_speed;
      aim_result = aim_.solveFromState(tracker_->targetState(), bullet_speed);
    } else {
      // 跟踪器还没收敛（LOST/DETECTING）→ 退回直接用本次观测，
      // 至少让云台朝靶板方向，不要傻等。
      aim_result = aim_.solveFromMeasurement(pos_gimbal, 22.0);
    }

    if (cfg_.debug_dump) {
      std::printf(
          "[track] %-11s mode=%-9s r=%.3f v_yaw=%+.2f posdiff=%.3f yawdiff=%.3f\n",
          tracker_->stateStr(),
          tracker_->activeMode() == tracker::RotationMode::SELF_SPIN ? "SELF_SPIN"
                                                                   : "CAROUSEL",
          tracker_->radius(), tracker_->vYaw(), tracker_->infoPositionDiff(),
          tracker_->infoYawDiff());
    }
  } else {
    // 最小闭环：不做跟踪，直接用本次观测瞄准。
    // 没有速度信息 → 没有提前量，对静止靶够用，对旋转靶会系统性落后。
    aim_result = aim_.solveFromMeasurement(pos_gimbal, 22.0);
  }

  if (!aim_result.valid) {
    publishHoldCommand();
    return;
  }

  // 瞄准角跳变闸门。
  //
  // 实测：转盘模式下检测中断（装甲板侧对相机）再重捕时，EKF 的 v_yaw
  // 会短暂冲到 55 rad/s（真值 2.0）。即使 aim_solver 里已经夹住 v_yaw，
  // 解出来的角度仍可能一帧跳几十度 —— 云台会跟着猛甩。
  //
  // 单帧角度变化超过阈值的，判为解算异常，保持上一次的角度不动。
  // 注意只在【已经有过一次有效瞄准】之后才判，否则启动时目标本来就在
  // 大角度上也过不去。
  if (have_last_aim_) {
    const double dyaw =
        std::abs(std::remainder(aim_result.yaw_rad - last_yaw_rad_, kTwoPi)) *
        kRad2Deg;
    const double dpitch =
        std::abs(aim_result.pitch_rad - last_pitch_rad_) * kRad2Deg;
    if (dyaw > cfg_.aim.max_jump_deg || dpitch > cfg_.aim.max_jump_deg) {
      jump_rejected_.fetch_add(1);
      publishHoldCommand();
      if (cfg_.debug_view) drawDebug(frame.image, "aim jump rejected");
      return;
    }
  }
  have_last_aim_ = true;

  // ── 6. 开火判定 ──────────────────────────────────────────
  int fire = 0;
  if (cfg_.enable_fire && aim_result.valid) {
    double gimbal_yaw_rad = 0.0, gimbal_pitch_rad = 0.0;
    // 反馈的年龄。串口断了的话 latest_feedback_ 会一直返回最后一帧，
    // 角度冻住 —— 必须让开火判定知道这件事，否则会对着一个
    // 位置未知的云台一直请求开火。
    double fb_age_s = 1e9;
    if (auto fb = latest_feedback_.tryGet()) {
      // 反馈是【角度制】，内部一律【弧度】
      gimbal_yaw_rad = fb->yaw_deg * kDeg2Rad;
      gimbal_pitch_rad = fb->pitch_deg * kDeg2Rad;
      // 无符号相减；now 是当前帧时间戳，必然 >= 反馈收到的时刻
      fb_age_s = toSeconds(now >= fb->timestamp_ns ? now - fb->timestamp_ns : 0);
    }
    fire = fire_.update(aim_result, gimbal_yaw_rad, gimbal_pitch_rad, tracking_ok,
                        toSeconds(now), fb_age_s);
    if (fire) fire_requests_.fetch_add(1);
  }

  publishCommand(aim_result.yaw_rad, aim_result.pitch_rad, fire, now);

  if (cfg_.debug_view) {
    char line[160];
    std::snprintf(line, sizeof(line), "d=%.2fm yaw=%.2f pitch=%.2f %s",
                  pos_gimbal.norm(), aim_result.yaw_rad * kRad2Deg,
                  aim_result.pitch_rad * kRad2Deg, fire ? "FIRE" : "");
    drawDebug(frame.image, line);
  }
}

void AutoAimApp::processLoop() {
  while (running_.load()) {
    const uint64_t seq = latest_frame_.seq();
    if (seq == last_frame_seq_) {
      // 没有新帧。让出 CPU，不要空转。
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    last_frame_seq_ = seq;

    auto frame = latest_frame_.tryGet();
    if (!frame) continue;

    // 目标过期检查：太久没瞄上就保持不动，不要拿着旧角度乱指
    processFrame(*frame);
  }
}

// ─────────────────────────────────────────────────────────────
// 发送线程：100Hz 定频。这里【只】做取最新值 → pack → write。
// ─────────────────────────────────────────────────────────────
void AutoAimApp::txLoop() {
  const auto period = std::chrono::microseconds(1000000 / cfg_.serial.tx_hz);
  auto next = std::chrono::steady_clock::now();

  while (running_.load()) {
    next += period;

    srm::TargetCommand cmd;
    cmd.yaw_deg = 0.0f;
    cmd.pitch_deg = 0.0f;
    cmd.fire_flag = 0;
    if (auto latest = latest_cmd_.tryGet()) cmd = *latest;

    const auto frame = srm::pack_target(cmd);
    if (serial_.write_bytes(frame.data(), frame.size()) < 0) {
      std::fprintf(stderr, "[tx] 串口写入失败，退出发送线程\n");
      running_.store(false);
      break;
    }
    tx_frames_.fetch_add(1);

    // sleep_until 比 sleep_for 准：不会累积漂移
    std::this_thread::sleep_until(next);

    // 如果落后超过一个周期（比如被系统调度打断），重新对齐而不是追赶
    const auto now = std::chrono::steady_clock::now();
    if (now > next + period) next = now;
  }
}

// ─────────────────────────────────────────────────────────────
// 接收线程：流式组帧 → 解析 → 发布反馈
// ─────────────────────────────────────────────────────────────
void AutoAimApp::rxLoop() {
  srm::FrameParser parser;
  uint8_t buf[512];

  while (running_.load()) {
    const ssize_t n = serial_.read_bytes(buf, sizeof(buf), cfg_.serial.read_timeout_ms);
    if (n < 0) {
      std::fprintf(stderr, "[rx] 串口读取失败，退出接收线程\n");
      break;
    }
    if (n == 0) continue;

    parser.feed(
        buf, static_cast<size_t>(n),
        [this](const srm::GimbalFeedback & fb) {
          rx_frames_.fetch_add(1);
          latest_feedback_.set(fb);
        },
        bsp::nowNs());
    // 重同步次数由组帧器自己数。用 buffered() 的变化去猜是错的 ——
    // 正常解析出一帧同样会让 buffered() 变小，会把好帧也算成丢帧。
    rx_resync_.store(parser.resyncCount());
  }
}

// ─────────────────────────────────────────────────────────────
// 指令发布
// ─────────────────────────────────────────────────────────────
void AutoAimApp::publishCommand(double yaw_rad, double pitch_rad, int fire,
                                uint64_t stamp_ns) {
  srm::TargetCommand cmd;
  cmd.yaw_deg = static_cast<float>(yaw_rad * kRad2Deg);
  cmd.pitch_deg = static_cast<float>(pitch_rad * kRad2Deg);
  cmd.fire_flag = fire;

  // 协议边界才转角度。内部一律弧度。
  // ★ 若现场实测发现下位机的 yaw/pitch 符号和这里相反，
  //   就在这里加负号 —— 只改这一处。
  latest_cmd_.set(cmd);

  last_yaw_rad_ = yaw_rad;
  last_pitch_rad_ = pitch_rad;
  last_aim_ns_ = stamp_ns;
}

void AutoAimApp::publishHoldCommand() {
  // 没瞄上：保持上一次的角度，但【绝不请求开火】。
  // 不要发零 —— 那会让云台猛地甩回原点。
  stale_cmds_.fetch_add(1);

  srm::TargetCommand cmd;
  cmd.yaw_deg = static_cast<float>(last_yaw_rad_ * kRad2Deg);
  cmd.pitch_deg = static_cast<float>(last_pitch_rad_ * kRad2Deg);
  cmd.fire_flag = 0;
  latest_cmd_.set(cmd);

  // 顺便重置开火状态机，避免丢失目标期间残留的收敛计时
  fire_.reset();
}

void AutoAimApp::drawDebug(const cv::Mat & rgb, const std::string & line) {
  // 先克隆再画。cv::Mat 的拷贝是引用计数共享，直接画会改到
  // Latest<Frame> 里那一份，而相机线程可能正在往里写。
  cv::Mat canvas = rgb.clone();
  detector_.drawResults(canvas);
  cv::cvtColor(canvas, canvas, cv::COLOR_RGB2BGR);  // imshow 要 BGR

  // 画主点，方便看 PnP 距离是否可信
  const auto & K = cfg_.solver.camera_matrix;
  cv::drawMarker(canvas, cv::Point(static_cast<int>(K[2]), static_cast<int>(K[5])),
                 cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 1);

  cv::putText(canvas, line, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(0, 255, 0), 2);
  cv::imshow("autoaim", canvas);
  cv::waitKey(1);
}

}  // namespace autoaim::app
