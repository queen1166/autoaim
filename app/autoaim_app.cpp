#include "app/autoaim_app.h"
#include "modules/common/units.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

#include "modules/solver/coord_transform.h"

namespace autoaim::app {

namespace {

constexpr uint64_t kNsPerSec = 1000000000ULL;

double toSeconds(uint64_t ns) { return static_cast<double>(ns) / 1e9; }

// 指数滑动平均，用来把分阶段耗时算成一个稳定的数。
void updateAverage(std::atomic<uint64_t> & avg, uint64_t sample) {
  const uint64_t prev = avg.load();
  const uint64_t next =
      prev == 0 ? sample : static_cast<uint64_t>(prev * 0.9 + sample * 0.1);
  avg.store(next);
}

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
      corner_optimizer_(cfg_.detect.corner_refine),
      aim_(cfg_.aim),
      fire_(cfg_.fire),
      planner_(cfg_.planner),
      attitude_buffer_(static_cast<size_t>(std::max(8, cfg_.attitude_buffer_size))) {
  // ── 探测器（教程 §5A.1）────────────────────────────────────────────
  if (cfg_.detect.kind == config::DetectorKind::YOLO_POSE) {
    auto yolo = std::make_unique<detect::YoloPoseDetector>(cfg_.detect.yolo);
    if (yolo->load()) {
      detector_ = std::move(yolo);
    } else if (!cfg_.detect.yolo_fallback_to_traditional) {
      std::fprintf(stderr,
                   "[app] YOLO 模型不可用: %s，且未允许回退，探测将全部失败\n",
                   cfg_.detect.yolo.model_path.c_str());
    } else {
      // 神经网络是默认路径，而模型文件被 .gitignore 排除，所以新 clone 的仓库一定
      // 会落到这里。说清用的是哪个路径、以及这不是故障，免得被当成坏了。
      std::fprintf(stderr,
                   "[app] YOLO 模型不可用: %s → 回退到传统视觉探测。\n"
                   "      该文件不在 Git 里（*.onnx 被排除），新 clone 的仓库都没有，属预期。\n"
                   "      想显式走传统视觉、不要这行提示，加 --no-yolo。\n",
                   cfg_.detect.yolo.model_path.c_str());
    }
  }
  if (!detector_) {
    auto trad = std::make_unique<detect::Detector>(
        cfg_.detect.binary_thres, cfg_.detect.detect_color, cfg_.detect.light,
        cfg_.detect.armor);
    trad->single_light_fallback = cfg_.detect.single_light_fallback;
    trad->max_single_light_count = cfg_.detect.max_single_light_count;
    detector_ = std::move(trad);
  }

  pnp_ = std::make_unique<solver::PnPSolver>(cfg_.solver.camera_matrix,
                                             cfg_.solver.dist_coeffs,
                                             cfg_.solver.geometry);
  pnp_->setYawFitConfig(cfg_.solver.yaw_fit);

  if (cfg_.enable_tracker) {
    tracker_ = std::make_unique<tracker::Tracker>(cfg_.tracker);
  }

  last_state_ = Eigen::VectorXd::Zero(tracker::kStateDim);
}

AutoAimApp::~AutoAimApp() {
  stop();
  if (camera_) camera_->close();
  serial_.close();
  if (cfg_.debug_view) cv::destroyAllWindows();
}

bool AutoAimApp::init() {
  bsp::CameraSpec spec;//相机配置
  spec.kind = (cfg_.camera.kind == config::CameraKind::HIK) ? bsp::CameraKind::HIK
                                                           : bsp::CameraKind::REPLAY;
  spec.replay_dir = cfg_.camera.replay_dir;
  spec.replay_fps = cfg_.camera.replay_fps;
  spec.replay_loop = cfg_.camera.replay_loop;
  spec.exposure_us = cfg_.camera.exposure_us;
  spec.gain = cfg_.camera.gain;
  spec.auto_exposure = cfg_.camera.auto_exposure;
  spec.width = cfg_.camera.width;
  spec.height = cfg_.camera.height;

  camera_ = bsp::makeCamera(spec);
  if (!camera_ || !camera_->open()) {
    std::fprintf(stderr, "[app] 相机打不开\n");
    return false;
  }

  if (!serial_.open(cfg_.serial.device, cfg_.serial.baud)) {
    std::fprintf(stderr, "[app] 串口打不开: %s\n", cfg_.serial.device.c_str());
    std::fprintf(stderr,
                 "  排查: ls /dev/serial/by-id/ | dmesg | tail | groups|grep dialout\n"
                 "  权限: sudo usermod -aG dialout $USER 后【重新登录】\n");
    return false;
  }

  std::printf("[app] 相机=%s  探测器=%s  串口=%s @ %d Hz  规划=%s(%d Hz)  跟踪器=%s  开火=%s\n",
              camera_->name(), detector_->name(), cfg_.serial.device.c_str(),
              cfg_.serial.tx_hz,
              cfg_.plan_hz > 0 ? (cfg_.planner.enable ? "MPC" : "直瞄") : "随帧",
              cfg_.plan_hz, cfg_.enable_tracker ? "开" : "关",
              cfg_.enable_fire ? "开" : "关");
  if (!cfg_.use_attitude) {
    std::printf("[app] ⚠️  --no-attitude：不做云台系→世界系变换。\n"
                "      只有在云台恒在 0° 附近调试时才对；正常比赛必须打开。\n");
  }
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
  s.yaw_fit_ok = yaw_fit_ok_.load();
  s.corner_refined = corner_refined_.load();
  s.tx_frames = tx_frames_.load();
  s.rx_frames = rx_frames_.load();
  s.rx_resync = rx_resync_.load();
  s.stale_cmds = stale_cmds_.load();
  s.jump_rejected = jump_rejected_.load();
  s.fire_requests = fire_requests_.load();
  s.ms_process = static_cast<double>(avg_process_ns_.load()) / 1e6;
  s.ms_plan = static_cast<double>(avg_plan_ns_.load()) / 1e6;
  s.ms_process_latency = static_cast<double>(avg_process_latency_ns_.load()) / 1e6;
  return s;
}

void AutoAimApp::cameraLoop() {
  while (running_.load()) {//只要相机开着，就一直循环取帧
    auto frame = camera_->grab(50);//取一帧，超时 50ms
    if (!frame) {//取帧失败
      if (camera_->exhausted()) {//回放结束
        std::printf("[camera] 回放结束\n");
        running_.store(false);//停止运行整个程序
        break;
      }
      continue;
    }
    frames_grabbed_.fetch_add(1);//取帧计数器加 1
    latest_frame_.set(*frame);//把取到的帧放到最新帧缓存里
  }
}

solver::Attitude AutoAimApp::attitudeAt(uint64_t t_ns) const {
  solver::Attitude att;
  if (!cfg_.use_attitude) return att;

  Eigen::Quaterniond q;
  if (!attitude_buffer_.query(t_ns, q)) {
    // 队列里还没有姿态（刚启动，或者串口没数据）。
    // 返回单位姿态 = 当作云台在零位，退化成旧行为，不会突然乱指。
    return att;
  }
  return solver::attitudeFromQuat(q);
}

const detect::Armor * AutoAimApp::selectArmor(
    const std::vector<detect::Armor> & armors, double * score_out) const {
  const detect::Armor * best = nullptr;
  double best_score = -1e9;

  for (const auto & a : armors) {
    if (a.type == detect::ArmorType::SINGLE || a.type == detect::ArmorType::INVALID) {
      continue;
    }
    // 前置硬条件（教程 §9.4.1A）：任一项不满足直接丢弃。
    // 本赛题只有一个靶板，这些门限默认放得很宽（见 DetectConfig 的注释）。
    const double pixel_dist = pnp_->calculateDistanceToCenter(a.center);
    if (pixel_dist > cfg_.detect.max_center_dist_px * 3) continue;

    const double center_score =
        std::clamp(1.0 - pixel_dist / std::max(1.0, cfg_.detect.max_center_dist_px), 0.0,
                   1.0);
    // 距离分要有距离才能算，这里先用"离画面中心"近似；
    // 真正的距离在 PnP 之后才算得出来，所以打分只用中心度 + 检测置信度。
    const double conf = a.confidence > 0.0f ? a.confidence : 1.0;
    const double score = (1.0 - cfg_.detect.select_w_dist) * center_score +
                         cfg_.detect.select_w_dist * conf;
    if (score > best_score) {
      best_score = score;
      best = &a;
    }
  }

  if (score_out) *score_out = best_score;
  return best;
}

void AutoAimApp::processFrame(const bsp::Frame & frame) {
  const uint64_t t_begin = bsp::nowNs();
  frames_processed_.fetch_add(1);

  const auto ext = toExtrinsic(cfg_.extrinsic);

  PlanInput input;
  input.frame_ns = frame.timestamp_ns;
  if (auto fb = latest_feedback_.tryGet()) input.bullet_speed = fb->bullet_speed;

  auto armors = detector_->detect(frame.image);
  if (!armors.empty()) armors_found_.fetch_add(1);

  // 角点精修（教程 §5A.3）：把端点从"二值化边界"挪到物理上的明暗交界。
  if (cfg_.detect.corner_refine.enable) {
    for (auto & a : armors) {
      if (corner_optimizer_.refine(frame.image, a)) corner_refined_.fetch_add(1);
    }
  }

  const detect::Armor * best = selectArmor(armors, nullptr);

  if (best != nullptr) {
    cv::Mat rvec, tvec;
    if (pnp_->solvePnP(*best, rvec, tvec)) {
      const double err = pnp_->reprojectionError(*best, rvec, tvec);
      if (err < 0 || err > cfg_.solver.max_reprojection_error_px) {
        pnp_rejected_.fetch_add(1);
        best = nullptr;
      } else {
        // 对齐到本帧图像时刻的云台姿态（教程场景 3），后面几步都要用。
        const solver::Attitude att = attitudeAt(frame.timestamp_ns);

        // ── yaw 降自由度优化（教程 §5A.4）────────────────────────────
        // 平移沿用 PnP 的 tvec（量级基本可信），pitch/roll 固定成机械常量，
        // 只在最敏感的 yaw 上做一维搜索。搜索在世界系里做 —— 教程的
        // R(yaw) = Rz(yaw)·Ry(pitch)·Rx(roll) 本身就是世界系约定。
        if (cfg_.solver.yaw_fit.enable) {
          const Eigen::Matrix3d R_c2w = solver::R_cam2world(ext, att);

          double yaw_fit = 0.0;
          cv::Mat tvec_fit;

          // 先验：跟踪器当前的 yaw 估计（在跟就最可信），否则用上一次被接受的精修值。
          double yaw_ref = 0.0;
          bool has_ref = false;
          if (tracker_ && tracker_->tracking() &&
              tracker_->targetState().size() == tracker::kStateDim) {
            yaw_ref = tracker_->targetState()(6);
            has_ref = true;
          } else if (have_last_yaw_fit_) {
            yaw_ref = last_yaw_fit_;
            has_ref = true;
          }

          // 注意：这里会同时给出"选中的那支 PnP 解"的平移 ——
          // 平面靶板两个解的平移不同，位置必须和姿态取自同一个解。
          const bool fit_ok =
              pnp_->fitYawSingle(*best, R_c2w, yaw_fit, tvec_fit, nullptr, yaw_ref,
                                 has_ref);

          if (fit_ok) {
            const double p = cfg_.solver.yaw_fit.armor_pitch_deg * kDeg2Rad;
            const double r = cfg_.solver.yaw_fit.armor_roll_deg * kDeg2Rad;
            const Eigen::Matrix3d R_m2w =
                Eigen::AngleAxisd(yaw_fit, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()).toRotationMatrix();
            const Eigen::Matrix3d R_m2c = R_c2w.inverse() * R_m2w;

            cv::Mat R_cv(3, 3, CV_64F);
            for (int i = 0; i < 3; ++i) {
              for (int j = 0; j < 3; ++j) R_cv.at<double>(i, j) = R_m2c(i, j);
            }
            cv::Mat rvec_fit;
            cv::Rodrigues(R_cv, rvec_fit);
            rvec = rvec_fit;
            tvec = tvec_fit;
            last_yaw_fit_ = yaw_fit;
            have_last_yaw_fit_ = true;
            yaw_fit_ok_.fetch_add(1);
          }
        }

        pnp_ok_.fetch_add(1);

        // ── 相机系 → 云台零位系 → 世界系（教程场景 1 模块 3 / 场景 3）──
        input.armor_pos_world = solver::cameraToWorld(tvec, ext, att);
        const Eigen::Quaterniond q_world = solver::rvecToWorldQuat(rvec, ext, att);
        input.has_target = input.armor_pos_world.allFinite();
        input.state = last_state_;  // 占位；跟踪失败时下游直瞄用不到

        if (input.has_target && tracker_) {
          tracker::ArmorMeasurement m;
          m.position = input.armor_pos_world;
          m.orientation = q_world;
          m.distance_to_image_center = pnp_->calculateDistanceToCenter(best->center);
          m.number = best->number;
          m.type = detect::armorTypeStr(best->type);

          double dt = 0.01;
          if (last_tracker_update_ns_ != 0) {
            dt = toSeconds(frame.timestamp_ns - last_tracker_update_ns_);
            if (dt <= 0 || dt > 1.0) dt = 0.01;
          }
          last_tracker_update_ns_ = frame.timestamp_ns;

          if (tracker_->state() == tracker::Tracker::LOST) {
            tracker_->init({m});
          } else {
            tracker_->update({m}, dt);
          }
          last_state_ = tracker_->targetState();
          input.state = last_state_;
          input.tracking = tracker_->tracking();

          if (cfg_.debug_dump) {
            std::printf(
                "[track] %-10s model=%-5s r=%.3f v_yaw=%+.2f posdiff=%.3f "
                "yawdiff=%.3f maha=%.1f jump=%.3f\n",
                tracker_->stateStr(), tracker_->modelStateStr(), tracker_->radius(),
                tracker_->vYaw(), tracker_->infoPositionDiff(),
                tracker_->infoYawDiff(), tracker_->infoMahalanobis(),
                tracker_->infoPositionJump());
          }
        } else if (input.has_target && !tracker_) {
          // 没开跟踪器：直接瞄测量点，没有任何提前量。
        }
      }
    } else {
      pnp_rejected_.fetch_add(1);
      best = nullptr;
    }
  }

  if (best == nullptr) {
    input.has_target = false;
    input.tracking = false;
  }

  latest_plan_input_.set(input);

  const uint64_t t_end = bsp::nowNs();
  updateAverage(avg_process_ns_, t_end - t_begin);
  updateAverage(avg_process_latency_ns_, t_end - frame.timestamp_ns);

  // plan_hz == 0 时不跑独立的规划线程，就在这里内联完成，保证只有一个指令生产者。
  if (cfg_.plan_hz <= 0) planAndPublish();

  if (cfg_.debug_view) {
    char line[200];
    std::snprintf(line, sizeof(line), "d=%.2fm yaw=%.2f pitch=%.2f %s%s",
                  input.armor_pos_world.norm(), last_yaw_rad_.load() * kRad2Deg,
                  last_pitch_rad_.load() * kRad2Deg, input.tracking ? "TRK " : "",
                  input.has_target ? "" : "no target");
    drawDebug(frame.image, line);
  }
}

void AutoAimApp::planAndPublish() {
  const uint64_t t0 = bsp::nowNs();

  auto input_opt = latest_plan_input_.tryGet();
  if (!input_opt || !input_opt->has_target) {
    publishHoldCommand();
    return;
  }
  const PlanInput & in = *input_opt;

  double gimbal_yaw_rad = 0.0, gimbal_pitch_rad = 0.0;
  double fb_age_s = 1e9;
  if (auto fb = latest_feedback_.tryGet()) {
    gimbal_yaw_rad = fb->yaw_deg * kDeg2Rad;
    gimbal_pitch_rad = fb->pitch_deg * kDeg2Rad;
    fb_age_s = toSeconds(t0 >= fb->timestamp_ns ? t0 - fb->timestamp_ns : 0);
  }

  // ── 延迟链（教程场景 2/3/6/7）────────────────────────────────────
  timeutil::DelayChain delay;
  delay.process_latency_s = toSeconds(t0 >= in.frame_ns ? t0 - in.frame_ns : 0);
  delay.comm_latency_s = cfg_.aim.comm_latency_s;
  {
    const double dist = in.armor_pos_world.norm();
    const double v = std::max(in.bullet_speed, cfg_.aim.min_bullet_speed);
    delay.fly_time_s = dist / v;
  }

  // 让瞄准解算器与 Planner 用同一套弹道参数。
  aim::AimConfig ac = aim_.config();
  ac.process_latency_s = delay.process_latency_s;
  ac.comm_latency_s = delay.comm_latency_s;
  aim_.setConfig(ac);
  planner_.setAimSolver(aim_);

  plan::Planner::Output out;
  if (in.tracking) {
    out = planner_.plan(in.state, in.bullet_speed, delay, gimbal_yaw_rad,
                        gimbal_pitch_rad);
  } else {
    // 还没进 TRACKING：直接瞄当前测量点，不做提前量、不走 MPC。
    const aim::AimResult a =
        aim_.solveFromMeasurement(in.armor_pos_world, in.bullet_speed);
    out.yaw_rad = a.yaw_rad;
    out.pitch_rad = a.pitch_rad;
    out.valid = a.valid;
    out.total_delay_s = delay.total();
    out.process_latency_s = delay.process_latency_s;
    out.fly_time_s = delay.fly_time_s;
  }

  if (!out.valid) {
    publishHoldCommand();
    return;
  }

  // ── 跳变闸门 ─────────────────────────────────────────────────────
  if (have_last_aim_) {
    const double dyaw =
        std::abs(std::remainder(out.yaw_rad - last_yaw_rad_.load(), kTwoPi)) * kRad2Deg;
    const double dpitch =
        std::abs(out.pitch_rad - last_pitch_rad_.load()) * kRad2Deg;
    if (dyaw > cfg_.aim.max_jump_deg || dpitch > cfg_.aim.max_jump_deg) {
      jump_rejected_.fetch_add(1);
      publishHoldCommand();
      return;
    }
  }
  have_last_aim_ = true;

  // ── 开火（教程 §7.4.1：发弹延迟只改时机，不改指向）─────────────────
  int fire = 0;
  if (cfg_.enable_fire) {
    double cmp_yaw = out.yaw_rad;
    double cmp_pitch = out.pitch_rad;
    if (cfg_.aim.fire_lead_s > 0.0 && in.tracking) {
      // 弹丸真正出膛还要晚 fire_lead 秒，所以对准判定要看那一刻的瞄准角。
      // 下发的瞄准角本身不变 —— 延迟只影响"什么时候扣扳机"。
      const aim::AimResult ahead =
          aim_.solveFromState(in.state, in.bullet_speed, cfg_.aim.fire_lead_s);
      if (ahead.valid) {
        cmp_yaw = ahead.yaw_rad;
        cmp_pitch = ahead.pitch_rad;
      }
    }
    aim::AimResult cmp;
    cmp.valid = true;
    cmp.yaw_rad = cmp_yaw;
    cmp.pitch_rad = cmp_pitch;
    fire = fire_.update(cmp, gimbal_yaw_rad, gimbal_pitch_rad, in.tracking,
                        toSeconds(t0), fb_age_s);
    if (fire) fire_requests_.fetch_add(1);
  }

  publishCommand(out.yaw_rad, out.pitch_rad, fire, t0);

  if (cfg_.debug_dump) {
    std::printf(
        "[plan] tgt=%.2fm delay=%.3fs(proc %.3f + comm %.3f + fly %.3f) "
        "yaw=%+.2f pitch=%+.2f v=%.2f a=%.1f %s%s%s\n",
        in.armor_pos_world.norm(), out.total_delay_s, out.process_latency_s,
        delay.comm_latency_s, out.fly_time_s, out.yaw_rad * kRad2Deg,
        out.pitch_rad * kRad2Deg, out.yaw_vel_rad, out.yaw_acc_rad,
        out.used_mpc ? "MPC" : "direct", fire ? " FIRE" : "",
        out.mpc_iterations > 0 ? " (约束打满/FISTA)" : "");
  }

  updateAverage(avg_plan_ns_, bsp::nowNs() - t0);
}

void AutoAimApp::processLoop() {
  while (running_.load()) {
    const uint64_t seq = latest_frame_.seq();
    if (seq == last_frame_seq_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    last_frame_seq_ = seq;

    auto frame = latest_frame_.tryGet();
    if (!frame) continue;

    processFrame(*frame);
  }
}

void AutoAimApp::planLoop() {
  if (cfg_.plan_hz <= 0) return;  // 指令由 processFrame 内联产生

  const auto period = std::chrono::microseconds(1000000 / cfg_.plan_hz);
  auto next = std::chrono::steady_clock::now();

  while (running_.load()) {
    planAndPublish();

    next += period;
    std::this_thread::sleep_until(next);

    const auto now = std::chrono::steady_clock::now();
    if (now > next + period) next = now;
  }
}

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

    std::this_thread::sleep_until(next);

    const auto now = std::chrono::steady_clock::now();
    if (now > next + period) next = now;
  }
}

void AutoAimApp::rxLoop() {
  srm::FrameParser parser;//帧解析器
  uint8_t buf[512];//接收缓冲区

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

          // 把姿态塞进时间戳队列，供"对齐到图像帧"用（教程场景 3）。
          const solver::Attitude att{fb.yaw_deg * kDeg2Rad, fb.pitch_deg * kDeg2Rad,
                                     fb.roll_deg * kDeg2Rad};
          attitude_buffer_.push(fb.timestamp_ns, solver::attitudeToQuat(att));
        },
        bsp::nowNs());
    rx_resync_.store(parser.resyncCount());
  }
}

void AutoAimApp::publishCommand(double yaw_rad, double pitch_rad, int fire,
                                uint64_t /*stamp_ns*/) {
  srm::TargetCommand cmd;
  cmd.yaw_deg = static_cast<float>(yaw_rad * kRad2Deg);
  cmd.pitch_deg = static_cast<float>(pitch_rad * kRad2Deg);
  cmd.fire_flag = fire;

  latest_cmd_.set(cmd);

  last_yaw_rad_.store(yaw_rad);
  last_pitch_rad_.store(pitch_rad);
}

void AutoAimApp::publishHoldCommand() {
  stale_cmds_.fetch_add(1);

  srm::TargetCommand cmd;
  cmd.yaw_deg = static_cast<float>(last_yaw_rad_.load() * kRad2Deg);
  cmd.pitch_deg = static_cast<float>(last_pitch_rad_.load() * kRad2Deg);
  cmd.fire_flag = 0;
  latest_cmd_.set(cmd);

  fire_.reset();
}

void AutoAimApp::drawDebug(const cv::Mat & rgb, const std::string & line) {
  cv::Mat canvas = rgb.clone();
  detector_->drawResults(canvas);
  cv::cvtColor(canvas, canvas, cv::COLOR_RGB2BGR);

  const auto & K = cfg_.solver.camera_matrix;
  cv::drawMarker(canvas, cv::Point(static_cast<int>(K[2]), static_cast<int>(K[5])),
                 cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 1);

  cv::putText(canvas, line, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(0, 255, 0), 2);
  cv::imshow("autoaim", canvas);
  cv::waitKey(1);
}

}
