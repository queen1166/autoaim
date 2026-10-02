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

}

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

void AutoAimApp::cameraLoop() {
  while (running_.load()) {
    auto frame = camera_->grab(50);
    if (!frame) {
      if (camera_->exhausted()) {
        std::printf("[camera] 回放结束\n");
        running_.store(false);
        break;
      }
      continue;
    }
    frames_grabbed_.fetch_add(1);
    latest_frame_.set(*frame);
  }
}

void AutoAimApp::processFrame(const bsp::Frame & frame) {
  frames_processed_.fetch_add(1);

  auto armors = detector_.detect(frame.image);
  if (armors.empty()) {
    publishHoldCommand();
    if (cfg_.debug_view) drawDebug(frame.image, "no armor");
    return;
  }
  armors_found_.fetch_add(1);

  const auto best = std::min_element(
      armors.begin(), armors.end(), [this](const detect::Armor & a, const detect::Armor & b) {
        return pnp_->calculateDistanceToCenter(a.center) <
               pnp_->calculateDistanceToCenter(b.center);
      });

  if (best->type == detect::ArmorType::SINGLE) {
    publishHoldCommand();
    if (cfg_.debug_view) drawDebug(frame.image, "single light (degraded)");
    return;
  }

  cv::Mat rvec, tvec;
  if (!pnp_->solvePnP(*best, rvec, tvec)) {
    publishHoldCommand();
    return;
  }

  const double err = pnp_->reprojectionError(*best, rvec, tvec);
  if (err < 0 || err > cfg_.solver.max_reprojection_error_px) {
    pnp_rejected_.fetch_add(1);
    publishHoldCommand();
    if (cfg_.debug_view) drawDebug(frame.image, "pnp rejected");
    return;
  }
  pnp_ok_.fetch_add(1);

  const auto ext = toExtrinsic(cfg_.extrinsic);
  const Eigen::Vector3d pos_gimbal = solver::cameraToGimbal(tvec, ext);

  if (!pos_gimbal.allFinite() || pos_gimbal.x() <= 0.1) {
    publishHoldCommand();
    return;
  }

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
      if (dt <= 0 || dt > 1.0) dt = 0.01;
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
    aim_result = aim_.solveFromMeasurement(pos_gimbal, 22.0);
  }

  if (!aim_result.valid) {
    publishHoldCommand();
    return;
  }

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

  int fire = 0;
  if (cfg_.enable_fire && aim_result.valid) {
    double gimbal_yaw_rad = 0.0, gimbal_pitch_rad = 0.0;
    double fb_age_s = 1e9;
    if (auto fb = latest_feedback_.tryGet()) {
      gimbal_yaw_rad = fb->yaw_deg * kDeg2Rad;
      gimbal_pitch_rad = fb->pitch_deg * kDeg2Rad;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    last_frame_seq_ = seq;

    auto frame = latest_frame_.tryGet();
    if (!frame) continue;

    processFrame(*frame);
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
    rx_resync_.store(parser.resyncCount());
  }
}

void AutoAimApp::publishCommand(double yaw_rad, double pitch_rad, int fire,
                                uint64_t stamp_ns) {
  srm::TargetCommand cmd;
  cmd.yaw_deg = static_cast<float>(yaw_rad * kRad2Deg);
  cmd.pitch_deg = static_cast<float>(pitch_rad * kRad2Deg);
  cmd.fire_flag = fire;

  latest_cmd_.set(cmd);

  last_yaw_rad_ = yaw_rad;
  last_pitch_rad_ = pitch_rad;
  last_aim_ns_ = stamp_ns;
}

void AutoAimApp::publishHoldCommand() {
  stale_cmds_.fetch_add(1);

  srm::TargetCommand cmd;
  cmd.yaw_deg = static_cast<float>(last_yaw_rad_ * kRad2Deg);
  cmd.pitch_deg = static_cast<float>(last_pitch_rad_ * kRad2Deg);
  cmd.fire_flag = 0;
  latest_cmd_.set(cmd);

  fire_.reset();
}

void AutoAimApp::drawDebug(const cv::Mat & rgb, const std::string & line) {
  cv::Mat canvas = rgb.clone();
  detector_.drawResults(canvas);
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
