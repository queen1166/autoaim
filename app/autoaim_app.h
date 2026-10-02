
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
  explicit AutoAimApp(config::Config cfg);

  ~AutoAimApp();

  AutoAimApp(const AutoAimApp &) = delete;
  AutoAimApp & operator=(const AutoAimApp &) = delete;

  bool init();


  void cameraLoop();

  void processLoop();

  void txLoop();

  void rxLoop();

  void stop() { running_.store(false); }

  bool running() const { return running_.load(); }

  struct Stats {
    uint64_t frames_grabbed = 0;
    uint64_t frames_processed = 0;
    uint64_t armors_found = 0;
    uint64_t pnp_ok = 0;
    uint64_t pnp_rejected = 0;
    uint64_t tx_frames = 0;
    uint64_t rx_frames = 0;
    uint64_t rx_resync = 0;
    uint64_t stale_cmds = 0;
    uint64_t jump_rejected = 0;
    uint64_t fire_requests = 0;
  };

  Stats stats() const;

  const config::Config & config() const { return cfg_; }

 private:
  void processFrame(const bsp::Frame & frame);

  void publishHoldCommand();

  void publishCommand(double yaw_rad, double pitch_rad, int fire, uint64_t stamp_ns);

  void drawDebug(const cv::Mat & rgb, const std::string & line);

  config::Config cfg_;


  std::unique_ptr<bsp::CameraSource> camera_;
  bsp::SerialPort serial_;
  detect::Detector detector_;
  std::unique_ptr<solver::PnPSolver> pnp_;
  std::unique_ptr<tracker::Tracker> tracker_;
  aim::AimSolver aim_;
  aim::FireDecision fire_;

  msg::Latest<bsp::Frame> latest_frame_;
  msg::Latest<srm::TargetCommand> latest_cmd_;
  msg::Latest<srm::GimbalFeedback> latest_feedback_;

  std::atomic<bool> running_{true};

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


  uint64_t last_frame_seq_ = 0;
  uint64_t last_aim_ns_ = 0;
  double last_yaw_rad_ = 0.0;
  double last_pitch_rad_ = 0.0;
  uint64_t last_tracker_update_ns_ = 0;

  bool have_last_aim_ = false;
};

}
