#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "app/config.h"
#include "bsp/camera/camera_factory.h"
#include "bsp/serial/serial_port.h"
#include "modules/aim/aim_solver.h"
#include "modules/aim/fire_decision.h"
#include "modules/detect/armor_corner_optimizer.h"
#include "modules/detect/armor_detector_base.h"
#include "modules/message_center/latest.h"
#include "modules/plan/planner.h"
#include "modules/protocol/srm_protocol.h"
#include "modules/solver/coord_transform.h"
#include "modules/solver/pnp_solver.h"
#include "modules/time/attitude_buffer.h"
#include "modules/time/delay_chain.h"
#include "modules/tracker/tracker.h"

namespace autoaim::app {

// 总指挥 —— 唯一认识所有人的类。
//
// 五条线程（教程场景 6 的高低频双线程架构）：
//
//   cameraLoop  相机帧率   抓图 → latest_frame_
//   processLoop 有新帧就跑  检测 → 角点精修 → 选靶 → PnP → yaw 降自由度
//                          → 云台零位系→世界系 → EKF 更新 → latest_plan_input_
//   planLoop    250 Hz     取最新状态 + 延迟链 → MPC → 跳变闸门 → 开火判定
//                          → latest_cmd_
//   txLoop      100 Hz     只做"取最新值 → 打包 → 写串口"
//   rxLoop      收到就读    收反馈 → latest_feedback_ + attitude_buffer_
//
// 为什么要把 planLoop 单独拆出来：只有相机新图才带来**目标状态**的新消息，
// 所以检测/更新的频率受相机帧率限制；但**指令下发**不必等新图 —— EKF 在帧间
// 也在按匀速模型外推。按更高频率下发，下位机的参考输入更连续、离散化误差更小。
class AutoAimApp {
 public:
  explicit AutoAimApp(config::Config cfg);//构造函数

  ~AutoAimApp();//析构函数

  AutoAimApp(const AutoAimApp &) = delete;//拷贝构造函数
  AutoAimApp & operator=(const AutoAimApp &) = delete;

  bool init();

  void cameraLoop();

  void processLoop();

  void planLoop();

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
    uint64_t yaw_fit_ok = 0;
    uint64_t corner_refined = 0;
    uint64_t tx_frames = 0;
    uint64_t rx_frames = 0;
    uint64_t rx_resync = 0;
    uint64_t stale_cmds = 0;
    uint64_t jump_rejected = 0;
    uint64_t fire_requests = 0;

    // 教程场景 6 的分阶段耗时（毫秒，滑动平均）
    double ms_process = 0.0;
    double ms_plan = 0.0;
    double ms_process_latency = 0.0;
  };

  Stats stats() const;

  const config::Config & config() const { return cfg_; }

 private:
  // 传给规划线程的一帧"快照"：检测线程产出的目标状态。
  struct PlanInput {
    Eigen::VectorXd state;  // 9 维 EKF 状态（世界系）
    Eigen::Vector3d armor_pos_world = Eigen::Vector3d::Zero();
    bool tracking = false;
    bool has_target = false;
    double bullet_speed = 22.0;
    uint64_t frame_ns = 0;  // 这一帧图像到达上位机的时刻
  };

  void processFrame(const bsp::Frame & frame);

  // 规划 + 开火 + 下发。只有 planLoop（或 plan_hz==0 时的 processFrame）会调，
  // 所以不需要为指令状态加锁。
  void planAndPublish();

  // 取这一帧图像时间戳对应的云台姿态。队列为空时返回单位姿态（等价于
  // "云台恒在零位"，即退化成旧行为）。
  solver::Attitude attitudeAt(uint64_t t_ns) const;

  // 加权打分选靶（教程 §9.4.1A）。
  const detect::Armor * selectArmor(const std::vector<detect::Armor> & armors,
                                    double * score_out) const;

  void publishHoldCommand();

  void publishCommand(double yaw_rad, double pitch_rad, int fire, uint64_t stamp_ns);

  void drawDebug(const cv::Mat & rgb, const std::string & line);

  config::Config cfg_;

  std::unique_ptr<bsp::CameraSource> camera_;
  bsp::SerialPort serial_;
  std::unique_ptr<detect::ArmorDetectorBase> detector_;
  detect::ArmorCornerOptimizer corner_optimizer_;
  std::unique_ptr<solver::PnPSolver> pnp_;
  std::unique_ptr<tracker::Tracker> tracker_;
  aim::AimSolver aim_;
  aim::FireDecision fire_;
  plan::Planner planner_;

  timeutil::AttitudeBuffer attitude_buffer_;

  msg::Latest<bsp::Frame> latest_frame_;
  msg::Latest<srm::TargetCommand> latest_cmd_;
  msg::Latest<srm::GimbalFeedback> latest_feedback_;
  msg::Latest<PlanInput> latest_plan_input_;

  std::atomic<bool> running_{true};

  std::atomic<uint64_t> frames_grabbed_{0};
  std::atomic<uint64_t> frames_processed_{0};
  std::atomic<uint64_t> armors_found_{0};
  std::atomic<uint64_t> pnp_ok_{0};
  std::atomic<uint64_t> pnp_rejected_{0};
  std::atomic<uint64_t> yaw_fit_ok_{0};
  std::atomic<uint64_t> corner_refined_{0};
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> rx_resync_{0};
  std::atomic<uint64_t> stale_cmds_{0};
  std::atomic<uint64_t> jump_rejected_{0};
  std::atomic<uint64_t> fire_requests_{0};

  // 滑动平均的阶段耗时（纳秒）
  std::atomic<uint64_t> avg_process_ns_{0};
  std::atomic<uint64_t> avg_plan_ns_{0};
  std::atomic<uint64_t> avg_process_latency_ns_{0};

  uint64_t last_frame_seq_ = 0;

  // 以下只在规划线程里读写
  std::atomic<double> last_yaw_rad_{0.0};
  std::atomic<double> last_pitch_rad_{0.0};
  bool have_last_aim_ = false;
  uint64_t last_tracker_update_ns_ = 0;

  // yaw 降自由度上一次被接受的结果，用于跨帧的"分支翻转"闸门。
  double last_yaw_fit_ = 0.0;
  bool have_last_yaw_fit_ = false;

  // 供 processFrame 复用的最近一次跟踪结果（只在检测线程里用）
  Eigen::VectorXd last_state_;
};

}
