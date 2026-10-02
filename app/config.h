
#pragma once

#include <array>
#include <string>
#include <vector>

#include "modules/aim/aim_solver.h"
#include "modules/aim/fire_decision.h"
#include "modules/detect/armor_detector.h"
#include "modules/solver/pnp_solver.h"
#include "modules/tracker/tracker_config.h"

namespace autoaim::config {

enum class CameraKind { REPLAY, HIK };

// 相机配置
struct CameraConfig {
  CameraKind kind = CameraKind::REPLAY;

  std::string replay_dir = "bags/latest";
  double replay_fps = 60.0;
  bool replay_loop = true;

  float exposure_us = 5000.0f;
  float gain = 10.0f;
  bool auto_exposure = false;
};

// 串口配置
struct SerialConfig {
  std::string device = "/dev/ttyACM0";
  int baud = 115200;
  int tx_hz = 100;
  int read_timeout_ms = 20;
};

// 检测配置
struct DetectConfig {
  int binary_thres = 160;
  int detect_color = 0;

  detect::Detector::LightParams light;
  detect::Detector::ArmorParams armor;

  bool single_light_fallback = true;
  int max_single_light_count = 3;
};

// PnP 求解配置
struct SolverConfig {
  solver::ArmorGeometry geometry;

  std::array<double, 9> camera_matrix = {1000.0, 0.0, 720.0,
                                         0.0, 1000.0, 540.0,
                                         0.0, 0.0, 1.0};
  std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};

  double max_reprojection_error_px = 3.0;
};

// 相机外参配置
struct ExtrinsicCfg {
  double cam_yaw_deg = 0.0;
  double cam_pitch_deg = -15.0;
  double cam_roll_deg = 0.0;
  double cam_x = 0.0;
  double cam_y = 0.0;
  double cam_z = 0.0;
};

//  总配置
struct Config {
  CameraConfig camera;   //相机
  SerialConfig serial;   //串口
  DetectConfig detect;  //检测
  SolverConfig solver;  //PnP求解
  ExtrinsicCfg extrinsic; //相机外参

  bool enable_tracker = false;
  tracker::TrackerConfig tracker;

  aim::AimConfig aim;
  aim::FireConfig fire;

  bool enable_fire = false;
  bool debug_view = false;
  bool debug_dump = false;
};

bool parseArgs(int argc, char ** argv, Config & cfg);

void printHelp(const char * prog);

}
