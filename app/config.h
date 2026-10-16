
#pragma once

#include <array>
#include <string>
#include <vector>

#include "modules/aim/aim_solver.h"
#include "modules/aim/fire_decision.h"
#include "modules/detect/armor_corner_optimizer.h"
#include "modules/detect/armor_detector.h"
#include "modules/detect/yolo_pose.h"
#include "modules/plan/planner.h"
#include "modules/solver/pnp_solver.h"
#include "modules/tracker/tracker_config.h"

namespace autoaim::config {

enum class CameraKind { REPLAY, HIK };

// 探测后端（教程 §5A.1 的三种方案）。
enum class DetectorKind {
  TRADITIONAL,  // 方案 A：灰度二值化 → 灯条 → 配对
  YOLO_POSE,    // 方案 B：神经网络端到端出 4 关键点
};

// 相机配置
struct CameraConfig {
  CameraKind kind = CameraKind::REPLAY;

  std::string replay_dir = "bags/latest";
  double replay_fps = 60.0;
  bool replay_loop = true;

  float exposure_us = 5000.0f;
  float gain = 10.0f;
  bool auto_exposure = false;

  // 诊断用：>0 时在开流前把采集分辨率改成这个值；0 = 不动，用相机自己的。
  // 为什么需要/为什么要小心，见 HikCamera::Config 里同名成员的注释。
  int width = 0;
  int height = 0;
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
  // 默认走神经网络（教程 §5A.1 方案 B）：模型路径和各项参数在
  // detect::YoloPoseConfig 里已经有默认值，不用传任何参数就能跑。
  // 想显式用传统视觉加 --no-yolo；模型文件缺失时会自动回退（见 yolo_fallback_to_traditional）。
  DetectorKind kind = DetectorKind::YOLO_POSE;

  int binary_thres = 160;
  // 灯条颜色过滤：0=RED，1=BLUE，-1=ANY（不过滤，单靶板推荐）。
  int detect_color = 0;

  detect::Detector::LightParams light;
  detect::Detector::ArmorParams armor;

  bool single_light_fallback = true;
  int max_single_light_count = 3;

  // YOLO-Pose（kind == YOLO_POSE 时生效）。默认值就是仓库自带的那个装甲板模型，
  // 见 detect::YoloPoseConfig 顶部的注释。
  detect::YoloPoseConfig yolo;
  bool yolo_fallback_to_traditional = true;  // 模型加载失败时退回传统视觉

  // 角点精修（教程 §5A.3）
  detect::CornerRefineConfig corner_refine;

  // ── 目标筛选与打分（教程 §9.4.1A）──────────────────────────────────
  // Score = w_dist·(1 − d/max_d) + (1−w_dist)·(1 − 像素距/maxSD)
  //
  // ⚠️ 教程的硬门限是 距离<4.5 m、|yaw|<0.85 rad、高度<1.5 m（那是为
  // 多目标赛场设计的：宁可放过也不打一个大角度侧对、PnP 很差的板子）。
  // 本赛题只有 5 m 处**一块**靶板，按教程的门限会把唯一的目标筛掉，
  // 所以默认值放宽。真要打多目标再按教程收紧。
  double select_w_dist = 0.6;
  double max_distance_accept = 10.0;
  double max_yaw_accept_rad = 1.5708;
  double max_height_accept = 3.0;
  double max_center_dist_px = 500.0;  // Center_Score 的归一化尺度
};

// PnP 求解配置
struct SolverConfig {
  solver::ArmorGeometry geometry;

  std::array<double, 9> camera_matrix = {1000.0, 0.0, 720.0,
                                         0.0, 1000.0, 540.0,
                                         0.0, 0.0, 1.0};
  std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};

  double max_reprojection_error_px = 3.0;

  // yaw 降自由度优化（教程 §5A.4）
  solver::YawFitConfig yaw_fit;
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
  plan::PlannerConfig planner;

  // 高频下发线程的频率（教程场景 6 的高低频双线程）。0 = 关掉，
  // 退化成"处理线程直接算角度、发送线程按 tx_hz 发"。
  int plan_hz = 250;

  // 是否使用云台反馈的姿态做"云台零位系 → 世界系"变换（教程场景 1/3）。
  // 关掉时退化为云台零位系（等价于云台恒在 0°，只有调试才这么用）。
  bool use_attitude = true;

  // 姿态队列容量（对齐图像时间戳用的样本数）。
  int attitude_buffer_size = 256;

  bool enable_fire = false;
  bool debug_view = false;
  bool debug_dump = false;
};

bool parseArgs(int argc, char ** argv, Config & cfg);

void printHelp(const char * prog);

}
