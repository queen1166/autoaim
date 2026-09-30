// 全部可调参数的唯一来源。代码里不许再出现魔数。
//
// 首版用常量头而不是 yaml —— 少一个依赖就少一类现场环境问题。
// 等把流程跑顺了再考虑上 yaml-cpp。
//
// ⚠️ 带 ★ 的值是【占位符】，必须在现场实测后改。默认值照抄上游，
//    而上游是他们实验室的环境，和 227 实验室/走廊完全不同。

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

// 相机相关参数。两个分支（回放 / 海康真机）共用这一个结构体，
// 各自只读自己那几个字段。
struct CameraConfig {
  CameraKind kind = CameraKind::REPLAY;

  // ── 回放（离线调参用）────────────────────────────────────
  std::string replay_dir = "bags/latest";
  double replay_fps = 60.0;
  bool replay_loop = true;

  // ── 海康真机 ────────────────────────────────────────────
  float exposure_us = 5000.0f;  // ★ 现场调：能看清装甲板的最短曝光
  float gain = 10.0f;           // ★ 尽量低，噪声会放大
  bool auto_exposure = false;   // ⭐ 必须关
};

// 串口相关参数。
struct SerialConfig {
  // ★ 强烈建议用 /dev/serial/by-id/... —— 重新插拔后 ttyACM 编号会变
  std::string device = "/dev/ttyACM0";
  int baud = 115200;  // USB CDC 下是废参数，填上无害
  int tx_hz = 100;    // 协议推荐的固定发送频率
  int read_timeout_ms = 20;  // 接收线程的单次读超时，决定退出响应有多快
};

// 检测相关参数。binary_thres 是现场最需要调的一个。
struct DetectConfig {
  // ★ 实测：这个值的实际含义是"多亮才算灯条"。
  //    纯红 (255,0,0) 灰度只有 76，低于 160 就完全抓不到。
  //    和曝光、补光、环境光强强相关，换场地必须重调。
  int binary_thres = 160;
  int detect_color = 0;  // 0 = RED

  detect::Detector::LightParams light;
  detect::Detector::ArmorParams armor;

  bool single_light_fallback = true;
  int max_single_light_count = 3;
};

// 解算相关参数：装甲板几何 + 相机内参 + 质量闸门。
struct SolverConfig {
  // ★ 这是【灯条几何】，不是装甲板外形尺寸！
  //   width  = 两根灯条中心之间的距离
  //   height = 灯条的长度
  //   实测：把 140 当成 132 填进去，5m 处距离直接错 +5.15%。
  //   反标方法：靶板放卷尺量准的 5.00m，调这两个数直到 tvec 模长对得上。
  solver::ArmorGeometry geometry;

  // ★ 相机内参，必须自己标定（棋盘格 + cv::calibrateCamera）
  std::array<double, 9> camera_matrix = {1000.0, 0.0, 720.0,
                                         0.0, 1000.0, 540.0,
                                         0.0, 0.0, 1.0};
  std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};

  // 重投影误差闸门（像素）。超过就丢弃这一帧的观测，
  // 挡住 PnP 的解分支翻转造成的离群（实测 IPPE 有这个问题，
  // ITERATIVE 好很多但仍需一道闸门）。
  double max_reprojection_error_px = 3.0;
};

// 相机安装外参（相机系 → 云台系）。字段含义与 coord_transform.h 的
// ExtrinsicConfig 完全一致，这里是给命令行用的副本。
struct ExtrinsicCfg {
  // ★ 相机安装角，必须实测反解。见 coord_transform.h
  double cam_yaw_deg = 0.0;
  double cam_pitch_deg = -15.0;  // 负值 = 相机向下俯
  double cam_roll_deg = 0.0;
  double cam_x = 0.0;            // 相机光心在云台系下的位置（米），
  double cam_y = 0.0;            // 首版可以全填 0 —— 5m 处 0.05m 的
  double cam_z = 0.0;            // 平移误差只相当于 0.6 度
};

// 总配置。持有上面所有子配置，外加几个全局开关。
// AutoAimApp 构造时拷一份（explicit AutoAimApp(config::Config cfg) 是值传递）。
struct Config {
  CameraConfig camera;
  SerialConfig serial;
  DetectConfig detect;
  SolverConfig solver;
  ExtrinsicCfg extrinsic;

  // 跟踪器开关。默认【关】—— 最小闭环先不接跟踪器，把"云台指向靶板"
  // 跑通再说。中期检查后再打开。
  bool enable_tracker = false;
  tracker::TrackerConfig tracker;

  aim::AimConfig aim;
  aim::FireConfig fire;

  bool enable_fire = false;   // 默认不开火，先确认瞄得准
  bool debug_view = false;    // 显示图像窗口（只在处理线程里）
  bool debug_dump = false;    // 打印跟踪状态
};

// 从命令行覆盖 cfg 里的默认值。argv[0] 是程序名，从 i=1 开始解析。
//
// 所有参数都用 "--名字 值" 的形式，逐个 if-else 匹配（参数不多，
// 不值得上 getopt）。遇到 --enable-* 这类开关就置 true，不看下一位。
//
// 返回 false 表示【应该退出】—— 两种情形：
//   1. 用户传了 -h/--help（已经打印了帮助）
//   2. 参数不合法（已经打印了错误和帮助）
// 两种情况下调用方都该直接 return 1，不能再往下跑。
//
// 注意它还在最后做了一次交叉校验：开了 fire 就按 enable_tracker
// 决定要不要强制要求跟踪器可用。
bool parseArgs(int argc, char ** argv, Config & cfg);

// 打印命令行帮助（参数清单 + 两个示例 + 安全提示）。
// prog 传 argv[0]，用于在用法行和示例里显示程序名。
// main.cpp 在解析失败时会自动调用它。
void printHelp(const char * prog);

}  // namespace autoaim::config
