#include "app/config.h"

#include "modules/common/units.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace autoaim::config {

namespace {

bool wantValue(int i, int argc, char ** argv) {
  if (i + 1 >= argc) {
    std::fprintf(stderr, "参数 %s 缺少值\n", argv[i]);
    return false;
  }
  return true;
}

}

void printHelp(const char * prog) {
  std::printf(
      "用法: %s [选项]\n"
      "\n"
      "相机:\n"
      "  --replay <目录>        用图像目录做离线回放（默认模式）\n"
      "  --replay-fps <fps>     回放帧率，默认 60\n"
      "  --no-loop              回放不循环\n"
      "  --hik                  用海康相机（需要 MVS SDK）\n"
      "  --exposure <us>        曝光时间，微秒，默认 5000\n"
      "  --gain <g>             增益，默认 10\n"
      "  --width <px>           压小采集宽度，默认用相机自己的\n"
      "  --height <px>          压小采集高度，同上\n"
      "                         ⚠️ 诊断用：`StartGrabbing` 报 0x80000006 时\n"
      "                         拿来验证是不是内核 USB 缓冲上限；会让标定\n"
      "                         内参失效，不是比赛配置\n"
      "\n"
      "串口:\n"
      "  --device <路径>        串口设备，默认 /dev/ttyACM0\n"
      "                        建议用 /dev/serial/by-id/...\n"
      "  --tx-hz <hz>           发送频率，默认 100\n"
      "\n"
      "算法:\n"
      "  --thres <n>            二值化阈值，默认 160（现场必须重调）\n"
      "  --color <red|blue|any> 灯条颜色过滤，默认 red\n"
      "                         单靶板配不出对时先试 any\n"
      "  --armor-w <mm>         两灯条中心距，默认 132（★ 必须实测反标）\n"
      "  --armor-h <mm>         灯条长度，默认 57（★ 必须实测反标）\n"
      "  --cam-pitch <deg>      相机俯仰安装角，负值向下，默认 -15\n"
      "  --cam-yaw <deg>        相机水平安装角，默认 0\n"
      "  --fx --fy --cx --cy    相机内参（★ 必须自己标定）\n"
      "  --enable-tracker       启用跟踪器（默认关）\n"
      "  --rotation <mode>      carousel | self_spin | auto（默认 auto）\n"
      "  --armor-pitch <deg>    装甲板自身俯仰（机械常量），默认 15\n"
      "  --no-yaw-fit           关掉 PnP 降自由度优化，退回完整 solvePnP\n"
      "  --armor-yaw-offset <deg> 装甲板朝向的 π 偏置；现场标 r 时用，见 README\n"
      "\n"
      "探测（教程 §5A）：默认用神经网络（仓库自带的装甲板模型）\n"
      "  --no-yolo              改用传统视觉（二值化 + 灯条配对）\n"
      "  --yolo <模型.onnx>     换成别的 YOLO-Pose 模型\n"
      "                         默认 models/Infantry-v8n-opencv.onnx\n"
      "  --yolo-layout <mode>   ultralytics | color-class-kpt，默认 color-class-kpt\n"
      "                         会一次设好下面 4 项的默认值（见 RUNBOOK §5.6）\n"
      "  --yolo-w <px>          输入宽，默认 640\n"
      "  --yolo-h <px>          输入高，默认 480（ultralytics 惯例是 640）\n"
      "  --yolo-classes <n>     类别数，默认 9（ultralytics 惯例是 1）\n"
      "  --yolo-kpt-dim <n>     每个关键点的数值个数，默认 2：3=(x,y,conf)\n"
      "  --yolo-input-scale <f> 输入缩放，默认 1.0（ultralytics 惯例是 1/255）\n"
      "  --yolo-conf <f>        检出置信度门限，默认 0.25\n"
      "  --no-corner-refine     关掉 PCA 角点精修（教程 §5A.3）\n"
      "\n"
      "规划（教程场景 6/8）：\n"
      "  --no-mpc               关掉 MPC，直接用延迟补偿后的瞄准角\n"
      "  --max-yaw-acc <rad/s2> MPC 偏航角加速度上限，默认 50\n"
      "  --max-pitch-acc <r/s2> MPC 俯仰角加速度上限，默认 100\n"
      "  --comm-latency <s>     兜底通讯延迟超参，默认 0\n"
      "  --plan-hz <hz>         高频规划线程频率，默认 250（0 = 关）\n"
      "  --no-attitude          不用下位机反馈姿态做世界系变换（仅调试）\n"
      "  --fire-lead <s>        发弹延迟；只影响开火时机，不改瞄准角\n"
      "  --enable-fire          允许开火（默认关）\n"
      "\n"
      "调试:\n"
      "  --debug-view           显示图像窗口（按 q 退出）\n"
      "  --debug-dump           打印跟踪状态\n"
      "  -h, --help             显示本帮助\n"
      "\n"
      "示例:\n"
      "  # 离线回放调检测参数\n"
      "  %s --replay bags/0929 --debug-view --thres 180\n"
      "\n"
      "  # 真机：先只瞄准不开火\n"
      "  %s --hik --device /dev/serial/by-id/usb-XXX --debug-dump\n"
      "\n"
      "安全提示: 默认不开火。确认瞄得准之后再加 --enable-fire。\n",
      prog, prog, prog);
}

bool parseArgs(int argc, char ** argv, Config & cfg) {
  // 记录哪些 --yolo-* 是显式给过的。收尾段按**最终** layout 给没给过的项填整组默认值，
  // 于是 --yolo-layout 是一键切换，而显式值永远赢——两种书写顺序结果一致，
  // 和 §5.1「后写的覆盖先写的」不冲突。
  struct {
    bool w = false, h = false, classes = false, kpt_dim = false, scale = false;
  } yolo_set;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];

    if (a == "-h" || a == "--help") {
      printHelp(argv[0]);
      return false;
    } else if (a == "--replay") { //--replay <目录>用图片目录做离线回放
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.kind = CameraKind::REPLAY;
      cfg.camera.replay_dir = argv[++i];
    } else if (a == "--replay-fps") { //--replay-fps <fps>回放帧率
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.replay_fps = std::atof(argv[++i]);
    } else if (a == "--no-loop") {//--no-loop回放不循环
      cfg.camera.replay_loop = false;
    } else if (a == "--hik") {//--hik用海康相机
      cfg.camera.kind = CameraKind::HIK;
    } else if (a == "--exposure") {//--exposure <us> 曝光时间
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.exposure_us = static_cast<float>(std::atof(argv[++i]));
    } else if (a == "--gain") {//--gain <g> 增益
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.gain = static_cast<float>(std::atof(argv[++i]));
    } else if (a == "--width") {//--width <px> 压小采集宽度（诊断用）
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.width = std::atoi(argv[++i]);
    } else if (a == "--height") {//--height <px> 压小采集高度（诊断用）
      if (!wantValue(i, argc, argv)) return false;
      cfg.camera.height = std::atoi(argv[++i]);
    } else if (a == "--device") {//--device <路径> 串口设备
      if (!wantValue(i, argc, argv)) return false;
      cfg.serial.device = argv[++i];
    } else if (a == "--tx-hz") {//--tx-hz <hz> 发送频率
      if (!wantValue(i, argc, argv)) return false;
      cfg.serial.tx_hz = std::atoi(argv[++i]);
      if (cfg.serial.tx_hz <= 0 || cfg.serial.tx_hz > 1000) {
        std::fprintf(stderr, "--tx-hz 要在 1~1000 之间\n");
        return false;
      }
    } else if (a == "--thres") {//--thres <n> 二值化阈值
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.binary_thres = std::atoi(argv[++i]);
    } else if (a == "--color") {//--color <red|blue|any> 灯条颜色过滤
      if (!wantValue(i, argc, argv)) return false;
      const std::string c = argv[++i];
      if (c == "red") {
        cfg.detect.detect_color = detect::RED;
      } else if (c == "blue") {
        cfg.detect.detect_color = detect::BLUE;
      } else if (c == "any") {
        cfg.detect.detect_color = detect::ANY_COLOR;
      } else {
        std::fprintf(stderr, "--color 只能是 red / blue / any\n");
        return false;
      }
    } else if (a == "--armor-w") {//--armor-w <mm> 两灯条中心距
      if (!wantValue(i, argc, argv)) return false;
      const float w = static_cast<float>(std::atof(argv[++i]));
      cfg.solver.geometry.small_width = w;
      cfg.solver.geometry.large_width = w;
    } else if (a == "--armor-h") {//--armor-h <mm> 灯条长度
      if (!wantValue(i, argc, argv)) return false;
      const float h = static_cast<float>(std::atof(argv[++i]));
      cfg.solver.geometry.small_height = h;
      cfg.solver.geometry.large_height = h;
    } else if (a == "--cam-pitch") {//--cam-pitch <deg> 相机俯仰安装角
      if (!wantValue(i, argc, argv)) return false;
      cfg.extrinsic.cam_pitch_deg = std::atof(argv[++i]);
    } else if (a == "--cam-yaw") {//--cam-yaw <deg> 相机偏航安装角
      if (!wantValue(i, argc, argv)) return false;
      cfg.extrinsic.cam_yaw_deg = std::atof(argv[++i]);
    } else if (a == "--cam-roll") {//--cam-roll <deg> 相机翻滚安装角
      if (!wantValue(i, argc, argv)) return false;
      cfg.extrinsic.cam_roll_deg = std::atof(argv[++i]);
    } else if (a == "--fx") {//--fx <f> X轴焦距
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[0] = std::atof(argv[++i]);
    } else if (a == "--fy") {//--fy <f> Y轴焦距
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[4] = std::atof(argv[++i]);
    } else if (a == "--cx") {//--cx <f> X轴主点
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[2] = std::atof(argv[++i]);
    } else if (a == "--cy") {//--cy <f> Y轴主点
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.camera_matrix[5] = std::atof(argv[++i]);
    } else if (a == "--enable-tracker") {//--enable-tracker 启用EKF跟踪器
      cfg.enable_tracker = true;
    } else if (a == "--rotation") {//--rotation <mode> 跟踪器旋转模式： carousel | self_spin | auto
      if (!wantValue(i, argc, argv)) return false;
      const std::string m = argv[++i];
      if (m == "carousel") {
        cfg.tracker.mode = tracker::RotationMode::CAROUSEL;
      } else if (m == "self_spin") {
        cfg.tracker.mode = tracker::RotationMode::SELF_SPIN;
      } else if (m == "auto") {
        cfg.tracker.mode = tracker::RotationMode::AUTO;
      } else {
        std::fprintf(stderr, "--rotation 只能是 carousel / self_spin / auto\n");
        return false;
      }
      //-----------------三个运行模式-------------
    } else if (a == "--armor-pitch") {//--armor-pitch 装甲板自身俯仰（机械常量）
      if (!wantValue(i, argc, argv)) return false;
      cfg.solver.yaw_fit.armor_pitch_deg = std::atof(argv[++i]);
    } else if (a == "--no-yaw-fit") {//关掉 PnP 降自由度优化
      cfg.solver.yaw_fit.enable = false;
    } else if (a == "--armor-yaw-offset") {//装甲板朝向的 π 偏置（现场标 r 时用）
      if (!wantValue(i, argc, argv)) return false;
      cfg.tracker.armor_yaw_offset_rad = std::atof(argv[++i]) * kDeg2Rad;
    } else if (a == "--no-yolo") {//--no-yolo 显式改用传统视觉探测
      cfg.detect.kind = DetectorKind::TRADITIONAL;
    } else if (a == "--yolo") {//--yolo 换成别的 YOLO-Pose 模型（默认已经是神经网络）
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.kind = DetectorKind::YOLO_POSE;
      cfg.detect.yolo.model_path = argv[++i];
    } else if (a == "--yolo-w") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.yolo.input_w = std::atoi(argv[++i]);
      yolo_set.w = true;
    } else if (a == "--yolo-h") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.yolo.input_h = std::atoi(argv[++i]);
      yolo_set.h = true;
    } else if (a == "--yolo-classes") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.yolo.num_classes = std::atoi(argv[++i]);
      yolo_set.classes = true;
    } else if (a == "--yolo-kpt-dim") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.yolo.kpt_dim = std::atoi(argv[++i]);
      yolo_set.kpt_dim = true;
    } else if (a == "--yolo-conf") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.yolo.conf_thres = static_cast<float>(std::atof(argv[++i]));
    } else if (a == "--yolo-layout") {//输出头布局，顺带决定输入缩放
      if (!wantValue(i, argc, argv)) return false;
      const std::string v = argv[++i];
      if (v == "ultralytics") {
        cfg.detect.yolo.head_layout = detect::HeadLayout::ULTRALYTICS;
      } else if (v == "color-class-kpt") {
        cfg.detect.yolo.head_layout = detect::HeadLayout::COLOR_CLASS_KPT;
      } else {
        std::fprintf(stderr, "--yolo-layout 只能是 ultralytics / color-class-kpt\n");
        return false;
      }
    } else if (a == "--yolo-input-scale") {//覆盖 layout 决定的输入缩放
      if (!wantValue(i, argc, argv)) return false;
      cfg.detect.yolo.input_scale = static_cast<float>(std::atof(argv[++i]));
      yolo_set.scale = true;
    } else if (a == "--no-corner-refine") {//关掉 PCA 角点精修
      cfg.detect.corner_refine.enable = false;
    } else if (a == "--no-mpc") {//关掉 MPC
      cfg.planner.enable = false;
    } else if (a == "--max-yaw-acc") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.planner.yaw_mpc.max_acc = std::atof(argv[++i]);
    } else if (a == "--max-pitch-acc") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.planner.pitch_mpc.max_acc = std::atof(argv[++i]);
    } else if (a == "--comm-latency") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.aim.comm_latency_s = std::atof(argv[++i]);
    } else if (a == "--plan-hz") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.plan_hz = std::atoi(argv[++i]);
      if (cfg.plan_hz < 0 || cfg.plan_hz > 1000) {
        std::fprintf(stderr, "--plan-hz 要在 0~1000 之间（0 = 关）\n");
        return false;
      }
    } else if (a == "--no-attitude") {//不用反馈姿态做世界系变换（仅调试）
      cfg.use_attitude = false;
    } else if (a == "--fire-lead") {
      if (!wantValue(i, argc, argv)) return false;
      cfg.aim.fire_lead_s = std::atof(argv[++i]);
    } else if (a == "--enable-fire") {//--enable-fire 启用射击
      cfg.enable_fire = true;
    } else if (a == "--debug-view") {//--debug-view 启用调试视图
      cfg.debug_view = true;
    } else if (a == "--debug-dump") {//--debug-dump 启用调试转储
      cfg.debug_dump = true;
    } else {
      std::fprintf(stderr, "未知参数: %s\n\n", a.c_str());
      printHelp(argv[0]);
      return false;
    }
  }

  // --yolo-layout 决定整套惯例值（输入尺寸 / 类别数 / kpt_dim / 输入缩放）。不放在
  // 分支里做，是因为 layout 和别的 --yolo-* 谁先写都可能，放在收尾段才能保证
  // "显式值永远赢" 与书写顺序无关。
  {
    auto & y = cfg.detect.yolo;
    const bool ultra = y.head_layout == detect::HeadLayout::ULTRALYTICS;
    if (!yolo_set.w) y.input_w = 640;
    if (!yolo_set.h) y.input_h = ultra ? 640 : 480;
    if (!yolo_set.classes) y.num_classes = ultra ? 1 : 9;
    if (!yolo_set.kpt_dim) y.kpt_dim = ultra ? 3 : 2;
    if (!yolo_set.scale) y.input_scale = ultra ? 1.0f / 255.0f : 1.0f;
  }

  if (cfg.enable_fire) {
    cfg.fire.require_tracking = cfg.enable_tracker;
  }
  return true;
}

}
