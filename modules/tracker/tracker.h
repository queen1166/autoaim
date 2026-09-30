// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.
//
// 派生自 rm_auto_aim/armor_tracker/{include/armor_tracker/tracker.hpp, src/tracker.cpp}
//
// 改动：
//   1. 去掉全部 ROS：angles → std::remainder、tf2 → Eigen、rclcpp → 无
//   2. Armors::SharedPtr → std::vector<ArmorMeasurement>
//   3. 半径从【硬钳位】改成【软约束】+ RotationMode 双模式（见 tracker.cpp）
//   4. updateArmorsNum 短路：校内赛是单靶板，恒为 1 块
//   5. 删掉 publishMarkers / adaptAngularVelocity（前者是 RViz，后者上游没实现）

#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "modules/tracker/armor_measurement.h"
#include "modules/tracker/extended_kalman_filter.h"
#include "modules/tracker/target_state.h"
#include "modules/tracker/tracker_config.h"

namespace autoaim::tracker {

// 装甲板跟踪器。输入一连串带噪观测，输出"靶板在哪、转多快、半径多大"。
//
// 两个核心价值：
//   1. 滤抖动 —— 单帧检测有噪声，EKF 把它平滑掉
//   2. 预测未来 —— 板子侧对相机时检测会中断（实测约 29% 的帧），
//      这段时间靠模型外推撑住，不然云台会丢目标
class Tracker {
 public:
  // 构造：只存配置，状态置 LOST。真正的初始化要等 init()。
  explicit Tracker(const TrackerConfig & cfg = {});

  // 用一帧观测初始化。上游选择"离图像中心最近的那块"。
  //
  // 会把状态机推到 DETECTING，并按这块板的观测值填初值和初始协方差。
  // 已经 TRACKING 时调它没用（应该走 update()）。
  //
  // armors 这一帧检测到的所有装甲板。为空时直接返回，状态不变。
  void init(const std::vector<ArmorMeasurement> & armors);

  // 主循环。每帧处理线程调一次。这是整个跟踪器的入口。
  //
  // 内部做：数据关联（这块板还在不在）→ EKF 预测 → EKF 更新
  //         → 半径约束 → 状态机推进 → 必要时 handleArmorJump()
  //
  // armors 这一帧的观测（可能为空 —— 检测没出结果）
  // dt     距上一帧的【秒数】。EKF 用它做状态外推，传错会导致
  //        速度估计整体偏移，所以必须是真实帧间隔。
  //
  // 观测为空时不会崩，会走"丢失"分支靠预测撑住。
  void update(const std::vector<ArmorMeasurement> & armors, double dt);

  // 清空所有状态回到 LOST：状态向量清零、计数器归零、EKF 重置。
  // 目标切换（换了块板子）或检测到跟踪彻底跑飞时调用。
  void reset();

  // 跟踪器的五种状态。
  enum State {
    LOST,           // 还没找到目标，或丢太久彻底放弃。不输出有效状态
    DETECTING,      // 找到了但还没连续匹配够 tracking_thres 帧，收敛中
    TRACKING,       // 正常跟踪中。可瞄准
    TEMP_LOST,      // 短暂丢失（板子侧对相机了）。靠预测撑住，仍可瞄准
    CHANGE_TARGET,  // 目标突变（可能是换板或误匹配），正在重新收敛
  };

  // 当前状态。调试和上层决策用。
  State state() const { return tracker_state; }

  // 状态的字符串形式（"LOST"/"TRACKING" 等），打印日志用。
  const char * stateStr() const;

  // 可以拿去瞄准的状态：TRACKING 和 TEMP_LOST 都算（TEMP_LOST 靠预测撑住）
  bool tracking() const {
    return tracker_state == TRACKING || tracker_state == TEMP_LOST;
  }

  // 9 维状态向量，布局见 target_state.h。
  // 注意：没初始化时可能还是【空的】（size() != 9），
  // 用之前先看 targetState().size() == kStateDim。
  const Eigen::VectorXd & targetState() const { return target_state; }

  // AUTO 模式下实际生效的模式（判定完之后），不是配置里那个 AUTO。
  // 调试时想看"到底判成了哪种"就调它。CAROUSEL / SELF_SPIN。
  RotationMode activeMode() const { return active_mode_; }

  // 当前估计的旋转半径（米），就是状态第 [8] 维。转盘模式有物理意义，
  // 自旋模式应该趋近 0。现场用它判断该用哪种模式（见 README）。
  // 状态还没初始化时返回 0。
  double radius() const { return target_state.size() == kStateDim ? target_state(8) : 0.0; }

  // 当前估计的角速度（rad/s），状态第 [7] 维。瞄准层的提前量直接用它。
  // ⚠️ 检测空窗后重捕时它会瞬间冲到 55 rad/s（真值 2.0），
  //    aim_solver 里有钳位，别直接信。状态没初始化时返回 0。
  double vYaw() const { return target_state.size() == kStateDim ? target_state(7) : 0.0; }

  // 当前正在跟踪的那块装甲板的原始观测（位置 + 姿态）。
  // 调试和可视化用。
  const ArmorMeasurement & trackedArmor() const { return tracked_armor; }

  // 正在跟踪的装甲板编号。校内赛单靶板、且不启用分类器，
  // 所以恒为空串（匹配逻辑照样工作，见 armor_measurement.h）。
  const std::string & trackedId() const { return tracked_id; }

  // 观测位置与预测位置的差（米），数据关联产生的诊断量。
  // 持续偏大说明模型和实际对不上（比如模式选错了）。
  double infoPositionDiff() const { return info_position_diff; }

  // 观测 yaw 与预测 yaw 的差（rad）。同上，是诊断量不是控制量。
  double infoYawDiff() const { return info_yaw_diff; }

  // 当前生效的跟踪器配置（只读）。
  const TrackerConfig & config() const { return cfg_; }

  // 公开的 EKF 实例。设为 public 是为了让测试能直接检查内部状态
  // （比如断言协方差有没有被正确膨胀）。业务代码不要碰它。
  ExtendedKalmanFilter ekf;

 private:
  // 按第一帧观测建立 EKF 模型：填初值 x0、初始协方差 P0，
  // 并把六个 lambda（f/h/雅可比/Q/R）注册进去。
  // 只在 init() 里调一次。
  void initEkf(const ArmorMeasurement & a);

  // 判定"目标变了"（换板/误匹配）后重建跟踪：
  // 保留状态向量但重置部分计数器，让滤波器重新收敛。
  // 从 CHANGE_TARGET 状态恢复时调。
  void initChange(const ArmorMeasurement & a);

  // 装甲板跳变处理：当观测到的 yaw 与预测差得太多
  // （多半是同一块板转到了另一面），直接把状态瞬移过去。
  //
  // 【必须配一次协方差膨胀】—— 否则 P 还停在"已经收敛"的小值上，
  // 滤波器会过度相信自己刚编出来的状态，接下来十几帧发散。
  // 这正是 setStateInflated() 存在的原因。
  void handleArmorJump(const ArmorMeasurement & a);

  // 把四元数转成【连续】的 yaw —— 往 last_yaw_ 上累加，
  // 跨越 ±π 时不跳变。自旋模式下 v_yaw ≠ 0 全靠它，否则 EKF 推不动。
  //
  // 举例：板子连续转了 3 圈，这个函数返回的是 6π 附近的值，
  // 而不是被折回 ±π —— 折回去的话角速度就永远是 0 了。
  double orientationToYaw(const Eigen::Quaterniond & q);

  // 半径约束。上游是【硬钳位】（把 r 强行夹进 [r_min, r_max]），
  // 这里改成了软约束，按当前模式区别对待：
  //   CAROUSEL  → 每帧朝边界拉回一点（r_pull）
  //   SELF_SPIN → 每帧乘 r_shrink 向 0 收缩
  // 每帧 update 里调一次。
  void applyRadiusConstraint();

  // AUTO 模式的判定逻辑：前 auto_grace_frames 帧先不动（等滤波器收敛），
  // 之后若 r 连续 auto_switch_frames 帧贴着下界，就切到 SELF_SPIN。
  // 只在 cfg_.mode == AUTO 时有实际动作。
  void updateAutoMode();

  TrackerConfig cfg_;
  RotationMode active_mode_;

  State tracker_state = LOST;
  std::string tracked_id;
  ArmorMeasurement tracked_armor;

  Eigen::VectorXd target_state;

  double dt_ = 0.01;
  double last_yaw_ = 0.0;
  int lost_thres = 30;

  int detect_count_ = 0;
  int lost_count_ = 0;
  int change_count_ = 0;

  int auto_frames_ = 0;
  int low_r_frames_ = 0;

  double info_position_diff = 0.0;
  double info_yaw_diff = 0.0;
};

}  // namespace autoaim::tracker
