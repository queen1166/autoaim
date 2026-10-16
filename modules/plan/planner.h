#pragma once

#include <Eigen/Dense>

#include "modules/aim/aim_solver.h"
#include "modules/plan/mpc.h"
#include "modules/time/delay_chain.h"

namespace autoaim::plan {

// Planner —— 把"逐帧算瞄准角"升级成"规划一条云台追得上的角度轨迹"（教程场景 6/8）。
//
// 一次 plan() 做的事：
//   1. 用 EKF 状态 + 延迟链，向前铺出 HORIZON 步的**参考轨迹**（目标未来该被
//      瞄到的绝对角 yaw/pitch）。每步都跑一遍弹道解算，所以高低差和重力都算进去。
//   2. 把参考轨迹的 yaw 解卷绕到连续分支，并对齐到云台当前角度所在的那一支
//      （否则 MPC 会为了差 2π 去转一整圈）。
//   3. 对 yaw / pitch 各跑一个 Mpc，求出满足 |α| ≤ max_acc 的最优轨迹。
//   4. 取"延迟链总时长"那一步的优化值作为下发的指令角度。
//
// 【为什么取延迟那一步】下发的角度是**绝对角**，下位机收到后立刻写入云台目标；
// 但指令要经过串口、下位机解包、云台跟随滞后才真正生效，弹丸还要再飞一段。
// 所以下发"延迟之后那一刻目标会在的角度"。
//
// 【和教程的一处偏差】教程说参考窗口"向前后各看 0.5 s，取 HALF_HORIZON 处"。
// 但 MPC 从**当前**云台状态开始积分，索引 k 只能对应"现在 + k·DT"；若把窗口
// 对称地摆到瞄准时刻两侧，索引 0 就跑到过去了，和初始状态对不上时间。
// 所以这里改成**从当前时刻向前铺 1.0 s 的窗口**，取第 (delay/DT) 步 ——
// 时间上是自洽的，而"向前看满 1 秒"同样给了 MPC 跨越装甲板切换的视野，
// 削尖峰的效果一致。想要教程的字面行为可以把 Mpc::Config::center_window
// 打开并改回默认取 half_horizon。
struct PlannerConfig {
  Mpc::Config yaw_mpc;
  Mpc::Config pitch_mpc;
  bool enable = true;

  // 参考轨迹里单个角度对时间的差分上限，防止 EKF 速度炸掉时参考轨迹飞出天际。
  double max_ref_vel_rad = 20.0;
};

class Planner {
 public:
  struct Output {
    double yaw_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_vel_rad = 0.0;
    double pitch_vel_rad = 0.0;
    double yaw_acc_rad = 0.0;
    double pitch_acc_rad = 0.0;

    // MPC 的迭代次数；0 表示走的是"无约束解即可行"的快路径（精确解）。
    // 大于 0 说明角加速度约束真的打满了，用的是 FISTA 迭代解。
    int mpc_iterations = 0;

    double total_delay_s = 0.0;
    double process_latency_s = 0.0;
    double fly_time_s = 0.0;

    bool valid = false;
    bool used_mpc = false;
  };

  explicit Planner(const PlannerConfig & cfg = {});

  void setConfig(const PlannerConfig & cfg);

  void setAimSolver(const aim::AimSolver & solver) { aim_ = solver; }

  const aim::AimSolver & aimSolver() const { return aim_; }

  // target_state：跟踪器的 9 维状态（**世界系**）
  // delay        ：延迟链（process / comm / fly_time）
  // gimbal_*_rad ：下位机反馈的当前云台角（度 → 弧度），作为 MPC 的初始状态
  Output plan(const Eigen::VectorXd & target_state, double bullet_speed,
              const timeutil::DelayChain & delay, double gimbal_yaw_rad,
              double gimbal_pitch_rad);

  // 不做 MPC 的退化路径：直接返回"延迟之后"的瞄准角。
  Output planDirect(const Eigen::VectorXd & target_state, double bullet_speed,
                    const timeutil::DelayChain & delay);

 private:
  // 把 EKF 状态按匀速模型外推 t 秒（t 可为负 = 往回推）。
  Eigen::VectorXd extrapolate(const Eigen::VectorXd & state, double t) const;

  // 铺参考轨迹：两个 horizon × 2 的矩阵，每行 [θ_ref, ω_ref]，
  // 索引 k 对应"现在 + k·DT"。yaw 与 pitch 各一份（俯仰的弹道补偿不同）。
  void buildReference(const Eigen::VectorXd & state, double bullet_speed,
                      double t_start_s, int horizon, Eigen::MatrixXd & ref_yaw,
                      Eigen::MatrixXd & ref_pitch) const;

  PlannerConfig cfg_;
  aim::AimSolver aim_;
  Mpc yaw_mpc_;
  Mpc pitch_mpc_;
};

}
