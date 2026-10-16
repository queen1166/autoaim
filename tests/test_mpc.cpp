// MPC 与弹道迭代的离线测试。
//
// 这一组用例专门盯住一个很隐蔽的坑：MPC 的二次型 H 元素量级由 dt² 决定
// （dt=0.01 时只有 1e-2），如果 ADMM 的惩罚系数 ρ 不跟着缩放，
// 求解器会"看起来在跑、实际一动不动"——输出贴着初始值，完全不跟踪参考。
// 症状是自瞄指令几乎恒定，端到端跑出来的 yaw 跨度接近 0。

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>

#include "modules/aim/aim_solver.h"
#include "modules/plan/mpc.h"
#include "modules/plan/planner.h"
#include "modules/tracker/target_state.h"

using namespace autoaim;
using namespace autoaim::plan;

namespace {

int g_failed = 0;
constexpr double kRad2Deg = 180.0 / M_PI;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

Mpc::Config makeCfg(double max_acc) {
  Mpc::Config c;
  c.horizon = 100;
  c.half_horizon = 50;
  c.dt = 0.01;
  c.max_acc = max_acc;
  return c;
}

Eigen::MatrixXd constantRef(int n, double theta, double omega) {
  Eigen::MatrixXd r(n, 2);
  r.col(0).setConstant(theta);
  r.col(1).setConstant(omega);
  return r;
}

}  // namespace

int main() {
  std::printf("用例 1：常值参考 —— MPC 必须跟上去（这是失控的那个坑）\n");
  {
    const double want = 5.0 * M_PI / 180.0;  // 5°
    Mpc mpc;
    mpc.setup(makeCfg(50.0));
    check(mpc.ready(), "setup 成功");

    const auto sol = mpc.solve(0.0, 0.0, constantRef(100, want, 0.0), 50);
    std::printf("       参考 %.3f° → 第 50 步输出 %.3f°\n", want * kRad2Deg,
                sol.angle * kRad2Deg);
    check(sol.ok, "求解成功");
    check(std::abs(sol.angle - want) < 0.2 * M_PI / 180.0,
          "在第 50 步跟上了参考（误差 < 0.2°）");
  }

  std::printf("用例 2：角加速度约束必须被尊重\n");
  {
    Mpc mpc;
    mpc.setup(makeCfg(50.0));

    // 大幅阶跃：不设约束的话 MPC 会输出爆表的角加速度。
    const Eigen::MatrixXd ref = constantRef(100, 1.0, 0.0);
    const auto sol = mpc.solve(0.0, 0.0, ref, 50);
    std::printf("       阶跃 57.3° → |α| = %.3f rad/s²（上限 50）\n",
                std::abs(sol.acc));
    check(std::abs(sol.acc) <= 50.0 + 1e-9, "|α| ≤ max_acc");
    check(std::abs(sol.vel) <= 50.0 * 0.5 + 1e-6, "角速度在物理可达范围内");
  }

  std::printf("用例 3：初始状态已经对准时，不应该乱动\n");
  {
    const double want = 3.0 * M_PI / 180.0;
    Mpc mpc;
    mpc.setup(makeCfg(50.0));
    const auto sol = mpc.solve(want, 0.0, constantRef(100, want, 0.0), 50);
    std::printf("       已对准 → 输出 %.4f°（参考 %.4f°）\n", sol.angle * kRad2Deg,
                want * kRad2Deg);
    check(std::abs(sol.angle - want) < 1e-3 * M_PI / 180.0, "保持在参考上");
  }

  std::printf("用例 4：参考是斜坡时，MPC 应该有一个稳定的跟随滞后而不是不动\n");
  {
    Mpc mpc;
    mpc.setup(makeCfg(50.0));
    Eigen::MatrixXd ref(100, 2);
    const double omega = 2.0;  // rad/s
    for (int k = 0; k < 100; ++k) {
      ref(k, 0) = omega * 0.01 * k;
      ref(k, 1) = omega;
    }
    const auto sol = mpc.solve(0.0, omega, ref, 50);
    std::printf("       参考第 50 步 %.4f°，输出 %.4f°，α=%.3f\n",
                ref(50, 0) * kRad2Deg, sol.angle * kRad2Deg, sol.acc);
    // 角速度已经匹配、只有位置误差 → 输出应接近参考。
    check(std::abs(sol.angle - ref(50, 0)) < 0.5 * M_PI / 180.0, "斜坡跟随误差 < 0.5°");
  }

  std::printf("用例 5：Planner —— 转盘目标下 yaw 指令必须随目标摆动\n");
  {
    // 与 tools/make_test_bag.py 的转盘工况一致：中心 5m，半径 0.3m，2 rad/s。
    PlannerConfig pc;
    pc.yaw_mpc = makeCfg(50.0);
    pc.pitch_mpc = makeCfg(100.0);
    Planner planner(pc);

    aim::AimConfig ac;  // 用默认弹道参数
    aim::AimSolver solver(ac);
    planner.setAimSolver(solver);

    double min_yaw = 1e9, max_yaw = -1e9;
    bool all_valid = true;
    bool any_mpc = false;
    for (int i = 0; i < 200; ++i) {
      const double t = i * 0.01;
      const double yaw_phase = 2.0 * t;
      Eigen::VectorXd s = Eigen::VectorXd::Zero(tracker::kStateDim);
      s(0) = 5.0;                       // 旋转中心 x
      s(2) = 0.0;                       // 旋转中心 y
      s(4) = 0.0;                       // 高度
      s(6) = yaw_phase;                 // 装甲板当前相位
      s(7) = 2.0;                       // 转速 rad/s
      s(8) = 0.3;                       // 半径

      timeutil::DelayChain delay;
      delay.process_latency_s = 0.005;
      delay.fly_time_s = 0.21;

      const auto out = planner.plan(s, 22.0, delay, 0.0, 0.0);
      if (!out.valid) {
        all_valid = false;
        break;
      }
      any_mpc = any_mpc || out.used_mpc;
      min_yaw = std::min(min_yaw, out.yaw_rad);
      max_yaw = std::max(max_yaw, out.yaw_rad);
    }

    const double span_deg = (max_yaw - min_yaw) * kRad2Deg;
    std::printf("       yaw 指令跨度 = %.2f°（真值摆幅约 ±3.4° → 跨度约 6.9°）\n",
                span_deg);
    check(all_valid, "全程输出有效");
    check(any_mpc, "走的是 MPC 路径");
    check(span_deg > 4.0, "yaw 随转盘摆动（> 4°）");
    check(span_deg < 9.0, "摆幅没有超出物理范围（< 9°）");
  }

  std::printf("用例 6：Planner 的俯仰必须带弹道补偿\n");
  {
    PlannerConfig pc;
    pc.yaw_mpc = makeCfg(50.0);
    pc.pitch_mpc = makeCfg(100.0);
    Planner planner(pc);
    planner.setAimSolver(aim::AimSolver(aim::AimConfig{}));

    Eigen::VectorXd s = Eigen::VectorXd::Zero(tracker::kStateDim);
    s(0) = 5.0;  // 静止目标，正前方 5m
    timeutil::DelayChain delay;
    delay.fly_time_s = 5.0 / 22.0;

    const auto out = planner.plan(s, 22.0, delay, 0.0, 0.0);
    std::printf("       静止 5m 目标 → pitch = %.3f°（弹道解算应为 2.905°）\n",
                out.pitch_rad * kRad2Deg);
    check(out.valid, "输出有效");
    // MPC 从 0 起步、参考是 2.905°，取第 k_cmd 步 —— 只要方向对、量级对即可。
    check(out.pitch_rad > 0.0, "俯仰为正（抬枪口补偿下坠）");
    check(out.pitch_rad * kRad2Deg < 3.6, "没有超过物理量级");
  }

  std::printf("用例 7：fly_time 迭代收敛\n");
  {
    aim::AimSolver solver(aim::AimConfig{});
    const aim::AimResult a = solver.solveFromMeasurement(Eigen::Vector3d(5, 0, 0), 22.0);
    const double want = 5.0 / 22.0;
    std::printf("       5m @ 22m/s → t_flight = %.4f s（真值 %.4f）\n", a.t_flight_s,
                want);
    check(std::abs(a.t_flight_s - want) < 1e-3, "飞行时间正确");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
