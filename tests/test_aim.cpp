
#include <Eigen/Dense>

#include <cmath>
#include <cstdio>

#include "modules/aim/aim_solver.h"
#include "modules/tracker/target_state.h"

using namespace autoaim;
using namespace autoaim::aim;

namespace {

int g_failed = 0;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

void checkNear(double got, double want, double tol, const char * what) {
  const bool ok = std::abs(got - want) < tol;
  std::printf("  %s %-38s got %+9.4f  want %+9.4f\n", ok ? "PASS" : "FAIL", what, got,
              want);
  if (!ok) g_failed++;
}

Eigen::VectorXd makeState(double xc, double yc, double za, double yaw, double v_yaw,
                          double r) {
  Eigen::VectorXd s = Eigen::VectorXd::Zero(tracker::kStateDim);
  s(0) = xc;
  s(2) = yc;
  s(4) = za;
  s(6) = yaw;
  s(7) = v_yaw;
  s(8) = r;
  return s;
}

}

int main() {
  std::printf("用例 1：弹道补偿\n");
  {
    const double th = AimSolver::ballisticPitch(5.0, 0.0, 22.0, 9.8);
    std::printf("       5m, 弹速22, 水平 → %.4f° （方案里估的是 2.9°）\n",
                th * kRad2Deg);
    checkNear(th * kRad2Deg, 2.905, 0.05, "俯仰角");

    const double d = 5.0, h = 0.0, v = 22.0, g = 9.8;
    const double k = g * d * d / (2 * v * v);
    const double u = std::tan(th);
    const double residual = d * u - k * (1.0 + u * u) - h;
    std::printf("       代回方程残差 = %+.3e m（应为 0）\n", residual);
    checkNear(residual, 0.0, 1e-12, "弹道方程残差");
  }

  std::printf("用例 2：距离越远，补偿越大\n");
  {
    double prev = -1;
    bool mono = true;
    for (double d : {1.0, 3.0, 5.0, 8.0, 12.0}) {
      const double th = AimSolver::ballisticPitch(d, 0.0, 22.0, 9.8) * kRad2Deg;
      std::printf("       %.0f m → %.3f°\n", d, th);
      if (th <= prev) mono = false;
      prev = th;
    }
    check(mono, "单调递增");
  }

  std::printf("用例 3：solveFromMeasurement —— 正前方 5m 的静止靶\n");
  {
    AimSolver solver;
    const AimResult r =
        solver.solveFromMeasurement(Eigen::Vector3d(5.0, 0.0, 0.0), 22.0);
    check(r.valid, "结果有效");
    checkNear(r.yaw_rad * kRad2Deg, 0.0, 1e-9, "yaw");
    checkNear(r.pitch_rad * kRad2Deg, 2.905, 0.05, "pitch（含弹道）");
    checkNear(r.t_lead_s, 0.0, 1e-12, "t_lead = 0（无速度信息）");
  }

  std::printf("用例 4：关掉弹道模型 + 经验偏置 3°\n");
  {
    AimConfig cfg;
    cfg.enable_ballistic = false;
    cfg.pitch_offset_deg = 3.0;
    AimSolver solver(cfg);
    const AimResult r =
        solver.solveFromMeasurement(Eigen::Vector3d(5.0, 0.0, 0.0), 22.0);
    checkNear(r.pitch_rad * kRad2Deg, 3.0, 1e-9, "pitch = 偏置");
  }

  std::printf("用例 5：转盘模式 r=0.5, v_yaw=+1.0 rad/s\n");
  {
    AimSolver solver;
    const Eigen::VectorXd s = makeState(5.0, 0.0, 0.0, 0.0, 1.0, 0.5);

    const Eigen::Vector3d p_now = tracker::armorPositionFromState(s);
    std::printf("       当前装甲板位置 = (%.4f, %.4f, %.4f)\n", p_now.x(), p_now.y(),
                p_now.z());

    const AimResult r = solver.solveFromState(s, 22.0);
    const double yaw_deg = r.yaw_rad * kRad2Deg;
    std::printf("       提前量 %.4f s（飞行 %.4f + 延迟 %.2f + 云台 %.2f）\n",
                r.t_lead_s, r.t_flight_s, solver.config().latency_s,
                solver.config().gimbal_lag_s);
    std::printf("       瞄准 yaw = %+.4f°\n", yaw_deg);

    check(r.t_lead_s > 0.25 && r.t_lead_s < 0.35, "t_lead 约 0.3s");
    check(yaw_deg < -1.0, "有提前量且方向为负（往右提前）");
    check(yaw_deg > -3.0, "提前量幅度合理（< 3°）");

    const double t = r.t_lead_s;
    const double yaw_pred = 0.0 + 1.0 * t;
    const double xa = 5.0 - 0.5 * std::cos(yaw_pred);
    const double ya = 0.0 - 0.5 * std::sin(yaw_pred);
    checkNear(yaw_deg, std::atan2(ya, xa) * kRad2Deg, 1e-9, "与独立复算一致");
  }

  std::printf("用例 6：自旋模式 r=0 —— 角度不应有提前量\n");
  {
    AimSolver solver;
    const Eigen::VectorXd s = makeState(5.0, 0.0, 0.0, 0.0, 2.0, 0.0);

    const AimResult r_state = solver.solveFromState(s, 22.0);
    const AimResult r_meas =
        solver.solveFromMeasurement(tracker::armorPositionFromState(s), 22.0);

    std::printf("       solveFromState     yaw=%+.6f°  pitch=%+.6f°\n",
                r_state.yaw_rad * kRad2Deg, r_state.pitch_rad * kRad2Deg);
    std::printf("       solveFromMeasurement yaw=%+.6f°  pitch=%+.6f°\n",
                r_meas.yaw_rad * kRad2Deg, r_meas.pitch_rad * kRad2Deg);

    checkNear(r_state.yaw_rad * kRad2Deg, r_meas.yaw_rad * kRad2Deg, 1e-9,
              "两种路径 yaw 一致");
    checkNear(r_state.pitch_rad * kRad2Deg, r_meas.pitch_rad * kRad2Deg, 1e-9,
              "两种路径 pitch 一致");
  }

  std::printf("用例 7：r=0 时换不同 v_yaw，角度不变\n");
  {
    AimSolver solver;
    const AimResult a = solver.solveFromState(makeState(5, 0, 0, 0.0, 0.0, 0.0), 22.0);
    const AimResult b = solver.solveFromState(makeState(5, 0, 0, 1.5, 9.0, 0.0), 22.0);
    checkNear(a.yaw_rad * kRad2Deg, b.yaw_rad * kRad2Deg, 1e-9, "yaw 不变");
    checkNear(a.pitch_rad * kRad2Deg, b.pitch_rad * kRad2Deg, 1e-9, "pitch 不变");
  }

  std::printf("用例 8：弹速为 0 或 NaN 时不产生 inf/nan\n");
  {
    AimSolver solver;
    const AimResult r0 = solver.solveFromMeasurement(Eigen::Vector3d(5, 0, 0), 0.0);
    check(r0.valid && std::isfinite(r0.pitch_rad), "弹速 0 → 仍有限");
    const AimResult rn =
        solver.solveFromMeasurement(Eigen::Vector3d(5, 0, 0), std::nan(""));
    check(std::isfinite(rn.pitch_rad) || !rn.valid, "弹速 NaN → 有限或置 invalid");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
