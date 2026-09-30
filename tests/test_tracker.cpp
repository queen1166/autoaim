// 跟踪器测试。
//
// 三块内容：
//   1. 数学自洽：armorPositionFromState 必须和 EKF 的 h(x) 前两维完全一致。
//      上游没有这个测试，靠肉眼核对；写错符号的话跟踪器不会报错，只会静默发散。
//   2. 转盘模式收敛：合成一条 r=0.3、ω=2rad/s 的圆周轨迹，看 r 和 v_yaw 能否收敛。
//   3. 自旋模式收敛：合成一条 r=0 的"位置不动、朝向在转"的轨迹。
//      这是上游模型会坏掉的情形 —— 它把 r 硬钳在 [0.12,0.4]。

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <vector>

#include "modules/tracker/tracker.h"

using namespace autoaim;
using namespace autoaim::tracker;

namespace {

int g_failed = 0;
constexpr double kRad2Deg = 180.0 / M_PI;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

ArmorMeasurement makeMeasurement(double xc, double yc, double za, double yaw, double r) {
  ArmorMeasurement m;
  // 与 target_state.h 里的几何关系一致：xa = xc - r*cos(yaw)
  m.position = {xc - r * std::cos(yaw), yc - r * std::sin(yaw), za};
  // 装甲板朝向。真实系统里朝向和角位置差一个常量，这里简化成相等，
  // 足够验证"yaw 的提取与展平"这条链路。
  m.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  m.distance_to_image_center = 0.0;
  return m;
}

struct RunResult {
  bool reached_tracking = false;
  double final_r = 0;
  double final_vyaw = 0;
  double position_err = 0;
  int tracking_frames = 0;
};

RunResult runSynthetic(RotationMode mode, double r_true, double omega, int frames) {
  TrackerConfig cfg;
  cfg.mode = mode;
  Tracker trk(cfg);

  const double dt = 0.01;  // 100 Hz
  const double xc_true = 5.0, yc_true = 2.0, za_true = 0.1;

  RunResult res;
  for (int i = 0; i < frames; ++i) {
    const double t = i * dt;
    const double yaw = omega * t;
    const auto m = makeMeasurement(xc_true, yc_true, za_true, yaw, r_true);

    if (trk.state() == Tracker::LOST) {
      trk.init({m});
    } else {
      trk.update({m}, dt);
    }
    if (trk.tracking()) res.tracking_frames++;
  }

  res.reached_tracking = trk.tracking();
  res.final_r = trk.radius();
  res.final_vyaw = trk.vYaw();
  const Eigen::Vector3d p = armorPositionFromState(trk.targetState());
  const double t_end = (frames - 1) * dt;
  const auto m_end = makeMeasurement(xc_true, yc_true, za_true, omega * t_end, r_true);
  res.position_err = (p - m_end.position).norm();
  return res;
}

}  // namespace

int main() {
  // ── 用例 1：h(x) 与 armorPositionFromState 必须一致 ──────────────
  std::printf("用例 1：EKF 的 h(x) 与 armorPositionFromState 自洽\n");
  {
    // 独立实现一遍 h(x) 的前两维（不是调用被测代码）
    auto h_independent = [](const Eigen::VectorXd & x) {
      return Eigen::Vector3d{x(0) - x(8) * std::cos(x(6)),
                             x(2) - x(8) * std::sin(x(6)), x(4)};
    };

    double worst = 0;
    for (double yaw : {-3.0, -1.0, 0.0, 0.7, 2.9}) {
      for (double r : {0.0, 0.12, 0.3, 0.5}) {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(kStateDim);
        x << 5.0, 0.1, 2.0, -0.2, 0.3, 0.0, yaw, 1.5, r;
        worst = std::max(worst, (armorPositionFromState(x) - h_independent(x)).norm());
      }
    }
    std::printf("       最大偏差 = %.3e\n", worst);
    check(worst < 1e-12, "两处实现逐元素一致（符号没写反）");
  }

  // ── 用例 2：转盘模式 r=0.3, ω=2rad/s ─────────────────────────
  std::printf("用例 2：CAROUSEL —— 真值 r=0.30, ω=2.0 rad/s\n");
  {
    const auto res = runSynthetic(RotationMode::CAROUSEL, 0.3, 2.0, 300);
    std::printf("       r=%.4f (真值 0.30)   v_yaw=%.4f (真值 2.0)   位置误差=%.4f m\n",
                res.final_r, res.final_vyaw, res.position_err);
    std::printf("       TRACKING 帧数 %d/300\n", res.tracking_frames);
    check(res.reached_tracking, "进入 TRACKING");
    check(std::abs(res.final_r - 0.3) < 0.05, "r 收敛（±0.05）");
    check(std::abs(res.final_vyaw - 2.0) < 0.5, "v_yaw 收敛（±0.5）");
    check(res.position_err < 0.05, "位置误差 < 5cm");
  }

  // ── 用例 3：自旋模式 r=0 ────────────────────────────────────
  // 这是上游模型会坏的情形：r 真值是 0，但上游硬钳在 [0.12,0.4]，
  // 会把 xc 往一个物理上不存在的圆心拉。
  std::printf("用例 3：SELF_SPIN —— 真值 r=0（位置不动，只有朝向在转）\n");
  {
    const auto res = runSynthetic(RotationMode::SELF_SPIN, 0.0, 2.0, 300);
    std::printf("       r=%.4f (真值 0.00)   v_yaw=%.4f (真值 2.0)   位置误差=%.4f m\n",
                res.final_r, res.final_vyaw, res.position_err);
    std::printf("       TRACKING 帧数 %d/300\n", res.tracking_frames);
    check(res.reached_tracking, "进入 TRACKING");
    check(res.final_r < 0.05, "r 保持在 0 附近");
    check(res.position_err < 0.05, "位置误差 < 5cm");
  }

  // ── 用例 4：同一条数据，两种模式对比 ──────────────────────────
  // 用 r=0 的数据去喂 CAROUSEL 模式，看它是不是真的会出问题。
  // 这解释了为什么必须加这个开关，而不是"多此一举"。
  std::printf("用例 4：r=0 的数据喂给 CAROUSEL 模式会怎样\n");
  {
    const auto good = runSynthetic(RotationMode::SELF_SPIN, 0.0, 2.0, 300);
    const auto bad = runSynthetic(RotationMode::CAROUSEL, 0.0, 2.0, 300);
    std::printf("       SELF_SPIN: r=%.4f  位置误差=%.4f m\n", good.final_r,
                good.position_err);
    std::printf("       CAROUSEL : r=%.4f  位置误差=%.4f m\n", bad.final_r,
                bad.position_err);
    std::printf("       → 用错模式时 r 被撑在半径下界附近，位置估计被拉偏\n");
    check(bad.final_r > good.final_r, "CAROUSEL 的 r 明显更大（模型不匹配）");
  }

  // ── 用例 5：yaw 跨 ±π 时必须连续 ─────────────────────────────
  // ω=2rad/s 跑 300 帧 = 6 秒 = 转了约 1.9 圈，必然多次跨越 ±π。
  // 如果 orientationToYaw 没做展平，v_yaw 会在正负间乱翻。
  std::printf("用例 5：跨 ±π 时 v_yaw 不跳变（跑 6 秒 ≈ 1.9 圈）\n");
  {
    const auto res = runSynthetic(RotationMode::SELF_SPIN, 0.0, 2.0, 600);
    std::printf("       1.9 圈后 v_yaw=%.4f（真值 2.0）\n", res.final_vyaw);
    check(std::abs(res.final_vyaw - 2.0) < 0.5, "v_yaw 稳定，没有因跨 ±π 翻转");
  }

  // ── 用例 6：观测量中断 → TEMP_LOST → LOST ────────────────────
  std::printf("用例 6：观测中断后的状态机\n");
  {
    TrackerConfig cfg;
    cfg.mode = RotationMode::CAROUSEL;
    cfg.lost_time_thres = 0.3;
    Tracker trk(cfg);
    const double dt = 0.01;
    trk.init({makeMeasurement(5, 0, 0, 0, 0.3)});
    for (int i = 0; i < 100; ++i) {
      trk.update({makeMeasurement(5, 0, 0, 0.0, 0.3)}, dt);
    }
    check(trk.state() == Tracker::TRACKING, "持续有观测 → TRACKING");

    // 丢观测
    int frames_to_lost = 0;
    for (int i = 0; i < 200 && trk.state() != Tracker::LOST; ++i) {
      trk.update({}, dt);
      frames_to_lost++;
    }
    std::printf("       丢观测后 %d 帧（%.2f s）转为 LOST，阈值 %.2f s\n",
                frames_to_lost, frames_to_lost * dt, cfg.lost_time_thres);
    check(trk.state() == Tracker::LOST, "最终回到 LOST");
    check(std::abs(frames_to_lost * dt - cfg.lost_time_thres) < 0.05,
          "LOST 时机与 lost_time_thres 相符");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
