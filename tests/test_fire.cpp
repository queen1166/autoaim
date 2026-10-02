
#include <cmath>
#include <cstdio>

#include "modules/aim/aim_solver.h"
#include "modules/aim/fire_decision.h"

using namespace autoaim::aim;

namespace {

int g_failed = 0;
constexpr double kDeg2Rad = M_PI / 180.0;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

AimResult aimAt(double yaw_deg, double pitch_deg) {
  AimResult r;
  r.yaw_rad = yaw_deg * kDeg2Rad;
  r.pitch_rad = pitch_deg * kDeg2Rad;
  r.valid = true;
  return r;
}

}

int main() {
  std::printf("用例 1：角误差超容差 → 不开火\n");
  {
    FireDecision fd;
    int fired = 0;
    for (int i = 0; i < 100; ++i) {
      fired += fd.update(aimAt(10.0, 0.0), 0.0, 0.0, true, i * 0.01, 0.0);
    }
    check(fired == 0, "100 帧（1 秒）内没有请求");
    std::printf("       最后误差 yaw=%.2f° pitch=%.2f°\n", fd.lastYawErrDeg(),
                fd.lastPitchErrDeg());
  }

  std::printf("用例 2：刚对准（< min_converge_s）→ 不开火\n");
  {
    FireConfig cfg;
    cfg.min_converge_s = 0.10;
    FireDecision fd(cfg);
    int fired = 0;
    for (int i = 0; i < 5; ++i) fired += fd.update(aimAt(0.5, 0.5), 0.0, 0.0, true, i * 0.01, 0.0);
    check(fired == 0, "50ms 内没有请求");
  }

  std::printf("用例 3：对准并稳定 100ms → 开火，且按 fire_hold_s 持续\n");
  {
    FireConfig cfg;
    cfg.fire_hold_s = 0.15;
    FireDecision fd(cfg);
    int fired = 0;
    int first_fire_frame = -1;
    for (int i = 0; i < 40; ++i) {
      const int f = fd.update(aimAt(0.5, 0.5), 0.0, 0.0, true, i * 0.01, 0.0);
      if (f && first_fire_frame < 0) first_fire_frame = i;
      fired += f;
    }
    std::printf("       首次请求在第 %d 帧（%.0f ms），共高电平 %d 帧（%.0f ms）\n",
                first_fire_frame, first_fire_frame * 10.0, fired, fired * 10.0);
    check(first_fire_frame >= 10, "等到收敛时间之后才开火");
    check(fired >= 14 && fired <= 17, "fire_flag 高电平持续约 150ms");
  }

  std::printf("用例 4：min_interval_s = 0.9 的节流\n");
  {
    FireConfig cfg;
    cfg.min_interval_s = 0.9;
    cfg.fire_hold_s = 0.15;
    FireDecision fd(cfg);
    int shots = 0;
    int last_high = 0;
    for (int i = 0; i < 300; ++i) {
      const int f = fd.update(aimAt(0.0, 0.0), 0.0, 0.0, true, i * 0.01, 0.0);
      if (f && !last_high) shots++;
      last_high = f;
    }
    std::printf("       3 秒内请求了 %d 次（间隔 0.9s+0.15s 保持 → 约 3 次）\n", shots);
    check(shots >= 2 && shots <= 4, "节流生效");
  }

  std::printf("用例 5：aim=179°, gimbal=-179° → 真实差 2°，应当开火\n");
  {
    FireConfig cfg;
    cfg.yaw_tol_deg = 3.0;
    FireDecision fd(cfg);
    int fired = 0;
    for (int i = 0; i < 40; ++i) {
      fired += fd.update(aimAt(179.0, 0.0), -179.0 * kDeg2Rad, 0.0, true, i * 0.01, 0.0);
    }
    std::printf("       算出的 yaw 误差 = %.2f°（真值差 2°）\n", fd.lastYawErrDeg());
    check(std::abs(fd.lastYawErrDeg()) < 3.0, "误差约 2°，不是 358°");
    check(std::abs(fd.lastYawErrDeg()) > 1.5, "确实是 2° 量级，没被抹平");
    check(fired > 0, "确实开火了");
  }

  std::printf("用例 6：max_shots 上限\n");
  {
    FireConfig cfg;
    cfg.min_interval_s = 0.05;
    cfg.fire_hold_s = 0.02;
    cfg.max_shots = 5;
    FireDecision fd(cfg);
    for (int i = 0; i < 2000; ++i) {
      fd.update(aimAt(0.0, 0.0), 0.0, 0.0, true, i * 0.01, 0.0);
    }
    std::printf("       20 秒内请求次数 = %d（上限 %d）\n", fd.shotsRequested(),
                cfg.max_shots);
    check(fd.shotsRequested() == cfg.max_shots, "到上限就停");
  }

  std::printf("用例 7：aim.valid = false → 不开火\n");
  {
    FireDecision fd;
    AimResult bad;
    int fired = 0;
    for (int i = 0; i < 100; ++i) fired += fd.update(bad, 0.0, 0.0, true, i * 0.01, 0.0);
    check(fired == 0, "没有请求");
  }

  std::printf("用例 8：require_tracking = true 时跟踪丢失不开火\n");
  {
    FireConfig cfg;
    cfg.require_tracking = true;
    FireDecision fd(cfg);
    int fired = 0;
    for (int i = 0; i < 100; ++i)
      fired += fd.update(aimAt(0.0, 0.0), 0.0, 0.0, false, i * 0.01, 0.0);
    check(fired == 0, "跟踪不可用时不开火");

    fired = 0;
    for (int i = 0; i < 100; ++i)
      fired += fd.update(aimAt(0.0, 0.0), 0.0, 0.0, true, i * 0.01, 0.0);
    check(fired > 0, "跟踪可用后开火");
  }

  std::printf("用例 9：反馈过期 → 不开火（安全）\n");
  {
    FireConfig cfg;
    cfg.max_data_age_s = 0.2;

    FireDecision fresh(cfg);
    int fired = 0;
    for (int i = 0; i < 100; ++i)
      fired += fresh.update(aimAt(0.0, 0.0), 0.0, 0.0, true, i * 0.01, 0.05);
    check(fired > 0, "数据新鲜时正常开火");

    FireDecision stale(cfg);
    fired = 0;
    for (int i = 0; i < 300; ++i)
      fired += stale.update(aimAt(0.0, 0.0), 0.0, 0.0, true, i * 0.01, 1.0);
    std::printf("       反馈年龄 1.0s（阈值 %.1fs）→ 开火 %d 次\n",
                cfg.max_data_age_s, fired);
    check(fired == 0, "反馈过期时一发都不发");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
