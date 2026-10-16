// 姿态时间戳队列（教程场景 3）的离线测试。
//
// 盯住两件事：
//   1. Slerp 插值出来的必须是**合法的单位四元数**（线性插值会掉出球面）
//   2. 插值结果要真的落在两个样本之间，而不是简单地取最近的一包
//
// 后者是这一站存在的唯一理由：IMU/姿态反馈和相机帧物理上异步，
// 用"缓冲区里最新那包"去解释几毫秒前的图像，在云台追旋转靶时会带来
// 与转速成正比的系统性角度偏差。

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>

#include "modules/solver/coord_transform.h"
#include "modules/time/attitude_buffer.h"
#include "modules/time/delay_chain.h"

using namespace autoaim;
using namespace autoaim::timeutil;

namespace {

int g_failed = 0;
constexpr double kRad2Deg = 180.0 / M_PI;

void check(bool ok, const char * what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

void checkNear(double got, double want, double tol, const char * what) {
  const bool ok = std::abs(got - want) < tol;
  std::printf("  %s %-40s got %+10.5f  want %+10.5f\n", ok ? "PASS" : "FAIL", what,
              got, want);
  if (!ok) g_failed++;
}

Eigen::Quaterniond yawQuat(double deg) {
  const solver::Attitude a{deg * M_PI / 180.0, 0.0, 0.0};
  return solver::attitudeToQuat(a);
}

}  // namespace

int main() {
  std::printf("用例 1：中间时刻取到的是插值，不是最近样本\n");
  {
    AttitudeBuffer buf;
    // 200 Hz 上报：t=105ms 时 yaw=0°，t=110ms 时 yaw=10°
    buf.push(105'000'000, yawQuat(0.0));
    buf.push(110'000'000, yawQuat(10.0));

    Eigen::Quaterniond q;
    check(buf.query(107'000'000, q), "查询成功");
    const double yaw = solver::attitudeFromQuat(q).yaw_rad * kRad2Deg;
    std::printf("       t=107ms（105 与 110 之间取 40%%）→ yaw = %.3f°\n", yaw);
    checkNear(yaw, 4.0, 1e-9, "插值结果（取最近样本会是 10°）");
  }

  std::printf("用例 2：插值结果仍是单位四元数（Slerp 而非 lerp）\n");
  {
    AttitudeBuffer buf;
    buf.push(1'000'000, yawQuat(0.0));
    buf.push(2'000'000, yawQuat(150.0));  // 大角度，lerp 会明显掉出球面

    double worst = 0;
    for (uint64_t t = 1'000'000; t <= 2'000'000; t += 100'000) {
      Eigen::Quaterniond q;
      if (!buf.query(t, q)) {
        g_failed++;
        continue;
      }
      worst = std::max(worst, std::abs(q.norm() - 1.0));
    }
    std::printf("       全程 |‖q‖ − 1| 最大 = %.3e\n", worst);
    check(worst < 1e-12, "四元数始终归一");
  }

  std::printf("用例 3：区间外取端点，不外推\n");
  {
    AttitudeBuffer buf;
    buf.push(1'000'000, yawQuat(0.0));
    buf.push(2'000'000, yawQuat(10.0));

    Eigen::Quaterniond q;
    buf.query(0, q);
    checkNear(solver::attitudeFromQuat(q).yaw_rad * kRad2Deg, 0.0, 1e-9, "比最旧还早 → 最旧");
    buf.query(9'000'000, q);
    checkNear(solver::attitudeFromQuat(q).yaw_rad * kRad2Deg, 10.0, 1e-9, "比最新还新 → 最新");
  }

  std::printf("用例 4：空队列返回 false（调用方据此退化为单位姿态）\n");
  {
    AttitudeBuffer buf;
    Eigen::Quaterniond q;
    check(!buf.query(12345, q), "空队列查询返回 false");
    uint64_t t = 0;
    check(!buf.latest(t, q), "空队列取最新返回 false");
  }

  std::printf("用例 5：乱序/重复时间戳被丢弃，队列保持单调\n");
  {
    AttitudeBuffer buf;
    buf.push(200, yawQuat(5.0));
    buf.push(200, yawQuat(5.0));  // 重复时间戳，应被丢弃
    buf.push(100, yawQuat(99.0));  // 时间倒流，应被丢弃
    buf.push(300, yawQuat(15.0));

    Eigen::Quaterniond q;
    buf.query(0, q);
    std::printf("       队列 %zu 个样本；最早的 yaw = %.1f°\n", buf.size(),
                solver::attitudeFromQuat(q).yaw_rad * kRad2Deg);
    check(buf.size() == 2, "只保留了 200 和 300 两个样本");
    checkNear(solver::attitudeFromQuat(q).yaw_rad * kRad2Deg, 5.0, 1e-9,
              "倒流的 100 被丢弃");
  }

  std::printf("用例 6：容量上限会丢最旧的\n");
  {
    AttitudeBuffer buf(4);
    for (uint64_t i = 1; i <= 10; ++i) buf.push(i * 1000, yawQuat(static_cast<double>(i)));
    check(buf.size() == 4, "容量被限制在 4");
    Eigen::Quaterniond q;
    buf.query(0, q);
    checkNear(solver::attitudeFromQuat(q).yaw_rad * kRad2Deg, 7.0, 1e-9,
              "留下的最旧的是第 7 个");
  }

  std::printf("用例 7：按 dt² 归一化的延迟链总量\n");
  {
    DelayChain d;
    d.process_latency_s = 0.01;
    d.comm_latency_s = 0.02;
    d.fly_time_s = 0.25;
    checkNear(d.total(), 0.28, 1e-12, "三项相加");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
