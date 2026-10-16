// SRM 协议帧解析的离线测试。
//
// 盯住两件事：
//   1. 线上字节序（小端）和帧布局，与 tools/protocol_check.py 那份独立 Python
//      实现对同一组输入产出同样的字节
//   2. 错误恢复策略贴合 self_aim_protocol.md §1：非法长度 / 未知 ID 一律
//      **丢弃整个缓存**并等下一帧，不做逐字节滑动
//
// 第 2 条是这一站存在的理由。协议没有魔数和 CRC，逐字节滑动可能撞上 payload 里
// 碰巧出现的 body_len（28）造成假同步，所以文档明确要求"丢弃当前缓存"。
// 下面用例 7 / 8 把这条行为钉死，避免后人又改回滑动。
//
// 只依赖协议模块本身，不碰 OpenCV / Eigen —— 缺 OpenCV 的环境也能构建。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "modules/protocol/srm_protocol.h"

namespace {

int g_failed = 0;

void check(bool ok, const char* what) {
  std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) g_failed++;
}

void checkNear(float got, float want, float tol, const char* what) {
  const bool ok = std::fabs(got - want) < tol;
  std::printf("  %s %-40s got %+10.5f  want %+10.5f\n", ok ? "PASS" : "FAIL",
              what, got, want);
  if (!ok) g_failed++;
}

void checkEq(long long got, long long want, const char* what) {
  const bool ok = (got == want);
  std::printf("  %s %-40s got %10lld  want %10lld\n", ok ? "PASS" : "FAIL",
              what, got, want);
  if (!ok) g_failed++;
}

// ── 小端打包（与 srm_protocol.cpp 的 put_* 同构，独立写一份以免自证） ──
void putI16(uint8_t* p, int16_t v) {
  const uint16_t u = static_cast<uint16_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFF);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}

void putI32(uint8_t* p, int32_t v) {
  const uint32_t u = static_cast<uint32_t>(v);
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((u >> (8 * i)) & 0xFF);
}

void putF32(uint8_t* p, float v) {
  uint32_t u;
  std::memcpy(&u, &v, sizeof(u));
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((u >> (8 * i)) & 0xFF);
}

// 造一帧下位机 → 上位机反馈（30 字节）。默认全部合法，可单独把某个 ID 改坏。
std::vector<uint8_t> recvFrame(float yaw, float pitch, float roll, int32_t mode,
                               int32_t color, float speed,
                               int16_t id1 = srm::kIdGimbal,
                               int16_t id2 = srm::kIdShoot) {
  std::vector<uint8_t> f(srm::kRecvFrameLen, 0);
  putI16(f.data() + 0, srm::kRecvBodyLen);
  putI16(f.data() + 2, id1);
  putF32(f.data() + 4, yaw);
  putF32(f.data() + 8, pitch);
  putF32(f.data() + 12, roll);
  putI32(f.data() + 16, mode);
  putI32(f.data() + 20, color);
  putI16(f.data() + 24, id2);
  putF32(f.data() + 26, speed);
  return f;
}

// 一次 feed 的采集器。
struct Sink {
  int count = 0;
  srm::GimbalFeedback last;
};

void feedBytes(srm::FrameParser& p, const std::vector<uint8_t>& b, Sink& s) {
  p.feed(b.data(), b.size(),
         [&s](const srm::GimbalFeedback& fb) {
           ++s.count;
           s.last = fb;
         },
         1000);
}

void feedRange(srm::FrameParser& p, const std::vector<uint8_t>& b, Sink& s,
               size_t from, size_t to) {
  p.feed(b.data() + from, to - from,
         [&s](const srm::GimbalFeedback& fb) {
           ++s.count;
           s.last = fb;
         },
         1000);
}

}  // namespace

int main() {
  // ── 1. 发送帧字节序锚点 ─────────────────────────────────────
  // 与 tools/protocol_check.py:57-66 的自检向量逐字节对上。
  std::printf("用例 1：发送帧字节序锚点（对齐 protocol_check.py 自检向量）\n");
  {
    const srm::TargetCommand cmd{1.0f, 0.0f, 0};
    const auto f = srm::pack_target(cmd);
    const uint8_t want[] = {0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00,
                            0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    checkEq(static_cast<long long>(f.size()), srm::kSendFrameLen, "帧长 18");
    check(std::memcmp(f.data(), want, sizeof(want)) == 0,
          "yaw=1.0 pitch=0.0 fire=0 逐字节匹配");
  }

  // ── 2. 半帧分两次喂入 ───────────────────────────────────────
  std::printf("\n用例 2：半帧分两次喂入，第二次才产出\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto f = recvFrame(1.0f, 2.0f, 3.0f, 0, 3, 22.0f);
    feedRange(p, f, s, 0, 15);
    checkEq(s.count, 0, "前 15 字节 → 0 回调");
    checkEq(static_cast<long long>(p.buffered()), 15, "缓冲保留 15 字节");
    feedRange(p, f, s, 15, 30);
    checkEq(s.count, 1, "补齐 → 1 回调");
    checkEq(static_cast<long long>(p.buffered()), 0, "缓冲清空");
    checkNear(s.last.yaw_deg, 1.0f, 1e-5f, "yaw 解出");
  }

  // ── 3. 只收到长度字段（2 字节边界） ─────────────────────────
  std::printf("\n用例 3：只喂到 body_len 两字节 → 应当继续等\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto f = recvFrame(5.0f, 0.0f, 0.0f, 0, 3, 22.0f);
    feedRange(p, f, s, 0, 2);
    checkEq(s.count, 0, "2 字节 → 0 回调");
    checkEq(static_cast<long long>(p.buffered()), 2, "缓冲保留 2 字节");
    feedRange(p, f, s, 2, 30);
    checkEq(s.count, 1, "补齐 → 1 回调");
  }

  // ── 4. 帧尾差一字节 ─────────────────────────────────────────
  std::printf("\n用例 4：差最后一字节 → 不产出\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto f = recvFrame(5.0f, 0.0f, 0.0f, 0, 3, 22.0f);
    feedRange(p, f, s, 0, 29);
    checkEq(s.count, 0, "29 字节 → 0 回调");
    checkEq(static_cast<long long>(p.buffered()), 29, "缓冲保留 29 字节");
    feedRange(p, f, s, 29, 30);
    checkEq(s.count, 1, "补齐 → 1 回调");
  }

  // ── 5. 背靠背两帧 ───────────────────────────────────────────
  std::printf("\n用例 5：一次喂入背靠背两帧 → 2 回调\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto a = recvFrame(1.0f, 0, 0, 0, 3, 22.0f);
    const auto b = recvFrame(7.0f, 0, 0, 0, 3, 22.0f);
    std::vector<uint8_t> two(a);
    two.insert(two.end(), b.begin(), b.end());
    feedBytes(p, two, s);
    checkEq(s.count, 2, "2 回调");
    checkNear(s.last.yaw_deg, 7.0f, 1e-5f, "第二帧的 yaw 生效");
    checkEq(static_cast<long long>(p.buffered()), 0, "缓冲清空");
  }

  // ── 6. 尾随半帧 ─────────────────────────────────────────────
  std::printf("\n用例 6：整帧 + 尾随半帧 → 半帧留着等下次\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto a = recvFrame(1.0f, 0, 0, 0, 3, 22.0f);
    const auto b = recvFrame(9.0f, 0, 0, 0, 3, 22.0f);
    std::vector<uint8_t> blob(a);
    blob.insert(blob.end(), b.begin(), b.begin() + 10);
    feedBytes(p, blob, s);
    checkEq(s.count, 1, "1 回调");
    checkEq(static_cast<long long>(p.buffered()), 10, "缓冲保留 10 字节");
    feedRange(p, b, s, 10, 30);
    checkEq(s.count, 2, "补齐 → 第 2 回调");
    checkNear(s.last.yaw_deg, 9.0f, 1e-5f, "第二帧的 yaw 生效");
  }

  // ── 7. 非法长度 → 丢弃整个缓存 ──────────────────────────────
  std::printf("\n用例 7：非法长度 → 丢弃整个缓存（含其后紧跟的合法帧）\n");
  {
    srm::FrameParser p;
    Sink s;
    std::vector<uint8_t> bad{0xFF, 0xFF};
    const auto f = recvFrame(1.0f, 0, 0, 0, 3, 22.0f);
    bad.insert(bad.end(), f.begin(), f.end());
    feedBytes(p, bad, s);
    // body_len 读出来是 -1，不等于 28。协议没有魔数/CRC，无法判断后面那帧
    // 是否真的是帧边界 —— 按 §1 必须整包丢弃。这是**有意为之的丢失**。
    checkEq(s.count, 0, "整包丢弃，合法帧也一并不产出");
    checkEq(static_cast<long long>(p.buffered()), 0, "缓冲清空");
    checkEq(static_cast<long long>(p.resyncCount()), 1, "重同步计数 +1");
  }
  std::printf("  变体：合法帧在坏前缀之前\n");
  {
    srm::FrameParser p;
    Sink s;
    std::vector<uint8_t> good = recvFrame(1.0f, 0, 0, 0, 3, 22.0f);
    good.insert(good.end(), {0xFF, 0xFF});
    feedBytes(p, good, s);
    // 丢失只波及损坏点之后的字节，不追溯前面的合法帧。
    checkEq(s.count, 1, "合法帧仍正常产出");
    checkEq(static_cast<long long>(p.buffered()), 0, "残余坏字节被清掉");
    checkEq(static_cast<long long>(p.resyncCount()), 1, "重同步计数 +1");
  }

  // ── 8. 未知 ID → 丢弃整个缓存 ───────────────────────────────
  std::printf("\n用例 8：未知 ID → 丢弃整个缓存\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto f = recvFrame(1.0f, 0, 0, 0, 3, 22.0f, /*id1=*/7);
    feedBytes(p, f, s);
    checkEq(s.count, 0, "ID1 未知 → 0 回调");
    checkEq(static_cast<long long>(p.buffered()), 0, "缓冲清空");
    checkEq(static_cast<long long>(p.resyncCount()), 1, "重同步计数 +1");
  }
  {
    srm::FrameParser p;
    Sink s;
    const auto f = recvFrame(1.0f, 0, 0, 0, 3, 22.0f, srm::kIdGimbal,
                             /*id2=*/7);
    feedBytes(p, f, s);
    checkEq(s.count, 0, "ID2 未知 → 0 回调");
    checkEq(static_cast<long long>(p.resyncCount()), 1, "重同步计数 +1");
  }

  // ── 9. resyncCount 只在丢弃时增长 ───────────────────────────
  std::printf("\n用例 9：resyncCount 只在丢弃时 +1\n");
  {
    srm::FrameParser p;
    Sink s;
    const auto f = recvFrame(1.0f, 0, 0, 0, 3, 22.0f);
    feedRange(p, f, s, 0, 15);  // 半帧等待
    checkEq(static_cast<long long>(p.resyncCount()), 0, "半帧等待不算丢弃");
    feedRange(p, f, s, 15, 30);  // 成功
    checkEq(static_cast<long long>(p.resyncCount()), 0, "成功解析不算丢弃");
    std::vector<uint8_t> bad{0x00, 0x10};  // body_len = 4096 ≠ 28
    feedBytes(p, bad, s);
    checkEq(static_cast<long long>(p.resyncCount()), 1, "丢弃才 +1");
  }

  // ── 10. 与 protocol_check.py 对表 ───────────────────────────
  std::printf("\n用例 10：与 protocol_check.py 的解析向量对表\n");
  {
    srm::FrameParser p;
    Sink s;
    feedBytes(p, recvFrame(-45.5f, 1.25f, 0.0f, 0, 3, 22.0f), s);
    checkEq(s.count, 1, "1 回调");
    checkNear(s.last.yaw_deg, -45.5f, 1e-5f, "yaw");
    checkNear(s.last.pitch_deg, 1.25f, 1e-5f, "pitch");
    checkNear(s.last.roll_deg, 0.0f, 1e-5f, "roll");
    checkEq(s.last.mode, 0, "mode");
    checkEq(s.last.color, 3, "color");
    checkNear(s.last.bullet_speed, 22.0f, 1e-5f, "bullet_speed");
  }

  // ── 11. 溢出兜底 ────────────────────────────────────────────
  // 单次 feed 灌进 > kMaxBufferLen(4096) 字节时，守卫在解析前先清缓存。
  std::printf("\n用例 11：单次灌入超长垃圾 → 丢弃，之后能恢复\n");
  {
    srm::FrameParser p;
    Sink s;
    std::vector<uint8_t> junk(5000, 0xAB);
    feedBytes(p, junk, s);
    checkEq(s.count, 0, "垃圾 → 0 回调");
    checkEq(static_cast<long long>(p.buffered()), 0, "缓冲清空");
    check(p.resyncCount() >= 1, "发生了丢弃");

    feedBytes(p, recvFrame(3.0f, 0, 0, 0, 3, 22.0f), s);  // 恢复
    checkEq(s.count, 1, "丢弃后仍能正常解析下一帧");
    checkNear(s.last.yaw_deg, 3.0f, 1e-5f, "yaw 正确");
  }

  std::printf("\n%s\n", g_failed == 0 ? "全部通过" : "有失败项");
  return g_failed == 0 ? 0 : 1;
}
