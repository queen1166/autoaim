#include "modules/protocol/srm_protocol.h"

#include <cstring>

namespace srm {

namespace {

// ── 小端写入（显式按字节写，不依赖主机字节序）────────────
inline void put_i16(uint8_t* p, int16_t v) {
    const uint16_t u = static_cast<uint16_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}

inline void put_i32(uint8_t* p, int32_t v) {
    const uint32_t u = static_cast<uint32_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

inline void put_f32(uint8_t* p, float v) {
    uint32_t u;
    std::memcpy(&u, &v, sizeof(u));   // 取位模式，不做数值转换
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

// ── 小端读取 ────────────────────────────────────────────
inline int16_t get_i16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

inline int32_t get_i32(const uint8_t* p) {
    const uint32_t u = static_cast<uint32_t>(p[0]) |
                       (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) |
                       (static_cast<uint32_t>(p[3]) << 24);
    return static_cast<int32_t>(u);
}

inline float get_f32(const uint8_t* p) {
    const uint32_t u = static_cast<uint32_t>(p[0]) |
                       (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) |
                       (static_cast<uint32_t>(p[3]) << 24);
    float v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

} // namespace

std::array<uint8_t, kSendFrameLen> pack_target(const TargetCommand& cmd) {
    std::array<uint8_t, kSendFrameLen> f{};

    // 偏移 0..1   body_len = 16
    put_i16(f.data() + 0, kSendBodyLen);
    // 偏移 2..3   ID 1
    put_i16(f.data() + 2, kIdGimbal);
    // 偏移 4..7   yaw   (float32, 角度制)
    put_f32(f.data() + 4, cmd.yaw_deg);
    // 偏移 8..11  pitch (float32, 角度制)
    put_f32(f.data() + 8, cmd.pitch_deg);
    // 偏移 12..13 ID 2
    put_i16(f.data() + 12, kIdShoot);
    // 偏移 14..17 fire_flag (int32)
    put_i32(f.data() + 14, cmd.fire_flag);

    return f;
}

std::optional<GimbalFeedback> parse_feedback(const uint8_t* frame,
                                             uint64_t now_ns) {
    if (frame == nullptr) return std::nullopt;

    // 长度字段
    if (get_i16(frame + 0) != kRecvBodyLen) return std::nullopt;

    // ID 1 数据体：yaw/pitch/roll/mode/color，共 20 字节，偏移 4..23
    if (get_i16(frame + 2) != kIdGimbal) return std::nullopt;

    GimbalFeedback fb;
    fb.yaw_deg   = get_f32(frame + 4);
    fb.pitch_deg = get_f32(frame + 8);
    fb.roll_deg  = get_f32(frame + 12);
    fb.mode      = get_i32(frame + 16);
    fb.color     = get_i32(frame + 20);

    // ID 2 数据体：bullet_speed，4 字节，偏移 26..29
    if (get_i16(frame + 24) != kIdShoot) return std::nullopt;
    fb.bullet_speed = get_f32(frame + 26);

    fb.timestamp_ns = now_ns;
    return fb;
}

void FrameParser::feed(const uint8_t* data, size_t len,
                       const Callback& on_frame, uint64_t now_ns) {
    if (data != nullptr && len > 0) {
        buf_.insert(buf_.end(), data, data + len);
    }

    // 防止异常字节流把缓存撑爆。正常情况永远到不了这里。
    if (buf_.size() > kMaxBufferLen) {
        buf_.clear();
        ++resyncs_;
        return;
    }

    while (buf_.size() >= kRecvFrameLen) {
        const int16_t body_len = get_i16(buf_.data());

        if (body_len != kRecvBodyLen) {
            // 长度不对 → 错位了。
            //
            // ⚠️ 这里【一次只丢一个字节】，不要 buf_.clear()。
            //    因为一次 read 完全可能是
            //        [上一帧的尾巴][一整帧有效数据]
            //    直接 clear 会把那整帧有效数据一起扔掉。
            //    每次至少推进一个字节，所以循环一定会结束。
            buf_.erase(buf_.begin());
            ++resyncs_;
            continue;
        }

        if (auto fb = parse_feedback(buf_.data(), now_ns)) {
            if (on_frame) on_frame(*fb);
        }

        buf_.erase(buf_.begin(),
                   buf_.begin() + static_cast<std::ptrdiff_t>(kRecvFrameLen));
    }
    // 循环退出时 buf_.size() < kRecvFrameLen，剩下的就是半帧碎片，
    // 等下一次 feed 补齐。
}

} // namespace srm
