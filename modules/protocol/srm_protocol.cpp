#include "modules/protocol/srm_protocol.h"

#include <cstring>

namespace srm {

namespace {

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
    std::memcpy(&u, &v, sizeof(u));
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

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

}

std::array<uint8_t, kSendFrameLen> pack_target(const TargetCommand& cmd) {
    std::array<uint8_t, kSendFrameLen> f{};

    put_i16(f.data() + 0, kSendBodyLen);
    put_i16(f.data() + 2, kIdGimbal);
    put_f32(f.data() + 4, cmd.yaw_deg);
    put_f32(f.data() + 8, cmd.pitch_deg);
    put_i16(f.data() + 12, kIdShoot);
    put_i32(f.data() + 14, cmd.fire_flag);

    return f;
}

std::optional<GimbalFeedback> parse_feedback(const uint8_t* frame,
                                             uint64_t now_ns) {
    if (frame == nullptr) return std::nullopt;

    if (get_i16(frame + 0) != kRecvBodyLen) return std::nullopt;

    if (get_i16(frame + 2) != kIdGimbal) return std::nullopt;

    GimbalFeedback fb;
    fb.yaw_deg   = get_f32(frame + 4);
    fb.pitch_deg = get_f32(frame + 8);
    fb.roll_deg  = get_f32(frame + 12);
    fb.mode      = get_i32(frame + 16);
    fb.color     = get_i32(frame + 20);

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

    // 溢出兜底：正常帧最长 30 字节，涨到这个量级说明已经没救了。语义同样是"丢弃当前缓存"。
    if (buf_.size() > kMaxBufferLen) {
        buf_.clear();
        ++resyncs_;
        return;
    }

    while (true) {
        // ① 连长度字段都没收齐 → 留着等下次。
        if (buf_.size() < sizeof(int16_t)) break;

        // ② 非法长度 → 丢弃整个缓存，等下一帧。
        // 协议没有魔数和 CRC，无法在任意字节处可靠重同步（逐字节滑动可能撞上 payload
        // 里碰巧出现的 28），所以这里不滑动、直接清空。见 self_aim_protocol.md §1。
        const int16_t body_len = get_i16(buf_.data());
        if (body_len != kRecvBodyLen) {
            buf_.clear();
            ++resyncs_;
            break;
        }

        // ③ 半帧 → 留着等下次。按长度字段现算，不直接用 kRecvFrameLen 常量。
        const size_t frame_len =
            sizeof(int16_t) + static_cast<size_t>(body_len);
        if (buf_.size() < frame_len) break;

        // ④ parse_feedback 内部已校验 body_len 与两个 ID。
        //    未知 ID 同样按协议丢弃整个缓存——即使后面紧跟一帧合法帧也会一起丢掉，
        //    这是文档要求的"有意丢弃"，不要改回滑动。
        auto fb = parse_feedback(buf_.data(), now_ns);
        if (!fb) {
            buf_.clear();
            ++resyncs_;
            break;
        }

        buf_.erase(buf_.begin(),
                   buf_.begin() + static_cast<std::ptrdiff_t>(frame_len));
        if (on_frame) on_frame(*fb);  // 循环继续 → 支持背靠背帧
    }
}

}
