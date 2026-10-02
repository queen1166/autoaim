#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace srm {

inline constexpr int16_t kSendBodyLen = 16;
inline constexpr int16_t kRecvBodyLen = 28;
inline constexpr size_t  kSendFrameLen = 18;
inline constexpr size_t  kRecvFrameLen = 30;

inline constexpr int16_t kIdGimbal = 1;
inline constexpr int16_t kIdShoot  = 2;

inline constexpr size_t kMaxBufferLen = 4096;

struct TargetCommand {
    float   yaw_deg   = 0.0f;
    float   pitch_deg = 0.0f;
    int32_t fire_flag = 0;
};

struct GimbalFeedback {
    float   yaw_deg      = 0.0f;
    float   pitch_deg    = 0.0f;
    float   roll_deg     = 0.0f;
    int32_t mode         = 0;
    int32_t color        = 0;
    float   bullet_speed = 22.0f;
    uint64_t timestamp_ns = 0;
};

std::array<uint8_t, kSendFrameLen> pack_target(const TargetCommand& cmd);

std::optional<GimbalFeedback> parse_feedback(const uint8_t* frame30,
                                             uint64_t now_ns);

class FrameParser {
public:
    using Callback = std::function<void(const GimbalFeedback&)>;

    void feed(const uint8_t* data, size_t len,
              const Callback& on_frame, uint64_t now_ns);

    void   reset()          { buf_.clear(); }

    size_t buffered() const { return buf_.size(); }

    uint64_t resyncCount() const { return resyncs_; }

private:
    std::vector<uint8_t> buf_;
    uint64_t resyncs_ = 0;
};

}
