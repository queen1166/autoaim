#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

// SRM 校内赛自瞄通信协议（self_aim_protocol.md）
//
// 帧格式：2 字节小端 body_len + 若干条 ID 记录
//   每条记录 = int16 id + 定长数据体
//   协议无魔数、无 CRC、无时间戳、无序列号 —— 长度字段是唯一的帧边界
//
//   上→下：body_len=16，总长 18    ID 1 云台目标 + ID 2 开火请求
//   下→上：body_len=28，总长 30    ID 1 云台状态 + ID 2 发射状态
//
// 所有整数和浮点数均为小端。结构体按 1 字节对齐（本文件逐字段读写，
// 不依赖结构体布局，因此不受编译器 padding 影响）。
namespace srm {

// ── 协议常量 ────────────────────────────────────────
inline constexpr int16_t kSendBodyLen = 16;
inline constexpr int16_t kRecvBodyLen = 28;
inline constexpr size_t  kSendFrameLen = 18;   // 2 + 16
inline constexpr size_t  kRecvFrameLen = 30;   // 2 + 28

inline constexpr int16_t kIdGimbal = 1;
inline constexpr int16_t kIdShoot  = 2;

// 组帧缓存上限。正常情况只留不到一帧的碎片，到不了这里；
// 纯粹是防御异常字节流把内存撑爆。
inline constexpr size_t kMaxBufferLen = 4096;

// ── 上→下：ID 1 + ID 2 ──────────────────────────────
struct TargetCommand {
    float   yaw_deg   = 0.0f;   // 角度制！不是弧度
    float   pitch_deg = 0.0f;   // 角度制！不是弧度
    int32_t fire_flag = 0;      // 0 = 不请求开火；非零 = 视觉侧请求
    // fire_flag 只是视觉侧请求。实际发射还必须同时满足下位机火控状态、
    // 摩擦轮状态和人工许可（鼠标左键），非零值不会绕过这些条件。
};

// ── 下→上：ID 1 + ID 2 ──────────────────────────────
struct GimbalFeedback {
    float   yaw_deg      = 0.0f;
    float   pitch_deg    = 0.0f;
    float   roll_deg     = 0.0f;
    int32_t mode         = 0;      // 当前分支固定发 0，保留字段
    int32_t color        = 0;      // 当前分支写入机器人 ID，字段名沿用源码
    float   bullet_speed = 22.0f;  // 裁判系统提供；未注入时模块默认 22.0
    uint64_t timestamp_ns = 0;     // 收到该帧的时刻（单调时钟）
};

// 打包成 18 字节固定帧：2 字节 body_len + ID1 记录 + ID2 记录。
// 逐字段小端写出，不依赖结构体布局，所以 padding 怎么排都无所谓。
//
// 由 txLoop 每 10ms 调一次（100Hz）。注意它【必须】把 ID 1 和 ID 2 都发全 ——
// 省略哪条，下位机对应的旧字段就不会被清零（README 坑 #4）。
std::array<uint8_t, kSendFrameLen> pack_target(const TargetCommand& cmd);

// 解析一个完整的 30 字节帧。长度字段或 ID 不对返回 nullopt。
//
// frame30 指向一个完整帧（长度必须 >= kRecvFrameLen）
// now_ns  收到这一帧的时刻，会被写进返回值的 timestamp_ns 字段。
//         调用方传 srm::nowNs 之类的单调时钟 —— 开火决策靠它判断
//         "这条反馈是不是已经过期了"，所以不要传真实时间或 0。
std::optional<GimbalFeedback> parse_feedback(const uint8_t* frame30,
                                             uint64_t now_ns);

// ── 流式组帧器 ──────────────────────────────────────
//
// USB 包边界 != 协议帧边界：一次 read 可能拿到半帧、整帧或多帧。
// 协议没有魔数/CRC，遇到非法长度无法在任意字节处可靠重同步，
// 正确做法是丢掉整个缓存，等下一次完整帧。
// 流式组帧器。rxLoop 每 read 到一批字节就 feed() 一次，
// 它负责把碎片拼成完整帧，每拼出一帧就回调一次。
//
// 内部就是一个待拼字节的缓冲区 buf_，只在 rxLoop 一个线程里用，
// 所以【不需要加锁】。
class FrameParser {
public:
    // 每解析出一帧完整数据就回调一次。参数是解析好的反馈结构体。
    using Callback = std::function<void(const GimbalFeedback&)>;

    // 喂入任意长度字节；on_frame 可能被调用 0 次或多次。
    //
    // data     刚 read 到的一批字节，长度随意（可能不足一帧、刚好一帧、
    //          或者好几帧粘在一起）
    // len      data 的字节数
    // on_frame 每拼出一帧就调一次，可能一次 feed 里调 0 次或多次
    // now_ns   这批字节的接收时刻，原样透传给 parse_feedback()
    //
    // 遇到非法长度会丢掉整个缓冲区重来（并让 resyncCount() +1）——
    // 协议没有魔数/CRC，没法在任意字节处可靠重同步，只能这样。
    void feed(const uint8_t* data, size_t len,
              const Callback& on_frame, uint64_t now_ns);

    // 丢弃所有待拼碎片，回到初始状态。重连串口、或者上层决定放弃
    // 当前这半帧时调用。注意它【不】清 resyncCount() 计数。
    void   reset()          { buf_.clear(); }

    // 当前还攒着多少字节没拼成帧（0 表示刚好在帧边界上）。
    // 调试用：长时间居高不下说明帧长对不上，多半是协议版本不一致。
    size_t buffered() const { return buf_.size(); }

    // 丢字节重同步的次数。
    //
    // 这个计数必须在【组帧器内部】累加 —— 调用方靠 buffered() 的变化去猜
    // 是猜不准的：正常解析出一帧也会让 buffered() 变小。
    uint64_t resyncCount() const { return resyncs_; }

private:
    std::vector<uint8_t> buf_;
    uint64_t resyncs_ = 0;
};

} // namespace srm
