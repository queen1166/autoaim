#pragma once

namespace autoaim::timeutil {

// 延迟链 —— Planner 做前向预测时要"往前推"的总时间窗口。
//
//   total_delay = process_latency + comm_latency + fly_time
//
// 三项各是什么（对应教程场景 2/3/6/7）：
//
//   process_latency : 从图像到达上位机，到 Planner 此刻做预测之间累积的耗时
//                     = now() - frame.timestamp_ns
//                     含前处理 + 推理 + 角点精修 + PnP + EKF 更新
//                     → 物理含义："我手上的目标状态其实是这么久以前的状态，
//                        目标在这段时间里已经动过了"
//
//   comm_latency    : 兜底超参，概括剩余所有杂项延迟
//                     = 串口从发出到字节落地的物理时间
//                     + 下位机解包 + 云台对位置指令的跟随滞后
//                     → 协议固定 18 字节、无时间戳，无法在协议里测；
//                        现场用"已知角速度的靶 → 看弹着点偏前还是偏后"反标
//
//   fly_time        : 弹丸从枪口飞到目标的耗时 = 距离 / 弹速
//                     → 弹速来自下位机反馈 ID 2 的 bullet_speed（裁判系统）
//
// 注意：协议里没有独立的 send_latency 字段，按教程约定把它并入 comm_latency。
struct DelayChain {
  double process_latency_s = 0.0;
  double comm_latency_s = 0.0;
  double fly_time_s = 0.0;

  double total() const {
    return process_latency_s + comm_latency_s + fly_time_s;
  }
};

}
