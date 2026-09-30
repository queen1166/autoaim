// 开火决策 —— rm_auto_aim 里也【没有】这一层。
//
// 规则 §2.2 的两个约束决定了这里的策略：
//   · 命中率 = 有效命中数 / 50，分母【固定】。只打 30 发，分母还是 50。
//   · 零发射本轮记 0 分。
//
// 所以：必须尽量打满 50 发，但每一发都要在瞄准收敛时才发。
// 乱开会白白浪费分子；不打则是 0 分。
//
// ⚠️ 协议里【没有"已发射"反馈】，也没有命中回执。
//    唯一的节流手段是 min_interval_s 这个软件计时器。
//    1 分钟打 50 发 ≈ 1.2 s 一发，min_interval_s 要从宽设定，
//    并靠"每轮结束后看靶板计数"来校正。

#pragma once

namespace autoaim::aim {

struct AimResult;  // 前置声明，避免头文件循环

struct FireConfig {
  // 收敛判据：瞄准角与云台当前角的差小于它就算"对准了"
  double yaw_tol_deg = 1.5;
  double pitch_tol_deg = 1.5;

  // 必须连续满足多久才开始请求开火。
  // 这个门槛是用来区分"刚甩到位"和"稳了片刻"的 —— 前者云台还在动，
  // 打出去大概率偏。
  double min_converge_s = 0.10;

  // 单次开火请求持续多久。下位机要求 fire_flag 持续为真才会走完供弹流程。
  double fire_hold_s = 0.15;

  // 两次请求之间的最小间隔。协议没有发射反馈，只能靠它节流。
  // 1 分钟 50 发 ≈ 1.2 s/发，这里先给保守值，现场按实际供弹速度调。
  double min_interval_s = 0.9;

  int max_shots = 50;

  // 云台反馈超过这个时长没更新，就不再请求开火。
  //
  // 为什么必须有：串口断了的话，latest_feedback_ 会一直返回【最后一帧】
  // 反馈，角度永远冻在旧值上。如果断开那一刻云台恰好接近指令角度，
  // aligned/converged 会一直为真，于是不停地请求开火 ——
  // 而实际上云台在哪、还在不在，你根本不知道。
  double max_data_age_s = 0.2;

  // 只有跟踪器处于可用状态时才开火。关掉可用于纯 PnP 的最小闭环调试。
  bool require_tracking = false;
};

// 开火决策器。每帧调一次 update()，它维护"对准了多久、上次打是什么时候"
// 这些跨帧状态，返回这一帧要不要请求开火。
//
// 注意这里是【无状态地算、有状态地判】—— 所有跨帧记忆都在私有成员里，
// 所以不能多个线程共用同一个实例（本工程里只有 processLoop 一个线程用它）。
class FireDecision {
 public:
  // 构造：只存配置。内部计时器全部归零（last_fire_end_ 初始化为 -1e9，
  // 意思是"很久以前"，保证第一次满足条件就能立刻打）。
  explicit FireDecision(const FireConfig & cfg = {}) : cfg_(cfg) {}

  // 返回 fire_flag（0 = 不请求，1 = 请求）
  //
  // 判定链：瞄准有效 → 跟踪器可用（若要求）→ 反馈没过期
  //         → 角度误差在容差内 → 连续稳定够 min_converge_s
  //         → 距上次开火够 min_interval_s。全过才返回 1。
  //
  // @param aim         瞄准结果。aim.valid == false 时直接返回 0
  // @param gimbal_yaw_rad   反馈帧里的当前云台角（已转弧度）
  // @param gimbal_pitch_rad 同上，pitch
  // @param tracking    跟踪器是否可用（TRACKING 或 TEMP_LOST）
  // @param now_s       单调时钟，秒。跨帧计时全部基于它
  // @param data_age_s  云台反馈的年龄（秒）。超过 max_data_age_s 就不开火。
  //                    这道闸门的存在理由见 FireConfig 里那段注释 ——
  //                    串口断了的话反馈会冻在旧值上，会一直误判"对准了"。
  int update(const AimResult & aim, double gimbal_yaw_rad, double gimbal_pitch_rad,
             bool tracking, double now_s, double data_age_s);

  // 清空所有跨帧状态：收敛计时归零、开火间隔重置、发弹计数清零。
  // 目标切换或重新开始时调用 —— 不重置的话上一轮残留的
  // last_fire_end_ 可能会无意中节流掉新一轮的第一发。
  void reset();

  // 累计已请求开火的次数。用来对照"这轮打了多少发"，
  // 和靶板计数比对可以反推实际命中率（协议没有命中回执）。
  int shotsRequested() const { return shots_requested_; }

  // 上一次 update() 时的 yaw 误差（度）。调试用：看云台是不是一直没跟上。
  double lastYawErrDeg() const { return last_yaw_err_deg_; }

  // 上一次 update() 时的 pitch 误差（度）。同上。
  double lastPitchErrDeg() const { return last_pitch_err_deg_; }

  // 配置的读写接口（现场调容差、间隔时用）。
  FireConfig & config() { return cfg_; }
  const FireConfig & config() const { return cfg_; }

 private:
  FireConfig cfg_;

  bool converged_ = false;        // 上一帧是否处于"对准且稳定"状态
  double converge_since_ = 0.0;   // 连续满足的起始时刻
  double fire_until_ = 0.0;       // 本次请求的截止时刻
  double last_fire_end_ = -1e9;   // 上次请求结束的时刻
  int shots_requested_ = 0;       // 累计请求次数

  // 上一帧的两个误差，只用于外部诊断（lastYawErrDeg 等）。
  double last_yaw_err_deg_ = 0.0;
  double last_pitch_err_deg_ = 0.0;
};

}  // namespace autoaim::aim
