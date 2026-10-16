#pragma once

#include <Eigen/Dense>

namespace autoaim::plan {

// 云台单轴线性 MPC（教程 §8.4.2）。
//
// 模型是双积分：
//     x = [θ, ω]ᵀ   （角度、角速度）
//     θ_{k+1} = θ_k + ω_k·DT
//     ω_{k+1} = ω_k + α_k·DT          α = 角加速度，是唯一的控制量
//     |α_k| ≤ max_acc                 ← 云台的物理约束（yaw ≈50, pitch ≈100 rad/s²）
//
// 代价：
//     min Σ_k (θ_k − θ_ref_k)² + w_vel·(ω_k − ω_ref_k)²
//
// 【为什么需要它】直接把逐帧解出的瞄准角当阶跃指令下发，装甲板切换/选板会让
// 目标角变成带尖峰的三角波；云台有惯量，跟不上这种尖峰，表现就是超调、震荡。
// MPC 的作用不是"瞄得更准"，而是"生成一条云台物理上追得上的平滑轨迹"。
//
// 【为什么向前看满一个窗口】只看当前一步，每次 EKF 更新后重新规划，轨迹会在
// 装甲板切换的瞬间跳变；向前铺满 1 s 的参考，MPC 就能看到"这块板从出现到
// 消失"的完整过程，从而给出平滑的跨板切换轨迹。
//
// 【怎么解】约束只有控制量的箱式约束、代价又是二次的 —— 这是个箱约束 QP，
// 把它化归成"二次型 + 箱投影"的复合优化，用**加速投影梯度（FISTA）**求解。
//
// 为什么不用 ADMM/TinyMPC：ADMM 对惩罚系数 ρ 极其敏感 —— H 的元素量级由 dt²
// 决定（dt=0.01 时只有 1e-2），ρ 取固定值时，小阶跃工况要求 ρ 小、大阶跃工况
// 要求 ρ 大，同一组参数下要么"一动不动"要么"炸到 1e60"（这两种现象在本项目
// 开发过程中都实际出现过）。FISTA 只需要一个精确的 Lipschitz 常数
// L = λ_max(H)，而 H 是常数矩阵、特征值在 setup 时算一次即可 ——
// 没有需要手调的参数，也不会发散。
class Mpc {
 public:
  struct Config {
    int horizon = 100;      // HORIZON：总步数
    int half_horizon = 50;  // HALF_HORIZON：默认取这一步的优化值
    double dt = 0.01;       // 步长，100 步 × 0.01 s = 1.0 s 窗口

    double max_acc = 50.0;  // rad/s²，云台角加速度上限

    double w_pos = 1.0;  // 角度跟踪权重

    // 角速度跟踪权重。**按 dt² 归一化后使用**（实际权重 = w_vel·dt²）。
    //
    // 为什么必须归一化：控制量是角加速度，它对角度的灵敏度是 dt²/2（1e-4 量级），
    // 对角度速度的灵敏度是 dt（1e-2 量级）—— 差了 100 倍，平方后就是 1e4 倍。
    // 如果直接罚 ω²，同一个权重下"罚速度"会压倒"跟位置"，最优解会变成
    // "慢慢挪"：参数取 0.05 时，理论上要 0.45 s 才能跟上一个 5° 的阶跃，
    // 表现就是云台指令系统性滞后。乘上 dt² 之后两者才是同一个量纲，
    // 罚的是"每一步由角速度偏差引起的角度变化"，调参才有意义。
    double w_vel = 1.0;

    int max_iters = 400;     // FISTA 迭代上限（只在无约束解不可行时才用）
    double step_tol = 1e-7;  // 相邻两步的最大变化小于它就提前收敛退出

    // 控制量（角加速度）正则权重，相对 λ_max(H) 取值。
    //
    // 为什么必须有：A 的奇异值跨度很大，而"高频正负交替的角加速度"这类方向
    // 几乎不影响轨迹（σ 很小）。无约束最小二乘解在这些方向上会被放大成
    // 幅度上千的抖动 —— 它不改变轨迹，却会让 |α| 轻松越过上限，
    // 于是每次规划都掉进 FISTA 慢路径（实测规划耗时 2.4 ms，占满了 250 Hz 的周期）。
    // 加一点控制代价既符合物理（云台本来就不该抖），又能把这类方向压掉，
    // 让绝大多数工况走"无约束解即可行"的精确快路径。
    double w_acc_rel = 1e-4;
  };

  struct Solution {
    double angle = 0.0;  // 第 take_index 步的优化角度（rad，连续分支）
    double vel = 0.0;    // 同一步的角速度（rad/s）
    double acc = 0.0;    // 同一步的角加速度（rad/s²）
    int iterations = 0;
    bool ok = false;
  };

  void setup(const Config & cfg);

  const Config & config() const { return cfg_; }

  bool ready() const { return ready_; }

  // ref 是 horizon × 2 的参考轨迹，每行 [θ_ref, ω_ref]，索引 k 对应
  // "当前时刻 + k·DT"。angle0 / vel0 是云台当前的（角度, 角速度），即 k=0 的状态。
  //
  // take_index 是要取哪一步的优化值作为下发的指令；-1 表示用 config 里的
  // half_horizon。Planner 传的是"延迟链总时长 / DT"——那一步才是指令真正
  // 生效的时刻（云台会滞后、弹丸还要飞一段）。
  Solution solve(double angle0, double vel0, const Eigen::MatrixXd & ref,
                 int take_index = -1) const;

 private:
  Config cfg_;

  bool ready_ = false;

  // 展开成 X_θ = Abar_theta·x0 + Bbar_theta·u，X_ω 同理。
  Eigen::MatrixXd Abar_theta_;  // horizon × 2
  Eigen::MatrixXd Abar_omega_;  // horizon × 2
  Eigen::MatrixXd Bbar_theta_;  // horizon × horizon
  Eigen::MatrixXd Bbar_omega_;  // horizon × horizon

  // 代价写成最小二乘形式 J = ‖A·u − b‖²，其中
  //     A = [√w_pos·Bbar_θ ; √w_vel·Bbar_ω]        （2N × N，与参考无关）
  //     b = −[√w_pos·(Abar_θ·x0 − ref_θ) ; √w_vel·(Abar_ω·x0 − ref_ω)]
  // 二者等价于 ½uᵀHu + gᵀu（H = AᵀA、g = −Aᵀb），但**绝不能真的去构造 H**：
  // A 的最小奇异值只有 1e-3 量级，AᵀA 会把条件数平方到 1e12 上下，
  // 这时解出来的 u 在近奇异方向全是数值垃圾 —— 现象就是 MPC"算得出解但跟不上参考"。
  // 所以统一对 A 做分解，而不是对 H 做。
  Eigen::MatrixXd A_;               // 2N × N
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_;
  double lipschitz_ = 1.0;          // = σ_max(A)²，即 λ_max(H)
  int n_steps_ = 0;

  // 上一次的解，作为热启动 —— 相邻两次规划周期参考变化很小，能省掉大量迭代。
  // 每个轴各持有一个 Mpc，且只有规划线程调用，不存在并发问题。
  mutable Eigen::VectorXd u_warm_;
};

}
