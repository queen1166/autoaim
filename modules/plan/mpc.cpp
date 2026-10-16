#include "modules/plan/mpc.h"

#include <algorithm>
#include <cmath>

namespace autoaim::plan {

void Mpc::setup(const Config &cfg) {
  cfg_ = cfg;
  ready_ = false;

  const int N = std::max(2, cfg_.horizon);
  const double dt = cfg_.dt;

  // A = [[1, dt], [0, 1]]，B = [dt²/2, dt]ᵀ，且 A^k = [[1, k·dt], [0, 1]]。
  // 状态递归 X_{i+1} = A·X_i + B·u_i，展开后：
  //     X_i = A^i·x0 + Σ_{j<i} A^{i-1-j}·B·u_j
  // 所以把行索引记成 k = i−1 时，Abar 取 A^{k+1}、Bbar(k,j) 取 A^{k-j}·B。
  auto Apow = [&](int k) {
    Eigen::Matrix2d M;
    M << 1.0, k * dt, 0.0, 1.0;
    return M;
  };
  const Eigen::Vector2d B(dt * dt / 2.0, dt);

  Abar_theta_.resize(N, 2);
  Abar_omega_.resize(N, 2);
  Bbar_theta_.setZero(N, N);
  Bbar_omega_.setZero(N, N);

  for (int k = 0; k < N; ++k) {
    const Eigen::Matrix2d Ak1 = Apow(k + 1);
    Abar_theta_.row(k) = Ak1.row(0);
    Abar_omega_.row(k) = Ak1.row(1);

    for (int j = 0; j <= k; ++j) {
      const Eigen::Vector2d col = Apow(k - j) * B;
      Bbar_theta_(k, j) = col(0);
      Bbar_omega_(k, j) = col(1);
    }
  }

  // A = [√w_pos·Bbar_θ ; √w_vel·Bbar_ω]，2N × N。只对 A 做分解，不构造 AᵀA。
  // 角度速度权重按 dt² 归一化，理由见 Mpc::Config::w_vel 的注释。
  const double sw_pos = std::sqrt(std::max(0.0, cfg_.w_pos));
  const double sw_vel = std::sqrt(std::max(0.0, cfg_.w_vel)) * dt;

  // σ_max 的量级由 Bbar 决定，而控制量的正则权重必须相对于它来取，
  // 所以先构造前两块、量一次 σ_max，再补上正则块。
  A_.setZero(2 * N, N);
  A_.topRows(N) = sw_pos * Bbar_theta_;
  A_.bottomRows(N) = sw_vel * Bbar_omega_;

  Eigen::JacobiSVD<Eigen::MatrixXd> probe(A_,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
  const double sigma_max =
      probe.singularValues().size() > 0 ? probe.singularValues()(0) : 1.0;
  lipschitz_ = std::max(sigma_max * sigma_max, 1e-12);

  const double sw_acc = std::sqrt(std::max(0.0, cfg_.w_acc_rel) * lipschitz_);
  A_.conservativeResize(3 * N, N);
  A_.middleRows(2 * N, N) = sw_acc * Eigen::MatrixXd::Identity(N, N);

  n_steps_ = N;
  svd_ = Eigen::JacobiSVD<Eigen::MatrixXd>(A_,
                                           Eigen::ComputeThinU | Eigen::ComputeThinV);

  u_warm_.setZero(N);
  ready_ = true;
}

Mpc::Solution Mpc::solve(double angle0, double vel0, const Eigen::MatrixXd &ref,
                         int take_index) const {
  Solution out;
  if (!ready_ || ref.rows() < cfg_.horizon || ref.cols() < 2) return out;

  const int N = cfg_.horizon;
  if (N != n_steps_) return out;
  const Eigen::Vector2d x0(angle0, vel0);

  const Eigen::VectorXd ref_theta = ref.col(0).head(N);
  const Eigen::VectorXd ref_omega = ref.col(1).head(N);

  // b = −[√w_pos·(Abar_θ·x0 − ref_θ) ; √w_vel·(Abar_ω·x0 − ref_ω)]
  const double sw_pos = std::sqrt(std::max(0.0, cfg_.w_pos));
  const double sw_vel = std::sqrt(std::max(0.0, cfg_.w_vel)) * cfg_.dt;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * N);  // 正则块对应的 b 为 0
  b.head(N) = -sw_pos * (Abar_theta_ * x0 - ref_theta);
  b.segment(N, N) = -sw_vel * (Abar_omega_ * x0 - ref_omega);
  if (!b.allFinite()) return out;

  const double alpha = cfg_.max_acc;

  auto finishWith = [&](const Eigen::VectorXd & u, int iters) {
    const int k =
        std::clamp(take_index < 0 ? cfg_.half_horizon : take_index, 0, N - 1);
    out.angle = (Abar_theta_.row(k) * x0)(0) + Bbar_theta_.row(k).dot(u);
    out.vel = (Abar_omega_.row(k) * x0)(0) + Bbar_omega_.row(k).dot(u);
    out.acc = u(k);
    out.iterations = iters;
    out.ok = std::isfinite(out.angle) && std::isfinite(out.vel) &&
             std::isfinite(out.acc);
    return out.ok;
  };

  // ── 快路径：无约束最小二乘解可行 ⇒ 它就是全局最优 ──────────────────
  // 凸 QP 的基本结论。正常跟踪时云台远没到角加速度上限，这条路径给出精确解。
  const Eigen::VectorXd u_unc = svd_.solve(b);
  if (u_unc.allFinite() && u_unc.cwiseAbs().maxCoeff() <= alpha) {
    u_warm_ = u_unc;
    if (!finishWith(u_unc, 0)) return out;
    out.acc = std::clamp(out.acc, -alpha, alpha);
    return out;
  }

  // ── 慢路径：约束真的打满了，用 FISTA 迭代 ─────────────────────────
  // 梯度 grad J = Aᵀ(A·u − b)，Lipschitz 常数 = σ_max(A)²，步长固定 1/L。
  // 投影就是逐元素夹取；没有需要手调的参数，收敛有保证。
  const double inv_L = 1.0 / lipschitz_;

  if (u_warm_.size() != N) u_warm_.setZero(N);
  Eigen::VectorXd u = u_warm_;
  Eigen::VectorXd y = u;
  double t = 1.0;
  int iter = 0;

  for (int k = 0; k < std::max(1, cfg_.max_iters); ++k) {
    ++iter;
    const Eigen::VectorXd grad = A_.transpose() * (A_ * y - b);
    const Eigen::VectorXd u_new =
        (y - inv_L * grad).cwiseMax(-alpha).cwiseMin(alpha);

    const double step = (u_new - u).cwiseAbs().maxCoeff();

    // FISTA 的外推项用的是**相邻两次迭代之差** (u_{k+1} − u_k)。
    const Eigen::VectorXd u_prev = u;
    u = u_new;

    const double t_new = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * t * t));
    y = u + ((t - 1.0) / t_new) * (u - u_prev);
    t = t_new;

    if (step < cfg_.step_tol) break;
  }

  u_warm_ = u;
  if (!finishWith(u, iter)) return out;
  out.acc = std::clamp(out.acc, -alpha, alpha);

  return out;
}

}
