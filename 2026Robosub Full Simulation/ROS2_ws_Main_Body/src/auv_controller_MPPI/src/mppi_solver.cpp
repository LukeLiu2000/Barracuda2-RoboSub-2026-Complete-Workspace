// -*- coding: utf-8 -*-
#include "auv_controller_mppi/mppi_solver.hpp"

#include <chrono>
#include <cmath>

namespace auv_controller_mppi
{

MppiSolver::MppiSolver(const DynamicsModel & model, const CostFunction & cost,
                       const ThrusterModel & thruster, const MppiParams & params)
: model_(model), cost_(cost), thruster_(thruster), rng_(params.seed)
{
  setParams(params);
}

void MppiSolver::setParams(const MppiParams & params)
{
  p_ = params;
  rng_.seed(p_.seed);
  nominal_ = Eigen::MatrixXd::Constant(p_.H, kNumThrusters, p_.shift_fill);
  noise_.resize(static_cast<long>(p_.K) * p_.H * kNumThrusters);
  traj_cost_.resize(p_.K);
  weights_.resize(p_.K);
  result_.U = nominal_;
  result_.states = Eigen::MatrixXd::Zero(p_.H + 1, 13);
}

void MppiSolver::reset()
{
  nominal_ = Eigen::MatrixXd::Constant(p_.H, kNumThrusters, p_.shift_fill);
  rng_.seed(p_.seed);
}

void MppiSolver::sampleNoise()
{
  std::normal_distribution<double> gauss(0.0, p_.sigma);
  for (long i = 0; i < noise_.size(); ++i) { noise_(i) = gauss(rng_); }
}

const MppiResult & MppiSolver::solve(const State & x0, const Goal & goal)
{
  const auto t_start = std::chrono::steady_clock::now();

  // 1. 预生成 K*H*8 高斯扰动(rollout 内核变成纯计算,方便未来搬 GPU)
  sampleNoise();

  const int H = p_.H;
  const int K = p_.K;
  const int N8 = kNumThrusters;
  const double sub_dt = p_.dt / std::max(1, p_.substeps);

  // 2. K 条 rollout(热路径:无内存分配、无虚调用;噪声已预采样,
  //    各 k 写 disjoint 槽位 -> OpenMP 并行且保持可复现确定性)
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int k = 0; k < K; ++k) {
    State x = x0;
    double cost = 0.0;
    ControlVector u, u_raw, u_prev;
    bool have_prev = false;
    const double * noise_k = noise_.data() + static_cast<long>(k) * H * N8;
    for (int h = 0; h < H; ++h) {
      u_raw = nominal_.row(h).transpose() +
              Eigen::Map<const Eigen::Matrix<double, 8, 1>>(noise_k + h * N8);
      u = thruster_.clamp(u_raw);
      // 动力学 H 步(每步 substeps 个欧拉子步)
      for (int s = 0; s < p_.substeps; ++s) {
        model_.stepInPlace(x, u, sub_dt);
      }
      cost += cost_.runningCost(x, goal, u_raw, u,
                                have_prev ? &u_prev : nullptr, h == H - 1);
      u_prev = u;
      have_prev = true;
    }
    traj_cost_(k) = cost;
  }

  // 3. 重要性权重:softmax(-(S - S_min)/lambda)
  const double s_min = traj_cost_.minCoeff();
  double w_sum = 0.0;
  double entropy = 0.0;
  for (int k = 0; k < K; ++k) {
    const double w = std::exp(-(traj_cost_(k) - s_min) / p_.lambda);
    weights_(k) = w;
    w_sum += w;
  }
  for (int k = 0; k < K; ++k) {
    weights_(k) /= w_sum;
    if (weights_(k) > 1e-12) { entropy -= weights_(k) * std::log(weights_(k)); }
  }

  // 4. 加权更新名义序列 U <- alpha*Σw*(U+eps 截断后) + (1-alpha)*U
  //    注意用截断后的采样控制,保证名义序列始终可行。
  Eigen::MatrixXd U_new = (1.0 - p_.alpha) * nominal_;
  for (int k = 0; k < K; ++k) {
    const double w = weights_(k);
    if (w < 1e-12) { continue; }
    const double * noise_k = noise_.data() + static_cast<long>(k) * H * N8;
    for (int h = 0; h < H; ++h) {
      ControlVector u_raw = nominal_.row(h).transpose() +
                            Eigen::Map<const Eigen::Matrix<double, 8, 1>>(noise_k + h * N8);
      U_new.row(h) += p_.alpha * w * thruster_.clamp(u_raw).transpose();
    }
  }
  nominal_ = U_new;

  // 5. 序列左移一步,末尾补 shift_fill(下一周期热启动)
  nominal_.block(0, 0, H - 1, N8) = nominal_.block(1, 0, H - 1, N8);
  nominal_.row(H - 1) = ControlVector::Constant(p_.shift_fill).transpose();

  // 名义轨迹与最终统计(用更新前的 U_new 评估,即本周期输出)
  model_.rollout(x0, U_new, p_.dt, result_.states);
  result_.U = U_new;
  result_.total_cost = cost_.trajectoryCost(x0, goal, U_new, model_, p_.dt);
  result_.min_cost = s_min;
  result_.rollouts = K;
  result_.weight_entropy = entropy;

  const auto t_end = std::chrono::steady_clock::now();
  result_.solve_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();
  return result_;
}

}  // namespace auv_controller_mppi
