// -*- coding: utf-8 -*-
/**
 * MPPI(Model Predictive Path Integral)求解器,CPU 单线程实现。
 *
 * 参考:Williams et al., "Aggressive Driving with Model Predictive Path
 * Integral Control", ICRA 2016。每次 solve():
 *   1. 以当前名义序列 U(  H×8  )为中心,采样 K 条扰动序列
 *      U_k = clip(U + eps_k),eps ~ N(0, diag(sigma^2));
 *   2. 每条序列用前向动力学滚动 H 步,累加代价;
 *   3. 重要性权重 w_k = softmax(-(S_k - S_min)/lambda);
 *   4. 名义序列更新 U <- alpha * Σ w_k U_k + (1-alpha) * U;
 *   5. 序列左移一步(末尾补零),供下一周期热启动。
 *
 * 设计要点(为未来 CUDA 迁移):
 *   - rollout 内核只读 model/cost,无任何共享可变状态;
 *   - 噪声一次性预生成到 (K,H,8) 缓冲,rollout 循环是纯计算;
 *   - 所有数据布局按 [k][h][dim] 连续,方便整体搬去 GPU。
 */
#pragma once

#include <atomic>
#include <random>

#include "auv_controller_mppi/cost_function.hpp"
#include "auv_controller_mppi/dynamics_model.hpp"
#include "auv_controller_mppi/thruster_model.hpp"
#include "auv_controller_mppi/types.hpp"

namespace auv_controller_mppi
{

struct MppiParams
{
  int K = 1000;            ///< 采样轨迹条数
  int H = 15;              ///< 预测时域步数
  double dt = 0.1;         ///< 预测步长 [s](时域 = H*dt = 1.5 s)
  double sigma = 10.0;     ///< 采样标准差 [N]
  double lambda = 8.0;     ///< 柔度温度(代价->权重)
  double alpha = 0.15;     ///< 名义序列动量(0=只信加权均值)
  double shift_fill = 0.0; ///< 左移后末尾填充控制 [N]
  uint64_t seed = 42;      ///< 随机种子(测试可复现)
  int substeps = 1;        ///< 每步内部欧拉子步数(动力学精度用)
};

/// 求解结果与统计
struct MppiResult
{
  Eigen::MatrixXd U;            ///< H×8 优化后名义控制序列
  Eigen::MatrixXd states;       ///< (H+1)×13 名义轨迹(调试/预测路径)
  double total_cost = 0.0;      ///< 名义序列总代价
  double min_cost = 0.0;        ///< 采样中最优代价
  double solve_ms = 0.0;        ///< 求解耗时
  int    rollouts = 0;          ///< 实际 rollout 条数
  double weight_entropy = 0.0;  ///< -Σw·log w(权重集中度诊断)
};

class MppiSolver
{
public:
  MppiSolver(const DynamicsModel & model,
             const CostFunction & cost,
             const ThrusterModel & thruster,
             const MppiParams & params = MppiParams());

  /// 求解一周期。x0 = 当前真实状态;goal = 目标。
  /// 返回引用在下次 solve 前有效。线程模型:仅 solver 线程调用。
  const MppiResult & solve(const State & x0, const Goal & goal);

  /// 重置名义序列(比如急停后重新开始)
  void reset();

  const MppiParams & params() const { return p_; }
  void setParams(const MppiParams & params);

  const MppiResult & lastResult() const { return result_; }

  /// 上一次 solve 的重要性权重(测试/诊断用)
  const Eigen::VectorXd & weights() const { return weights_; }
  /// 上一次 solve 的各轨迹代价(测试/诊断用)
  const Eigen::VectorXd & trajectoryCosts() const { return traj_cost_; }

private:
  /// 生成 K*H*8 高斯噪声到 noise_(线性索引 k*H*8+h*8+d)
  void sampleNoise();

  DynamicsModel model_;   // 拷贝一份:辨识在线更新时不影响 solver 线程
  CostFunction  cost_;
  ThrusterModel thruster_;
  MppiParams    p_;

  Eigen::MatrixXd nominal_;      ///< H×8 当前名义序列
  Eigen::VectorXd noise_;        ///< K*H*8
  Eigen::VectorXd traj_cost_;    ///< K
  Eigen::VectorXd weights_;      ///< K
  MppiResult     result_;
  std::mt19937_64 rng_;
};

}  // namespace auv_controller_mppi
