// MPPI 求解器测试(采样、重要性权重、收敛)
#include <gtest/gtest.h>

#include <cmath>

#include "auv_controller_mppi/mppi_solver.hpp"

using namespace auv_controller_mppi;

namespace
{
struct Fixture
{
  DynamicsParams dp;
  ThrusterParams tp;
  DynamicsModel model;
  CostFunction cost;
  ThrusterModel thruster;
  MppiParams mp;
  Fixture()
  : dp([] {
      DynamicsParams p;
      p.buoyancy_force = p.mass * 9.81;   // 中性悬浮
      p.cb_position = Eigen::Vector3d::Zero();
      p.world_up = Eigen::Vector3d(0, 1, 0);
      p.gravity_world = Eigen::Vector3d(0, -9.81, 0);
      return p;
    }()),
    model(dp, tp), thruster(tp)
  {
    mp.K = 300; mp.H = 12; mp.dt = 0.1; mp.sigma = 8.0;
    mp.lambda = 5.0; mp.alpha = 0.2; mp.seed = 7;
  }
};
}  // namespace

TEST(MppiSolver, WeightsSumToOneAndNonNegative)
{
  Fixture f;
  MppiSolver s(f.model, f.cost, f.thruster, f.mp);
  State x0;
  Goal g;
  g.p = Eigen::Vector3d(0, 0, 1.0);
  s.solve(x0, g);
  double sum = 0.0;
  for (int k = 0; k < f.mp.K; ++k) {
    EXPECT_GE(s.weights()(k), 0.0);
    sum += s.weights()(k);
  }
  EXPECT_NEAR(sum, 1.0, 1e-9) << "重要性权重必须归一化";
}

TEST(MppiSolver, LargeLambdaGivesNearUniformWeights)
{
  Fixture f;
  f.mp.lambda = 1e9;  // 温度极大 -> softmax 平坦
  MppiSolver s(f.model, f.cost, f.thruster, f.mp);
  Goal g; g.p = Eigen::Vector3d(0.5, 0, 0.5);
  s.solve(State{}, g);
  const double uniform_entropy = std::log(static_cast<double>(f.mp.K));
  EXPECT_NEAR(s.lastResult().weight_entropy, uniform_entropy, 0.01);
}

TEST(MppiSolver, SmallLambdaConcentratesWeight)
{
  Fixture f;
  f.mp.lambda = 1e-4;  // 温度极小 -> 权重集中到最优轨迹
  MppiSolver s(f.model, f.cost, f.thruster, f.mp);
  Goal g; g.p = Eigen::Vector3d(0.5, 0, 0.5);
  s.solve(State{}, g);
  EXPECT_LT(s.lastResult().weight_entropy, 0.1);
  const double wmax = s.weights().maxCoeff();
  EXPECT_GT(wmax, 0.9);
}

TEST(MppiSolver, CostDecreasesOverIterations)
{
  Fixture f;
  f.mp.lambda = 100.0;  // 代价量级 ~1e3,温度过小会让权重退化成单样本随机游走
  MppiSolver s(f.model, f.cost, f.thruster, f.mp);
  Goal g;
  g.p = Eigen::Vector3d(0, 0, 1.0);
  g.q = Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY());
  double first_cost = 0, mid_cost = 0, last_cost = 0;
  // 闭环推进:每轮执行序列首步再重求解(与真实控制回路一致,
  // warm-start 的左移恰好与状态推进对齐)
  State x;
  for (int i = 0; i < 60; ++i) {
    const auto & r = s.solve(x, g);
    if (i == 0) { first_cost = r.total_cost; }
    if (i == 29) { mid_cost = r.total_cost; }
    last_cost = r.total_cost;
    const ControlVector u0 = f.thruster.clamp(r.U.row(0).transpose());
    x = f.model.predict(x, u0, f.mp.dt);
  }
  EXPECT_LT(last_cost, first_cost * 0.5)
    << "60 轮闭环迭代后名义代价应显著下降(实际 " << first_cost << " -> "
    << mid_cost << "(30) -> " << last_cost << "(60))";
}

TEST(MppiSolver, OutputRespectsThrusterLimits)
{
  Fixture f;
  f.mp.sigma = 100.0;  // 疯狂采样也必须输出可行控制
  MppiSolver s(f.model, f.cost, f.thruster, f.mp);
  Goal g; g.p = Eigen::Vector3d(2, 2, 2);
  const auto & r = s.solve(State{}, g);
  for (int h = 0; h < r.U.rows(); ++h) {
    for (int i = 0; i < kNumThrusters; ++i) {
      EXPECT_LE(r.U(h, i), f.thruster.params().force_max + 1e-9);
      EXPECT_GE(r.U(h, i), f.thruster.params().force_min - 1e-9);
    }
  }
}

TEST(MppiSolver, ReachedGoalKeepsLowCostAndSmallThrust)
{
  // 已在目标处:最优策略近似零推力(代价以 effort 主导)
  Fixture f;
  MppiSolver s(f.model, f.cost, f.thruster, f.mp);
  Goal g;  // 目标 = 原点 + 单位姿态(与初态一致)
  double u_norm = 0;
  for (int i = 0; i < 40; ++i) {
    const auto & r = s.solve(State{}, g);
    u_norm = r.U.cwiseAbs().maxCoeff();
  }
  EXPECT_LT(u_norm, 8.0) << "到达目标后应几乎不推(最大 |u|=" << u_norm << ")";
}

TEST(MppiSolver, DeterministicWithSameSeed)
{
  Fixture f;
  Goal g; g.p = Eigen::Vector3d(0.3, 0.2, -0.4);
  MppiSolver a(f.model, f.cost, f.thruster, f.mp);
  MppiSolver b(f.model, f.cost, f.thruster, f.mp);
  a.solve(State{}, g);
  b.solve(State{}, g);
  EXPECT_TRUE(a.lastResult().U.isApprox(b.lastResult().U, 1e-12));
}
