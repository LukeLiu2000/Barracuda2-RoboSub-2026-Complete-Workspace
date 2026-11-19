// 代价函数测试
#include <gtest/gtest.h>

#include "auv_controller_mppi/cost_function.hpp"

using namespace auv_controller_mppi;

namespace
{
Goal goalAtOrigin()
{
  Goal g;
  g.p = Eigen::Vector3d::Zero();
  g.q = Eigen::Quaterniond::Identity();
  return g;
}
}  // namespace

TEST(CostFunction, ZeroAtGoalWithZeroControl)
{
  CostFunction c;
  State x;  // 原点单位姿态零速度
  const Goal g = goalAtOrigin();
  ControlVector u = ControlVector::Zero();
  EXPECT_NEAR(c.runningCost(x, g, u, u, nullptr, false), 0.0, 1e-12);
  EXPECT_NEAR(c.runningCost(x, g, u, u, nullptr, true), 0.0, 1e-12);
}

TEST(CostFunction, PositionTerm)
{
  CostFunction c;
  const Goal g = goalAtOrigin();
  State x;
  x.p = Eigen::Vector3d(1.0, 0, 0);
  ControlVector u = ControlVector::Zero();
  EXPECT_NEAR(c.runningCost(x, g, u, u, nullptr, false),
              c.params().position_weight * 1.0, 1e-9);
}

TEST(CostFunction, OrientationTermUsesAngleNotQuaternionDistance)
{
  CostFunction c;
  const Goal g = goalAtOrigin();
  State x;
  x.q = Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY());  // 30°
  ControlVector u = ControlVector::Zero();
  EXPECT_NEAR(c.runningCost(x, g, u, u, nullptr, false),
              c.params().orientation_weight * (M_PI / 6) * (M_PI / 6), 1e-9);
}

TEST(CostFunction, TerminalMultiplier)
{
  CostFunction c;
  const Goal g = goalAtOrigin();
  State x;
  x.p = Eigen::Vector3d(0.5, 0, 0);
  ControlVector u = ControlVector::Zero();
  const double run = c.runningCost(x, g, u, u, nullptr, false);
  const double term = c.runningCost(x, g, u, u, nullptr, true);
  EXPECT_NEAR(term, run * c.params().terminal_multiplier, 1e-9);
}

TEST(CostFunction, SaturationPenaltyOnRawOverrun)
{
  CostFunction c;
  const Goal g = goalAtOrigin();
  State x;
  ControlVector u_raw = ControlVector::Zero();
  u_raw(0) = 1.5 * c.params().u_max;   // 想推 150% 限值
  ControlVector u = u_raw.cwiseMin(c.params().u_max);
  const double cost = c.runningCost(x, g, u_raw, u, nullptr, false);
  const double over = 0.5 * c.params().u_max;
  // 注意截断后的实际推力 = u_max(非零) -> 还有一项 effort_weight*1
  EXPECT_NEAR(cost,
              c.params().saturation_weight * over * over /
                (c.params().u_max * c.params().u_max) +
              c.params().effort_weight * 1.0,
              1e-9);
}

TEST(CostFunction, SmoothnessPenalty)
{
  CostFunction c;
  const Goal g = goalAtOrigin();
  State x;
  ControlVector u = ControlVector::Zero();
  u(0) = 10.0;
  ControlVector u_prev = ControlVector::Zero();
  const double with = c.runningCost(x, g, u, u, &u_prev, false);
  const double without = c.runningCost(x, g, u, u, nullptr, false);
  EXPECT_NEAR(with - without, c.params().smoothness_weight * 100.0 /
              (c.params().u_max * c.params().u_max), 1e-9);
}

TEST(CostFunction, WorkspaceViolationPenalized)
{
  CostFunction c;
  const Goal g = goalAtOrigin();
  State x;
  x.p = Eigen::Vector3d(0, c.params().workspace_max(1) + 0.5, 0);  // 顶部越界 0.5 m
  ControlVector u = ControlVector::Zero();
  const double cost = c.runningCost(x, g, u, u, nullptr, false);
  // 目标在原点,机器人 y=5.5 -> 位置项 != 0,必须一并计入
  const double pos2 = x.p.squaredNorm();
  EXPECT_NEAR(cost,
              c.params().position_weight * pos2 +
              c.params().safety_weight * 0.25 /
                (c.params().u_max * c.params().u_max),
              1e-6);
}

TEST(CostFunction, TrajectoryCostIntegratesHorizon)
{
  CostFunction c;
  DynamicsModel m;
  const Goal g = goalAtOrigin();
  Eigen::MatrixXd U = Eigen::MatrixXd::Zero(5, kNumThrusters);
  State x0;
  x0.p = Eigen::Vector3d(0.5, 0, 0);
  const double total = c.trajectoryCost(x0, g, U, m, 0.1);
  // 每步位置项 = wp*0.25(静止机器人近似不动),终端 ×mult
  EXPECT_GT(total, 0.0);
  EXPECT_LT(total, c.params().position_weight * 0.25 * 6 * 10);
}
