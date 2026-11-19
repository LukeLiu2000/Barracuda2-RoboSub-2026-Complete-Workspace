// 前向动力学模型测试
#include <gtest/gtest.h>

#include "auv_controller_mppi/dynamics_model.hpp"

using namespace auv_controller_mppi;

namespace
{
DynamicsParams neutralHoverParams()
{
  // 中性悬浮:浮力 = 重力,浮心过质心 -> 恢复力为零
  DynamicsParams p;
  p.buoyancy_force = p.mass * 9.81;
  p.cb_position = Eigen::Vector3d::Zero();
  p.world_up = Eigen::Vector3d(0, 1, 0);
  p.gravity_world = Eigen::Vector3d(0, -9.81, 0);
  return p;
}
DynamicsParams noDampingParams()
{
  auto p = neutralHoverParams();
  p.linear_damping.setZero();
  p.quadratic_damping.setZero();
  return p;
}
}  // namespace

TEST(DynamicsModel, HoverEquilibriumIsFixedPoint)
{
  DynamicsModel m(neutralHoverParams());
  State x;  // 原点、单位姿态、零速度
  const ControlVector u = ControlVector::Zero();
  for (int i = 0; i < 100; ++i) { x = m.predict(x, u, 0.1); }
  EXPECT_LT(x.p.norm(), 1e-9) << "无输入中性悬浮应保持位置";
  EXPECT_NEAR(x.q.w(), 1.0, 1e-9);
  EXPECT_LT(x.v.norm(), 1e-9);
  EXPECT_LT(x.w.norm(), 1e-9);
}

TEST(DynamicsModel, GravitySinkAccelMatchesMassModel)
{
  // 浮力置零:竖直下落加速度 = m*g / (m + ma_y)(体 y 轴)
  DynamicsParams p = noDampingParams();
  p.buoyancy_force = 0.0;
  DynamicsModel m(p);
  State x;
  const double dt = 0.01;
  x = m.predict(x, ControlVector::Zero(), dt);
  const double expected = 40.0 * 9.81 / (40.0 + 14.0);
  EXPECT_NEAR(x.v(1), -expected * dt, 1e-6);
}

TEST(DynamicsModel, BuoyancyTorusRightingMoment)
{
  // 浮心在质心上方 0.02 m,横滚 10°(绕纵轴 z)应产生恢复力矩(绕 z)
  DynamicsParams p = neutralHoverParams();
  p.cb_position = Eigen::Vector3d(0, 0.02, 0);
  DynamicsModel m(p);
  State x;
  x.q = Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());
  const WrenchVector g = m.restoringWrench(x);
  // 恢复力矩方向:把艇扶正 -> 与横滚角反号(绕 z 为负)
  EXPECT_LT(g(5), 0.0);
  // 力矩量级 ~ B * cb_z * sin(10°)
  EXPECT_NEAR(g(5), -387.4 * 0.02 * std::sin(10.0 * M_PI / 180.0), 1.0);
}

TEST(DynamicsModel, PureYawSpinConservesAngularRate)
{
  // 无阻尼、无恢复力矩:绕 y(偏航轴)匀速旋转应保持速率(对角惯量下
  // C_RB 项自相消),姿态角随时间线性增长 —— 验证四元数积分。
  DynamicsModel m(noDampingParams());
  State x;
  x.w = Eigen::Vector3d(0, 0.5, 0);  // 0.5 rad/s 纯偏航
  const double T = 2.0;
  for (int i = 0; i < 200; ++i) { x = m.predict(x, ControlVector::Zero(), T / 200); }
  EXPECT_NEAR(x.w(1), 0.5, 1e-3);
  EXPECT_NEAR(x.w(0), 0.0, 1e-9);
  EXPECT_NEAR(x.w(2), 0.0, 1e-9);
  const Eigen::Quaterniond q_expect(
    Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY()));
  EXPECT_LT(orientationError(x.q, q_expect), 1e-3) << "2s * 0.5rad/s = 1 rad 偏航";
  EXPECT_NEAR(x.q.norm(), 1.0, 1e-9) << "四元数应保持归一化";
}

TEST(DynamicsModel, SurgeThrustAccel)
{
  // 纯纵推力:tau = B*T = [0,0,F],加速度 = F/(m+ma_z)
  ThrusterModel tm;
  DynamicsModel m(noDampingParams());
  const double F = 30.0;
  WrenchVector tau;
  tau << 0, 0, F, 0, 0, 0;
  const ControlVector u = tm.distribute(tau);
  State x;
  x = m.predict(x, u, 0.01);
  const double expected = F / (40.0 + 10.0);
  EXPECT_NEAR(x.v(2), expected * 0.01, 1e-6);
}

TEST(DynamicsModel, QuaternionDoubleCoverStable)
{
  // q 与 -q 是同一姿态:误差角应为 0
  Eigen::Quaterniond qa(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond qb = qa;
  qb.coeffs() = -qb.coeffs();
  EXPECT_LT(orientationError(qa, qb), 1e-12);
}

TEST(DynamicsModel, StateVectorRoundTrip)
{
  State x;
  x.p = Eigen::Vector3d(1, 2, 3);
  x.q = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  x.v = Eigen::Vector3d(0.1, 0.2, 0.3);
  x.w = Eigen::Vector3d(0.01, 0.02, 0.03);
  const auto v = x.toVector();
  EXPECT_EQ(v.size(), 13);
  const State y = State::fromVector(v);
  EXPECT_LT((x.p - y.p).norm(), 1e-12);
  EXPECT_LT(orientationError(x.q, y.q), 1e-12);
  EXPECT_LT((x.v - y.v).norm(), 1e-12);
  EXPECT_LT((x.w - y.w).norm(), 1e-12);
}
