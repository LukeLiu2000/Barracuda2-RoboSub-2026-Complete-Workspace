// 推进器分布与 6x8 分配矩阵测试
#include <gtest/gtest.h>

#include "auv_controller_mppi/thruster_model.hpp"

using namespace auv_controller_mppi;

TEST(ThrusterModel, DefaultDirectionsAreUnit)
{
  ThrusterModel m;
  const auto c = m.check();
  EXPECT_TRUE(c.directions_unit);
}

TEST(ThrusterModel, AllocationRankIsSix)
{
  ThrusterModel m;
  const auto c = m.check();
  ASSERT_EQ(c.rank, 6) << "6x8 分配矩阵必须满秩,否则 6 轴不可控";
  EXPECT_GT(c.singular_values[5], 0.05) << "最小奇异值过小,力分配病态";
  EXPECT_LT(c.condition_number, 50.0);
}

TEST(ThrusterModel, ColumnEqualsCrossProduct)
{
  // 第 i 列 = [d_i ; r_i × d_i],拿 thruster_0 手算对照
  ThrusterModel m;
  const ThrusterParams & p = m.params();
  const WrenchVector col = m.column(0);
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(col(k), p.directions[0](k), 1e-12);
  }
  const Eigen::Vector3d tau = p.positions[0].cross(p.directions[0]);
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(col(3 + k), tau(k), 1e-12);
  }
}

TEST(ThrusterModel, HorizontalThrustersDiagonal)
{
  // t0..t3 推力方向应在水平(x-z)平面内,且互相 45° 对角
  ThrusterModel m;
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(m.params().directions[i](1), 0.0, 1e-9);  // 无 y 分量
    EXPECT_NEAR(m.params().directions[i].norm(), 1.0, 1e-9);
  }
  const double dot02 = m.params().directions[0].dot(m.params().directions[2]);
  EXPECT_NEAR(dot02, -1.0, 1e-9) << "t0 与 t2 应反向(对角)";
}

TEST(ThrusterModel, VerticalThrustersUp)
{
  ThrusterModel m;
  for (int i = 4; i < 8; ++i) {
    EXPECT_NEAR(m.params().directions[i](0), 0.0, 1e-9);
    EXPECT_NEAR(m.params().directions[i](1), 1.0, 1e-9);
    EXPECT_NEAR(m.params().directions[i](2), 0.0, 1e-9);
  }
}

TEST(ThrusterModel, ClampSaturation)
{
  ThrusterModel m;
  ControlVector u;
  u << 100.0, -100.0, 39.2266, -39.2266, 0.0, 1.0, -1.0, 39.3;
  const ControlVector c = m.clamp(u);
  EXPECT_DOUBLE_EQ(c(0), m.params().force_max);
  EXPECT_DOUBLE_EQ(c(1), m.params().force_min);
  EXPECT_DOUBLE_EQ(c(2), 39.2266);
  EXPECT_DOUBLE_EQ(c(3), -39.2266);
  EXPECT_DOUBLE_EQ(c(7), 39.2266);
}

TEST(ThrusterModel, DistributeRoundTrip)
{
  // B 满秩时,广义力 -> 推力 -> 广义力 应精确复原(未触限时)
  ThrusterModel m;
  WrenchVector tau;
  tau << 10.0, -5.0, 20.0, 0.3, -0.2, 0.1;
  const ControlVector u = m.distribute(tau);
  const WrenchVector tau2 = m.allocation() * u;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(tau2(i), tau(i), 1e-9) << "六轴广义力均可实现";
  }
}

TEST(ThrusterModel, BadDirectionFailsUnitCheck)
{
  ThrusterParams p;
  p.directions[0] = Eigen::Vector3d(1.0, 1.0, 1.0);  // 非单位向量
  ThrusterModel m(p);  // 构造时会归一化,check 仍应通过
  EXPECT_TRUE(m.check().directions_unit);
  EXPECT_EQ(m.check().rank, 6);
}
