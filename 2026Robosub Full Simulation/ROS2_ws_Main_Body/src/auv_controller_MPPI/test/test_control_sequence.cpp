// 控制序列与执行器测试
#include <gtest/gtest.h>

#include "auv_controller_mppi/control_sequence.hpp"
#include "auv_controller_mppi/sequence_executor.hpp"

using namespace auv_controller_mppi;

namespace
{
Eigen::MatrixXd makeU(int H)
{
  Eigen::MatrixXd U(H, kNumThrusters);
  for (int h = 0; h < H; ++h) {
    for (int i = 0; i < kNumThrusters; ++i) { U(h, i) = h * 10.0 + i; }
  }
  return U;
}
}  // namespace

TEST(ControlSequence, SampleByTimestamp)
{
  const Eigen::MatrixXd U = makeU(10);
  ControlSequence seq(100.0, 0.1, U);
  ControlVector u;
  ASSERT_TRUE(seq.sampleControl(100.05, u));
  EXPECT_DOUBLE_EQ(u(0), 0.0);        // 第 0 步
  ASSERT_TRUE(seq.sampleControl(100.15, u));
  EXPECT_DOUBLE_EQ(u(0), 10.0);       // 第 1 步
  ASSERT_TRUE(seq.sampleControl(100.99, u));
  EXPECT_DOUBLE_EQ(u(0), 90.0);       // 第 9 步(域内最后)
  // 末尾之后保持最后一步(失效判断是执行器的职责)
  ASSERT_TRUE(seq.sampleControl(101.5, u));
  EXPECT_DOUBLE_EQ(u(0), 90.0);
}

TEST(ControlSequence, ClockSlightlyBackUsesFirst)
{
  ControlSequence seq(100.0, 0.1, makeU(3));
  ControlVector u;
  ASSERT_TRUE(seq.sampleControl(99.99, u));
  EXPECT_DOUBLE_EQ(u(0), 0.0);
}

TEST(SequenceExecutor, ActiveWithinHorizon)
{
  ThrusterModel tm;
  SequenceExecutor ex(tm);
  ex.setSequence(ControlSequence(0.0, 0.5, makeU(4)));
  SequenceExecutor::Status st;
  const ControlVector u = ex.command(0.6, st);
  EXPECT_EQ(st, SequenceExecutor::Status::ACTIVE);
  EXPECT_DOUBLE_EQ(u(0), 10.0);  // 第 1 步
}

TEST(SequenceExecutor, StaleAfterTimeoutGivesZero)
{
  ThrusterModel tm;
  SequenceExecutor::Params p;
  p.sequence_timeout_s = 0.3;
  SequenceExecutor ex(tm, p);
  ex.setSequence(ControlSequence(0.0, 0.5, makeU(4)));  // 末尾 = 2.0 s
  SequenceExecutor::Status st;
  ControlVector u = ex.command(2.5, st);                // 超出末尾 0.5 s > 0.3
  EXPECT_EQ(st, SequenceExecutor::Status::STALE);
  EXPECT_EQ(u.norm(), 0.0);
  // 末尾 +0.2 s 仍在宽限内
  u = ex.command(2.2, st);
  EXPECT_EQ(st, SequenceExecutor::Status::ACTIVE);
}

TEST(SequenceExecutor, NewSequenceReplacesOldImmediately)
{
  ThrusterModel tm;
  SequenceExecutor ex(tm);
  ex.setSequence(ControlSequence(0.0, 1.0, makeU(4)));
  Eigen::MatrixXd U2 = Eigen::MatrixXd::Constant(4, kNumThrusters, 5.0);
  ex.setSequence(ControlSequence(10.0, 1.0, U2));  // 新序列从 10 s 开始
  SequenceExecutor::Status st;
  const ControlVector u = ex.command(10.5, st);
  EXPECT_EQ(st, SequenceExecutor::Status::ACTIVE);
  EXPECT_DOUBLE_EQ(u(3), 5.0);
}

TEST(SequenceExecutor, ExcitationBypassAndTimeout)
{
  ThrusterModel tm;
  SequenceExecutor::Params p;
  p.excitation_timeout_s = 0.2;
  SequenceExecutor ex(tm, p);
  ex.setSequence(ControlSequence(0.0, 10.0, makeU(4)));  // 一直有效
  ControlVector e = ControlVector::Constant(3.0);
  ex.setExcitation(e, 5.0);
  SequenceExecutor::Status st;
  ControlVector u = ex.command(5.1, st);
  EXPECT_EQ(st, SequenceExecutor::Status::EXCITATION);
  EXPECT_DOUBLE_EQ(u(0), 3.0);
  u = ex.command(5.4, st);   // 激励超时
  EXPECT_EQ(st, SequenceExecutor::Status::ACTIVE);  // 回到 MPPI 序列
}

TEST(SequenceExecutor, ExcitationIsSaturated)
{
  ThrusterModel tm;
  SequenceExecutor ex(tm);
  ControlVector e = ControlVector::Constant(500.0);
  ex.setExcitation(e, 1.0);
  SequenceExecutor::Status st;
  const ControlVector u = ex.command(1.05, st);
  EXPECT_EQ(st, SequenceExecutor::Status::EXCITATION);
  for (int i = 0; i < kNumThrusters; ++i) {
    EXPECT_DOUBLE_EQ(u(i), tm.params().force_max);
  }
}

TEST(SequenceExecutor, ZeroedOverridesEverything)
{
  ThrusterModel tm;
  SequenceExecutor ex(tm);
  ex.setSequence(ControlSequence(0.0, 10.0, makeU(4)));
  ex.setZeroed(true);
  SequenceExecutor::Status st;
  const ControlVector u = ex.command(0.5, st);
  EXPECT_EQ(st, SequenceExecutor::Status::ZEROED);
  EXPECT_EQ(u.norm(), 0.0);
  ex.setZeroed(false);
  const ControlVector u2 = ex.command(0.5, st);
  EXPECT_EQ(st, SequenceExecutor::Status::ACTIVE);
  EXPECT_DOUBLE_EQ(u2(1), 1.0);  // makeU 第 0 行第 1 列
}
