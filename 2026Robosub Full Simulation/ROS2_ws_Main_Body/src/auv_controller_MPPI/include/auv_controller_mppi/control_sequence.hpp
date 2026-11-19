// -*- coding: utf-8 -*-
/**
 * 控制序列:MPPI 输出的 H 步未来推力序列 + 时间戳管理。
 *
 * 序列以 t0(稳态时钟秒)为起点,步长 dt,第 h 步在 [t0+h*dt, t0+(h+1)*dt)
 * 内有效。超过序列末尾 -> 按超时策略处理(见 SequenceExecutor)。
 */
#pragma once

#include "auv_controller_mppi/types.hpp"

namespace auv_controller_mppi
{

class ControlSequence
{
public:
  ControlSequence() = default;

  /// 构造:t0 秒(稳态时钟),dt 步长,U 为 H×8
  ControlSequence(double t0, double dt, const Eigen::MatrixXd & U);

  /// t 时刻的控制;超出序列末尾时保持最后一步(超时判失效由执行器负责)。
  /// 返回 false 仅当序列为空或 dt 无效。
  bool sampleControl(double t, ControlVector & u) const;

  int horizon() const { return static_cast<int>(U_.rows()); }
  double t0() const { return t0_; }
  double dt() const { return dt_; }
  double endTime() const { return t0_ + horizon() * dt_; }
  const Eigen::MatrixXd & controls() const { return U_; }

private:
  double t0_ = 0.0;
  double dt_ = 0.0;
  Eigen::MatrixXd U_;  ///< H×8
};

}  // namespace auv_controller_mppi
