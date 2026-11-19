// -*- coding: utf-8 -*-
#include "auv_controller_mppi/control_sequence.hpp"

#include <algorithm>

namespace auv_controller_mppi
{

ControlSequence::ControlSequence(double t0, double dt, const Eigen::MatrixXd & U)
: t0_(t0), dt_(dt), U_(U)
{
}

bool ControlSequence::sampleControl(double t, ControlVector & u) const
{
  if (U_.rows() == 0 || dt_ <= 0.0) { return false; }
  const int H = static_cast<int>(U_.rows());
  const double rel = t - t0_;
  if (rel < 0.0) {
    u = U_.row(0).transpose();  // 尚未开始(时钟微小回退等)
    return true;
  }
  // +1e-9 容差:消除 rel/dt 恰为整数时的浮点下取整抖动
  int idx = static_cast<int>(std::floor(rel / dt_ + 1e-9));
  if (idx >= H) {
    // 序列末尾之后:保持最后一步(执行器负责超时判 STALE 并置零)
    idx = H - 1;
  }
  u = U_.row(idx).transpose();
  return true;
}

}  // namespace auv_controller_mppi
