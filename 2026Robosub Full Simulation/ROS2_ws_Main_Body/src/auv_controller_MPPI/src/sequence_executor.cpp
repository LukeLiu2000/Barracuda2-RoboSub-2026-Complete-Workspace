// -*- coding: utf-8 -*-
#include "auv_controller_mppi/sequence_executor.hpp"

namespace auv_controller_mppi
{

SequenceExecutor::SequenceExecutor()
: SequenceExecutor(ThrusterModel(ThrusterParams()), Params())
{
}

SequenceExecutor::SequenceExecutor(const ThrusterModel & thruster)
: SequenceExecutor(thruster, Params())
{
}

SequenceExecutor::SequenceExecutor(const ThrusterModel & thruster,
                                   const Params & params)
: thruster_(thruster), p_(params)
{
}

void SequenceExecutor::setSequence(const ControlSequence & seq)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sequence_ = seq;
}

void SequenceExecutor::setExcitation(const ControlVector & u, double stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  excitation_ = u;
  excitation_stamp_ = stamp;
}

void SequenceExecutor::setZeroed(bool zeroed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  zeroed_ = zeroed;
}

ControlVector SequenceExecutor::command(double t, Status & status)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (zeroed_) {
    status = Status::ZEROED;
    return ControlVector::Zero();
  }
  // 激励旁路(训练实验):新命令优先,超时自动退出旁路
  if (t - excitation_stamp_ <= p_.excitation_timeout_s &&
      excitation_stamp_ > -1e8)
  {
    status = Status::EXCITATION;
    return thruster_.clamp(excitation_);
  }
  if (sequence_ && t <= sequence_->endTime() + p_.sequence_timeout_s) {
    ControlVector u;
    if (sequence_->sampleControl(t, u)) {
      status = Status::ACTIVE;
      return thruster_.clamp(u);
    }
  }
  status = Status::STALE;
  return ControlVector::Zero();  // 序列耗尽/超时 -> 安全零推力
}

}  // namespace auv_controller_mppi
