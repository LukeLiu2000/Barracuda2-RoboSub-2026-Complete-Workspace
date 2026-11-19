// -*- coding: utf-8 -*-
/**
 * 控制序列执行器:高频(50-100 Hz)按时间戳从最新序列取当前控制量。
 *
 * 线程模型:
 *   - solver 线程调用 setSequence()(新序列立即替换旧序列剩余部分);
 *   - 执行定时器线程调用 command();
 *   - 内部互斥锁保护。
 *
 * 处理:序列超时(滞留)/ 激励旁路(训练模式直接给推力)/ 限幅 / 强制置零。
 */
#pragma once

#include <mutex>
#include <optional>

#include "auv_controller_mppi/control_sequence.hpp"
#include "auv_controller_mppi/thruster_model.hpp"

namespace auv_controller_mppi
{

class SequenceExecutor
{
public:
  enum class Status
  {
    ACTIVE,       ///< 正常执行 MPPI 序列
    EXCITATION,   ///< 训练激励旁路
    STALE,        ///< 序列超时(掉到安全值)
    ZEROED,       ///< 被外部强制置零(急停/看门狗)
  };

  struct Params
  {
    double sequence_timeout_s = 0.5;  ///< 序列尾部超时(超出末尾多久视为失效)
    double excitation_timeout_s = 0.5;///< 激励命令超时
  };

  SequenceExecutor();
  explicit SequenceExecutor(const ThrusterModel & thruster);
  SequenceExecutor(const ThrusterModel & thruster, const Params & params);

  /// solver 线程:装入新序列(t0 = 当前时间)
  void setSequence(const ControlSequence & seq);

  /// 激励旁路(训练实验):stamp = 该命令时间戳
  void setExcitation(const ControlVector & u, double stamp);

  /// 外部急停 / 恢复
  void setZeroed(bool zeroed);

  /// 执行线程:t 时刻应输出的控制
  ControlVector command(double t, Status & status);

  const Params & params() const { return p_; }

private:
  ThrusterModel thruster_;
  Params p_;
  std::mutex mutex_;
  std::optional<ControlSequence> sequence_;
  ControlVector excitation_{ControlVector::Zero()};
  double excitation_stamp_ = -1e9;
  bool zeroed_ = false;
};

}  // namespace auv_controller_mppi
