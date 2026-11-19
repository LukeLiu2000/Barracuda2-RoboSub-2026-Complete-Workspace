// -*- coding: utf-8 -*-
/**
 * MPPI 代价函数。
 *
 * 每步代价 c(x, u_k, u_{k-1}) =
 *    wp  * ||p - p_goal||^2                         位置误差
 *  + wo  * angle(q, q_goal)^2                       姿态误差
 *  + wv  * ||v - v_goal||^2                         线速度误差
 *  + ww  * ||w - w_goal||^2                         角速度误差
 *  + wu  * ||u||^2 / umax^2                         推进器做功(归一化)
 *  + wdu * ||u_k - u_{k-1}||^2 / umax^2             控制平滑度
 *  + wsat* Σ max(0, |u_raw|-umax)^2                 饱和惩罚(截断前)
 *  + wsafe* Σ max(0, p_outside_workspace)^2         工作空间安全边界
 * 终端步再乘 terminal_multiplier。
 * 全部权重 YAML 可配(config/mppi.yaml)。
 */
#pragma once

#include "auv_controller_mppi/dynamics_model.hpp"
#include "auv_controller_mppi/types.hpp"

namespace auv_controller_mppi
{

struct CostParams
{
  double position_weight   = 60.0;  ///< wp [1/m^2]
  double orientation_weight = 25.0; ///< wo [1/rad^2]
  double lin_vel_weight    = 8.0;   ///< wv [s^2/m^2]
  double ang_vel_weight    = 3.0;   ///< ww [s^2/rad^2]
  double effort_weight     = 0.02;  ///< wu(无量纲,u 已归一化)
  double smoothness_weight = 0.3;   ///< wdu(无量纲)
  double saturation_weight = 50.0;  ///< wsat(归一化后)
  double safety_weight     = 200.0; ///< wsafe(归一化到 umax^2 量级)
  double terminal_multiplier = 8.0; ///< 终端加权

  /// 世界系工作空间边界(盒);软约束:hinge^2
  Eigen::Vector3d workspace_min{-3.0, -1.0, -3.0};
  Eigen::Vector3d workspace_max{+3.0, +5.0, +3.0};

  /// 饱和限幅值(与 ThrusterParams.force_max 一致,由节点装载时同步)
  double u_max = 39.2266;
};

class CostFunction
{
public:
  explicit CostFunction(const CostParams & params = CostParams());

  /// 单步代价。u_raw = 未截断采样控制(算饱和惩罚),
  /// u = 已截断(送入动力学)。u_prev 取上一时刻控制(平滑项),首步传 nullptr。
  double runningCost(const State & x, const Goal & goal,
                     const ControlVector & u_raw, const ControlVector & u,
                     const ControlVector * u_prev, bool terminal) const;

  /// 整条 rollout 的总代价(测试/调试用;solver 内联热路径不用它)
  double trajectoryCost(const State & x0, const Goal & goal,
                        const Eigen::MatrixXd & U, const DynamicsModel & model,
                        double dt) const;

  const CostParams & params() const { return p_; }
  void setParams(const CostParams & params) { p_ = params; }

private:
  /// 盒外越界量(hinge):每轴 max(0, min-p) + max(0, p-max)
  double workspaceViolation(const Eigen::Vector3d & p) const;

  CostParams p_;
};

}  // namespace auv_controller_mppi
