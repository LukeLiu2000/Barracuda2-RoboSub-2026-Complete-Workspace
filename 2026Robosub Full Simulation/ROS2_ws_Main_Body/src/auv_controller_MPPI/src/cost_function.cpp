// -*- coding: utf-8 -*-
#include "auv_controller_mppi/cost_function.hpp"

#include <cmath>

namespace auv_controller_mppi
{

CostFunction::CostFunction(const CostParams & params)
: p_(params)
{
}

double CostFunction::workspaceViolation(const Eigen::Vector3d & p) const
{
  double s = 0.0;
  for (int i = 0; i < 3; ++i) {
    s += std::max(0.0, p_.workspace_min(i) - p(i));
    s += std::max(0.0, p(i) - p_.workspace_max(i));
  }
  return s;  // 总越界距离 [m]
}

double CostFunction::runningCost(const State & x, const Goal & goal,
                                 const ControlVector & u_raw,
                                 const ControlVector & u,
                                 const ControlVector * u_prev,
                                 bool terminal) const
{
  const double m = terminal ? p_.terminal_multiplier : 1.0;
  const double umax2 = p_.u_max * p_.u_max;

  double c = 0.0;
  c += m * p_.position_weight * (x.p - goal.p).squaredNorm();
  c += m * p_.orientation_weight *
       std::pow(orientationError(x.q, goal.q), 2);
  c += m * p_.lin_vel_weight * (x.v - goal.v).squaredNorm();
  c += m * p_.ang_vel_weight * (x.w - goal.w).squaredNorm();

  // 推力做功(归一化,用截断后的实际施加值)
  c += p_.effort_weight * u.squaredNorm() / umax2;

  // 控制平滑度(相邻两步差分;首步不计)
  if (u_prev != nullptr) {
    c += p_.smoothness_weight * (u - *u_prev).squaredNorm() / umax2;
  }

  // 饱和惩罚:采样想越限的量(截断前),hinge^2
  double over = 0.0;
  for (int i = 0; i < u_raw.size(); ++i) {
    const double e = std::max(0.0, std::abs(u_raw(i)) - p_.u_max);
    over += e * e;
  }
  c += p_.saturation_weight * over / umax2;

  // 工作空间软约束(世界系盒)
  const double viol = workspaceViolation(x.p);
  c += m * p_.safety_weight * viol * viol / umax2;

  return c;
}

double CostFunction::trajectoryCost(const State & x0, const Goal & goal,
                                    const Eigen::MatrixXd & U,
                                    const DynamicsModel & model,
                                    double dt) const
{
  State x = x0;
  ControlVector u_prev;
  bool have_prev = false;
  double total = 0.0;
  const int H = static_cast<int>(U.rows());
  for (int h = 0; h < H; ++h) {
    const ControlVector u_raw = U.row(h).transpose();
    const ControlVector u = model.thruster().clamp(u_raw);
    x = model.predict(x, u, dt);
    total += runningCost(x, goal, u_raw, u,
                         have_prev ? &u_prev : nullptr, h == H - 1);
    u_prev = u;
    have_prev = true;
  }
  return total;
}

}  // namespace auv_controller_mppi
