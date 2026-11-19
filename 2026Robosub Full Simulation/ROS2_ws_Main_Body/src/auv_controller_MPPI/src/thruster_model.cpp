// -*- coding: utf-8 -*-
#include "auv_controller_mppi/thruster_model.hpp"

#include <algorithm>

namespace auv_controller_mppi
{

ThrusterModel::ThrusterModel(const ThrusterParams & params)
{
  setParams(params);
}

void ThrusterModel::setParams(const ThrusterParams & params)
{
  params_ = params;
  for (int i = 0; i < kNumThrusters; ++i) {
    // 方向先归一化(输入理论上已是单位向量,这里兜底)
    params_.directions[i].normalize();
    B_.block<3, 1>(0, i) = params_.directions[i];
    B_.block<3, 1>(3, i) = params_.positions[i].cross(params_.directions[i]);
  }
}

ControlVector ThrusterModel::clamp(const ControlVector & u) const
{
  return u.cwiseMin(params_.force_max).cwiseMax(params_.force_min);
}

AllocationCheck ThrusterModel::check() const
{
  AllocationCheck c;
  for (const auto & d : params_.directions) {
    if (std::abs(d.norm() - 1.0) > 1e-6) { c.directions_unit = false; }
  }
  // 只需要奇异值:固定尺寸矩阵上 thin U/V 会被 Eigen 断言拒绝
  Eigen::JacobiSVD<AllocationMatrix> svd(B_);
  const auto sv = svd.singularValues();
  for (int i = 0; i < sv.size() && i < 6; ++i) { c.singular_values[i] = sv(i); }
  // 奇异值阈值:数值上远小于其它奇异值即视为亏秩
  const double tol = sv.size() ? sv(0) * 1e-9 : 0.0;
  c.rank = 0;
  for (int i = 0; i < sv.size(); ++i) {
    if (sv(i) > tol) { ++c.rank; }
  }
  if (sv.size()) { c.condition_number = sv(0) / sv(sv.size() - 1); }
  return c;
}

ControlVector ThrusterModel::distribute(const WrenchVector & tau) const
{
  // 最小范数解 T = B^T (B B^T)^-1 tau(B 列满秩时等价伪逆)
  Eigen::Matrix<double, 6, 6> BBt = B_ * B_.transpose();
  return clamp(B_.transpose() * BBt.llt().solve(tau));
}

}  // namespace auv_controller_mppi
