// -*- coding: utf-8 -*-
#include "auv_controller_mppi/dynamics_model.hpp"

namespace auv_controller_mppi
{

namespace
{
/// 3x1 -> 反对称矩阵
inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<    0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return S;
}
}  // namespace

DynamicsModel::DynamicsModel(const DynamicsParams & params,
                             const ThrusterParams & thruster)
: thruster_(thruster)
{
  setParams(params);
}

void DynamicsModel::setParams(const DynamicsParams & params)
{
  p_ = params;
  updateCache();
}

void DynamicsModel::updateCache()
{
  const Eigen::Matrix<double, 6, 1> m_diag =
    (Eigen::Matrix<double, 6, 1>() <<
      p_.mass + p_.added_mass(0), p_.mass + p_.added_mass(1),
      p_.mass + p_.added_mass(2), p_.inertia(0) + p_.added_mass(3),
      p_.inertia(1) + p_.added_mass(4), p_.inertia(2) + p_.added_mass(5))
    .finished();
  minv_ = m_diag.cwiseInverse();
  m_plus_mav_ = Eigen::Vector3d(m_diag(0), m_diag(1), m_diag(2));
  i_plus_maw_ = Eigen::Vector3d(m_diag(3), m_diag(4), m_diag(5));
  gravity_force_world_ = p_.mass * p_.gravity_world;
  buoy_force_world_ = p_.buoyancy_force * p_.world_up.normalized();
}

Eigen::Matrix<double, 6, 6> DynamicsModel::massMatrix() const
{
  Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();
  M(0, 0) = p_.mass + p_.added_mass(0);
  M(1, 1) = p_.mass + p_.added_mass(1);
  M(2, 2) = p_.mass + p_.added_mass(2);
  M(3, 3) = p_.inertia(0) + p_.added_mass(3);
  M(4, 4) = p_.inertia(1) + p_.added_mass(4);
  M(5, 5) = p_.inertia(2) + p_.added_mass(5);
  return M;
}

Eigen::Matrix<double, 6, 6> DynamicsModel::coriolis(
  const Eigen::Vector3d & v, const Eigen::Vector3d & w) const
{
  // 体坐标系原点在质心(Fossen 2011, Eq. 3.63):
  // C_RB = [ 0            -m*S(v1) ]
  //        [ m*S(v1)      -S(I v2) ]
  // 加质量(对角 A11=diag(ma_v), A22=diag(ma_w),无交叉项):
  // C_A  = [ 0            -S(A11 v1) ]
  //        [ S(A11 v1)    -S(A22 v2) ]
  Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Vector3d ma_v(p_.added_mass(0), p_.added_mass(1), p_.added_mass(2));
  const Eigen::Vector3d ma_w(p_.added_mass(3), p_.added_mass(4), p_.added_mass(5));

  const Eigen::Matrix3d S_rb_lin = skew(p_.mass * v);
  const Eigen::Matrix3d S_rb_ang = skew(
    (Eigen::Vector3d(p_.inertia(0), p_.inertia(1), p_.inertia(2)).cwiseProduct(w)));
  const Eigen::Matrix3d S_a_lin  = skew(ma_v.cwiseProduct(v));
  const Eigen::Matrix3d S_a_ang  = skew(ma_w.cwiseProduct(w));

  C.block<3, 3>(0, 3) = -(S_rb_lin + S_a_lin);
  C.block<3, 3>(3, 0) =   S_rb_lin + S_a_lin;
  C.block<3, 3>(3, 3) = -(S_rb_ang + S_a_ang);
  return C;
}

Eigen::Matrix<double, 6, 6> DynamicsModel::damping(
  const Eigen::Vector3d & v, const Eigen::Vector3d & w) const
{
  Eigen::Matrix<double, 6, 1> nu_abs;
  nu_abs << std::abs(v.x()), std::abs(v.y()), std::abs(v.z()),
            std::abs(w.x()), std::abs(w.y()), std::abs(w.z());
  Eigen::Matrix<double, 6, 1> d = p_.linear_damping +
                                  p_.quadratic_damping.cwiseProduct(nu_abs);
  return d.asDiagonal();
}

WrenchVector DynamicsModel::restoringWrench(const State & x) const
{
  // 重力:世界系 m*g,转到体坐标系;作用点 = 质心 = 体坐标系原点 -> 无力矩
  const Eigen::Quaterniond q_inv = x.q.conjugate();
  const Eigen::Vector3d f_gravity_b = q_inv * gravity_force_world_;
  // 浮力:世界系 +buoyancy_force * up,转到体坐标系;作用点 cb_position
  const Eigen::Vector3d f_buoy_b = q_inv * buoy_force_world_;
  WrenchVector g;
  g.segment<3>(0) = f_gravity_b + f_buoy_b;
  g.segment<3>(3) = p_.cb_position.cross(f_buoy_b);
  return g;
}

void DynamicsModel::eulerStep(State & x, const ControlVector & u, double dt) const
{
  // ---- 热路径:不组装 6x6 矩阵,全部写成逐元素/叉积形式 ----
  // tau = B * (scale * u)
  const WrenchVector tau = thruster_.allocation() * (p_.thrust_scale * u);

  // 恢复力(四元数共轭旋转 = R^T * v,免显式旋转矩阵)
  const Eigen::Quaterniond q_inv = x.q.conjugate();
  const Eigen::Vector3d f_g = q_inv * gravity_force_world_;
  const Eigen::Vector3d f_b = q_inv * buoy_force_world_;

  // Coriolis 直接形式:a = (m+ma_v).*v,b = (I+ma_w).*w
  //   C*nu = [ -a×w ; a×v - b×w ]
  const Eigen::Vector3d a = m_plus_mav_.cwiseProduct(x.v);
  const Eigen::Vector3d b = i_plus_maw_.cwiseProduct(x.w);

  // 对角阻尼 D*nu = (lin + quad*|nu|).*nu,与推力/恢复/科氏合成右端项
  Eigen::Matrix<double, 6, 1> rhs;
  rhs.segment<3>(0) =
    tau.segment<3>(0) + f_g + f_b - a.cross(x.w) -
    (p_.linear_damping.segment<3>(0) +
     p_.quadratic_damping.segment<3>(0).cwiseProduct(x.v.cwiseAbs()))
      .cwiseProduct(x.v);
  rhs.segment<3>(3) =
    tau.segment<3>(3) + p_.cb_position.cross(f_b) + a.cross(x.v) -
    b.cross(x.w) -
    (p_.linear_damping.segment<3>(3) +
     p_.quadratic_damping.segment<3>(3).cwiseProduct(x.w.cwiseAbs()))
      .cwiseProduct(x.w);

  // M 为对角 -> 逐元素除(免 LLT 分解)
  const Eigen::Matrix<double, 6, 1> nu_dot = minv_.cwiseProduct(rhs);

  // 姿态运动学:q_dot = 0.5 * q ⊗ [0, w](w 用步进前的值,与 sysid 复刻一致)
  const Eigen::Quaterniond wq(0.0, x.w.x(), x.w.y(), x.w.z());
  const Eigen::Quaterniond qd = x.q * wq;
  Eigen::Quaterniond q_next(
    x.q.w() + 0.5 * qd.w() * dt,
    x.q.x() + 0.5 * qd.x() * dt,
    x.q.y() + 0.5 * qd.y() * dt,
    x.q.z() + 0.5 * qd.z() * dt);
  q_next.normalize();

  // 位置积分用步进前的速度方向(显式欧拉,与 sysid python 复刻一致)
  const Eigen::Vector3d p_dot = x.q * x.v;
  x.v += nu_dot.segment<3>(0) * dt;
  x.w += nu_dot.segment<3>(3) * dt;
  x.p += p_dot * dt;
  x.q = q_next;
}

State DynamicsModel::predict(const State & x, const ControlVector & u,
                             double dt) const
{
  State s = x;
  eulerStep(s, thruster_.clamp(u), dt);
  return s;
}

void DynamicsModel::stepInPlace(State & x, const ControlVector & u,
                                double dt) const
{
  eulerStep(x, u, dt);  // solver 已保证传入的 u 先行截断
}

void DynamicsModel::rollout(const State & x0, const Eigen::MatrixXd & U,
                            double dt, Eigen::MatrixXd & out_states) const
{
  const int H = static_cast<int>(U.rows());
  out_states.resize(H + 1, 13);
  State s = x0;
  out_states.row(0) = s.toVector().transpose();
  const double sub_dt = dt;
  for (int h = 0; h < H; ++h) {
    eulerStep(s, thruster_.clamp(U.row(h).transpose()), sub_dt);
    out_states.row(h + 1) = s.toVector().transpose();
  }
}

}  // namespace auv_controller_mppi
