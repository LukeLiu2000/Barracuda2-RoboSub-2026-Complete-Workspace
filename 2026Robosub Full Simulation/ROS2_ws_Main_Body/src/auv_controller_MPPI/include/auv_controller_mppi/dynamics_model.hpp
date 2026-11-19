// -*- coding: utf-8 -*-
/**
 * 六自由度 AUV 前向动力学模型(Fossen 汇总形式,体坐标系原点在质心):
 *
 *   M * nu_dot + C(nu) * nu + D(nu) * nu + g(eta) = B * T
 *
 *   eta = (p_w, q_wb),nu = (v_b, w_b)
 *   M  = M_RB + M_A(对角近似)
 *   C  = C_RB(ν) + C_A(ν)(skew 形式)
 *   D  = 线性 + 二次对角阻尼
 *   g  = 重力(作用于质心,无力臂)+ 浮力(作用于_cb_position_)
 *   B  :ThrusterModel 提供的 6x8 分配矩阵
 *
 * 积分:显式欧拉(可配子步);四元数运动学 q_dot = 0.5*q⊗[0,w],逐步归一化。
 * 所有未知水动力参数集中在一个 Params 结构里,支持系统辨识整体替换。
 */
#pragma once

#include "auv_controller_mppi/thruster_model.hpp"
#include "auv_controller_mppi/types.hpp"

namespace auv_controller_mppi
{

struct DynamicsParams
{
  // ---- 刚体(USD 实测:mass=40 kg 已 author;惯量为箱体公式估计,未标定)----
  double mass = 40.0;                       ///< [kg]
  Eigen::Vector3d inertia{4.23, 5.60, 3.17};///< [kg·m^2] Ixx,Iyy,Izz(箱体估计)

  // ---- 附加质量(对角,未标定,量级取车体质量/惯量的 25%~35%)----
  Eigen::Matrix<double, 6, 1> added_mass{
    12.0, 14.0, 10.0, 0.60, 0.80, 0.50};    ///< [kg]x3 + [kg·m^2]x3

  // ---- 水动力阻尼(对角,未标定)----
  Eigen::Matrix<double, 6, 1> linear_damping{
    15.0, 18.0, 15.0, 1.0, 1.2, 1.0};       ///< 线性项
  Eigen::Matrix<double, 6, 1> quadratic_damping{
    45.0, 60.0, 60.0, 2.0, 3.0, 3.0};       ///< 二次项 |nu|*nu

  // ---- 恢复力 ----
  Eigen::Vector3d gravity_world{0.0, 0.0, -9.81};  ///< 世界系重力加速度 [m/s^2]
  /// 浮力合力 [N],沿世界系竖直向上(0 表示不计浮力)。
  /// 默认值来自缩放后网格排水体积 0.03949 m^3 * rho*g(未标定)。
  double buoyancy_force = 387.4;
  /// 世界系竖直向上单位向量(必须与里程计世界系一致!ROS REP103 = z,
  /// Isaac Y-up 直通 = y;参数化避免写死)
  Eigen::Vector3d world_up{0.0, 0.0, 1.0};
  /// 浮心相对质心位置(体坐标系,米;假设值,未标定)
  Eigen::Vector3d cb_position{0.0, 0.02, 0.0};

  /// 推力全局缩放(辨识用;真实推进器推力曲线未知时的校准因子)
  double thrust_scale = 1.0;
};

class DynamicsModel
{
public:
  explicit DynamicsModel(const DynamicsParams & params = DynamicsParams(),
                         const ThrusterParams & thruster = ThrusterParams());

  /// 单步预测:next_state = predict(x, T, dt)(dt 内部可再分子步)
  State predict(const State & x, const ControlVector & u, double dt) const;

  /// 原地单步(solver 热路径用,避免按值拷贝 13 维状态)
  void stepInPlace(State & x, const ControlVector & u, double dt) const;

  /// 多步滚动(MPPI rollout 用);out_states 需预留 H 行,每行 13 列
  void rollout(const State & x0, const Eigen::MatrixXd & U, double dt,
               Eigen::MatrixXd & out_states) const;

  /// 当前体坐标系广义恢复力 g(eta) = [f_g+f_b ; r_cb × f_b]
  WrenchVector restoringWrench(const State & x) const;

  /// 广义质量矩阵 M = diag(m+ma_v, I+ma_w)(6x6 对角)
  Eigen::Matrix<double, 6, 6> massMatrix() const;

  /// 科氏项 C(nu)(6x6,RB + A,skew 形式)
  Eigen::Matrix<double, 6, 6> coriolis(const Eigen::Vector3d & v,
                                       const Eigen::Vector3d & w) const;

  /// 阻尼项 D(nu)(6x6 对角:线性 + 二次)
  Eigen::Matrix<double, 6, 6> damping(const Eigen::Vector3d & v,
                                      const Eigen::Vector3d & w) const;

  const DynamicsParams & params() const { return p_; }
  void setParams(const DynamicsParams & params);
  const ThrusterModel & thruster() const { return thruster_; }

private:
  /// 一步欧拉(公开接口做子步循环)
  void eulerStep(State & x, const ControlVector & u, double dt) const;
  /// 由 p_ 重建热路径缓存(对角质量逆、组合惯量、世界系恢复力向量)
  void updateCache();

  DynamicsParams p_;
  ThrusterModel thruster_;
  // ---- 热路径缓存(eulerStep 不再组装 6x6 矩阵)----
  Eigen::Matrix<double, 6, 1> minv_{Eigen::Matrix<double, 6, 1>::Ones()};
  Eigen::Vector3d m_plus_mav_{Eigen::Vector3d::Ones()};   ///< m + ma_v(逐轴)
  Eigen::Vector3d i_plus_maw_{Eigen::Vector3d::Ones()};   ///< I + ma_w(逐轴)
  Eigen::Vector3d gravity_force_world_{Eigen::Vector3d::Zero()};  ///< m*g
  Eigen::Vector3d buoy_force_world_{Eigen::Vector3d::Zero()};     ///< B*up
};

}  // namespace auv_controller_mppi
