// -*- coding: utf-8 -*-
/**
 * 公共数据类型:机器人状态 / 目标状态 / 控制向量。
 *
 * 状态约定(与任务书一致,13 维):
 *   [px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]
 *   - p:世界系(=里程计 frame)位置
 *   - q:世界->体坐标系旋转四元数(Hamilton,Eigen coeffs 顺序 x,y,z,w)
 *   - v:体坐标系线速度
 *   - w:体坐标系角速度
 *
 * 体坐标系约定(来自 USD base_link,已在 docs/thruster_geometry.md 论证):
 *   原点在(假设)质心,轴平行于 base_link:
 *   +x = 一侧水平方向(port),+y = 竖直向上,+z = 艇体纵轴(前向)。
 *   即:roll 绕 z、pitch 绕 x、yaw 绕 y。
 *   注意这与 REP 103(x 前 z 上)不同 —— 因为 Isaac 的 base_link 本来就是
 *   z 前 y 上;里程计的 twist 就是在 base_link 系给出的,控制器沿用同一
 *   坐标系可以避免任何换轴错误。
 */
#pragma once

#include <Eigen/Dense>

namespace auv_controller_mppi
{

/// 推进器个数(固定 8)
constexpr int kNumThrusters = 8;
/// 广义力维数(6:三力 + 三力矩)
constexpr int kNumWrench = 6;

/// 8 维控制向量:各推进器推力 [N],顺序 thruster_0..thruster_7
using ControlVector = Eigen::Matrix<double, kNumThrusters, 1>;
/// 6 维广义力 [Fx,Fy,Fz,Mx,My,Mz](体坐标系)
using WrenchVector  = Eigen::Matrix<double, kNumWrench, 1>;
/// 6x8 推进器分配矩阵
using AllocationMatrix = Eigen::Matrix<double, kNumWrench, kNumThrusters>;

/// 机器人状态(13 维)
struct State
{
  Eigen::Vector3d    p{Eigen::Vector3d::Zero()};  ///< 世界系位置 [m]
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};  ///< 世界->体
  Eigen::Vector3d    v{Eigen::Vector3d::Zero()};  ///< 体坐标系线速度 [m/s]
  Eigen::Vector3d    w{Eigen::Vector3d::Zero()};  ///< 体坐标系角速度 [rad/s]

  /// 打包成任务书规定的 13 维向量
  Eigen::Matrix<double, 13, 1> toVector() const
  {
    Eigen::Matrix<double, 13, 1> x;
    x.segment<3>(0)  = p;
    x.segment<4>(3)  = q.coeffs();   // x, y, z, w
    x.segment<3>(7)  = v;
    x.segment<3>(10) = w;
    return x;
  }
  static State fromVector(const Eigen::Matrix<double, 13, 1> & x)
  {
    State s;
    s.p = x.segment<3>(0);
    s.q = Eigen::Quaterniond(x(6), x(3), x(4), x(5));  // (w,x,y,z)
    s.q.normalize();
    s.v = x.segment<3>(7);
    s.w = x.segment<3>(10);
    return s;
  }
};

/// 目标状态(默认目标速度为零 = 到点停住)
struct Goal
{
  Eigen::Vector3d    p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d    v{Eigen::Vector3d::Zero()};  ///< 体坐标系目标线速度
  Eigen::Vector3d    w{Eigen::Vector3d::Zero()};  ///< 体坐标系目标角速度
};

/// 姿态误差角 [rad](处理双重覆盖:q 与 -q 表示同一姿态)
inline double orientationError(const Eigen::Quaterniond & qa,
                               const Eigen::Quaterniond & qb)
{
  Eigen::Quaterniond qe = qa.inverse() * qb;
  if (qe.w() < 0.0) { qe.coeffs() = -qe.coeffs(); }
  return 2.0 * std::atan2(qe.vec().norm(), std::abs(qe.w()));
}

}  // namespace auv_controller_mppi
