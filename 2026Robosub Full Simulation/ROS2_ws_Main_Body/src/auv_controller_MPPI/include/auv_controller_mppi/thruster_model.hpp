// -*- coding: utf-8 -*-
/**
 * 八推进器分布模型 + 6x8 推进器分配矩阵 B。
 *
 * 几何来源(全部细节见 docs/thruster_geometry.md 与 config/thrusters.yaml):
 *   - USD 原始 translate(mm)经统一缩放 s = 1/0.6975 = 1.433692
 *     (艇体实测长 0.6975 m,按任务书假设拉到 1.0 m,三轴同比缩放);
 *   - 质心取碰撞网格体积质心(USD 未 author COM,已明确标记为计算值);
 *   - 推力方向 = 各推进器局部 +X 轴在 base_link 系的方向
 *     【重要假设,未经实机/仿真驱动确认,待验证项见 yaml 注释】。
 *
 * 分配关系:tau = B * T,tau = [Fx,Fy,Fz,Mx,My,Mz](体坐标系,N / N·m),
 * 第 i 列 = [d_i ; r_i × d_i],d_i 单位推力方向,r_i 相对质心力臂。
 */
#pragma once

#include <array>
#include <string>
#include <vector>

#include "auv_controller_mppi/types.hpp"

namespace auv_controller_mppi
{

struct ThrusterParams
{
  /// 各推进器相对质心位置(体坐标系,米)
  std::array<Eigen::Vector3d, kNumThrusters> positions{
    Eigen::Vector3d(-0.3021, -0.0916, +0.4321),  // thruster_0
    Eigen::Vector3d(+0.3130, -0.0916, +0.4321),  // thruster_1
    Eigen::Vector3d(+0.3130, -0.0916, -0.4109),  // thruster_2
    Eigen::Vector3d(-0.3021, -0.0916, -0.4109),  // thruster_3
    Eigen::Vector3d(-0.3494, +0.0374, -0.1700),  // thruster_4
    Eigen::Vector3d(-0.3494, +0.0374, +0.1956),  // thruster_5
    Eigen::Vector3d(+0.3416, +0.0374, +0.1956),  // thruster_6
    Eigen::Vector3d(+0.3416, +0.0374, -0.1700),  // thruster_7
  };
  /// 各推进器单位推力方向(体坐标系)——假设:沿各自局部 +X
  std::array<Eigen::Vector3d, kNumThrusters> directions{
    Eigen::Vector3d(+M_SQRT1_2, 0.0, +M_SQRT1_2),  // thruster_0
    Eigen::Vector3d(-M_SQRT1_2, 0.0, +M_SQRT1_2),  // thruster_1
    Eigen::Vector3d(-M_SQRT1_2, 0.0, -M_SQRT1_2),  // thruster_2
    Eigen::Vector3d(+M_SQRT1_2, 0.0, -M_SQRT1_2),  // thruster_3
    Eigen::Vector3d(0.0, 1.0, 0.0),                // thruster_4
    Eigen::Vector3d(0.0, 1.0, 0.0),                // thruster_5
    Eigen::Vector3d(0.0, 1.0, 0.0),                // thruster_6
    Eigen::Vector3d(0.0, 1.0, 0.0),                // thruster_7
  };
  /// 推力上下限 [N](4000 gf = 39.2266 N,正反对称,待标定)
  double force_min = -39.2266;
  double force_max = +39.2266;
};

/// 分配矩阵健康检查结果
struct AllocationCheck
{
  int rank = 0;                            ///< B 的秩(要求 6)
  std::array<double, 6> singular_values{}; ///< 奇异值(降序)
  double condition_number = 0.0;
  bool directions_unit = true;             ///< 所有方向向量是否为单位向量
  bool ok() const { return rank == kNumWrench && directions_unit; }
};

class ThrusterModel
{
public:
  explicit ThrusterModel(const ThrusterParams & params = ThrusterParams());

  /// 6x8 分配矩阵 tau = B * T
  const AllocationMatrix & allocation() const { return B_; }

  /// 单推进器单位推力的广义力列(测试用)
  WrenchVector column(int i) const { return B_.col(i); }

  /// 推力饱和限幅
  ControlVector clamp(const ControlVector & u) const;

  /// 秩 / 奇异值 / 单位方向检查
  AllocationCheck check() const;

  const ThrusterParams & params() const { return params_; }

  /// 广义力 -> 最小范数推力分配(训练激励与调试用;实时控制走 MPPI)
  ControlVector distribute(const WrenchVector & tau) const;

  /// 供 ROS 参数装载用(节点代码负责把参数塞进来)
  void setParams(const ThrusterParams & params);

private:
  ThrusterParams params_;
  AllocationMatrix B_{AllocationMatrix::Zero()};
};

}  // namespace auv_controller_mppi
