// -*- coding: utf-8 -*-
/**
 * basic_mission_test —— 第十二章"基础控制测试"(自包含闭环,无 ROS 通信)。
 *
 * 场景:机器人静止起航,前进 1 m(体 +z,初始姿态下 = 世界 +z),
 * 同时绕竖直轴旋转 30°,最终停在目标位姿。
 * 世界系约定:初始时刻体坐标系 = 世界坐标系(USD stage 同款 y-up);
 * 隔离动力学仿真 5 ms 欧拉步;solver 按 solver_hz 周期解算,
 * 执行器按 exec_hz 从序列取控制 —— 与 ROS 链路同一套
 * MppiSolver / SequenceExecutor / ThrusterModel 代码。
 *
 * 输出:stdout 打印 "RESULT:" 行(最终误差/求解耗时/频率),并写
 * basic_mission.csv(t, p, q, v, w, u0..u7, solve_ms)到 basic_mission.output_dir。
 * 运行:ros2 launch auv_controller_MPPI basic_mission.launch.py
 */
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "auv_controller_mppi/mppi_solver.hpp"
#include "auv_controller_mppi/sequence_executor.hpp"

#include <rclcpp/rclcpp.hpp>

namespace auv_controller_mppi
{
namespace
{
double deg2rad(double d) { return d * M_PI / 180.0; }
}  // namespace

class BasicMissionNode : public rclcpp::Node
{
public:
  explicit BasicMissionNode(const rclcpp::NodeOptions & options)
  : Node("basic_mission_test", options)
  {
    loadParams();
    thruster_ = std::make_unique<ThrusterModel>(thruster_params_);
    model_   = std::make_unique<DynamicsModel>(dyn_params_, thruster_params_);
    cost_    = std::make_unique<CostFunction>(cost_params_);
    solver_  = std::make_unique<MppiSolver>(*model_, *cost_, *thruster_, mppi_params_);
    executor_ = std::make_unique<SequenceExecutor>(*thruster_);

    // 目标:前进 1 m + 竖直轴(y)旋转 30°
    goal_.p = Eigen::Vector3d(0.0, 0.0, forward_m_);
    goal_.q = Eigen::AngleAxisd(deg2rad(yaw_deg_), Eigen::Vector3d::UnitY())
                * Eigen::Quaterniond::Identity();
  }

  int run()
  {
    const double sim_dt = 0.005;
    const double solve_period = 1.0 / solver_hz_;
    double t = 0.0;
    double next_solve_t = 0.0;
    State x;  // 静止,原点,单位姿态
    x.q = Eigen::Quaterniond::Identity();

    std::ofstream csv(output_dir_ + "/basic_mission.csv");
    csv << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,"
           "u0,u1,u2,u3,u4,u5,u6,u7,solve_ms\n";

    double solve_ms_sum = 0, solve_ms_max = 0;
    int solve_count = 0;
    double arrived_since = -1;
    const double t_start_wall = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

    RCLCPP_INFO(get_logger(),
      "基础任务开始:前进 %.2f m + 偏航 %.1f°,K=%d H=%d dt=%.2f solver=%.0fHz",
      forward_m_, yaw_deg_, mppi_params_.K, mppi_params_.H, mppi_params_.dt,
      solver_hz_);

    while (t < timeout_s_ && rclcpp::ok()) {
      // solver 周期
      if (t >= next_solve_t) {
        const auto & r = solver_->solve(x, goal_);
        executor_->setSequence(ControlSequence(t, mppi_params_.dt, r.U));
        solve_ms_sum += r.solve_ms;
        solve_ms_max = std::max(solve_ms_max, r.solve_ms);
        ++solve_count;
        last_solve_ms_ = r.solve_ms;
        next_solve_t += solve_period;
      }
      // 执行 + 仿真一步
      SequenceExecutor::Status st;
      const ControlVector u = executor_->command(t, st);
      model_->stepInPlace(x, u, sim_dt);
      t += sim_dt;

      // 到达判定(保持 hold_s)
      const double pe = (x.p - goal_.p).norm();
      const double ae = orientationError(x.q, goal_.q);
      const bool in_tol = pe < pos_tol_ && ae < ang_tol_ &&
                          x.v.norm() < v_tol_ && x.w.norm() < w_tol_;
      if (in_tol && arrived_since < 0) { arrived_since = t; }
      if (!in_tol) { arrived_since = -1; }

      const auto xv = x.toVector();
      csv << std::fixed << std::setprecision(6) << t;
      for (int i = 0; i < 13; ++i) { csv << ',' << xv(i); }
      for (int i = 0; i < 8; ++i) { csv << ',' << u(i); }
      csv << ',' << last_solve_ms_ << '\n';
      if (solve_count % 10 == 0) { csv.flush(); }

      if (arrived_since > 0 && (t - arrived_since) >= hold_s_) { break; }
    }

    const double wall_used = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count() - t_start_wall;
    const double pe = (x.p - goal_.p).norm();
    const double ae = orientationError(x.q, goal_.q);
    const bool arrived = arrived_since > 0;

    RCLCPP_INFO(get_logger(), "RESULT:arrived=%d", arrived ? 1 : 0);
    RCLCPP_INFO(get_logger(), "RESULT:final_pos_error_m=%.4f", pe);
    RCLCPP_INFO(get_logger(), "RESULT:final_ang_error_deg=%.2f",
      ae * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "RESULT:final_lin_vel=%.4f", x.v.norm());
    RCLCPP_INFO(get_logger(), "RESULT:final_ang_vel=%.4f", x.w.norm());
    RCLCPP_INFO(get_logger(), "RESULT:solve_ms_mean=%.2f",
      solve_count ? solve_ms_sum / solve_count : 0.0);
    RCLCPP_INFO(get_logger(), "RESULT:solve_ms_max=%.2f", solve_ms_max);
    RCLCPP_INFO(get_logger(), "RESULT:capable_solver_hz=%.1f",
      solve_ms_max > 0 ? 1000.0 / solve_ms_max : 0.0);
    RCLCPP_INFO(get_logger(), "RESULT:virtual_mission_time_s=%.2f", t);
    RCLCPP_INFO(get_logger(), "RESULT:wall_time_s=%.2f", wall_used);
    csv.close();
    return arrived ? 0 : 1;
  }

private:
  void loadParams()
  {
    output_dir_ = declare_parameter<std::string>(
      "basic_mission.output_dir", "/tmp");
    forward_m_ = declare_parameter<double>("basic_mission.forward_m", 1.0);
    yaw_deg_ = declare_parameter<double>("basic_mission.yaw_deg", 30.0);
    pos_tol_ = declare_parameter<double>("basic_mission.pos_tol", 0.10);
    ang_tol_ = declare_parameter<double>("basic_mission.ang_tol", 0.087);
    v_tol_ = declare_parameter<double>("basic_mission.v_tol", 0.10);
    w_tol_ = declare_parameter<double>("basic_mission.w_tol", 0.15);
    hold_s_ = declare_parameter<double>("basic_mission.hold_s", 1.0);
    timeout_s_ = declare_parameter<double>("basic_mission.timeout_s", 60.0);
    solver_hz_ = declare_parameter<double>("rates.solver_hz", 10.0);

    auto dvec = [this](const char * n, std::vector<double> def) {
      return declare_parameter<std::vector<double>>(n, std::move(def));
    };

    auto pos = dvec("thrusters.positions", [] {
      std::vector<double> v;
      for (const auto & p : ThrusterParams().positions) {
        v.push_back(p.x()); v.push_back(p.y()); v.push_back(p.z());
      }
      return v;
    }());
    auto dir = dvec("thrusters.directions", [] {
      std::vector<double> v;
      for (const auto & d : ThrusterParams().directions) {
        v.push_back(d.x()); v.push_back(d.y()); v.push_back(d.z());
      }
      return v;
    }());
    for (int i = 0; i < kNumThrusters; ++i) {
      thruster_params_.positions[i] =
        Eigen::Vector3d(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]);
      thruster_params_.directions[i] =
        Eigen::Vector3d(dir[3 * i], dir[3 * i + 1], dir[3 * i + 2]);
    }
    thruster_params_.force_min = declare_parameter<double>(
      "thrusters.force_min", thruster_params_.force_min);
    thruster_params_.force_max = declare_parameter<double>(
      "thrusters.force_max", thruster_params_.force_max);

    dyn_params_.mass = declare_parameter<double>("dynamics.mass", 40.0);
    dyn_params_.inertia = Eigen::Vector3d(
      dvec("dynamics.inertia", {4.23, 5.60, 3.17}).data());
    dyn_params_.added_mass = Eigen::Matrix<double, 6, 1>(
      dvec("dynamics.added_mass", {12, 14, 10, 0.6, 0.8, 0.5}).data());
    dyn_params_.linear_damping = Eigen::Matrix<double, 6, 1>(
      dvec("dynamics.linear_damping", {15, 18, 15, 1.0, 1.2, 1.0}).data());
    dyn_params_.quadratic_damping = Eigen::Matrix<double, 6, 1>(
      dvec("dynamics.quadratic_damping", {45, 60, 60, 2, 3, 3}).data());
    dyn_params_.gravity_world = Eigen::Vector3d(
      dvec("dynamics.gravity_world", {0, 0, -9.81}).data());
    dyn_params_.buoyancy_force = declare_parameter<double>(
      "dynamics.buoyancy_force", 387.4);
    // 自包含仿真:世界系 = 初始体坐标系(y-up,与 USD stage 一致)
    dyn_params_.world_up = Eigen::Vector3d(
      dvec("dynamics.world_up", {0, 1, 0}).data());
    dyn_params_.cb_position = Eigen::Vector3d(
      dvec("dynamics.cb_position", {0, 0.02, 0}).data());

    mppi_params_.K = declare_parameter<int>("mppi.K", mppi_params_.K);
    mppi_params_.H = declare_parameter<int>("mppi.H", mppi_params_.H);
    mppi_params_.dt = declare_parameter<double>("mppi.dt", mppi_params_.dt);
    mppi_params_.sigma = declare_parameter<double>("mppi.sigma", mppi_params_.sigma);
    mppi_params_.lambda = declare_parameter<double>("mppi.lambda", mppi_params_.lambda);
    mppi_params_.alpha = declare_parameter<double>("mppi.alpha", mppi_params_.alpha);
    mppi_params_.shift_fill = declare_parameter<double>(
      "mppi.shift_fill", mppi_params_.shift_fill);
    mppi_params_.seed = static_cast<uint64_t>(
      declare_parameter<int>("mppi.seed", static_cast<int>(mppi_params_.seed)));
    mppi_params_.substeps = declare_parameter<int>("mppi.substeps", mppi_params_.substeps);

    cost_params_.position_weight = declare_parameter<double>(
      "cost.position_weight", cost_params_.position_weight);
    cost_params_.orientation_weight = declare_parameter<double>(
      "cost.orientation_weight", cost_params_.orientation_weight);
    cost_params_.lin_vel_weight = declare_parameter<double>(
      "cost.lin_vel_weight", cost_params_.lin_vel_weight);
    cost_params_.ang_vel_weight = declare_parameter<double>(
      "cost.ang_vel_weight", cost_params_.ang_vel_weight);
    cost_params_.effort_weight = declare_parameter<double>(
      "cost.effort_weight", cost_params_.effort_weight);
    cost_params_.smoothness_weight = declare_parameter<double>(
      "cost.smoothness_weight", cost_params_.smoothness_weight);
    cost_params_.saturation_weight = declare_parameter<double>(
      "cost.saturation_weight", cost_params_.saturation_weight);
    cost_params_.safety_weight = declare_parameter<double>(
      "cost.safety_weight", cost_params_.safety_weight);
    cost_params_.terminal_multiplier = declare_parameter<double>(
      "cost.terminal_multiplier", cost_params_.terminal_multiplier);
    cost_params_.workspace_min = Eigen::Vector3d(
      dvec("cost.workspace_min", {-3.0, -1.0, -3.0}).data());
    cost_params_.workspace_max = Eigen::Vector3d(
      dvec("cost.workspace_max", {3.0, 5.0, 3.0}).data());
    cost_params_.u_max = thruster_params_.force_max;
  }

  std::string output_dir_;
  double forward_m_ = 1.0, yaw_deg_ = 30.0;
  double pos_tol_ = 0.10, ang_tol_ = 0.087, v_tol_ = 0.10, w_tol_ = 0.15;
  double hold_s_ = 1.0, timeout_s_ = 60.0, solver_hz_ = 10.0;
  double last_solve_ms_ = 0.0;
  ThrusterParams thruster_params_;
  DynamicsParams dyn_params_;
  CostParams cost_params_;
  MppiParams mppi_params_;
  Goal goal_;
  std::unique_ptr<ThrusterModel> thruster_;
  std::unique_ptr<DynamicsModel> model_;
  std::unique_ptr<CostFunction> cost_;
  std::unique_ptr<MppiSolver> solver_;
  std::unique_ptr<SequenceExecutor> executor_;
};

}  // namespace auv_controller_mppi

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_controller_mppi::BasicMissionNode>(
    rclcpp::NodeOptions());
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
