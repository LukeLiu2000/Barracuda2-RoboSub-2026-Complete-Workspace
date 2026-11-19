// -*- coding: utf-8 -*-
#pragma once
/**
 * mppi_controller_node 节点类(拆出头文件以便进程内集成测试)。
 *
 * mppi_controller_node —— MPPI 控制器 ROS 2 节点。
 * —— MPPI 控制器 ROS 2 节点。
 *
 * 输入:
 *   /auv/state/control        nav_msgs/msg/Odometry     当前状态(外部状态估计/真值)
 *   /auv/control/goal         geometry_msgs/PoseStamped 目标位姿(目标速度默认零)
 *   /auv/training/excitation_cmd std_msgs/Float64MultiArray (可选,训练激励旁路)
 * 输出:
 *   /auv/control/thruster_cmd std_msgs/Float64MultiArray 8 个推力 [N](t0..t7)
 *   /auv/control/predicted_path  nav_msgs/Path            名义预测轨迹(调试)
 *   /auv/control/optimal_sequence Float64MultiArray H*8   优化序列(调试)
 *   /auv/control/diagnostics  diagnostic_msgs/DiagnosticArray(1 Hz)
 * 服务:
 *   ~/estop std_srvs/srv/SetBool  true=紧急置零推力,false=恢复
 *
 * 线程模型:solver 线程(solver_hz,默认 10 Hz)+ 执行定时器(exec_hz,默认
 * 100 Hz)+ 诊断定时器(1 Hz)。状态/目标经互斥锁交换;SequenceExecutor
 * 负新旧序列切换、超时、限幅、激励旁路。
 *
 * 话题名固定,需要换源时用标准 remap,例如真值调试:
 *   ros2 launch auv_controller_MPPI controller.launch.py \
 *     state_topic:=/auv/ground_truth/odometry
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "auv_controller_mppi/dynamics_model.hpp"
#include "auv_controller_mppi/mppi_solver.hpp"
#include "auv_controller_mppi/sequence_executor.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace auv_controller_mppi
{
namespace
{
using namespace std::chrono_literals;
double nowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class MppiControllerNode : public rclcpp::Node
{
public:
  explicit MppiControllerNode(const rclcpp::NodeOptions & options)
  : Node("mppi_controller_node", options)
  {
    loadParameters();

    thruster_ = std::make_unique<ThrusterModel>(thruster_params_);
    {
      const auto c = thruster_->check();
      RCLCPP_INFO(get_logger(),
        "allocation matrix B: rank=%d cond=%.2f sv=[%.3f %.3f %.3f %.3f %.3f %.3f]",
        c.rank, c.condition_number, c.singular_values[0], c.singular_values[1],
        c.singular_values[2], c.singular_values[3], c.singular_values[4],
        c.singular_values[5]);
      if (!c.ok()) {
        RCLCPP_FATAL(get_logger(),
          "allocation matrix rank/units check FAILED —— 拒绝启动(不伪造可控性)");
        throw std::runtime_error("thruster allocation check failed");
      }
    }
    model_   = std::make_unique<DynamicsModel>(dyn_params_, thruster_params_);
    cost_    = std::make_unique<CostFunction>(cost_params_);
    cost_->setParams(cost_params_);
    solver_  = std::make_unique<MppiSolver>(*model_, *cost_, *thruster_, mppi_params_);
    executor_ = std::make_unique<SequenceExecutor>(*thruster_, exec_params_);

    // ---- 订阅 / 发布 ----
    const std::string state_topic = declare_parameter<std::string>(
      "topics.state", "/auv/state/control");
    const std::string goal_topic = declare_parameter<std::string>(
      "topics.goal", "/auv/control/goal");
    sub_state_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onState(msg); });
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { onGoal(msg); });
    sub_excitation_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/auv/training/excitation_cmd", 10,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() != kNumThrusters) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "excitation_cmd 需要 %d 元素,收到 %zu,忽略",
            kNumThrusters, msg->data.size());
          return;
        }
        ControlVector u;
        for (int i = 0; i < kNumThrusters; ++i) { u(i) = msg->data[i]; }
        executor_->setExcitation(u, nowSeconds());
      });

    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/auv/control/thruster_cmd", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
      "/auv/control/predicted_path", 10);
    pub_seq_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/auv/control/optimal_sequence", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/auv/control/diagnostics", 10);

    // ---- 轻量调试话题(发布控制器实际使用的 state / 实际接受的 goal)----
    // current_pose/current_state 仅在状态有效且有数据时发布(无效不发零);
    // state_debug 始终发布,带 validity/age/mode 字段。
    pub_cur_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/auv/control/current_pose", 10);
    pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/auv/control/goal_pose", 10);
    pub_cur_state_ = create_publisher<nav_msgs::msg::Odometry>(
      "/auv/control/current_state", 10);
    pub_state_debug_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/auv/control/state_debug", 10);

    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/estop",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        executor_->setZeroed(req->data);
        estop_ = req->data;
        res->success = true;
        res->message = req->data ? "estop engaged" : "estop released";
      });

    // ---- 线程与定时器 ----
    running_ = true;
    solver_thread_ = std::thread([this] { solverLoop(); });
    exec_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / exec_hz_),
      [this] { execTick(); });
    diag_timer_ = create_wall_timer(1s, [this] { diagTick(); });
    debug_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.5, debug_hz_)),
      [this] { debugTick(); });

    RCLCPP_INFO(get_logger(),
      "MPPI controller 启动:K=%d H=%d dt=%.2fs | solver %.1f Hz, exec %.1f Hz "
      "| 输入 %s -> 输出 /auv/control/thruster_cmd",
      mppi_params_.K, mppi_params_.H, mppi_params_.dt, solver_hz_, exec_hz_,
      state_topic.c_str());
  }

  ~MppiControllerNode() override
  {
    running_ = false;
    cv_.notify_all();
    if (solver_thread_.joinable()) { solver_thread_.join(); }
  }

private:
  // ============================ 参数装载 ============================
  void loadParameters()
  {
    auto get_double_vec = [this](const std::string & name,
                                 std::vector<double> def) {
      auto v = declare_parameter<std::vector<double>>(name, std::move(def));
      return v;
    };

    // thrusters.*
    const auto pos = get_double_vec("thrusters.positions", [] {
      std::vector<double> v;
      for (const auto & p : ThrusterParams().positions) {
        v.push_back(p.x()); v.push_back(p.y()); v.push_back(p.z());
      }
      return v;
    }());
    const auto dir = get_double_vec("thrusters.directions", [] {
      std::vector<double> v;
      for (const auto & d : ThrusterParams().directions) {
        v.push_back(d.x()); v.push_back(d.y()); v.push_back(d.z());
      }
      return v;
    }());
    if (pos.size() != 24 || dir.size() != 24) {
      throw std::runtime_error("thrusters.positions/directions 必须各 24 个数");
    }
    for (int i = 0; i < kNumThrusters; ++i) {
      thruster_params_.positions[i]  = Eigen::Vector3d(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]);
      thruster_params_.directions[i] = Eigen::Vector3d(dir[3 * i], dir[3 * i + 1], dir[3 * i + 2]);
    }
    thruster_params_.force_min = declare_parameter<double>(
      "thrusters.force_min", thruster_params_.force_min);
    thruster_params_.force_max = declare_parameter<double>(
      "thrusters.force_max", thruster_params_.force_max);
    com_offset_ = Eigen::Vector3d(
      get_double_vec("thrusters.com_offset_base_link",
                     {0.418208, 0.234982, 0.518423}).data());
    apply_com_offset_ = declare_parameter<bool>(
      "thrusters.apply_com_offset", true);

    // dynamics.*
    dyn_params_.mass = declare_parameter<double>("dynamics.mass", dyn_params_.mass);
    dyn_params_.inertia = Eigen::Vector3d(
      get_double_vec("dynamics.inertia", {4.23, 5.60, 3.17}).data());
    dyn_params_.added_mass = Eigen::Matrix<double, 6, 1>(
      get_double_vec("dynamics.added_mass", {12, 14, 10, 0.6, 0.8, 0.5}).data());
    dyn_params_.linear_damping = Eigen::Matrix<double, 6, 1>(
      get_double_vec("dynamics.linear_damping", {15, 18, 15, 1.0, 1.2, 1.0}).data());
    dyn_params_.quadratic_damping = Eigen::Matrix<double, 6, 1>(
      get_double_vec("dynamics.quadratic_damping", {45, 60, 60, 2, 3, 3}).data());
    dyn_params_.gravity_world = Eigen::Vector3d(
      get_double_vec("dynamics.gravity_world", {0, 0, -9.81}).data());
    dyn_params_.buoyancy_force = declare_parameter<double>(
      "dynamics.buoyancy_force", dyn_params_.buoyancy_force);
    dyn_params_.world_up = Eigen::Vector3d(
      get_double_vec("dynamics.world_up", {0, 0, 1}).data());
    dyn_params_.cb_position = Eigen::Vector3d(
      get_double_vec("dynamics.cb_position", {0, 0.02, 0}).data());
    dyn_params_.thrust_scale = declare_parameter<double>(
      "dynamics.thrust_scale", dyn_params_.thrust_scale);

    // mppi.*
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

    // cost.*
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
      get_double_vec("cost.workspace_min", {-3.0, -1.0, -3.0}).data());
    cost_params_.workspace_max = Eigen::Vector3d(
      get_double_vec("cost.workspace_max", {3.0, 5.0, 3.0}).data());
    cost_params_.u_max = thruster_params_.force_max;

    // rates / safety / debug
    solver_hz_ = declare_parameter<double>("rates.solver_hz", 10.0);
    exec_hz_   = declare_parameter<double>("rates.exec_hz", 100.0);
    state_timeout_ = declare_parameter<double>("safety.state_timeout_s", 0.5);
    goal_timeout_  = declare_parameter<double>("safety.goal_timeout_s", 10.0);
    solver_stall_  = declare_parameter<double>("safety.solver_stall_s", 2.0);
    publish_debug_ = declare_parameter<bool>("debug.publish_debug", true);
    // 轻量调试话题频率(current_pose/goal_pose/current_state/state_debug);
    // 只影响调试发布,不改变 solver/exec 内部频率。
    debug_hz_  = declare_parameter<double>("debug.state_debug_hz", 10.0);
  }

  // ============================ 回调 ============================
  void onState(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    State x;
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    x.p = Eigen::Vector3d(p.x, p.y, p.z);
    // 防御:零四元数(默认构造的 odom)直接归一化会产生 NaN
    Eigen::Quaterniond qm(q.w, q.x, q.y, q.z);
    x.q = qm.norm() > 1e-9 ? qm.normalized() : Eigen::Quaterniond::Identity();
    x.v = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);
    x.w = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                          msg->twist.twist.angular.z);

    // 里程计给的是 base_link 原点位姿;动力学体坐标系原点在质心:
    // p_com = p_bl + R * com_offset;v_com = v_bl + w × com_offset
    if (apply_com_offset_) {
      x.p += x.q.toRotationMatrix() * com_offset_;
      x.v += x.w.cross(com_offset_);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    state_ = x;
    state_stamp_ = nowSeconds();
    state_frame_ = msg->header.frame_id;
    cv_.notify_one();
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
  {
    Goal g;
    const auto & p = msg->pose.position;
    const auto & q = msg->pose.orientation;
    g.p = Eigen::Vector3d(p.x, p.y, p.z);
    Eigen::Quaterniond qm(q.w, q.x, q.y, q.z);
    g.q = qm.norm() > 1e-9 ? qm.normalized() : Eigen::Quaterniond::Identity();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!state_frame_.empty() && !msg->header.frame_id.empty() &&
          msg->header.frame_id != state_frame_)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "goal frame '%s' != state frame '%s' —— 拒绝该目标(本版不做 TF 变换)",
          msg->header.frame_id.c_str(), state_frame_.c_str());
        return;
      }
      goal_ = g;
      goal_stamp_ = nowSeconds();
    }
  }

  // ============================ 线程 ============================
  void solverLoop()
  {
    const double period = 1.0 / std::max(0.1, solver_hz_);
    while (running_) {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait_for(lock, std::chrono::duration<double>(period),
                   [this] { return !running_; });
      if (!running_) { break; }
      if (!state_ || !goal_) { continue; }
      const State x = *state_;
      const Goal g = *goal_;
      const double t_now = nowSeconds();
      const bool state_fresh = (t_now - state_stamp_) < state_timeout_;
      const bool goal_fresh = (t_now - goal_stamp_) < goal_timeout_;
      lock.unlock();

      if (!state_fresh || !goal_fresh) {
        // 状态/目标超时:不解算;执行器序列很快耗尽 -> STALE -> 零推力
        continue;
      }

      const MppiResult & r = solver_->solve(x, g);
      last_solve_ms_ = r.solve_ms;
      ++solve_count_;

      const double t0 = nowSeconds();
      ControlSequence seq(t0, mppi_params_.dt, r.U);
      executor_->setSequence(seq);
      last_seq_stamp_ = t0;

      if (publish_debug_ && pub_path_->get_subscription_count() > 0) {
        auto path = std::make_unique<nav_msgs::msg::Path>();
        path->header.frame_id = state_frame_.empty() ? "world" : state_frame_;
        path->header.stamp = now();
        for (int h = 0; h < r.states.rows(); ++h) {
          geometry_msgs::msg::PoseStamped ps;
          ps.header = path->header;
          ps.pose.position.x = r.states(h, 0);
          ps.pose.position.y = r.states(h, 1);
          ps.pose.position.z = r.states(h, 2);
          ps.pose.orientation.x = r.states(h, 3);
          ps.pose.orientation.y = r.states(h, 4);
          ps.pose.orientation.z = r.states(h, 5);
          ps.pose.orientation.w = r.states(h, 6);
          path->poses.push_back(ps);
        }
        pub_path_->publish(std::move(path));
      }
      if (publish_debug_ && pub_seq_->get_subscription_count() > 0) {
        auto m = std::make_unique<std_msgs::msg::Float64MultiArray>();
        m->data.reserve(static_cast<size_t>(r.U.size()));
        for (int h = 0; h < r.U.rows(); ++h) {
          for (int i = 0; i < kNumThrusters; ++i) { m->data.push_back(r.U(h, i)); }
        }
        pub_seq_->publish(std::move(m));
      }
    }
  }

  void execTick()
  {
    SequenceExecutor::Status st;
    const ControlVector u = executor_->command(nowSeconds(), st);
    auto m = std::make_unique<std_msgs::msg::Float64MultiArray>();
    m->data.resize(kNumThrusters);
    for (int i = 0; i < kNumThrusters; ++i) { m->data[i] = u(i); }
    pub_cmd_->publish(std::move(m));
    last_status_ = st;
    ++exec_count_;
  }

  void diagTick()
  {
    auto arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "auv_controller_MPPI";
    st.hardware_id = "barracuda2";

    const double t = nowSeconds();
    double state_age = -1, goal_age = -1;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_age = state_ ? (t - state_stamp_) : -1;
      goal_age = goal_ ? (t - goal_stamp_) : -1;
    }
    const bool seq_ok = (t - last_seq_stamp_) < solver_stall_;
    const char * st_str = "UNKNOWN";
    switch (last_status_) {
      case SequenceExecutor::Status::ACTIVE:    st_str = "ACTIVE"; break;
      case SequenceExecutor::Status::EXCITATION: st_str = "EXCITATION"; break;
      case SequenceExecutor::Status::STALE:     st_str = "STALE"; break;
      case SequenceExecutor::Status::ZEROED:    st_str = "ZEROED"; break;
    }
    st.level = (estop_ || !seq_ok) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
             : (last_status_ == SequenceExecutor::Status::ACTIVE ||
                last_status_ == SequenceExecutor::Status::EXCITATION)
               ? diagnostic_msgs::msg::DiagnosticStatus::OK
               : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    std::ostringstream oss;
    oss << "status=" << st_str << " estop=" << (estop_ ? 1 : 0)
        << " solve_ms=" << last_solve_ms_
        << " solver_count=" << solve_count_.load()
        << " exec_count=" << exec_count_.load()
        << " state_age_s=" << state_age << " goal_age_s=" << goal_age;
    st.message = oss.str();
    arr->status.push_back(st);
    pub_diag_->publish(std::move(arr));

    RCLCPP_INFO(get_logger(), "[diag] %s", st.message.c_str());
  }

  // ============================ 轻量调试 ============================
  // 发布控制器【实际使用】的 state(onState 里已做质心换算后存入 state_)
  // 与【实际接受】的 goal(通过 frame 校验、写入 goal_ 的那份)。
  // current_pose/current_state 只在状态存在且未超时时发布(无效不发零);
  // state_debug 始终发布,缺失字段留空字符串,不发零冒充数据。
  // 四元数为权威;欧拉角仅显示:体约定 roll 绕 z / pitch 绕 x / yaw 绕 y,
  // 单位度,pitch ±90° 奇异,禁止用于控制运算。
  static Eigen::Vector3d eulerDegBody(const Eigen::Quaterniond & q)
  {
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const double roll  = std::atan2(R(1, 0), R(1, 1));
    const double pitch = std::asin(std::clamp(-R(1, 2), -1.0, 1.0));
    const double yaw   = std::atan2(R(0, 2), R(2, 2));
    return {roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};
  }

  const char * execStatusStr() const
  {
    switch (last_status_) {
      case SequenceExecutor::Status::ACTIVE:    return "ACTIVE";
      case SequenceExecutor::Status::EXCITATION: return "EXCITATION";
      case SequenceExecutor::Status::STALE:     return "STALE";
      case SequenceExecutor::Status::ZEROED:    return "ZEROED";
    }
    return "UNKNOWN";
  }

  void debugTick()
  {
    std::optional<State> x;
    std::optional<Goal> g;
    double state_age = -1.0, goal_age = -1.0;
    std::string frame = state_frame_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      x = state_;
      g = goal_;
      const double t = nowSeconds();
      state_age = state_ ? (t - state_stamp_) : -1.0;
      goal_age  = goal_  ? (t - goal_stamp_)  : -1.0;
    }
    const bool state_valid = x.has_value() && state_age >= 0.0 &&
                             state_age <= state_timeout_;
    const bool goal_valid  = g.has_value() && goal_age >= 0.0 &&
                             goal_age <= goal_timeout_;
    const auto stamp = now();
    const std::string fr = frame.empty() ? "world" : frame;

    if (state_valid) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = fr;
      ps.pose.position.x = x->p.x();
      ps.pose.position.y = x->p.y();
      ps.pose.position.z = x->p.z();
      ps.pose.orientation.x = x->q.x();
      ps.pose.orientation.y = x->q.y();
      ps.pose.orientation.z = x->q.z();
      ps.pose.orientation.w = x->q.w();
      pub_cur_pose_->publish(ps);

      nav_msgs::msg::Odometry od;
      od.header = ps.header;
      od.child_frame_id = "auv_com";     // 体坐标系(质心原点)
      od.pose.pose = ps.pose;
      od.twist.twist.linear.x = x->v.x();
      od.twist.twist.linear.y = x->v.y();
      od.twist.twist.linear.z = x->v.z();
      od.twist.twist.angular.x = x->w.x();
      od.twist.twist.angular.y = x->w.y();
      od.twist.twist.angular.z = x->w.z();
      pub_cur_state_->publish(od);
    }
    if (g.has_value()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = fr;
      ps.pose.position.x = g->p.x();
      ps.pose.position.y = g->p.y();
      ps.pose.position.z = g->p.z();
      ps.pose.orientation.x = g->q.x();
      ps.pose.orientation.y = g->q.y();
      ps.pose.orientation.z = g->q.z();
      ps.pose.orientation.w = g->q.w();
      pub_goal_pose_->publish(ps);
    }

    // state_debug(字段化;空字符串 = 无数据,不是 0)
    auto arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "mppi_state_debug";
    st.hardware_id = "barracuda2";
    auto kv = [](const char * k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k; e.value = v;
        return e;
      };
    auto f3 = [](const Eigen::Vector3d & v) {
        std::ostringstream os;
        os << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
        return os.str();
      };
    std::ostringstream mode;
    if (estop_.load()) {mode << "estop";} else {mode << execStatusStr();}
    if (!state_valid) {mode << (x ? "|state_stale" : "|no_state");}
    if (!goal_valid) {mode << (g ? "|goal_stale" : "|no_goal");}
    st.level = (estop_.load() || !state_valid)
      ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
      : diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = mode.str();
    st.values.push_back(kv("state_valid", state_valid ? "1" : "0"));
    st.values.push_back(kv("state_age_s", x ? std::to_string(state_age) : ""));
    st.values.push_back(kv("goal_valid", goal_valid ? "1" : "0"));
    st.values.push_back(kv("goal_age_s", g ? std::to_string(goal_age) : ""));
    st.values.push_back(kv("frame_id", fr));
    st.values.push_back(kv("current_xyz_m", x ? f3(x->p) : ""));
    st.values.push_back(kv("current_rpy_deg", x ? f3(eulerDegBody(x->q)) : ""));
    st.values.push_back(kv("goal_xyz_m", g ? f3(g->p) : ""));
    st.values.push_back(kv("goal_rpy_deg", g ? f3(eulerDegBody(g->q)) : ""));
    if (x && g) {
      const double perr = (g->p - x->p).norm();
      const double d = std::abs(x->q.dot(g->q));
      const double oerr = 2.0 * std::acos(std::clamp(d, 0.0, 1.0)) * 180.0 / M_PI;
      st.values.push_back(kv("pos_err_norm_m", std::to_string(perr)));
      st.values.push_back(kv("ori_err_deg", std::to_string(oerr)));
    } else {
      st.values.push_back(kv("pos_err_norm_m", ""));
      st.values.push_back(kv("ori_err_deg", ""));
    }
    st.values.push_back(kv("mode", mode.str()));
    st.values.push_back(kv("estop", estop_.load() ? "1" : "0"));
    st.values.push_back(kv("last_solve_ms", std::to_string(last_solve_ms_.load())));
    st.values.push_back(kv("solve_count", std::to_string(solve_count_.load())));
    arr->status.push_back(st);
    pub_state_debug_->publish(std::move(arr));
  }

  // ============================ 成员 ============================
  ThrusterParams thruster_params_;
  DynamicsParams dyn_params_;
  CostParams cost_params_;
  MppiParams mppi_params_;
  SequenceExecutor::Params exec_params_;
  Eigen::Vector3d com_offset_{Eigen::Vector3d::Zero()};
  bool apply_com_offset_ = true;
  double solver_hz_ = 10.0, exec_hz_ = 100.0;
  double state_timeout_ = 0.5, goal_timeout_ = 10.0, solver_stall_ = 2.0;
  bool publish_debug_ = true;
  double debug_hz_ = 10.0;

  std::unique_ptr<ThrusterModel> thruster_;
  std::unique_ptr<DynamicsModel> model_;
  std::unique_ptr<CostFunction> cost_;
  std::unique_ptr<MppiSolver> solver_;
  std::unique_ptr<SequenceExecutor> executor_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<State> state_;
  std::optional<Goal> goal_;
  double state_stamp_ = -1e9, goal_stamp_ = -1e9;
  std::string state_frame_;

  std::atomic<bool> running_{false};
  std::atomic<bool> estop_{false};
  std::atomic<double> last_solve_ms_{0.0};
  std::atomic<uint64_t> solve_count_{0};
  std::atomic<uint64_t> exec_count_{0};
  double last_seq_stamp_ = -1e9;
  SequenceExecutor::Status last_status_ = SequenceExecutor::Status::STALE;

  std::thread solver_thread_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_state_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_excitation_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_seq_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cur_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_cur_state_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_state_debug_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr exec_timer_, diag_timer_, debug_timer_;
};

}  // namespace auv_controller_mppi
