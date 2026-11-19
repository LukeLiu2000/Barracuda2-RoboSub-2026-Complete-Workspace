// -*- coding: utf-8 -*-
/**
 * training_mission_node —— 训练任务生成节点。
 *
 * 两种模式(training.mode):
 *  1) random_goals:在可配置工作空间内随机生成目标位姿(相对当前状态限幅),
 *     监测到达(位置/角度容差 + 保持时间)或超时,然后生成下一目标。
 *  2) excitation:推进器激励实验序列(提高辨识数据可辨识性):
 *     在广义力空间(step/chirp/rand 波形)或直接推进器空间施加,
 *     经 B 伪逆分配为 8 推力,经 /auv/training/excitation_cmd 发布,
 *     由 mppi_controller_node 旁路执行(单写者原则)。
 *
 * 发布:
 *   /auv/control/goal           geometry_msgs/PoseStamped(目标)
 *   /auv/training/experiment_id std_msgs/String(当前实验编号,collector 记录用)
 *   /auv/training/excitation_cmd std_msgs/Float64MultiArray(仅 excitation 模式)
 * 订阅:
 *   /auv/state/control          nav_msgs/Odometry(到达检测;参数可改)
 */
#include <cmath>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "auv_controller_mppi/thruster_model.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace auv_controller_mppi
{
namespace
{
double deg2rad(double d) { return d * M_PI / 180.0; }
/// 受限随机姿态:相对参考姿态,在【体坐标系】三个轴各转一个受限随机角。
/// 轴向必须符合机体定义(与控制器/桥一致):
///   roll  绕体 z(艇体纵轴/前进方向)
///   pitch 绕体 x(左舷方向)
///   yaw   绕体 y(竖直向上)
/// 右乘 = 在体坐标系局部施加旋转。(旧实现把 max_roll 放在 UnitX、
/// max_pitch 放在 UnitY、max_yaw 放在 UnitZ,与体定义不符,已修正。)
Eigen::Quaterniond randomOrientation(std::mt19937 & rng, double max_roll,
                                     double max_pitch, double max_yaw,
                                     const Eigen::Quaterniond & q_ref)
{
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const Eigen::AngleAxisd r_pitch(u(rng) * max_pitch, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd r_yaw(u(rng) * max_yaw, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd r_roll(u(rng) * max_roll, Eigen::Vector3d::UnitZ());
  return (q_ref * r_pitch * r_yaw * r_roll).normalized();
}
}  // namespace

class TrainingMissionNode : public rclcpp::Node
{
public:
  explicit TrainingMissionNode(const rclcpp::NodeOptions & options)
  : Node("training_mission_node", options)
  {
    loadParams();
    model_ = std::make_unique<ThrusterModel>(thruster_params_);

    const std::string state_topic = declare_parameter<std::string>(
      "training.state_topic", "/auv/state/control");
    sub_state_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        have_state_ = true;
        if (!m->header.frame_id.empty()) { state_frame_ = m->header.frame_id; }
        cur_p_ = Eigen::Vector3d(m->pose.pose.position.x, m->pose.pose.position.y,
                                 m->pose.pose.position.z);
        cur_q_ = Eigen::Quaterniond(m->pose.pose.orientation.w,
                                    m->pose.pose.orientation.x,
                                    m->pose.pose.orientation.y,
                                    m->pose.pose.orientation.z).normalized();
        last_state_t_ = now().seconds();
      });
    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("training.goal_topic", "/auv/control/goal"), 10);
    pub_exp_id_ = create_publisher<std_msgs::msg::String>(
      "/auv/training/experiment_id", 10);
    pub_excite_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/auv/training/excitation_cmd", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
      [this] { tick(); });

    RCLCPP_INFO(get_logger(), "training_mission_node 启动,mode=%s", mode_.c_str());
  }

private:
  void loadParams()
  {
    mode_ = declare_parameter<std::string>("training.mode", "random_goals");
    rate_hz_ = declare_parameter<double>("training.rate_hz", 20.0);
    goal_frame_default_ = declare_parameter<std::string>(
      "training.goal_frame", std::string("world"));
    auto vec3 = [this](const char * n, double a, double b, double c) {
      const auto v = declare_parameter<std::vector<double>>(
        n, std::vector<double>{a, b, c});
      return Eigen::Vector3d(v[0], v[1], v[2]);
    };
    ws_min_ = vec3("training.workspace_min", -2.0, 0.0, -2.0);
    ws_max_ = vec3("training.workspace_max", 2.0, 4.0, 2.0);
    max_disp_ = declare_parameter<double>("training.max_displacement", 1.5);
    max_roll_ = deg2rad(declare_parameter<double>("training.max_roll_deg", 15.0));
    max_pitch_ = deg2rad(declare_parameter<double>("training.max_pitch_deg", 15.0));
    max_yaw_ = deg2rad(declare_parameter<double>("training.max_yaw_deg", 30.0));
    pos_tol_ = declare_parameter<double>("training.pos_tol", 0.15);
    ang_tol_ = deg2rad(declare_parameter<double>("training.ang_tol_deg", 8.0));
    hold_s_ = declare_parameter<double>("training.hold_s", 1.5);
    goal_timeout_s_ = declare_parameter<double>("training.goal_timeout_s", 40.0);
    num_goals_ = declare_parameter<int>("training.num_goals", 0);
    // 目标重发频率:控制器的 safety.goal_timeout_s(默认 10 s)内目标必须
    // 刷新,否则判定过期停输出。旧实现只在生成时发一次,10 s 后必然超时。
    goal_republish_hz_ = declare_parameter<double>("training.goal_republish_hz", 2.0);

    // excitation 模式参数
    // ROS 2 整数数组参数类型是 int64_t,需转换后存 int
    const auto axes64 = declare_parameter<std::vector<int64_t>>(
      "training.excitation_axes", std::vector<int64_t>{2});   // 默认:纯 surge 力
    excite_axes_.assign(axes64.begin(), axes64.end());
    excite_amp_ = declare_parameter<double>("training.excitation_amplitude", 20.0);
    excite_profile_ = declare_parameter<std::string>(
      "training.excitation_profile", "chirp");
    excite_duration_s_ = declare_parameter<double>("training.excitation_duration_s", 15.0);
    excite_rest_s_ = declare_parameter<double>("training.excitation_rest_s", 5.0);
    excite_period_s_ = declare_parameter<double>("training.excitation_period_s", 4.0);
    excite_reps_ = declare_parameter<int>("training.excitation_repetitions", 8);

    auto pos = declare_parameter<std::vector<double>>(
      "thrusters.positions", std::vector<double>(24, 0.0));
    auto dir = declare_parameter<std::vector<double>>(
      "thrusters.directions", std::vector<double>(24, 0.0));
    if (pos.size() == 24 && dir.size() == 24) {
      for (int i = 0; i < kNumThrusters; ++i) {
        thruster_params_.positions[i] =
          Eigen::Vector3d(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]);
        thruster_params_.directions[i] =
          Eigen::Vector3d(dir[3 * i], dir[3 * i + 1], dir[3 * i + 2]);
      }
    }
    thruster_params_.force_min = declare_parameter<double>(
      "thrusters.force_min", thruster_params_.force_min);
    thruster_params_.force_max = declare_parameter<double>(
      "thrusters.force_max", thruster_params_.force_max);
  }

  std::string newExperimentId(const char * kind)
  {
    ++exp_seq_;
    std::ostringstream oss;
    oss << kind << "_" << std::setw(4) << std::setfill('0') << exp_seq_;
    std_msgs::msg::String m;
    m.data = oss.str();
    pub_exp_id_->publish(m);
    return oss.str();
  }

  void publishGoal(const Eigen::Vector3d & p, const Eigen::Quaterniond & q)
  {
    geometry_msgs::msg::PoseStamped g;
    g.header.stamp = now();
    // 目标坐标系 = 状态里程计坐标系(避免控制器拒收)
    g.header.frame_id = state_frame_.empty() ? goal_frame_default_ : state_frame_;
    g.pose.position.x = p.x(); g.pose.position.y = p.y(); g.pose.position.z = p.z();
    g.pose.orientation.x = q.x(); g.pose.orientation.y = q.y();
    g.pose.orientation.z = q.z(); g.pose.orientation.w = q.w();
    pub_goal_->publish(g);
  }

  void publishExcitation(const WrenchVector & tau)
  {
    const ControlVector u = model_->distribute(tau);
    std_msgs::msg::Float64MultiArray m;
    m.data.resize(kNumThrusters);
    for (int i = 0; i < kNumThrusters; ++i) { m.data[i] = u(i); }
    pub_excite_->publish(m);
  }

  // ================= random_goals 状态机 =================
  enum class GPhase { WAIT_STATE, REACHED_PUBLISHING, MONITORING };
  void tickRandomGoals()
  {
    if (!have_state_) { return; }
    const double t = now().seconds();
    if (phase_ == GPhase::WAIT_STATE) {
      newExperimentId("goal");
      // 生成目标:当前位置 + 限幅随机位移,夹到工作空间
      std::uniform_real_distribution<double> u(-1.0, 1.0);
      Eigen::Vector3d d(u(rng_) * max_disp_, u(rng_) * max_disp_, u(rng_) * max_disp_);
      goal_p_ = (cur_p_ + d).cwiseMax(ws_min_).cwiseMin(ws_max_);
      goal_q_ = randomOrientation(rng_, max_roll_, max_pitch_, max_yaw_, cur_q_);
      publishGoal(goal_p_, goal_q_);
      last_goal_pub_ = t;
      phase_ = GPhase::MONITORING;
      goal_t0_ = t; in_tol_since_ = -1.0;
      RCLCPP_INFO(get_logger(), "新目标 #%d: pos=(%.2f,%.2f,%.2f)",
        exp_seq_, goal_p_.x(), goal_p_.y(), goal_p_.z());
      return;
    }
    // MONITORING:周期性重发同一目标,防止控制器侧 goal_timeout 过期
    if (goal_republish_hz_ > 0.0 &&
        (t - last_goal_pub_) >= 1.0 / goal_republish_hz_)
    {
      publishGoal(goal_p_, goal_q_);
      last_goal_pub_ = t;
    }
    const double pos_err = (cur_p_ - goal_p_).norm();
    const double ang_err = orientationError(cur_q_, goal_q_);
    const bool in_tol = pos_err < pos_tol_ && ang_err < ang_tol_;
    if (in_tol && in_tol_since_ < 0) { in_tol_since_ = t; }
    if (!in_tol) { in_tol_since_ = -1; }
    const bool reached = in_tol && (t - in_tol_since_) >= hold_s_;
    const bool timeout = (t - goal_t0_) > goal_timeout_s_;
    if (reached || timeout) {
      RCLCPP_INFO(get_logger(),
        "目标 #%d %s:pos_err=%.3f m ang_err=%.1f deg 用时 %.1f s",
        exp_seq_, reached ? "到达" : "超时", pos_err,
        ang_err * 180.0 / M_PI, t - goal_t0_);
      ++goals_done_;
      if (num_goals_ > 0 && goals_done_ >= num_goals_) {
        RCLCPP_INFO(get_logger(), "完成全部 %d 个目标,任务结束", goals_done_);
        timer_->cancel();
        return;
      }
      phase_ = GPhase::WAIT_STATE;  // 下一轮
    }
  }

  // ================= excitation 状态机 =================
  void tickExcitation()
  {
    const double t = now().seconds();
    if (seg_t0_ < 0) { seg_t0_ = t; newExperimentId("exc"); }
    const double seg_t = t - seg_t0_;
    const double cycle = excite_duration_s_ + excite_rest_s_;
    const int rep = static_cast<int>(seg_t / cycle);
    if (rep >= excite_reps_) {
      publishExcitation(WrenchVector::Zero());
      RCLCPP_INFO(get_logger(), "激励实验序列完成(%d 组)", excite_reps_);
      timer_->cancel();
      return;
    }
    const double t_in = std::fmod(seg_t, cycle);
    WrenchVector tau = WrenchVector::Zero();
    if (t_in < excite_duration_s_) {
      // 波形
      double s = 0.0;
      if (excite_profile_ == "step") {
        s = (std::fmod(t_in, excite_period_s_) < excite_period_s_ * 0.5) ? 1.0 : -1.0;
      } else if (excite_profile_ == "chirp") {
        const double f = 0.1 + 0.9 * t_in / std::max(0.1, excite_duration_s_);  // 0.1->1.0 Hz
        s = std::sin(2 * M_PI * (0.1 * t_in +
               0.5 * (f - 0.1) / std::max(0.1, excite_duration_s_) * t_in));
      } else {  // rand
        static thread_local std::uniform_real_distribution<double> u(-1.0, 1.0);
        s = u(rng_);
      }
      for (const int ax : excite_axes_) {
        if (ax >= 0 && ax < 6) { tau(ax) = excite_amp_ * s; }
      }
    }
    publishExcitation(tau);
  }

  void tick()
  {
    if (mode_ == "excitation") { tickExcitation(); } else { tickRandomGoals(); }
  }

  // ---- 参数 ----
  std::string mode_;
  std::string goal_frame_default_ = "world";
  double rate_hz_ = 20.0;
  Eigen::Vector3d ws_min_{-2, 0, -2}, ws_max_{2, 4, 2};
  double max_disp_ = 1.5, max_roll_ = 0, max_pitch_ = 0, max_yaw_ = 0;
  double pos_tol_ = 0.15, ang_tol_ = 0.14, hold_s_ = 1.5, goal_timeout_s_ = 40.0;
  double goal_republish_hz_ = 2.0;
  int num_goals_ = 0;
  std::vector<int> excite_axes_{2};
  double excite_amp_ = 20.0, excite_duration_s_ = 15.0, excite_rest_s_ = 5.0;
  double excite_period_s_ = 4.0;
  int excite_reps_ = 8;
  std::string excite_profile_ = "chirp";
  ThrusterParams thruster_params_;

  // ---- 运行状态 ----
  std::unique_ptr<ThrusterModel> model_;
  std::mt19937 rng_{1234};
  int exp_seq_ = 0, goals_done_ = 0;
  GPhase phase_ = GPhase::WAIT_STATE;
  bool have_state_ = false;
  Eigen::Vector3d cur_p_{Eigen::Vector3d::Zero()}, goal_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond cur_q_{Eigen::Quaterniond::Identity()},
                      goal_q_{Eigen::Quaterniond::Identity()};
  double goal_t0_ = 0, in_tol_since_ = -1, last_state_t_ = 0, seg_t0_ = -1;
  double last_goal_pub_ = -1e9;
  std::string state_frame_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_state_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_exp_id_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_excite_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auv_controller_mppi

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_controller_mppi::TrainingMissionNode>(
    rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
