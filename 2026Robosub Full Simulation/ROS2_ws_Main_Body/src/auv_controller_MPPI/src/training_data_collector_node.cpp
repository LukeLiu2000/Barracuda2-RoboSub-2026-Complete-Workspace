// -*- coding: utf-8 -*-
/**
 * training_data_collector_node —— 训练数据记录节点。
 *
 * 订阅:
 *   collector.state_topic(默认 /auv/ground_truth/odometry,优先仿真真值)
 *   /auv/control/thruster_cmd    实际施加的 8 推力
 *   /auv/training/experiment_id  当前实验编号
 * 输出(CSV,collector.output_dir,默认包 share 目录 data/ 下,授权目录内):
 *   t,experiment_id, 13 维状态, 8 维推力, 下一时刻 13 维状态, dt
 *
 * 对齐规则:状态消息到达沿触发;距上一条记录 >= sample_dt 才写一行,
 *   dt = 两状态时间戳差;dt 超过 10*sample_dt(乱序/断流)或 < 1e-4 的行丢弃。
 *   u 是该区间内的【时间加权平均】推力(推力变化时不得把末尾命令当成
 *   整段输入 —— 旧实现取最新命令,已修正);区间内命令变化次数记入
 *   运行日志(变化频繁的行辨识权重应降低)。
 *   Float64MultiArray 无时间戳,命令时间用节点时钟(联调 launch 已设
 *   use_sim_time,与状态 header.stamp 同为仿真时间轴)。
 * 服务:~/set_enabled(std_srvs/SetBool)暂停/恢复记录。
 */
#include <chrono>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "auv_controller_mppi/types.hpp"

namespace auv_controller_mppi
{

class TrainingDataCollectorNode : public rclcpp::Node
{
public:
  explicit TrainingDataCollectorNode(const rclcpp::NodeOptions & options)
  : Node("training_data_collector_node", options)
  {
    sample_dt_ = declare_parameter<double>("collector.sample_dt", 0.05);
    output_dir_ = declare_parameter<std::string>(
      "collector.output_dir", "/tmp");  // launch 里必须覆盖到授权目录!
    const std::string state_topic = declare_parameter<std::string>(
      "collector.state_topic", "/auv/ground_truth/odometry");

    sub_state_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onState(m); });
    sub_cmd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/auv/control/thruster_cmd", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
        if (m->data.size() != 8) { return; }
        const double t = now().seconds();   // use_sim_time 下 = 仿真时间
        std::lock_guard<std::mutex> lk(buf_mutex_);
        if (!have_cmd_) {
          cur_u_ = m->data;
          cur_u_t_ = t;
          have_cmd_ = true;
          return;
        }
        // 把上一条命令的持续段计入积分,再切换新命令
        const double dur = std::max(0.0, t - cur_u_t_);
        if (dur > 0.0) {
          for (int i = 0; i < 8; ++i) { seg_u_sum_[i] += cur_u_[i] * dur; }
          seg_dt_ += dur;
        }
        cur_u_ = m->data;
        cur_u_t_ = t;
        ++seg_changes_;
      });
    sub_exp_ = create_subscription<std_msgs::msg::String>(
      "/auv/training/experiment_id", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(buf_mutex_);
        exp_id_ = m->data;
      });
    srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lk(buf_mutex_);
        enabled_ = req->data;
        res->success = true;
        res->message = req->data ? "collecting" : "paused";
      });

    openFile();
    RCLCPP_INFO(get_logger(), "collector 启动:state=%s sample_dt=%.3f s -> %s (%s)",
      state_topic.c_str(), sample_dt_, path_.c_str(),
      csv_.is_open() ? "ok" : "OPEN FAILED");
  }

  ~TrainingDataCollectorNode() override
  {
    std::lock_guard<std::mutex> lk(buf_mutex_);
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

private:
  void onState(const nav_msgs::msg::Odometry::ConstSharedPtr & m)
  {
    State x;
    const auto & p = m->pose.pose.position;
    const auto & q = m->pose.pose.orientation;
    x.p = Eigen::Vector3d(p.x, p.y, p.z);
    // 防御:零四元数直接归一化会 NaN;零时间戳(合成/畸形输入)回退到节点时钟
    Eigen::Quaterniond qm(q.w, q.x, q.y, q.z);
    x.q = qm.norm() > 1e-9 ? qm.normalized() : Eigen::Quaterniond::Identity();
    x.v = Eigen::Vector3d(m->twist.twist.linear.x, m->twist.twist.linear.y,
                          m->twist.twist.linear.z);
    x.w = Eigen::Vector3d(m->twist.twist.angular.x, m->twist.twist.angular.y,
                          m->twist.twist.angular.z);
    const double t = m->header.stamp.sec != 0 || m->header.stamp.nanosec != 0
      ? rclcpp::Time(m->header.stamp).seconds()
      : now().seconds();
    const Eigen::Matrix<double, 13, 1> xv = x.toVector();

    std::lock_guard<std::mutex> lk(buf_mutex_);
    // 先把当前命令段积分到状态时刻 t(无论是否写行,都对齐到 t)
    if (have_cmd_ && t > cur_u_t_) {
      const double dur = t - cur_u_t_;
      for (int i = 0; i < 8; ++i) { seg_u_sum_[i] += cur_u_[i] * dur; }
      seg_dt_ += dur;
      cur_u_t_ = t;
    }
    if (enabled_ && have_cmd_ && have_prev_ &&
        t > prev_t_ && (t - prev_t_) >= 1e-4 && (t - prev_t_) <= 10.0 * sample_dt_ &&
        t - last_emit_t_ >= sample_dt_)
    {
      const double dt = t - prev_t_;
      // 区间输入 = 时间加权平均推力(命令变化时不得用末尾值冒充整段)
      std::vector<double> u_avg(8, 0.0);
      if (seg_dt_ > 1e-9) {
        for (int i = 0; i < 8; ++i) { u_avg[i] = seg_u_sum_[i] / seg_dt_; }
      } else {
        u_avg = cur_u_;   // 区间内无命令到达(保持段),用当前命令
      }
      csv_ << std::fixed << std::setprecision(6)
           << prev_t_ << ',' << exp_id_ << ',';
      for (int i = 0; i < 13; ++i) { csv_ << prev_x_(i) << ','; }
      for (int i = 0; i < 8; ++i) { csv_ << u_avg[i] << ','; }
      for (int i = 0; i < 13; ++i) { csv_ << xv(i) << ','; }
      csv_ << dt << '\n';
      last_emit_t_ = t;
      total_changes_ += seg_changes_;
      if (seg_changes_ > max_changes_) { max_changes_ = seg_changes_; }
      if (++rows_ % 200 == 0) {
        csv_.flush();
        RCLCPP_INFO(get_logger(),
          "rows=%llu 区间命令变化: max=%d avg=%.1f(变化多的行辨识权重应低)",
          static_cast<unsigned long long>(rows_), max_changes_,
          rows_ > 0 ? double(total_changes_) / double(rows_) : 0.0);
      }
      // 写行后开启新区间
      seg_u_sum_.assign(8, 0.0);
      seg_dt_ = 0.0;
      seg_changes_ = 0;
    } else {
      // 本条状态未触发写行:prev_t_/prev_x_ 会前移到 t,区间积分也必须
      // 同步重开(否则把已跳过子区间的命令混进下一段,时间加权失真)。
      // cur_u_t_ 在上面的积分步已推进到 t,无需再动。
      seg_u_sum_.assign(8, 0.0);
      seg_dt_ = 0.0;
      seg_changes_ = 0;
    }
    prev_x_ = xv;
    prev_t_ = t;
    have_prev_ = true;
  }

  void openFile()
  {
    char buf[64];
    const std::time_t tt = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&tt));
    path_ = output_dir_ + "/training_" + buf + ".csv";
    csv_.open(path_);
    if (!csv_.is_open()) { return; }
    csv_ << "t,experiment_id,"
         << "px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,"
         << "u0,u1,u2,u3,u4,u5,u6,u7,"
         << "nx_px,nx_py,nx_pz,nx_qx,nx_qy,nx_qz,nx_qw,nx_vx,nx_vy,nx_vz,"
            "nx_wx,nx_wy,nx_wz,dt\n";
  }

  double sample_dt_ = 0.05;
  std::string output_dir_, path_;
  std::string exp_id_ = "none";
  // 区间命令积分(时间加权平均用):
  //   cur_u_/cur_u_t_  当前持续中的命令及其起始时刻
  //   seg_u_sum_/seg_dt_  自上一写行点(或起点)以来的 ∫u dt 与 ∫dt
  //   seg_changes_     当前区间内命令变化次数
  std::vector<double> cur_u_{std::vector<double>(8, 0.0)};
  double cur_u_t_ = 0.0;
  std::vector<double> seg_u_sum_{std::vector<double>(8, 0.0)};
  double seg_dt_ = 0.0;
  int seg_changes_ = 0;
  uint64_t total_changes_ = 0;
  int max_changes_ = 0;
  bool have_cmd_ = false, enabled_ = true, have_prev_ = false;
  double prev_t_ = -1, last_emit_t_ = -1e9;
  Eigen::Matrix<double, 13, 1> prev_x_{Eigen::Matrix<double, 13, 1>::Zero()};
  uint64_t rows_ = 0;
  std::mutex buf_mutex_;
  std::ofstream csv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_exp_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
};

}  // namespace auv_controller_mppi

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auv_controller_mppi::TrainingDataCollectorNode>(
    rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
