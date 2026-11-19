// ROS 2 话题接口集成测试(进程内真实节点):
// 实例化 MppiControllerNode(小规模参数),以 20 Hz 发布合成里程计 +
// 一次目标,断言 /auv/control/thruster_cmd 持续输出 8 维有限值且
// solver 生效后推力非零。
// 【合成状态数据仅用于接口测试,非生产链路】
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "auv_controller_mppi/mppi_controller_node.hpp"

using namespace std::chrono_literals;
using namespace auv_controller_mppi;

class RosInterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosInterfaceTest, ControllerNodeProducesThrusterCmd)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    // 小规模求解 + 高频小参数,测试要快
    rclcpp::Parameter("mppi.K", 200),
    rclcpp::Parameter("mppi.H", 10),
    rclcpp::Parameter("rates.solver_hz", 20.0),
    rclcpp::Parameter("rates.exec_hz", 50.0),
    rclcpp::Parameter("rates.diag_hz", 1.0),
    // 中性悬浮(y-up 自包含世界系,与基础任务一致)
    rclcpp::Parameter("dynamics.world_up", std::vector<double>{0.0, 1.0, 0.0}),
    rclcpp::Parameter("dynamics.gravity_world", std::vector<double>{0.0, -9.81, 0.0}),
    rclcpp::Parameter("dynamics.buoyancy_force", 392.4),
    rclcpp::Parameter("dynamics.cb_position", std::vector<double>{0.0, 0.0, 0.0}),
    rclcpp::Parameter("safety.state_timeout_s", 0.5),
    rclcpp::Parameter("safety.goal_timeout_s", 30.0),
    rclcpp::Parameter("debug.publish_debug", false),
    rclcpp::Parameter("topics.state", std::string("/auv/itf_test/state")),
    rclcpp::Parameter("topics.goal", std::string("/auv/itf_test/goal")),
  });
  auto controller = std::make_shared<MppiControllerNode>(opts);

  auto driver = std::make_shared<rclcpp::Node>("itf_driver");
  auto pub_state = driver->create_publisher<nav_msgs::msg::Odometry>(
    "/auv/itf_test/state", 10);
  auto pub_goal = driver->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/auv/itf_test/goal", 10);

  std::atomic<int> received{0};
  std::atomic<double> max_abs_u{0.0};
  std::atomic<bool> shape_ok{true};
  auto sub_cmd = driver->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/auv/control/thruster_cmd", 10,
    [&received, &max_abs_u, &shape_ok](
      std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
      if (m->data.size() != 8) { shape_ok = false; }
      for (double v : m->data) {
        if (!std::isfinite(v)) { shape_ok = false; }
        max_abs_u = std::max(max_abs_u.load(), std::abs(v));
      }
      ++received;
    });

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(controller);
  exe.add_node(driver);
  std::thread spinner([&exe] { exe.spin(); });

  // 目标:沿 z 前进 1 m(状态从原点出发,solver 应给出非零推力)
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.position.z = 1.0;
  goal.pose.orientation.w = 1.0;

  const auto t_end = std::chrono::steady_clock::now() + 3s;
  int i = 0;
  while (std::chrono::steady_clock::now() < t_end) {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "world";
    odom.header.stamp = driver->now();
    odom.pose.pose.orientation.w = 1.0;
    pub_state->publish(odom);
    if (i == 5) { pub_goal->publish(goal); }   // 等话题发现后发目标
    std::this_thread::sleep_for(50ms);         // 20 Hz
    ++i;
  }
  std::this_thread::sleep_for(300ms);          // 收尾消息
  exe.cancel();
  spinner.join();

  // 50 Hz × ~3.3 s,扣除发现延迟,至少应收到一半以上
  EXPECT_GE(received.load(), 80) << "exec 定时器应持续发布,实际 " << received;
  EXPECT_TRUE(shape_ok.load()) << "所有消息必须 8 元素且有限";
  EXPECT_GT(max_abs_u.load(), 0.5)
    << "solver 生效后应有非零推力(实测最大 |u|=" << max_abs_u << ")";
}

TEST_F(RosInterfaceTest, StateTimeoutDrivesZeroThrust)
{
  // 状态断流后,序列耗尽 -> STALE -> 零推力(看门狗验证)
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("mppi.K", 100),
    rclcpp::Parameter("mppi.H", 5),
    rclcpp::Parameter("mppi.dt", 0.1),
    rclcpp::Parameter("rates.solver_hz", 20.0),
    rclcpp::Parameter("rates.exec_hz", 50.0),
    rclcpp::Parameter("dynamics.world_up", std::vector<double>{0.0, 1.0, 0.0}),
    rclcpp::Parameter("dynamics.gravity_world", std::vector<double>{0.0, -9.81, 0.0}),
    rclcpp::Parameter("dynamics.buoyancy_force", 392.4),
    rclcpp::Parameter("dynamics.cb_position", std::vector<double>{0.0, 0.0, 0.0}),
    rclcpp::Parameter("topics.state", std::string("/auv/itf2/state")),
    rclcpp::Parameter("topics.goal", std::string("/auv/itf2/goal")),
  });
  auto controller = std::make_shared<MppiControllerNode>(opts);
  auto driver = std::make_shared<rclcpp::Node>("itf_driver2");
  auto pub_state = driver->create_publisher<nav_msgs::msg::Odometry>(
    "/auv/itf2/state", 10);
  auto pub_goal = driver->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/auv/itf2/goal", 10);

  std::atomic<double> last_max_u{0.0};
  auto sub_cmd = driver->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/auv/control/thruster_cmd", 10,
    [&last_max_u](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
      double mx = 0.0;
      for (double v : m->data) { mx = std::max(mx, std::abs(v)); }
      last_max_u = mx;
    });

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(controller);
  exe.add_node(driver);
  std::thread spinner([&exe] { exe.spin(); });

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.position.z = 1.0;
  goal.pose.orientation.w = 1.0;

  // 先正常跑 1.5 s(solver 应已输出非零)
  for (int i = 0; i < 30; ++i) {
    odom.header.stamp = driver->now();
    pub_state->publish(odom);
    if (i == 3) { pub_goal->publish(goal); }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GT(last_max_u.load(), 0.1) << "正常阶段应有推力";

  // 停发状态 2.5 s(状态 0.5 s 超时 + 序列 1.5 s 时域 + 0.5 s 宽限)
  const auto t_stop = std::chrono::steady_clock::now() + 2500ms;
  while (std::chrono::steady_clock::now() < t_stop) {
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_NEAR(last_max_u.load(), 0.0, 1e-9) << "断流后必须安全零推力";
  exe.cancel();
  spinner.join();
}
