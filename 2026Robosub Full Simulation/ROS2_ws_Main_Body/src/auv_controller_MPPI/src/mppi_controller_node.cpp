// -*- coding: utf-8 -*-
/**
 * mppi_controller_node 可执行入口。
 * 节点类实现见 include/auv_controller_mppi/mppi_controller_node.hpp
 * (拆出头文件是为了让 test/test_ros_interface.cpp 能进程内实例化节点
 * 做真实话题级集成测试)。
 */
#include "auv_controller_mppi/mppi_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auv_controller_mppi::MppiControllerNode>(
    rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
