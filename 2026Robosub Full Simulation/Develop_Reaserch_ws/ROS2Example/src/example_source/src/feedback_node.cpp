// feedback_node.cpp —— 反馈接收节点(包:example_source,语言:C++/rclcpp)
//
// 职责:订阅相对话题 `feedback`(std_msgs/msg/String),打印收到的每条反馈,
//       并维护本进程实例累计收到的反馈条数。
// 教学点:同一个 C++ 包里可以有职责完全不同的两个节点;
//         它接收的是 Python 包 report_node 发来的消息(跨语言、跨包)。

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class FeedbackNode : public rclcpp::Node
{
public:
  FeedbackNode()
  : rclcpp::Node("feedback_node"), received_(0)
  {
    // 统一教学 QoS:KEEP_LAST / depth=10 / RELIABLE / VOLATILE
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    // 订阅相对话题 feedback;回调里只做打印与计数,没有 Timer
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "feedback", qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        received_++;  // 累计条数:随本实例生命周期,从 1 开始
        RCLCPP_INFO(
          this->get_logger(), "[第 %lu 条反馈] %s",
          static_cast<unsigned long>(received_), msg->data.c_str());
      });

    RCLCPP_INFO(this->get_logger(), "feedback_node 已启动,等待反馈消息...");
  }

private:
  std::uint64_t received_ = 0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);   // 初始化上下文
  rclcpp::spin(std::make_shared<FeedbackNode>());  // 等回调
  rclcpp::shutdown();         // 退出前释放
  return 0;
}
