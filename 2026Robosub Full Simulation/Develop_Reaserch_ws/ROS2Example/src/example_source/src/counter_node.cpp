// counter_node.cpp —— 计数发布节点(包:example_source,语言:C++/rclcpp)
//
// 职责:按 publish_rate_hz 的频率,从 start_value 开始以 increment 为步长,
//       向相对话题 `count` 发布 std_msgs/msg/Int64。
// 注意:代码里只写相对话题名 "count",不写死 /demo/。
//       命名空间由 launch 参数或 ros2 run 的 __ns:= 决定。

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class CounterNode : public rclcpp::Node
{
public:
  explicit CounterNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("counter_node", options)
  {
    // 1) 声明参数并读取默认值(可被 YAML 文件或命令行 -p 覆盖)
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 2.0);
    start_value_  = this->declare_parameter<std::int64_t>("start_value", 0);
    increment_    = this->declare_parameter<std::int64_t>("increment", 1);

    // 2) 参数校验:频率必须大于 0,否则报错退出,不用非法值硬撑
    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "参数 publish_rate_hz 必须大于 0,当前值为 %f", publish_rate_hz_);
      throw std::invalid_argument("publish_rate_hz must be > 0");
    }

    // 3) 统一教学 QoS:KEEP_LAST / depth=10 / RELIABLE / VOLATILE
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    pub_ = this->create_publisher<std_msgs::msg::Int64>("count", qos);

    // 4) 用 ROS Timer 驱动发布:回调式,不用阻塞 sleep 或业务 while 循环
    const std::chrono::duration<double> period_sec(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_sec),
      std::bind(&CounterNode::on_timer, this));

    current_ = start_value_;
    RCLCPP_INFO(
      this->get_logger(),
      "counter_node 已启动: rate=%.2fHz start=%ld increment=%ld",
      publish_rate_hz_, static_cast<long>(start_value_),
      static_cast<long>(increment_));
  }

private:
  void on_timer()
  {
    // 每次回调:先发布当前值,再更新下一次要发布的值
    std_msgs::msg::Int64 msg;
    msg.data = current_;
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "发布 count = %ld", static_cast<long>(current_));

    // 溢出防护:若 current_ + increment_ 会越过 Int64 范围,
    // 明确报错并停止计数发布,不允许出现未定义行为。
    if (increment_ > 0 &&
      current_ > std::numeric_limits<std::int64_t>::max() - increment_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "计数将溢出 Int64 上限,停止发布(current=%ld, increment=%ld)",
        static_cast<long>(current_), static_cast<long>(increment_));
      timer_->cancel();  // 取消定时器 = 不再发布,进程仍在(spin 继续)
      return;
    }
    if (increment_ < 0 &&
      current_ < std::numeric_limits<std::int64_t>::min() - increment_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "计数将溢出 Int64 下限,停止发布(current=%ld, increment=%ld)",
        static_cast<long>(current_), static_cast<long>(increment_));
      timer_->cancel();
      return;
    }
    current_ += increment_;
  }

  double publish_rate_hz_ = 2.0;
  std::int64_t start_value_ = 0;
  std::int64_t increment_ = 1;
  std::int64_t current_ = 0;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // 初始化全局 rclcpp 上下文(进程级,只做一次)

  try {
    // 构造节点并进入 spin:线程阻塞在这里分发回调,直到 Ctrl+C 或出错
    rclcpp::spin(std::make_shared<CounterNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    // 参数非法等构造期错误会走到这里:打印后以非零码退出
    std::fprintf(stderr, "counter_node 启动失败: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();  // 释放上下文资源,正常退出
  return 0;
}
