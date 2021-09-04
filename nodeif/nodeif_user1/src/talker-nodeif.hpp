#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

struct SendStringMessage
{
  using DataType = std_msgs::msg::String;
  using Behavior = rclcpp::Publisher<DataType>;

  static std::string name()
  {
    return "chatter";
  }
  static rclcpp::QoS qos()
  {
    return rclcpp::QoS(rclcpp::KeepLast(7));
  }
};

struct RecvStringMessage
{
  using DataType = std_msgs::msg::String;
  using Behavior = rclcpp::Subscription<DataType>;

  static std::string name()
  {
    return "chatter";
  }
  static rclcpp::QoS qos()
  {
    return rclcpp::QoS(rclcpp::KeepLast(7));
  }
};
