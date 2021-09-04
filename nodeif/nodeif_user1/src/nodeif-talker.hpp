#include "rclcpp/rclcpp.hpp"

class TalkerNodeIF : public rclcpp::Node
{
public:
  explicit TalkerNodeIF(const rclcpp::NodeOptions & options) : Node("talker", options)
  {
  }
};
