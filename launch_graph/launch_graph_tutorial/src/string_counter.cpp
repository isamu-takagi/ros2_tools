// Copyright 2014 Open Source Robotics Foundation, Inc.
// Copyright 2021 Takagi Isamu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace launch_graph_tutorial
{

class StringCounter : public rclcpp::Node
{
public:
  explicit StringCounter(const rclcpp::NodeOptions & options)
  : Node("string_counter", options)
  {
    using namespace std::chrono_literals;

    auto publish_message =
      [this]() -> void
      {
        std_msgs::msg::String msg;
        msg.data = "Hello World: " + std::to_string(count_++);
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Pub: %s", msg.data.c_str());
      };

    pub_ = create_publisher<std_msgs::msg::String>("~/output", rclcpp::QoS(1));
    timer_ = create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace launch_graph_tutorial

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(launch_graph_tutorial::StringCounter)
