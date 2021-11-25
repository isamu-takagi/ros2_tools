// Copyright 2021 Takagi, Isamu
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

#include "test.hpp"
#include "access.hpp"

#include "debug.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

std::ostream& operator<<(std::ostream& os, const rclcpp::SerializedMessage & msg)
{
  const auto array = msg.get_rcl_serialized_message();
  const auto flags = os.flags();
  os << "size     : " << msg.size() << std::endl;
  os << "capacity : " << msg.capacity() << std::endl;
  os << "size     : " << array.buffer_length << std::endl;
  os << "capacity : " << array.buffer_capacity << std::endl;
  os << "buffer   : " << std::hex << std::setfill('0');
  for (size_t i = 0; i < msg.size(); ++i)
  {
    os << std::setw(2) << static_cast<uint32_t>(array.buffer[i]) << " ";
  }
  os.flags(flags);
  return os;
}

std::shared_ptr<generic_type_access::GenericMessageSupport> support;
void callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized)
{
  std::cout << "==================== Message1 ====================" << std::endl;
  std::cout << *serialized << std::endl;
  std::cout << "==================== Message2 ====================" << std::endl;
  std::cout << support->DeserializeYAML(serialized) << std::endl;
  std::cout << "==================================================" << std::endl;
}

int main(int argc, char **argv)
{
  auto access = generic_type_access::GenericTypeAccess();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("generic");
  auto type = node->declare_parameter("type", "std_msgs/msg/Header");
  auto subs = node->create_generic_subscription("/generic", type, rclcpp::QoS(1), callback);

  RCLCPP_INFO(node->get_logger(), "type: %s", type.c_str());
  support = access.GetMessageSupport(type);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
