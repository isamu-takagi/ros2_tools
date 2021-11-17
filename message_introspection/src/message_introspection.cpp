// Copyright 2021 Tier IV, Inc.
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

#include "message_introspection.hpp"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <std_msgs/msg/header.hpp>

const auto type_name = "std_msgs/msg/Header";
const auto identifier = "rosidl_typesupport_introspection_cpp";

std::ostream& operator<<(std::ostream& os, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  os << "name               : " << member.name_ << std::endl;
  os << "type_id            : " << static_cast<uint32_t>(member.type_id_) << std::endl;
  os << "string_upper_bound : " << member.string_upper_bound_ << std::endl;
  os << "members            : " << member.members_ << std::endl;
  os << "is_array           : " << member.is_array_ << std::endl;
  os << "array_size         : " << member.array_size_ << std::endl;
  os << "is_upper_bound     : " << member.is_upper_bound_ << std::endl;
  os << "offset             : " << member.offset_ << std::endl;
  os << "default_value      : " << member.default_value_ << std::endl;
  os << "size_function      : " << member.size_function << std::endl;
  os << "get_const_function : " << member.get_const_function << std::endl;
  os << "get_function       : " << member.get_function << std::endl;
  os << "resize_function    : " << member.resize_function << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const rosidl_typesupport_introspection_cpp::MessageMembers & members)
{
  os << "message_namespace : " << members.message_namespace_ << std::endl;
  os << "message_name      : " << members.message_name_ << std::endl;
  os << "member_count      : " << members.member_count_ << std::endl;
  os << "size_of           : " << members.size_of_ << std::endl;
  os << "members           : " << members.members_ << std::endl;
  os << "init_function     : " << (void*)members.init_function << std::endl;
  os << "fini_function     : " << (void*)members.fini_function;
  return os;
}

void typesupport()
{
  auto type_library = rclcpp::get_typesupport_library(type_name, identifier);
  auto type_support = rclcpp::get_typesupport_handle(type_name, identifier, *type_library);

  const auto members = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);
  std::cout << type_support->typesupport_identifier << std::endl;
  std::cout << std::endl;
  std::cout << *members << std::endl;
  std::cout << std::endl;

  for (uint32_t i = 0; i < members->member_count_; ++i)
  {
    std::cout << "Member[" << i << "]" << std::endl;
    std::cout << members->members_[i] << std::endl;
  }
}

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

void callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  std::cout << *msg << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  typesupport();
  std::cout << sizeof(std_msgs::msg::Header) << std::endl;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("generic");
  auto sub = node->create_generic_subscription("/generic", type_name, rclcpp::QoS(1), callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
