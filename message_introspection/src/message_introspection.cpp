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

int main()
{
  using namespace std;

  auto library = rclcpp::get_typesupport_library("std_msgs/msg/String", "rosidl_typesupport_introspection_cpp");
  auto type_support = rclcpp::get_typesupport_handle("std_msgs/msg/String", "rosidl_typesupport_introspection_cpp", *library);

  const auto member = reinterpret_cast<const rosidl_typesupport_introspection_cpp::MessageMember *>(type_support->data);
  cout << type_support->typesupport_identifier << endl;
  cout << member->name_ << endl;
}
