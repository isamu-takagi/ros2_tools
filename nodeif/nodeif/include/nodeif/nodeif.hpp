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

#ifndef NODEIF__NODEIF_HPP_
#define NODEIF__NODEIF_HPP_

#include "rclcpp/rclcpp.hpp"

namespace nodeifs
{

class NodeIF : public rclcpp::Node
{
public:
  using rclcpp::Node::Node;

  template<class T>
  typename T::Behavior::SharedPtr create_publisher()
  {
    return create_publisher<typename T::DataType>(T::name(), T::qos());
  }

  template<class T, class CallbackT>
  typename T::Behavior::SharedPtr create_subscription(CallbackT && callback)
  {
    return create_subscription<typename T::DataType>(T::name(), T::qos(), std::forward<CallbackT>(callback));
  }

private:
  // Prohibit direct access to original methods.
  using rclcpp::Node::create_publisher;
  using rclcpp::Node::create_subscription;
  using rclcpp::Node::create_client;
  using rclcpp::Node::create_service;
};

}  // namespace nodeif

#endif  // NODEIF__NODEIF_HPP_
