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

#ifndef TOPIC__SUBSCRIPTION_HPP_
#define TOPIC__SUBSCRIPTION_HPP_

#include "monitor.hpp"
#include <rclcpp/rclcpp.hpp>

namespace monitors
{

class TopicSubscription
{
public:
  TopicSubscription(const rclcpp::Node::SharedPtr node, std::vector<Monitor *> monitors);
  ~TopicSubscription();
  void Callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized) const;

private:
  std::unique_ptr<generic_type_support::GenericMessageSupport> support_;
  std::vector<Monitor *> monitors_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
};

}  // namespace monitors

#endif  // TOPIC__SUBSCRIPTION_HPP_
