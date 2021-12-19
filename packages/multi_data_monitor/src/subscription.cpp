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

#include "subscription.hpp"

#include <iostream>

namespace monitors
{

TopicSubscription::TopicSubscription(const std::string & name, const generic_type_support::GenericMessageSupport * support)
: name_(name), support_(support)
{
}

void TopicSubscription::Add(Monitor * monitor)
{
  if (support_ != monitor->GetTypeSupport())
  {
    const auto type1 = support_->GetTypeName();
    const auto type2 = monitor->GetTypeSupport()->GetTypeName();
    throw std::runtime_error("Topic '" + name_ + "' has multiple types [" + type1 + ", " + type2 + "]");
  }
  monitors_.push_back(monitor);
}

void TopicSubscription::Start(const rclcpp::Node::SharedPtr & node)
{
  std::cout << "start subscription: " << name_ << " " << support_->GetTypeName() << std::endl;

  using namespace std::placeholders;
  subscription_ = node->create_generic_subscription(
  name_,
  support_->GetTypeName(),
  rclcpp::QoS(1),
  std::bind(&TopicSubscription::Callback, this, _1));
}

void TopicSubscription::Callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized) const
{
  const YAML::Node yaml = support_->DeserializeYAML(*serialized);
  for (const auto & monitor : monitors_)
  {
    monitor->Callback(yaml);
  }
}

}  // namespace monitors
