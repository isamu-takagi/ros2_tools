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

#ifndef monitors__MANAGER_HPP_
#define monitors__MANAGER_HPP_

#include "monitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <vector>

#include "generic_type_support/generic_type_support.hpp"

namespace monitors
{

struct TopicSubscription
{
  TopicSubscription(const rclcpp::Node::SharedPtr node, std::vector<Monitor *> monitors)
  {
    using namespace std::placeholders;
    subscription_ = node->create_generic_subscription(
      monitors[0]->GetTopicName(),
      monitors[0]->GetTopicType(),
      rclcpp::QoS(1),
      std::bind(&TopicSubscription::Callback, this, _1));
    std::cout << "create_generic_subscription: " << monitors[0]->GetTopicName() << " " << monitors[0]->GetTopicType() << std::endl;

    monitors_ = monitors;
    support_ = std::make_unique<generic_type_support::GenericMessageSupport>(monitors[0]->GetTopicType());
  }

  ~TopicSubscription()
  {
    std::cout << "Destruct TopicSubscription" << std::endl;
  }

  void Callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized) const
  {
    std::cout << "callback" << std::endl;
    const YAML::Node yaml = support_->DeserializeYAML(*serialized);
    for (const Monitor * monitor : monitors_)
    {
      monitor->Callback(yaml);
    }
  }

  std::unique_ptr<generic_type_support::GenericMessageSupport> support_;
  std::vector<Monitor *> monitors_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
};

class Manager
{
public:
  void CreateNode(const std::string & name, const YAML::Node & yaml);
  void Subscribe(const rclcpp::Node::SharedPtr node);
  void Build(QWidget * panel, const std::string & name);

private:
  Monitors monitors_;
  std::map<std::string, std::unique_ptr<TopicSubscription>> subscriptions_;
};

}  // namespace monitors

#endif  // monitors__MANAGER_HPP_
