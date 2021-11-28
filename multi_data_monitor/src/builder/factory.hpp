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

#ifndef BUILDER__FACTORY_HPP_
#define BUILDER__FACTORY_HPP_

#include "interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <vector>

#include "generic_type_support/generic_type_support.hpp"

namespace builder
{

struct TopicSubscription
{
  TopicSubscription(const rclcpp::Node::SharedPtr node, std::vector<Interface *> interfaces)
  {
    using namespace std::placeholders;
    subscription = node->create_generic_subscription(
      interfaces[0]->GetTopicName(),
      interfaces[0]->GetTopicType(),
      rclcpp::QoS(1),
      std::bind(&TopicSubscription::Callback, this, _1));
    std::cout << "create_generic_subscription: " << interfaces[0]->GetTopicName() << " " << interfaces[0]->GetTopicType() << std::endl;

    views = interfaces;
    support = std::make_unique<generic_type_support::GenericMessageSupport>(interfaces[0]->GetTopicType());
  }

  ~TopicSubscription()
  {
    std::cout << "Destruct TopicSubscription" << std::endl;
  }

  void Callback(const std::shared_ptr<rclcpp::SerializedMessage> serialized) const
  {
    std::cout << "callback" << std::endl;
    for (const Interface * view : views)
    {
      view->Callback(YAML::Node());
    }
  }

  std::unique_ptr<generic_type_support::GenericMessageSupport> support;
  std::vector<Interface *> views;
  rclcpp::GenericSubscription::SharedPtr subscription;
};

class Factory
{
public:
  void CreateNode(const std::string & name, const YAML::Node & yaml);
  void Subscribe(const rclcpp::Node::SharedPtr node);
  void Build(QWidget * panel);

private:
  Dictionary dictionary_;
  std::map<std::string, std::unique_ptr<TopicSubscription>> subscriptions_;
};

}  // namespace builder

#endif  // BUILDER__FACTORY_HPP_
