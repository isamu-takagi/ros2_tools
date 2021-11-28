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
#include "generic_type_support/message.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <map>
#include <vector>

namespace builder
{

/*
struct TopicSubscription
{
  generic_type_support::GenericMessageSupport support;
  rclcpp::GenericSubscription::SharedPtr subscription;
  std::vector<std::shared_ptr<Interface> callbacks;
};
*/

class Factory
{
public:
  void CreateNode(const std::string & name, const YAML::Node & yaml);
  void Subscribe();
  void Build(QWidget * panel);

private:
  Dictionary dictionary_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
};

}  // namespace builder

#endif  // BUILDER__FACTORY_HPP_
