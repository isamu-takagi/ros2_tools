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

#include "monitors/monitor.hpp"
#include "monitors/subscription.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "generic_type_support/generic_type_support.hpp"

namespace monitors
{

class Manager
{
public:
  void Load(const std::string & path);
  void CreateMonitors();
  void CreateSubscription(const rclcpp::Node::SharedPtr & node);
  void Build(QWidget * panel);

private:

  MonitorDict monitors_;
  std::vector<std::unique_ptr<TopicSubscription>> subscriptions_;
  YAML::Node yaml_;
};

}  // namespace monitors

#endif  // monitors__MANAGER_HPP_
