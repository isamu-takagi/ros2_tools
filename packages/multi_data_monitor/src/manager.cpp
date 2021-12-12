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

#include "manager.hpp"
#include "monitors/layout/matrix.hpp"
#include "monitors/widget/simple.hpp"
#include "monitors/widget/titled.hpp"
#include <QWidget>
#include <string>

#include <iostream>

namespace monitors
{

void Manager::CreateMonitors()
{
  const auto CreateMonitor = [](const std::string & name, const YAML::Node & yaml) -> std::shared_ptr<Monitor>
  {
    const auto type = yaml["class"].as<std::string>();
    std::cout << "create node: " << name << "  " << type << std::endl;

    if (type == "matrix")
    {
      return std::make_shared<Matrix>(name, yaml);
    }
    if (type == "titled")
    {
      return std::make_shared<Titled>(name, yaml);
    }
    if (type == "simple")
    {
      return std::make_shared<Simple>(name, yaml);
    }

    std::cout << "  unknown type: " << type << std::endl;
    return nullptr;
  };

  for(const auto & monitor : yaml_["monitors"])
  {
    const auto name = monitor.first.as<std::string>();
    monitors_[name] = CreateMonitor(name, monitor.second);
  }
}

void Manager::CreateSubscription(const rclcpp::Node::SharedPtr & node)
{
  std::map<std::string, MonitorList> topics;
  for (const auto & [_, monitor] : monitors_)
  {
    const std::string name = monitor->GetTopicName();
    if (!name.empty())
    {
      topics[name].push_back(monitor);
    }
  }

  for (const auto & topic : topics)
  {
    std::cout << topic.first << std::endl;
    for (const auto & monitor : topic.second)
    {
      std::cout << "  " << monitor->GetName() << std::endl;
    }
    auto subscription = std::make_unique<TopicSubscription>(node, topic.second);
    subscriptions_.push_back(std::move(subscription));
  }
}

void Manager::Build(QWidget * panel)
{
  const auto name = yaml_["root"].as<std::string>();
  const auto root = monitors_.at(name);
  root->Build(monitors_);

  const auto widget = root->GetWidget();
  std::cout << "widget: " << widget << std::endl;
  if (widget)
  {
    // TODO: use dummy layout
    // panel->Widget(widget)
  }

  const auto layout = root->GetLayout();
  std::cout << "layout: " << layout << std::endl;
  if (layout)
  {
  panel->setLayout(layout);
  }
}

void Manager::Load(const std::string & path)
{
  yaml_ = YAML::LoadFile(path);
  std::cout << "format version: " << yaml_["version"].as<std::string>() << std::endl;
}

}  // namespace monitors
