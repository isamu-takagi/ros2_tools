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
#include "monitors/layout/grid.hpp"
#include "monitors/widget/titled.hpp"
#include <QWidget>
#include <string>

#include <iostream>

namespace monitors
{

void Manager::CreateNode(const std::string & name, const YAML::Node & yaml)
{
	const auto type = yaml["class"].as<std::string>();
	std::cout << "create node: " << name << "  " << type << std::endl;

	std::unique_ptr<Monitor> monitors = nullptr;
	if (type == "matrix")
	{
		monitors_[name] = std::make_unique<Grid>(name, yaml);
		return;
	}
	if (type == "titled")
	{
		monitors_[name] = std::make_unique<Titled>(name, yaml);
		return;
	}
	std::cout << "  unknown type: " << type << std::endl;
}

void Manager::Subscribe(const rclcpp::Node::SharedPtr node)
{
	std::map<std::string, std::vector<Monitor *>> groups;
	for (const auto & pair : monitors_)
	{
		const std::string topic = pair.second->GetTopicName();
		if (!topic.empty())
		{
			groups[topic].push_back(pair.second.get());  // check release order
		}
	}

  for (const auto & [topic, monitors] : groups)
  {
    std::cout << topic << std::endl;
    for (const auto & monitor : monitors)
    {
      std::cout << "  " << monitor->GetName() << std::endl;
    }
    subscriptions_[topic] = std::make_unique<TopicSubscription>(node, monitors);
  }

}

void Manager::Build(QWidget * panel, const std::string & name)
{
  const auto & root = monitors_.at(name);
  root->Build(monitors_);

  const auto widget = root->GetWidget();
  std::cout << "widget: " << widget << std::endl;
  if (widget)
  {
	// TODO: use dummy layout
	//panel->Widget(widget)
  }

  const auto layout = root->GetLayout();
  std::cout << "layout: " << layout << std::endl;
  if (layout)
  {
	panel->setLayout(layout);
  }
}

}  // namespace monitors
