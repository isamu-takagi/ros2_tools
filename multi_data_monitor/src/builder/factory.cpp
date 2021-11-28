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

#include "factory.hpp"
#include "layout/grid.hpp"
#include "widget/titled.hpp"
#include <QWidget>
#include <string>

#include <iostream>

namespace builder
{

void Factory::CreateNode(const std::string & name, const YAML::Node & yaml)
{
	const auto type = yaml["class"].as<std::string>();
	std::cout << "create node: " << name << "  " << type << std::endl;

	std::unique_ptr<Interface> builder = nullptr;
	if (type == "matrix")
	{
		dictionary_[name] = std::make_unique<Grid>(name, yaml);
		return;
	}
	if (type == "titled")
	{
		dictionary_[name] = std::make_unique<Titled>(name, yaml);
		return;
	}
	std::cout << "  unknown type: " << type << std::endl;
}

void Factory::Subscribe()
{
	for (const auto & pair : dictionary_)
	{
		const std::string topic = pair.second->GetTopic();
		if (!topic.empty())
		{
			if (subscriptions_.count(topic))
			{
				std::cout << "subscribe: " << topic << std::endl;
			}

			// TopicSubscription
			// message access
			// callbacks
		}
	}
}

void Factory::Build(QWidget * panel)
{
  const auto & root = dictionary_.at("root");
  root->Build(dictionary_);

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

}  // namespace builder
