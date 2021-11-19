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

void Factory::Build(QWidget * panel)
{
  std::cout << "===============" << std::endl;
  for (const auto & pair : dictionary_)
  {
	std::cout << pair.first << "  " << pair.second.get() << std::endl;
  }
  const auto & root = dictionary_.at("root");
  root->Build(panel, dictionary_);

  const auto widget = root->GetWidget();
  const auto layout = root->GetLayout();
  std::cout << widget << "  " << layout << std::endl;
  if (layout)
  {
	  panel->setLayout(layout);
  }
}

void Factory::CreateNode(const std::string & name, const YAML::Node & yaml)
{
	const auto type = yaml["class"].as<std::string>();
	std::cout << "  " << name << "  " << type << std::endl;

	std::unique_ptr<Interface> builder = nullptr;
	if (type == "matrix")
	{
		dictionary_[name] = std::make_unique<Grid>(yaml);
		return;
	}
	if (type == "float_with_title")
	{
		dictionary_[name] = std::make_unique<Titled>(yaml);
		return;
	}
	std::cout << "  unknown type: " << type << std::endl;
}

}  // namespace builder
