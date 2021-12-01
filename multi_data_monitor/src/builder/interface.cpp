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

#include "interface.hpp"
#include <string>

namespace builder
{

Interface::Interface(const std::string & name, const YAML::Node & yaml)
{
  name_ = name;
  yaml_ = yaml;

  if (yaml_["topic"]["data"])
  {
    access = generic_type_support::GeneticTypeAccess(yaml_["topic"]["data"].as<std::string>());
  }
}

std::string Interface::GetName()
{
  return name_;
}

std::string Interface::GetTopicName()
{
  if (!yaml_["topic"]["name"]) { return ""; }
  return yaml_["topic"]["name"].as<std::string>();
}

std::string Interface::GetTopicType()
{
  if (!yaml_["topic"]["type"]) { return ""; }
  return yaml_["topic"]["type"].as<std::string>();
}

/*
void NodeBase::AddChild(QWidget * parent, const std::unique_ptr<NodeBase> & base)
{
  auto layout = base->GetLayout();
  auto widget = base->GetWidget();
  std::cout << widget << " " << layout << std::endl;

  if (layout)
  {
    parent->setLayout(layout);
  }
}

void NodeBase::AddChild(QLayout * parent, const std::unique_ptr<NodeBase> & base)
{
  (void)parent;
  (void)base;
}
*/

}  // namespace builder
