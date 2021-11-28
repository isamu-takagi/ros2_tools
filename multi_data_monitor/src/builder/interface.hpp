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

#ifndef BUILDER__INTERFACE_HPP_
#define BUILDER__INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <string>
#include <map>
#include <memory>

class QWidget;
class QLayout;

namespace builder
{

class Interface;
using Dictionary = std::map<std::string, std::unique_ptr<Interface>>;

class Interface
{
public:
  Interface(const std::string & name, const YAML::Node & yaml);
  virtual ~Interface() = default;

  // TODO: merge (Build, GetWidget, GetLayout)
  QWidget * GetWidget() {return widget_;}
  QLayout * GetLayout() {return layout_;}
  virtual void Build(Dictionary & dict) = 0;
  // static void AddChild(QWidget * parent, const std::unique_ptr<Interface> & base);
  // static void AddChild(QLayout * parent, const std::unique_ptr<Interface> & base);

  std::string GetTopic();

protected:
  std::string name_;
  YAML::Node yaml_;
  QWidget * widget_ = nullptr;
  QLayout * layout_ = nullptr;
};

}  // namespace builder

#endif  // BUILDER__INTERFACE_HPP_
