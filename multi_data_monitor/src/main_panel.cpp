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

#include "main_panel.hpp"
#include <QLabel>
#include <QGridLayout>
#include <yaml-cpp/yaml.h>
#include <iostream>

MultiDataMonitor::MultiDataMonitor(QWidget * parent)
: rviz_common::Panel(parent)
{

}

void MultiDataMonitor::save(rviz_common::Config config) const
{
  std::cout << "config: save" << std::endl;
  Panel::save(config);
  config.mapSetValue("File", path_);
}

void MultiDataMonitor::onInitialize()
{
}

void MultiDataMonitor::load(const rviz_common::Config & config)
{
  Panel::load(config);
  config.mapGetString("File", &path_);

  const auto rviz_ros_node = getDisplayContext()->getRosNodeAbstraction();

  try
  {
    std::cout << std::string(100, '=') << std::endl;
    std::cout << "File: " << path_.toStdString() << std::endl;
    YAML::Node yaml = YAML::LoadFile(path_.toStdString());
    YAML::Node format = yaml["format"];
    yaml.remove("format");
    std::cout << "format version: " << format["version"].as<int>() << std::endl;

    std::cout << std::string(100, '=') << std::endl;
    for(const auto & node : yaml)
    {
      factory_.CreateNode(node.first.as<std::string>(), node.second);
    }
    std::cout << std::string(100, '=') << std::endl;
    factory_.Subscribe(rviz_ros_node.lock()->get_raw_node());
    std::cout << std::string(100, '=') << std::endl;
    factory_.Build(this);
    std::cout << std::string(100, '=') << std::endl;
  }
  catch(YAML::BadFile & error)
  {
    std::cout << error.what()<< std::endl;
  }
}

void MultiDataMonitor::update()
{
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MultiDataMonitor, rviz_common::Panel)
