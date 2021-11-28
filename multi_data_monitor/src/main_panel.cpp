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
#include "builder/factory.hpp"
#include <QLabel>
#include <QGridLayout>
#include <yaml-cpp/yaml.h>
#include <iostream>

MultiDataMonitorPanel::MultiDataMonitorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{

}

void MultiDataMonitorPanel::save(rviz_common::Config config) const
{
  std::cout << "config: save" << std::endl;
  Panel::save(config);
  config.mapSetValue("File", path_);
}

void MultiDataMonitorPanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
}

void MultiDataMonitorPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  config.mapGetString("File", &path_);

  try
  {
    std::cout << std::string(100, '=') << std::endl;
    std::cout << "File: " << path_.toStdString() << std::endl;
    YAML::Node yaml = YAML::LoadFile(path_.toStdString());
    YAML::Node format = yaml["format"];
    yaml.remove("format");
    std::cout << "format version: " << format["version"].as<int>() << std::endl;

    std::cout << std::string(100, '=') << std::endl;
    builder::Factory factory;
    for(const auto & node : yaml)
    {
      factory.CreateNode(node.first.as<std::string>(), node.second);
    }
    std::cout << std::string(100, '=') << std::endl;
    factory.Subscribe();
    std::cout << std::string(100, '=') << std::endl;
    factory.Build(this);
    std::cout << std::string(100, '=') << std::endl;
  }
  catch(YAML::BadFile & error)
  {
    std::cout << error.what()<< std::endl;
  }
}

void MultiDataMonitorPanel::update()
{
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MultiDataMonitorPanel, rviz_common::Panel)
