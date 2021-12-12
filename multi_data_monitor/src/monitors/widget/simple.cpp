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

#include "simple.hpp"
#include <QLabel>

namespace monitors
{

constexpr auto default_style = "border-width: 1px 1px 1px 1px; border-style: solid; font-size: 14px;";

void Simple::Build([[maybe_unused]] MonitorDict & monitors)
{
  widget_ = label = new QLabel();
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet(default_style);

  if (yaml_["color"])
  {
    for (const auto & data : yaml_["color"])
    {
      const auto match = data["match"].as<std::string>();
      const auto color = data["color"].as<std::string>();
      style_color_[match] = " background-color: " + color + ";";
    }
  }
}

void Simple::Callback(const YAML::Node & message)
{
  const auto text = access.Get(message).as<std::string>();
  if (prev_ != text)
  {
    std::string style = default_style;
    if (style_color_.count(text))
    {
      style += style_color_.at(text);
    }
    label->setText(QString::fromStdString(text));
    label->setStyleSheet(QString::fromStdString(style));

    prev_ = text;
  }
}

}  // namespace monitors
