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

#include <iostream>

namespace monitors
{

constexpr auto default_style = "border-width: 1px 1px 1px 1px; border-style: solid; font-size: 14px;";

void Simple::Build([[maybe_unused]] MonitorDict & monitors)
{
  widget_ = label = new QLabel();
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet(default_style);

  const auto rules = yaml_["rules"];
  if (rules)
  {
    for (const auto & rule : rules)
    {
      rules_.emplace_back(rule);
    }
  }
}

void Simple::Callback(const YAML::Node & message)
{
  const auto data = access.Get(message);
  const auto text = data.as<std::string>();
  if (prev_ != text)
  {
    FunctionResult result{data, YAML::Node()};  // TODO: fix style
    for (const auto & rule : rules_)
    {
      result = rule.Apply(data);
    }
    label->setText(QString::fromStdString(result.value.as<std::string>()));
    label->setStyleSheet(QString::fromStdString(default_style + result.style.GetStyleSheet() ));

    prev_ = text;
  }
}

}  // namespace monitors
