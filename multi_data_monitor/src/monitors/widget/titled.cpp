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

#include "titled.hpp"
#include <QLabel>
#include <QVBoxLayout>
#include <string>

namespace monitors
{

void Titled::Build([[maybe_unused]] MonitorDict & monitors)
{
  layout_ = new QVBoxLayout();
  value = new QLabel();
  title = new QLabel(QString::fromStdString(yaml_["title"].as<std::string>()));

  value->setAlignment(Qt::AlignCenter);
  title->setAlignment(Qt::AlignCenter);
  // setToolTip();

  value->setStyleSheet("border-width: 1px 1px 1px 1px; border-style: solid; font-size: 24px;");
  title->setStyleSheet("border-width: 0px 1px 1px 1px; border-style: solid; font-size: 12px;");

  layout_->addWidget(value);
  layout_->addWidget(title);
  layout_->setSpacing(0);
}

void Titled::Callback(const YAML::Node & message)
{
  const auto text = access.Get(message).as<std::string>();
  value->setText(QString::fromStdString(text));
}

}  // namespace monitors
