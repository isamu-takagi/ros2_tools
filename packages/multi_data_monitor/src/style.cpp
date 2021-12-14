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

#include "style.hpp"

#include <iostream>

StyleDefinition::StyleDefinition()
{
  text_size = 0;
  text_color = "";
  back_color = "";
}

StyleDefinition::StyleDefinition(const YAML::Node & yaml)
{
  text_size = 0;
  text_color = "";
  back_color = "";

  if (!yaml) { return; }

  if (yaml["text-size"])  { text_size  = yaml["text-size"].as<int>(); }
  if (yaml["text-color"]) { text_color = yaml["text-color"].as<std::string>(); }
  if (yaml["back-color"]) { back_color = yaml["back-color"].as<std::string>(); }
}

std::string StyleDefinition::GetStyleSheet() const
{
  std::string style_sheet;

  if (text_size != 0)
  {
    style_sheet += "font-size: " + std::to_string(text_size) + ";";
  }
  if (text_color != "")
  {
    style_sheet += "color: " + text_color + ";";
  }
  if (back_color != "")
  {
    style_sheet += "background-color: " + back_color + ";";
  }
  return style_sheet;
}

StyleDefinition StyleDefinition::Merge(const StyleDefinition & input) const
{
  StyleDefinition style = *this;

  if (input.text_size != 0)
  {
    style.text_size = input.text_size;
  }
  if (input.text_color != "")
  {
    style.text_color = input.text_color;
  }
  if (input.back_color != "")
  {
    style.back_color = input.back_color;
  }
  return style;
}
