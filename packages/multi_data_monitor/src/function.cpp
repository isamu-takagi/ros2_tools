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

#include "function.hpp"
#include "style.hpp"

#include <iostream>

SwitchFunction::SwitchFunction(const YAML::Node & yaml)
{
  for (const auto & node : yaml["args"])
  {
    const auto key = node.first.as<std::string>();
    cases.insert(std::make_pair(key, FunctionResult{node.second["value"], node.second["style"]}));
  }
}

FunctionResult SwitchFunction::Apply(const YAML::Node & value) const
{
  const auto iter = cases.find(value.as<std::string>());
  if (iter == cases.end())
  {
    return FunctionResult{value, YAML::Node()};
  }

  if (!iter->second.value.IsDefined())
  {
    return FunctionResult{value, iter->second.style};
  }
  return iter->second;
}
