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

std::unique_ptr<BaseFunction> CreateFunction(const YAML::Node & rule)
{
  const auto func = rule["func"].as<std::string>();
  if (func == "switch")
  {
    return std::make_unique<SwitchFunction>(rule);
  }
  std::cout << "unknown function" << std::endl;
  return nullptr;
}

void FunctionRules::Load(const YAML::Node & rules)
{
  // TODO: check reload
  if (!rules) { return; }

  for (const auto & rule : rules)
  {
    functions_.push_back(CreateFunction(rule));
  }
}

FunctionResult FunctionRules::Apply(const FunctionResult & base) const
{
  FunctionResult result = base;
  for (const auto & function : functions_)
  {
    result = function->Apply(result);
  }
  return result;
}

FunctionResult BaseFunction::Apply(const FunctionResult & base, const FunctionResult & input)
{
  return FunctionResult{input.value ? input.value : base.value, base.style.Merge(input.style)};
}

SwitchFunction::SwitchFunction(const YAML::Node & yaml)
{
  for (const auto & node : yaml["args"])
  {
    const auto key = node.first.as<std::string>();
    cases.insert(std::make_pair(key, FunctionResult{node.second["value"], node.second["style"]}));
  }
}

FunctionResult SwitchFunction::Apply(const FunctionResult & base) const
{
  const auto iter = cases.find(base.value.as<std::string>());
  if (iter == cases.end())
  {
    return base;
  }
  return BaseFunction::Apply(base, iter->second);
}
