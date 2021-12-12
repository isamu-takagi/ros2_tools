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

#ifndef FUNCTION_HPP_
#define FUNCTION_HPP_

#include "style.hpp"
#include <yaml-cpp/yaml.h>

struct FunctionResult
{
  YAML::Node value;
  StyleDefinition style;
};

class SwitchFunction
{
public:
  SwitchFunction(const YAML::Node & yaml);
  FunctionResult Apply(const YAML::Node & value /* TODO: use FunctionResult */) const;

private:
  std::map<std::string, FunctionResult> cases;
};

#endif  // FUNCTION_HPP_
