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

#include "generic_type_support/access.hpp"

#include <iostream>

namespace generic_type_support
{

std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::vector<std::string> result;
  size_t found = input.find(delimiter);
  size_t start = 0;
  while(found != std::string::npos)
  {
    result.push_back(input.substr(start, found - start));
    start = found + 1;
    found = input.find(delimiter, start);
  }
  result.push_back(input.substr(start));
  return result;
}

GenericTypeAccessField::GenericTypeAccessField(const std::string field)
{
  const auto token = split(field, '@');
  if (token.size() == 1)
  {
    type = Type::VALUE;
    name = token[0];
    index = 0;
    return;
  }
  if (token.size() == 2)
  {
    type = Type::LIST;
    name = token[0];
    index = std::stoi(token[1]);
    return;
  }
  throw std::runtime_error("Invalid field '" + field + "'");
}

GenericTypeAccess::GenericTypeAccess(const std::string access)
{
  for (const auto & field : split(access, '.'))
  {
    fields.emplace_back(field);
  }
  debug = access;
}

const YAML::Node GenericTypeAccess::Get(const YAML::Node & yaml) const
{
  YAML::Node node = yaml;
  for (const auto & field : fields)
  {
    node.reset(node[field.name]);
  }
  return node;
}


bool GenericTypeAccess::Validate(const TypeSupportClass & support) const
{
  return true;
}

}  // namespace generic_type_support
