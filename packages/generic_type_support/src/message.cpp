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

#include "generic_type_support/message.hpp"

#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/serialized_message.hpp>

#include <iostream>  // DEBUG

namespace generic_type_support
{

#if 0
std::vector<std::string> split(const std::string & input)
{
  std::vector<std::string> result;
  size_t found = input.find('.');
  size_t start = 0;
  while(found != std::string::npos)
  {
    result.push_back(input.substr(start, found - start));
    start = found + 1;
    found = input.find('.', start);
  }
  result.push_back(input.substr(start));
  return result;
}

GeneticTypeAccess::GeneticTypeAccess(const std::string access)
{
  for (const auto & name : split(access))
  {
    GenericTypeAccessField field;
    field.name = name;
    fields.push_back(field);
  }
  debug = access;
}

const YAML::Node GeneticTypeAccess::Get(const YAML::Node & yaml) const
{
  YAML::Node node = yaml;
  for (const auto & field : fields)
  {
    node.reset(node[field.name]);
  }
  return node;
}
#endif

GenericMessageSupport::GenericMessageSupport(const std::string & type)
: introspection_(type), serialization_(type)
{
}

YAML::Node GenericMessageSupport::DeserializeYAML(const rclcpp::SerializedMessage & serialized)
{
  TypeSupportMessageMemory memory(introspection_);
  serialization_.GetSerialization().deserialize_message(&serialized, memory.GetData());
  return introspection_.GetClass().Get<YAML::Node>(memory.GetData());
}

}  // namespace generic_type_support
