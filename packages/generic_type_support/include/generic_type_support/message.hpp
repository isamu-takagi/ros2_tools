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

#ifndef GENERIC_TYPE_SUPPORT__MESSAGE_HPP_
#define GENERIC_TYPE_SUPPORT__MESSAGE_HPP_

#include "typesupport.hpp"

#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>

namespace generic_type_support
{

struct GenericTypeAccessField
{
  enum class Type {DATA, LIST, DICT};
  Type type;
  std::string name;
  std::string key;
  int index;
};

struct GeneticTypeAccess
{
public:
  GeneticTypeAccess() = default;
  GeneticTypeAccess(const std::string access);
  const YAML::Node Get(const YAML::Node & yaml) const;

public:
  std::string debug;
private:
  std::vector<GenericTypeAccessField> fields;
};

class GenericMessageSupport
{
public:
  GenericMessageSupport(const std::string & type);
  YAML::Node DeserializeYAML(const rclcpp::SerializedMessage & serialized);

private:
  std::shared_ptr<IntrospectionMessage> introspection_;
  std::shared_ptr<MessageSerialization> serialization_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__MESSAGE_HPP_
