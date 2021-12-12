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
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <iostream>  // DEBUG

namespace generic_type_support
{

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

GenericMessageSupport::GenericMessageSupport(const std::string & type)
{
  introspection_ = IntrospectionMessage::Load(type);
  serialization_ = MessageSerialization::Load(type);

  /*
  introspection_->Dump();
  for (const auto field : *introspection_)
  {
    std::cout << std::endl;
    field.Dump();
  }
  */
}

YAML::Node parse_message(const void * data, const TypeSupportMessage & message);

YAML::Node parse_primitive(const void * data, const TypeSupportField & field)
{
  using namespace rosidl_typesupport_introspection_cpp;

  switch (field.type_id_)
  {
    case ROS_TYPE_FLOAT:
      return YAML::Node(*reinterpret_cast<const float*>(data));
    case ROS_TYPE_DOUBLE:
      return YAML::Node(*reinterpret_cast<const double*>(data));
    case ROS_TYPE_LONG_DOUBLE:
      return YAML::Node(*reinterpret_cast<const long double*>(data));
    case ROS_TYPE_CHAR:
      return YAML::Node(*reinterpret_cast<const char*>(data));
    case ROS_TYPE_WCHAR:
      return YAML::Node("[WCHAR IS NOT IMPLEMENTED]");
    case ROS_TYPE_BOOLEAN:
      return YAML::Node(*reinterpret_cast<const bool*>(data));
    case ROS_TYPE_OCTET:
      return YAML::Node(*reinterpret_cast<const uint8_t*>(data));
    case ROS_TYPE_UINT8:
      return YAML::Node(*reinterpret_cast<const uint8_t*>(data));
    case ROS_TYPE_INT8:
      return YAML::Node(*reinterpret_cast<const int8_t*>(data));
    case ROS_TYPE_UINT16:
      return YAML::Node(*reinterpret_cast<const uint16_t*>(data));
    case ROS_TYPE_INT16:
      return YAML::Node(*reinterpret_cast<const int16_t*>(data));
    case ROS_TYPE_UINT32:
      return YAML::Node(*reinterpret_cast<const uint32_t*>(data));
    case ROS_TYPE_INT32:
      return YAML::Node(*reinterpret_cast<const int32_t*>(data));
    case ROS_TYPE_UINT64:
      return YAML::Node(*reinterpret_cast<const uint64_t*>(data));
    case ROS_TYPE_INT64:
      return YAML::Node(*reinterpret_cast<const int64_t*>(data));
    case ROS_TYPE_STRING:
      return YAML::Node(*reinterpret_cast<const std::string*>(data));
    case ROS_TYPE_WSTRING:
      return YAML::Node("[WSTRING IS NOT IMPLEMENTED]");
    case ROS_TYPE_MESSAGE:
      return parse_message(data, *reinterpret_cast<const TypeSupportMessage*>(field.members_->data));
  }
  return YAML::Node("[PARSE_ERROR]");
}

YAML::Node parse_array(const void * data, const TypeSupportField & field)
{
  YAML::Node node;
  size_t size = field.size_function(data);
  for (size_t i = 0; i < size; ++i)
  {
    const void * element = field.get_const_function(data, i);
    node.push_back(parse_primitive(element, field));
  }
  return node;
}

YAML::Node parse_field(const void * data, const TypeSupportField & field)
{
  if (field.is_array_)
  {
    return parse_array(data, field);
  }
  return parse_primitive(data, field);
}

YAML::Node parse_message(const void * data, const TypeSupportMessage & message)
{
  YAML::Node node;
  for (uint32_t i = 0; i < message.member_count_; ++i)
  {
    const TypeSupportField & field = message.members_[i];
    node[field.name_] = parse_field(static_cast<const uint8_t*>(data) + field.offset_, field);
  }
  return node;
}

YAML::Node GenericMessageSupport::DeserializeYAML(const rclcpp::SerializedMessage & serialized)
{
  std::cout << "allocate size: " << introspection_->message_.size_of_ << std::endl;

  void * data = static_cast<uint8_t*>(std::malloc(introspection_->message_.size_of_));
  introspection_->message_.init_function(data, rosidl_runtime_cpp::MessageInitialization::SKIP);
  serialization_->deserialize_message(&serialized, data);

  YAML::Node yaml = parse_message(data, introspection_->message_);

  introspection_->message_.fini_function(data);
  std::free(data);

  std::cout << "delete message" << std::endl;

  return yaml;
}

}  // namespace generic_type_support
