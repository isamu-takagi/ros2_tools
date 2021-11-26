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

#include "generic_type_access/message.hpp"
#include "debug.hpp"
#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <iostream>

namespace generic_type_access
{

GenericMessageSupport::GenericMessageSupport(const std::string & type)
{
	constexpr auto type_identifier = "rosidl_typesupport_cpp";
	constexpr auto meta_identifier = "rosidl_typesupport_introspection_cpp";

  type_library_ = rclcpp::get_typesupport_library(type, type_identifier);
  meta_library_ = rclcpp::get_typesupport_library(type, meta_identifier);

  const auto type_handle_ = rclcpp::get_typesupport_handle(type, type_identifier, *type_library_);
  const auto meta_handle_ = rclcpp::get_typesupport_handle(type, meta_identifier, *meta_library_);

  serialization_ = std::make_unique<rclcpp::SerializationBase>(type_handle_);
  meta_message_ = reinterpret_cast<const MetaMessage *>(meta_handle_->data);
  dump(*meta_message_);
  for (uint32_t i = 0; i < meta_message_->member_count_; ++i)
  {
    std::cout << std::endl;
    dump(meta_message_->members_[i]);
  }
}

std::shared_ptr<GenericMessage> GenericMessageSupport::Deserialize(const std::shared_ptr<rclcpp::SerializedMessage> serialized)
{
  auto message = std::make_shared<GenericMessage>(shared_from_this());
  serialization_->deserialize_message(serialized.get(), message->message_);
  return message;
}

YAML::Node parse_message(const void * data, const MetaMessage & message);

YAML::Node parse_primitive(const void * data, const MetaField & field)
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
      return parse_message(data, *reinterpret_cast<const MetaMessage*>(field.members_->data));
  }
  return YAML::Node("[PARSE_ERROR]");
}

YAML::Node parse_array(const void * data, const MetaField & field)
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

YAML::Node parse_field(const void * data, const MetaField & field)
{
  if (field.is_array_)
  {
    return parse_array(data, field);
  }
  return parse_primitive(data, field);
}

YAML::Node parse_message(const void * data, const MetaMessage & message)
{
  YAML::Node node;
  for (uint32_t i = 0; i < message.member_count_; ++i)
  {
    const MetaField & field = message.members_[i];
    node[field.name_] = parse_field(static_cast<const uint8_t*>(data) + field.offset_, field);
  }
  return node;
}

YAML::Node GenericMessageSupport::DeserializeYAML(const std::shared_ptr<rclcpp::SerializedMessage> serialized)
{
  const auto message = Deserialize(serialized);
  return parse_message(message->message_, *meta_message_);
}

GenericMessage::GenericMessage(const std::shared_ptr<GenericMessageSupport> & support)
{
  std::cout << "init deserialize message" << std::endl;
  support_ = support;
  message_ = static_cast<uint8_t*>(std::malloc(support_->meta_message_->size_of_));
  support_->meta_message_->init_function(message_, rosidl_runtime_cpp::MessageInitialization::SKIP);
}

GenericMessage::~GenericMessage()
{
  std::cout << "fini deserialize message" << std::endl;
  support_->meta_message_->fini_function(message_);
  std::free(message_);
}

}  // namespace generic_type_access
