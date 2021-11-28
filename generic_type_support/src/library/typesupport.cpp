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

#include "generic_type_support/typesupport.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace generic_type_support
{

std::shared_ptr<IntrospectionMessage> IntrospectionMessage::Load(const std::string & type)
{
  constexpr auto identifier = "rosidl_typesupport_introspection_cpp";
  const auto library = rclcpp::get_typesupport_library(type, identifier);
  const auto handle = rclcpp::get_typesupport_handle(type, identifier, *library);
  return std::make_shared<IntrospectionMessage>(library, handle);
}

IntrospectionMessage::IntrospectionMessage(const TypeSupportLibrary & library, const TypeSupportHandle * handle)
: library_(library), message_(*reinterpret_cast<const TypeSupportMessage *>(handle->data))
{
  for (uint32_t i = 0; i < message_.member_count_; ++i) {
    fields_.emplace_back(message_.members_[i]);
  }
}

void IntrospectionMessage::Dump() const
{
  std::cout << "namespace     : " << message_.message_namespace_ << std::endl;
  std::cout << "name          : " << message_.message_name_ << std::endl;
  std::cout << "member_count  : " << message_.member_count_ << std::endl;
  std::cout << "size_of       : " << message_.size_of_ << std::endl;
  std::cout << "members       : " << message_.members_ << std::endl;
  std::cout << "init_function : " << reinterpret_cast<void *>(message_.init_function) << std::endl;
  std::cout << "fini_function : " << reinterpret_cast<void *>(message_.fini_function) << std::endl;
}

IntrospectionField::IntrospectionField(const TypeSupportField & field)
: field_(field)
{
}

void IntrospectionField::Dump() const
{
  std::cout << "name               : " << field_.name_ << std::endl;
  std::cout << "type_id            : " << static_cast<uint32_t>(field_.type_id_) << std::endl;
  std::cout << "string_upper_bound : " << field_.string_upper_bound_ << std::endl;
  std::cout << "members            : " << field_.members_ << std::endl;
  std::cout << "is_array           : " << field_.is_array_ << std::endl;
  std::cout << "array_size         : " << field_.array_size_ << std::endl;
  std::cout << "is_upper_bound     : " << field_.is_upper_bound_ << std::endl;
  std::cout << "offset             : " << field_.offset_ << std::endl;
  std::cout << "default_value      : " << field_.default_value_ << std::endl;
  std::cout << "size_function      : " << field_.size_function << std::endl;
  std::cout << "get_const_function : " << field_.get_const_function << std::endl;
  std::cout << "get_function       : " << field_.get_function << std::endl;
  std::cout << "resize_function    : " << field_.resize_function << std::endl;
}

std::shared_ptr<MessageSerialization> MessageSerialization::Load(const std::string & type)
{
  constexpr auto identifier = "rosidl_typesupport_cpp";
  const auto library = rclcpp::get_typesupport_library(type, identifier);
  const auto handle = rclcpp::get_typesupport_handle(type, identifier, *library);
  return std::make_shared<MessageSerialization>(library, handle);
}

MessageSerialization::MessageSerialization(const TypeSupportLibrary & library, const TypeSupportHandle * handle)
: SerializationBase(handle), library_(library)
{
}


}  // namespace generic_type_support
