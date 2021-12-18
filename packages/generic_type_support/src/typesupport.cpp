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

TypeSupportLibrary TypeSupportLibrary::LoadTypeSupport(const std::string & type)
{
  constexpr auto identifier = "rosidl_typesupport_cpp";
  const auto library = rclcpp::get_typesupport_library(type, identifier);
  const auto handle = rclcpp::get_typesupport_handle(type, identifier, *library);
  return TypeSupportLibrary{handle, library};
}

TypeSupportLibrary TypeSupportLibrary::LoadIntrospection(const std::string & type)
{
  constexpr auto identifier = "rosidl_typesupport_introspection_cpp";
  const auto library = rclcpp::get_typesupport_library(type, identifier);
  const auto handle = rclcpp::get_typesupport_handle(type, identifier, *library);
  return TypeSupportLibrary{handle, library};
}

TypeSupportField::TypeSupportField(const IntrospectionField & field) : field_(field)
{
}

void TypeSupportField::Dump() const
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

TypeSupportClass::TypeSupportClass(const IntrospectionMessage & message) : message_(message)
{
  for (uint32_t i = 0; i < message_.member_count_; ++i) {
    fields_.emplace_back(message_.members_[i]);
  }
}

void TypeSupportClass::Dump() const
{
  std::cout << "namespace     : " << message_.message_namespace_ << std::endl;
  std::cout << "name          : " << message_.message_name_ << std::endl;
  std::cout << "member_count  : " << message_.member_count_ << std::endl;
  std::cout << "size_of       : " << message_.size_of_ << std::endl;
  std::cout << "members       : " << message_.members_ << std::endl;
  std::cout << "init_function : " << reinterpret_cast<void *>(message_.init_function) << std::endl;
  std::cout << "fini_function : " << reinterpret_cast<void *>(message_.fini_function) << std::endl;
}

void TypeSupportClass::CreateMemory(void *& data)
{
  data = std::malloc(message_.size_of_);
  message_.init_function(data, rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY);
}

void TypeSupportClass::DeleteMemory(void *& data)
{
  message_.fini_function(data);
  std::free(data);
  data = nullptr;
}

TypeSupportMessage TypeSupportMessage::Load(const std::string & type)
{
  return TypeSupportLibrary(TypeSupportLibrary::LoadIntrospection(type));
}

TypeSupportMessage::TypeSupportMessage(const TypeSupportLibrary & library) : library_(library)
{
}

TypeSupportClass TypeSupportMessage::GetClass() const
{
  return {*reinterpret_cast<const IntrospectionMessage *>(library_.handle->data)};
}

TypeSupportSerialization TypeSupportSerialization::Load(const std::string & type)
{
  return TypeSupportLibrary(TypeSupportLibrary::LoadIntrospection(type));
}

TypeSupportSerialization::TypeSupportSerialization(const TypeSupportLibrary & library)
: SerializationBase(library.handle), library_(library)
{
}

TypeSupportMessageMemory::TypeSupportMessageMemory(const TypeSupportMessage & message) : message_(message)
{
  message_.GetClass().CreateMemory(data_);
}

TypeSupportMessageMemory::~TypeSupportMessageMemory()
{
  message_.GetClass().DeleteMemory(data_);
}

}  // namespace generic_type_support
