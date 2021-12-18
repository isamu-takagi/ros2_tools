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

#ifndef GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_
#define GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_

#include "typesupport.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <string>
#include <vector>

namespace generic_type_support
{

class TypeSupportMessage;
class TypeSupportService;
class TypeSupportSerialization;
class TypeSupportMessageMemory;

class TypeSupportClass;
class TypeSupportField;
using TypeSupportHandle = rosidl_message_type_support_t;
using IntrospectionMessage = rosidl_typesupport_introspection_cpp::MessageMembers;
using IntrospectionField = rosidl_typesupport_introspection_cpp::MessageMember;
using IntrospectionService = rosidl_typesupport_introspection_cpp::ServiceMembers;

// Do not create this class directly.
struct TypeSupportLibrary
{
  static TypeSupportLibrary LoadTypeSupport(const std::string & type);
  static TypeSupportLibrary LoadIntrospection(const std::string & type);

  const rosidl_message_type_support_t * handle;
  const std::shared_ptr<rcpputils::SharedLibrary> library;
};

// Do not create this class directly.
class TypeSupportField
{
public:
  TypeSupportField(const IntrospectionField & field);
  void Dump() const;

private:
  const IntrospectionField & field_;
};

// Do not create this class directly.
class TypeSupportClass
{
public:
  TypeSupportClass(const IntrospectionMessage & message);
  void Dump() const;
  void CreateMemory(void *& data);
  void DeleteMemory(void *& data);
  const auto begin() const { return fields_.begin(); }
  const auto end() const { return fields_.end(); }

private:
  const IntrospectionMessage & message_;
  std::vector<TypeSupportField> fields_;
};

// This is the interface class.
class TypeSupportMessage
{
public:
  static TypeSupportMessage Load(const std::string & type);
  TypeSupportMessage(const TypeSupportLibrary & library);
  TypeSupportClass GetClass() const;
  bool HasField();

private:
  const TypeSupportLibrary library_;
};

// This is the interface class.
class TypeSupportSerialization : public rclcpp::SerializationBase
{
public:
  static TypeSupportSerialization Load(const std::string & type);
  TypeSupportSerialization(const TypeSupportLibrary & library);

private:
  const TypeSupportLibrary library_;
};

class TypeSupportMessageMemory
{
public:
  TypeSupportMessageMemory(const TypeSupportMessage & message);
  ~TypeSupportMessageMemory();
  void * GetData() { return data_; };

private:
  void * data_;
  const TypeSupportMessage message_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_
