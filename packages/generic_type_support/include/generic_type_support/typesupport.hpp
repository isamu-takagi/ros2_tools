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
#include <vector>

namespace generic_type_support
{

class IntrospectionMessage;
class IntrospectionField;
class IntrospectionService;

using TypeSupportLibrary = std::shared_ptr<rcpputils::SharedLibrary>;
using TypeSupportHandle = rosidl_message_type_support_t;
using TypeSupportMessage = rosidl_typesupport_introspection_cpp::MessageMembers;
using TypeSupportField = rosidl_typesupport_introspection_cpp::MessageMember;
using TypeSupportService = rosidl_typesupport_introspection_cpp::ServiceMembers;


class IntrospectionMessage
{
public:
  static std::shared_ptr<IntrospectionMessage> Load(const std::string & type);
  IntrospectionMessage(const TypeSupportLibrary & library, const TypeSupportHandle * handle);

  void Dump() const;

  const auto begin() const { return fields_.begin(); }
  const auto end() const { return fields_.end(); }

//private:
  const TypeSupportLibrary library_;
  const TypeSupportMessage message_;
  std::vector<IntrospectionField> fields_;
};

class IntrospectionField
{
public:
  IntrospectionField(const TypeSupportField & field);

  void Dump() const;

//private:
  const TypeSupportField field_;
};

class IntrospectionService
{
  // not implemented
};

class MessageSerialization : public rclcpp::SerializationBase
{
public:
  static std::shared_ptr<MessageSerialization> Load(const std::string & type);
  MessageSerialization(const TypeSupportLibrary & library, const TypeSupportHandle * handle);

private:
  const TypeSupportLibrary library_;
};

}  // namespace generic_type_support

#endif  // GENERIC_TYPE_SUPPORT__TYPESUPPORT_HPP_
