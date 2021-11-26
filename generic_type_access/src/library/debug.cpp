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

#include "debug.hpp"

#include <iostream>

namespace generic_type_access
{

void debug(const MetaMessage & message, int indent);
void debug(const MetaField & message, int indent);

void debug(const rosidl_message_type_support_t * handle, int indent)
{
  debug(*reinterpret_cast<const MetaMessage *>(handle->data), indent);
}

void debug(const MetaMessage & message, int indent)
{
  std::cout << message.message_namespace_ << "::" << message.message_name_;
  std::cout << " (" << message.size_of_ << ")";

  std::cout << " [" << &message << "]" << std::endl;
  for (uint32_t i = 0; i < message.member_count_; ++i)
  {
    debug(message.members_[i], indent + 2);
  }
}

void debug(const MetaField & field, int indent)
{
  const auto space = std::string(indent, ' ');
  std::cout << space << field.name_;
  std::cout << " (" << static_cast<uint32_t>(field.type_id_) << ", " << field.offset_ << ")";
  std::cout << " [" << &field << "] ";
  if (field.members_)
  {
    debug(field.members_, indent);
  }
  else
  {
    std::cout << std::endl;
  }
}

void dump(const MetaMessage & message, int indent)
{
  (void)indent;
  std::cout << "message_namespace : " << message.message_namespace_ << std::endl;
  std::cout << "message_name      : " << message.message_name_ << std::endl;
  std::cout << "member_count      : " << message.member_count_ << std::endl;
  std::cout << "size_of           : " << message.size_of_ << std::endl;
  std::cout << "members           : " << message.members_ << std::endl;
  std::cout << "init_function     : " << (void*)message.init_function << std::endl;
  std::cout << "fini_function     : " << (void*)message.fini_function << std::endl;
}

void dump(const MetaField & field, int indent)
{
  (void)indent;
  std::cout << "name               : " << field.name_ << std::endl;
  std::cout << "type_id            : " << static_cast<uint32_t>(field.type_id_) << std::endl;
  std::cout << "string_upper_bound : " << field.string_upper_bound_ << std::endl;
  std::cout << "members            : " << field.members_ << std::endl;
  std::cout << "is_array           : " << field.is_array_ << std::endl;
  std::cout << "array_size         : " << field.array_size_ << std::endl;
  std::cout << "is_upper_bound     : " << field.is_upper_bound_ << std::endl;
  std::cout << "offset             : " << field.offset_ << std::endl;
  std::cout << "default_value      : " << field.default_value_ << std::endl;
  std::cout << "size_function      : " << field.size_function << std::endl;
  std::cout << "get_const_function : " << field.get_const_function << std::endl;
  std::cout << "get_function       : " << field.get_function << std::endl;
  std::cout << "resize_function    : " << field.resize_function << std::endl;
}

}  // namespace generic_type_access
