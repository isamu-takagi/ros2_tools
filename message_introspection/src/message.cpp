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

#include "message.hpp"
#include "debug.hpp"
#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/serialized_message.hpp>
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
}

std::shared_ptr<GenericMessage> GenericMessageSupport::Deserialize(const std::shared_ptr<rclcpp::SerializedMessage> serialized)
{
  auto message = std::make_shared<GenericMessage>(shared_from_this());
  serialization_->deserialize_message(serialized.get(), message->message_);
  return message;
}


YAML::Node parse_dict(uint8_t * data, const MetaMessage * message)
{

}

YAML::Node GenericMessageSupport::DeserializeYAML()
{
  YAML::Node node;
  node["aaa"] = 123;
  node["bbb"] = "str";
  return node;
}

GenericMessage::GenericMessage(const std::shared_ptr<GenericMessageSupport> & support)
{
  support_ = support;
  message_ = static_cast<uint8_t*>(std::malloc(support_->meta_message_->size_of_));
  support_->meta_message_->init_function(message_, rosidl_runtime_cpp::MessageInitialization::SKIP);
}

GenericMessage::~GenericMessage()
{
  support_->meta_message_->fini_function(message_);
  std::free(message_);
}

template <>
int32_t GenericMessage::Access<int32_t>(const GenericAccessor accessor)
{
  (void)accessor;
  return 0;
}



}  // namespace generic_type_access
