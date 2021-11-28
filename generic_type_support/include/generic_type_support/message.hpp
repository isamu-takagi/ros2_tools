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

#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_

#include "typesupport.hpp"

#include <yaml-cpp/yaml.h>
#include <memory>

namespace generic_type_support
{

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

#endif  // MESSAGE_HPP_
