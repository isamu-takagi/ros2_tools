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

#include "types.hpp"
#include <rcpputils/shared_library.hpp>
#include <rclcpp/serialization.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>


namespace generic_type_access
{

class GenericMessageSupport;
class GenericMessage;

class GenericMessageSupport : public std::enable_shared_from_this<GenericMessageSupport>
{
public:
	friend GenericMessage;
	GenericMessageSupport(const std::string & type);  // TODO: move to private
	YAML::Node DeserializeYAML();

	std::shared_ptr<GenericMessage> Deserialize(const std::shared_ptr<rclcpp::SerializedMessage> serialized);

	// noncopyable
	GenericMessageSupport(const GenericMessageSupport&) = delete;
	GenericMessageSupport& operator=(const GenericMessageSupport&) = delete;

private:
	std::shared_ptr<rcpputils::SharedLibrary> type_library_;
	std::shared_ptr<rcpputils::SharedLibrary> meta_library_;
	std::unique_ptr<rclcpp::SerializationBase> serialization_;
	const MetaMessage * meta_message_;
};

class GenericAccessor
{
public:
	GenericAccessor(const std::string & field) : field_(field) {}
	std::string field_;
};

class GenericMessage
{
public:
	friend GenericMessageSupport;
	GenericMessage(const std::shared_ptr<GenericMessageSupport> & support);
	~GenericMessage();

	template<class T>
	T Access(const GenericAccessor accessor);

	template<class T>
	T Access(const std::string & fields) { return Access<T>(GenericAccessor(fields)); }

	// noncopyable
	GenericMessage(const GenericMessage&) = delete;
	GenericMessage& operator=(const GenericMessage&) = delete;

private:
	uint8_t * message_;
	std::shared_ptr<GenericMessageSupport> support_;
};

}  // namespace generic_type_access

#endif  // MESSAGE_HPP_
