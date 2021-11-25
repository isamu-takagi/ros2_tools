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

#include "access.hpp"

#include <memory>

#include "debug.hpp"
#include <iostream>

namespace generic_type_access
{


std::shared_ptr<GenericMessageSupport> GenericTypeAccess::GetMessageSupport(const std::string & type)
{
	std::cout << "================================================================================" << std::endl;
	std::cout << "message init: " << type << std::endl;
	if (messages_.count(type) == 0)
	{
		messages_[type] = std::make_shared<GenericMessageSupport>(type);
	}
	return messages_[type];
}

}  // namespace generic_type_access
