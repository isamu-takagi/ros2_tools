# Copyright 2021 Takagi Isamu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

message(STATUS "[NODEIF] nodeif_generators-extra")

ament_index_get_resources(_nodeif_generator_packages "nodeif_generator_packages")
set(_dependent_packages "nodeif_cmake" ${_nodeif_generator_packages})

foreach(_dependent_package ${_dependent_packages})
  find_package("${_dependent_package}")
endforeach()
