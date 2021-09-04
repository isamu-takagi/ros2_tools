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

message(STATUS "[NODEIF] nodeif_cmake-extra")

# register nodeif functions
include("${nodeif_cmake_DIR}/nodeif_generate.cmake")

# register ament_package hook
ament_register_extension("ament_package" "nodeif_cmake" "nodeif_ament_package_hook.cmake")
