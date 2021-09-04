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

function(nodeif_get_typesupport_target variable target typesupport)

  if(NOT TARGET ${target})
    message(FATAL_ERROR "${target} is not a CMake target. Maybe nodeif_generate was given a different target name?")
  endif()

  set(output_target "${target}__${typesupport}")

  if(NOT TARGET ${output_target})
    message(FATAL_ERROR "${output_target} is not a CMake target - maybe the typesupport '${typesupport}' doesn't exist?")
  endif()

  set("${variable}" "${output_target}" PARENT_SCOPE)

endfunction()
