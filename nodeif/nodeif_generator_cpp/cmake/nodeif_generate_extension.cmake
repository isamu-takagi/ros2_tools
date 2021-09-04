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

message(STATUS "[NODEIF]   nodeif_generator_cpp-nodeif_generate_extension")

message(STATUS "[NODEIF]     TARGET=${nodeif_generate_TARGET}")
message(STATUS "[NODEIF]     SOURCE=${nodeif_generate_SOURCE}")

# camel to snake
get_filename_component(_source_path ${nodeif_generate_SOURCE} DIRECTORY)
get_filename_component(_source_name ${nodeif_generate_SOURCE} NAME)
string(REGEX REPLACE "[A-Z]" "_\\0" _source_name "${_source_name}")
string(TOLOWER "${_source_name}" _source_name)
string(REGEX REPLACE "^_" "" _source_name "${_source_name}")

# init directory
normalize_path(_build_path "${PROJECT_BINARY_DIR}/nodeif_generate_cpp")
normalize_path(_share_path "${nodeif_generator_cpp_DIR}/..")
set(_target_source "${PROJECT_SOURCE_DIR}/${nodeif_generate_SOURCE}.yaml")
set(_target_config "${_build_path}/config/${_source_path}/${_source_name}.cmake")
set(_target_output "${_build_path}/output/${_source_path}/${_source_name}.hpp")

message(STATUS "[NODEIF]     TARGET=${nodeif_generate_TARGET}-cpp")
message(STATUS "[NODEIF]     SCRIPT=${_share_path}/script/configure")
message(STATUS "[NODEIF]     SOURCE=${_target_source}")
message(STATUS "[NODEIF]     SOURCE=${_target_config}")
message(STATUS "[NODEIF]     SOURCE=${_target_output}")

# set dependency
add_custom_target(${nodeif_generate_TARGET}-cpp DEPENDS ${_target_output})
add_dependencies(${nodeif_generate_TARGET} ${nodeif_generate_TARGET}-cpp)

# generate configure_file script
add_custom_command(
    OUTPUT ${_target_config}
    COMMAND python3 ${_share_path}/script/configure
        "--package" ${PROJECT_NAME}
        "--source" ${_target_source}
        "--output" ${_target_config}
    DEPENDS
        ${_target_source}
)

# execute configure_file script
add_custom_command(
    OUTPUT ${_target_output}
    COMMAND ${CMAKE_COMMAND}
        -D INPUT=${_share_path}/template/template.hpp.in
        -D OUTPUT=${_target_output}
        -P ${_target_config}
    DEPENDS
        ${_target_config}
        ${_share_path}/template/template.hpp.in
)
