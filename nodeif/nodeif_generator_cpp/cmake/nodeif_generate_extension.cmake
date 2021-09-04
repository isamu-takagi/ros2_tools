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

message(STATUS "[NODEIF] nodeif_generator_cpp-nodeif_generate_extension")
message(STATUS "[NODEIF]   TARGET=${nodeif_generator_TARGET}")
message(STATUS "[NODEIF]   FILES=${nodeif_generator_FILES}")

# init directory
normalize_path(_build_path "${PROJECT_BINARY_DIR}/nodeif_generator_cpp")
normalize_path(_share_path "${nodeif_generator_cpp_DIR}/..")

# create output files and path tuples
set(_output_files "")
set(_path_tuples "")
foreach(_input_file ${nodeif_generator_FILES})
  get_filename_component(_folder ${_input_file} DIRECTORY)
  get_filename_component(_camel ${_input_file} NAME_WE)
  string(REGEX REPLACE "[A-Z]" "_\\0" _snake "${_camel}")
  string(TOLOWER "${_snake}" _snake)
  string(REGEX REPLACE "^_" "" _snake "${_snake}")
  list(APPEND _output_files "${_build_path}/output/${_folder}/${_snake}.hpp")
  list(APPEND _path_tuples "${_folder}\;${_snake}\;${_camel}")
endforeach()

# set dependencies for cpp main target
add_custom_target(${nodeif_generator_TARGET}__cpp DEPENDS ${_output_files})
add_dependencies(${nodeif_generator_TARGET} ${nodeif_generator_TARGET}__cpp)

# set dependencies for each file
foreach(_path_tuple ${_path_tuples})

  list(GET _path_tuple 0 _folder)
  list(GET _path_tuple 1 _snake)
  list(GET _path_tuple 2 _camel)
  set(_target_source "${PROJECT_SOURCE_DIR}/${_folder}/${_camel}.yaml")
  set(_target_config "${_build_path}/config/${_folder}/${_snake}.cmake")
  set(_target_output "${_build_path}/output/${_folder}/${_snake}.hpp")

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

endforeach()

# install output files if exists
if(EXISTS ${_build_path}/output)
  install(
    DIRECTORY ${_build_path}/output/
    DESTINATION include/${PROJECT_NAME}
    PATTERN *.hpp
  )
  ament_export_include_directories(include)
endif()
