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

macro(nodeif_generate _project)

    message(STATUS "[NODEIF] nodeif_generate: package=${_project}")
    foreach(_target_path ${ARGN})

        string(REGEX REPLACE "/" "-" _target_name "${_target_path}")
        set(nodeif_generate_TARGET "${_target_name}")
        set(nodeif_generate_SOURCE "${_target_path}")

        add_custom_target(${nodeif_generate_TARGET} ALL SOURCES "${nodeif_generate_SOURCE}.yaml")
        ament_execute_extensions("nodeif_generate")

    endforeach()

endmacro(nodeif_generate)
