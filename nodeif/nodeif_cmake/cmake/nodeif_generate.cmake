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

macro(nodeif_generate target)

    # TODO(Takagi, Isamu): check ARGN
    set(nodeif_generator_TARGET "${target}")
    set(nodeif_generator_FILES "${ARGN}")
    message(STATUS "[NODEIF]   TARGET=${nodeif_generator_TARGET}")
    message(STATUS "[NODEIF]   FILES=${nodeif_generator_FILES}")

    add_custom_target(${nodeif_generator_TARGET} ALL SOURCES "${nodeif_generator_FILES}")
    ament_execute_extensions("nodeif_generate")

endmacro(nodeif_generate)
