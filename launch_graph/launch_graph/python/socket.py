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

class Socket(object):

    def __init__(self, *, node, name):
        self._node = node
        self._name = name
        self._connection = None

    def _bind(self, connection):
        if self._connection:
            raise Exception("This socket is already bound.")
        self._connection = connection

        full_name = "{}.{}".format(self._node._kwargs["name"], self._name)
        print("{} = {}".format(full_name, self._connection._name))
        self._node._kwargs.setdefault("remappings", [])
        self._node._kwargs["remappings"].append((self._name, self._connection._name))
        print(self._node._kwargs["remappings"])
