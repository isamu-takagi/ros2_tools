# Copyright 2021 Takagi Isamu
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from .connection import Connection
from .nodeif import NodeIF


class Graph(object):

    def __init__(self):
        self._nodes = []
        self._links = []
        # self._binds = [] (node-link)

    def nodeif(self, node, *args, **kwargs):
        entity = NodeIF(node, *args, **kwargs)
        self._nodes.append(entity)
        return entity

    def connection(self, **kwargs):
        entity = Connection(**kwargs)
        self._links.append(entity)
        return entity

    def description(self):
        return [node.action() for node in self._nodes]

    def dump(self):
        print("!! graph !!")
