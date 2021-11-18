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

from launch_ros.actions import Node as RosNode
from .graph import Graph
from .graph import Process
from .graph import Channel
from .graph import Socket


class LaunchGraph(Graph):
    pass


class TopicSocket(Socket):
    pass


class Topic(Channel):

    def __init__(self, graph: Graph, name, type):
        super().__init__(graph)
        self._name = name
        self._type = type

    def __str__(self):
        return super().__str__(self._name)

    def bind(self, sockets):
        super()._bind(TopicSocket, sockets)


class Node(Process):

    def __init__(self, graph: Graph, **kwargs):
        super().__init__(graph)
        self._kwargs = kwargs

    def __str__(self):
        return super().__str__(self._kwargs['name'])

    def _descriptions(self):
        self._kwargs["remappings"] = []
        for socket in self._search_sockets(TopicSocket):
            self._kwargs["remappings"].append((socket._name, socket._channel._name))
        return [RosNode(**self._kwargs)]
