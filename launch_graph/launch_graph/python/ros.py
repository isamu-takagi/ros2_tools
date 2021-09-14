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

from .graph import Graph
from .graph import Process
from .graph import Channel
from .graph import Socket
from launch_ros.actions import Node as RosNode

class Node(Process):

    def __init__(self, graph: Graph, **kwargs):
        self._kwargs = kwargs
        super().__init__(graph)

    def descriptions(self):
        print(self._connected_channels())
        return [RosNode(**self._kwargs)]

    @property
    def _identifier(self):
        return self._kwargs['executable']


class TopicSocket(Socket):

    type_process = Node

    def __init__(self, node: Node, name: str, **kwargs):
        self._name = name
        self._kwargs = kwargs
        super().__init__(node)

    @property
    def _identifier(self):
        return self._name


class Topic(Channel):

    def __init__(self, graph: Graph, **kwargs):
        self._kwargs = kwargs
        super().__init__(graph)

    def bind(self, sockets):
        super().bind(map(self.__convert_socket, sockets))

    @staticmethod
    def __convert_socket(socket):
        if isinstance(socket, TopicSocket):
            return socket
        return socket[0]._socket(TopicSocket, socket[1])

    @property
    def _identifier(self):
        return self._kwargs['name']
