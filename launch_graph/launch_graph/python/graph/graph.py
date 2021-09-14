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


from .utils import connect_graph_process
from .utils import connect_graph_channel
from .utils import connect_socket_process
from .utils import connect_socket_channel


class GraphObject(object):

    def dump(self, indent, kind):
        return "{}{}[{}] {}".format(" " * indent, kind, self.__class__.__name__, self._identifier)

    @property
    def _identifier(self):
        return id(self)


class Graph(object):

    PROCESS = 'process'
    CHANNEL = 'channel'

    def __init__(self):
        self._elements = {}
        self._elements[Graph.PROCESS] = []
        self._elements[Graph.CHANNEL] = []

    def descriptions(self):
        entities = []
        for process in self._elements[Graph.PROCESS]:
            entities.extend(process.descriptions())
        for channel in self._elements[Graph.CHANNEL]:
            entities.extend(channel.descriptions())
        return entities

    def dump(self, indent=0):
        print("PROCESS:")
        for process in self._elements[Graph.PROCESS]:
            print(process.dump(indent + 2))
            for socket in process._sockets.values():
                print(socket.dump(indent + 4))
        print("CHANNEL:")
        for channel in self._elements[Graph.CHANNEL]:
            print(channel.dump(indent + 2))
            for socket in channel._sockets:
                print(socket.dump(indent + 4))
        print("SOCKET:")
        sockets = sum((list(process._sockets.values()) for process in self._elements[Graph.PROCESS]), [])
        for socket in sockets:
            print(socket.dump(indent + 2))
            print(socket._process.dump(indent + 4) if socket._process else "    None")
            print(socket._channel.dump(indent + 4) if socket._channel else "    None")


class Process(GraphObject):

    def __init__(self, graph: Graph):
        self._graph = None
        self._sockets = {}
        connect_graph_process(graph, self)

    def descriptions(self):
        return []

    def dump(self, indent=0):
        return super().dump(indent, "P")

    def _socket(self, type: type, name: str):
        socket = self._sockets.get((type, name))
        if socket:
            return socket
        return type(self, name)

    def _connected_channels(self):
        for key, val in self._sockets.items():
            print(key, val)
        return ["aaaaa"]



class Channel(GraphObject):

    def __init__(self, graph: Graph):
        self._graph = None
        self._sockets = []
        connect_graph_channel(graph, self)

    def descriptions(self):
        return []

    def dump(self, indent=0):
        return super().dump(indent, "C")

    def bind(self, sockets):
        for socket in sockets:
            connect_socket_channel(socket, self)

    # def processes (graph search)


class Socket(GraphObject):

    # TODO(Takgai, Isamu): apply others
    PROCESS_TYPE = Process
    CHANNEL_TYPE = Channel

    def __init__(self, process: Process):
        self._process = None
        self._channel = None
        connect_socket_process(self, process)

    def dump(self, indent=0):
        return super().dump(indent,  "S")

    # def channel (graph search)
