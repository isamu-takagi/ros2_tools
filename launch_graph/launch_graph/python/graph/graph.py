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

from typing import Sequence, Type
from .utils import check_is_subclass
from .utils import check_is_instance


class Graph(object):

    def __init__(self):
        self.__elements = {}
        self.__elements[Process] = []
        self.__elements[Channel] = []
        self.__elements[Socket] = []

    def descriptions(self):
        entities = []
        for process in self.__elements[Process]:
            entities.extend(process._descriptions())
        for channel in self.__elements[Channel]:
            entities.extend(channel._descriptions())
        return entities

    def dump(self):
        print("PROCESS:")
        for process in self.__elements[Process]:
            print("  {}".format(process))
            for socket in process._Process__sockets.values():
                print("    {}".format(socket))
        print("CHANNEL:")
        for channel in self.__elements[Channel]:
            print("  {}".format(channel))
            for socket in channel._Channel__sockets:
                print("    {}".format(socket))
        print("SOCKET:")
        for socket in self.__elements[Socket]:
            print("  {}".format(socket))
            print("    {}".format(socket._process))
            print("    {}".format(socket._channel))


class GraphElement(object):

    def __init__(self, element_class: type, graph: Graph):
        check_is_instance(graph, Graph)
        check_is_instance(self, element_class)
        self.__graph = graph
        self.__graph._Graph__elements[element_class].append(self)

    def __str__(self, kind="X", info=None):
        info = info or "0x{:x}".format(id(self))
        return "{}[{}] ({})".format(kind, self.__class__.__name__, info)

    def _descriptions(self):
        return []

    @property
    def _graph(self):
        return self.__graph


class Socket(GraphElement):

    def __init__(self, graph: Graph, name: str):
        super().__init__(Socket, graph)
        self.__name = name
        self.__process = None
        self.__channel = None

    def __str__(self):
        return super().__str__("S", self._name)

    def _register_process(self, process: GraphElement):
        check_is_instance(process, self._process_class)
        if self.__process:
            raise Exception("overwrite socket process")
        self.__process = process

    def _register_channel(self, channel: GraphElement):
        check_is_instance(channel, self._channel_class)
        if self.__channel:
            raise Exception("overwrite socket channel")
        self.__channel = channel

    @property
    def _process_class(self):
        return Process

    @property
    def _channel_class(self):
        return Channel

    @property
    def _name(self):
        return self.__name

    @property
    def _process(self):
        # TODO: graph search
        return self.__process

    @property
    def _channel(self):
        # TODO: graph search
        return self.__channel


class Process(GraphElement):

    def __init__(self, graph: Graph):
        super().__init__(Process, graph)
        self.__sockets = {}

    def __str__(self, info=None):
        return super().__str__("P", info)

    def __create_socket(self, socket_class: Type[Socket], socket_name: str):
        check_is_subclass(socket_class, Socket)
        socket = socket_class(self._graph, socket_name)
        socket._register_process(self)
        self.__sockets[socket_class, socket_name] = socket
        return socket

    def _socket(self, socket_class: Type[Socket], socket_name: str):
        socket = self.__sockets.get((socket_class, socket_name))
        return socket or self.__create_socket(socket_class, socket_name)

    def _search_sockets(self, socket_class):
        for socket in self.__sockets.values():
            if isinstance(socket, socket_class):
                yield socket


class Channel(GraphElement):

    def __init__(self, graph: Graph):
        super().__init__(Channel, graph)
        self.__sockets = []

    def __str__(self, info=None):
        return super().__str__("C", info)

    def _bind(self, socket_class: Type[Socket], sockets: Sequence[Socket]):
        for socket in sockets:
            if type(socket) is tuple:
                process, socket_name = socket
                if not isinstance(process, Process):
                    raise TypeError("")  # TODO: error message
                if not isinstance(socket_name, str):
                    raise TypeError("")  # TODO: error message
                socket = process._socket(socket_class, socket_name)
            check_is_instance(socket, Socket)
            socket._register_channel(self)
            self.__sockets.append(socket)

    @property
    def Socket(self):
        return Socket(self.__class__)
