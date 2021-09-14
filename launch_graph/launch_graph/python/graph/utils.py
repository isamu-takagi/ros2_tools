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


def check_connection_object(object, expect):
    if not isinstance(object, expect):
        object_type = object.__class__.__name__
        expect_type = expect.__name__
        raise TypeError("Connection target expects {} but {}.".format(expect_type, object_type))


def connect_graph_process(graph, process):
    from .graph import Graph
    from .graph import Process
    check_connection_object(graph, Graph)
    check_connection_object(process, Process)
    process._graph = graph
    graph._elements[Graph.PROCESS].append(process)


def connect_graph_channel(graph, channel):
    from .graph import Graph
    from .graph import Channel
    check_connection_object(graph, Graph)
    check_connection_object(channel, Channel)
    channel._graph = graph
    graph._elements[Graph.CHANNEL].append(channel)


def connect_socket_process(socket, process):
    from .graph import Socket
    check_connection_object(socket, Socket)
    check_connection_object(process, socket.PROCESS_TYPE)
    if socket._process:
        raise Exception("overwrite socket process")
    if (socket.__class__, socket._identifier) in process._sockets:
        raise Exception("conflict socket identifier")
    socket._process = process
    process._sockets[socket.__class__, socket._identifier] = socket


def connect_socket_channel(socket, channel):
    from .graph import Socket
    from .graph import Channel
    check_connection_object(socket, Socket)
    check_connection_object(channel, socket.CHANNEL_TYPE)
    if socket._channel:
        raise Exception("overwrite socket channel")
    socket._channel = channel
    channel._sockets.append(socket)
