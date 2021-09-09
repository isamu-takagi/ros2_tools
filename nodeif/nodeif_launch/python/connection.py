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

class Connection(object):

    def __init__(self, *, name, type):
        self._name = name
        self._type = type

    def bind(self, sockets):
        for socket in sockets:
            if type(socket) is tuple:
                process, socket_name = socket
                socket = process.socket(socket_name)
            socket._bind(self)
