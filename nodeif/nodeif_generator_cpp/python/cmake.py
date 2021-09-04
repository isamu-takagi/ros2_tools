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

class ConfigureFileScript(object):

    def __init__(self):
        self._params = {}

    def configure(self, name, data):
        self._params[name] = data

    def dump(self):
        import yaml
        return yaml.safe_dump(self._params)

    def text(self):
        params = ['set({} "{}")'.format(*param) for param in self._params.items()]
        config = ['configure_file(${INPUT} ${OUTPUT} @ONLY)']
        return '\n'.join(params + config + [''])
