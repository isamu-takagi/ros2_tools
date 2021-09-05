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

import argparse
import pathlib
import yaml
from .cmake import ConfigureFileScript
from .nodeif import InterfaceSpecification
from . import utils


def configure():

    parser = argparse.ArgumentParser(description='Generate the node interface for C++.')
    parser.add_argument('--package', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=False)
    args = parser.parse_args()

    path = pathlib.Path(args.source)
    info = yaml.safe_load(path.read_text())
    spec = InterfaceSpecification(path.stem, info)

    script = ConfigureFileScript()
    script.configure('NODEIF_SOCKET',        spec.socket)
    script.configure('NODEIF_TYPE',          spec.type.capitalize())
    script.configure('NODEIF_NAME',          spec.name)
    script.configure('NODEIF_QOS_CODE',      create_qos_code(spec))
    script.configure('NODEIF_INCLUDE_GUARD', create_include_guard(args.package, spec))
    script.configure('NODEIF_NAMESPACE',     create_namespace(args.package))
    script.configure('NODEIF_DATA_NAME',     create_data_name(spec))
    script.configure('NODEIF_DATA_HEADER',   create_data_header(spec))

    if args.output:
        path = pathlib.Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(script.text())
    else:
        print(script.text())


def create_qos_code(spec):
    return 'rclcpp::QoS(7)' ## TODO(Takagi, Isamu): temporary


def create_include_guard(package, spec):
    socket = utils.camel_to_snake(spec.socket)
    return '__'.join([package, 'nodeif', socket]).upper()


def create_namespace(package):
    return '{}::socket'.format(package)


def create_data_name(spec):
    return '::'.join(spec.data)


def create_data_header(spec):
    last = utils.camel_to_snake(spec.data[-1]) + '.hpp'
    return '/'.join(spec.data[:-1] + (last,))
