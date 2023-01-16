# Copyright 2015-2016 Open Source Robotics Foundation, Inc.
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

from collections import OrderedDict
import os
import re
import sys

import ament_index_python
from catkin_pkg.package import parse_package
# ROS 1 imports
import genmsg
import genmsg.msg_loader

import rosidl_adapter.parser
from rosidl_cmake import expand_template
import rosidl_parser.parser

import yaml

# import rospkg which is required by rosmsg
# and likely only available for Python 2
try:
    import rospkg
except ImportError:
    from importlib.machinery import SourceFileLoader
    import subprocess
    for python_executable in ['python2', 'python2.7']:
        try:
            rospkg_path = subprocess.check_output(
                [python_executable, '-c', 'import rospkg; print(rospkg.__file__)'])
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
        rospkg_path = rospkg_path.decode().strip()
        if rospkg_path.endswith('.pyc'):
            rospkg_path = rospkg_path[:-1]
        rospkg = SourceFileLoader('rospkg', rospkg_path).load_module()
    if not rospkg:
        raise

# more ROS 1 imports
# since ROS 2 should be sourced after ROS 1 it will overlay
# e.g. the message packages, to prevent that from happening (and ROS 1 code to
# fail) ROS 1 paths are moved to the front of the search path
rpp = os.environ.get('ROS_PACKAGE_PATH', '').split(os.pathsep)
for package_path in reversed([p for p in rpp if p]):
    if not package_path.endswith(os.sep + 'share'):
        continue
    ros1_basepath = os.path.dirname(package_path)
    for sys_path in sys.path:
        if sys_path.startswith(os.path.join(ros1_basepath, '')):
            sys.path.remove(sys_path)
            sys.path.insert(0, sys_path)
import rosmsg  # noqa


def generate_cpp(output_path, template_dir):
    rospack = rospkg.RosPack()
    data = generate_messages(rospack)
    message_string_pairs = {
        (
            '%s/%s' % (m.ros1_msg.package_name, m.ros1_msg.message_name),
            '%s/%s' % (m.ros2_msg.package_name, m.ros2_msg.message_name))
        for m in data['mappings']}
    data.update(
        generate_services(rospack, message_string_pairs=message_string_pairs))

    template_file = os.path.join(template_dir, 'get_mappings.cpp.em')
    output_file = os.path.join(output_path, 'get_mappings.cpp')
    data_for_template = {
        'mappings': data['mappings'], 'services': data['services']}
    expand_template(template_file, data_for_template, output_file)

    unique_package_names = set(data['ros2_package_names_msg'] + data['ros2_package_names_srv'])
    # skip builtin_interfaces since there is a custom implementation
    unique_package_names -= {'builtin_interfaces'}
    data['ros2_package_names'] = list(unique_package_names)

    template_file = os.path.join(template_dir, 'get_factory.cpp.em')
    output_file = os.path.join(output_path, 'get_factory.cpp')
    expand_template(template_file, data, output_file)

    for ros2_package_name in data['ros2_package_names']:
        data_pkg_hpp = {
            'ros2_package_name': ros2_package_name,
            # include directives and template types
            'mapped_ros1_msgs': [
                m.ros1_msg for m in data['mappings']
                if m.ros2_msg.package_name == ros2_package_name],
            'mapped_ros2_msgs': [
                m.ros2_msg for m in data['mappings']
                if m.ros2_msg.package_name == ros2_package_name],
            # forward declaration of factory functions
            'ros2_msg_types': [
                m for m in data['all_ros2_msgs']
                if m.package_name == ros2_package_name],
            'ros2_srv_types': [
                s for s in data['all_ros2_srvs']
                if s.package_name == ros2_package_name],
            # forward declaration of template specializations
            'mappings': [
                m for m in data['mappings']
                if m.ros2_msg.package_name == ros2_package_name],
        }
        template_file = os.path.join(template_dir, 'pkg_factories.hpp.em')
        output_file = os.path.join(
            output_path, '%s_factories.hpp' % ros2_package_name)
        expand_template(template_file, data_pkg_hpp, output_file)

        data_pkg_cpp = {
            'ros2_package_name': ros2_package_name,
            # call interface specific factory functions
            'ros2_msg_types': data_pkg_hpp['ros2_msg_types'],
            'ros2_srv_types': data_pkg_hpp['ros2_srv_types'],
        }
        template_file = os.path.join(template_dir, 'pkg_factories.cpp.em')
        output_file = os.path.join(
            output_path, '%s_factories.cpp' % ros2_package_name)
        expand_template(template_file, data_pkg_cpp, output_file)

        for interface_type, interfaces in zip(
            ['msg', 'srv'], [data['all_ros2_msgs'], data['all_ros2_srvs']]
        ):
            for interface in interfaces:
                if interface.package_name != ros2_package_name:
                    continue
                data_idl_cpp = {
                    'ros2_package_name': ros2_package_name,
                    'interface_type': interface_type,
                    'interface': interface,
                    'mapped_msgs': [],
                    'mapped_services': [],
                }
                if interface_type == 'msg':
                    data_idl_cpp['mapped_msgs'] += [
                        m for m in data['mappings']
                        if m.ros2_msg.package_name == ros2_package_name and
                        m.ros2_msg.message_name == interface.message_name]
                if interface_type == 'srv':
                    data_idl_cpp['mapped_services'] += [
                        s for s in data['services']
                        if s['ros2_package'] == ros2_package_name and
                        s['ros2_name'] == interface.message_name]
                template_file = os.path.join(template_dir, 'interface_factories.cpp.em')
                output_file = os.path.join(
                    output_path, '%s__%s__%s__factories.cpp' %
                    (ros2_package_name, interface_type, interface.message_name))
                expand_template(template_file, data_idl_cpp, output_file)


def generate_messages(rospack=None):
    ros1_msgs = get_ros1_messages(rospack=rospack)
    ros2_package_names, ros2_msgs, mapping_rules = get_ros2_messages()

    package_pairs = determine_package_pairs(ros1_msgs, ros2_msgs, mapping_rules)
    message_pairs = determine_message_pairs(ros1_msgs, ros2_msgs, package_pairs, mapping_rules)

    mappings = []
    # add custom mapping for builtin_interfaces
    for msg_name in ('Duration', 'Time'):
        ros1_msg = [
            m for m in ros1_msgs
            if m.package_name == 'std_msgs' and m.message_name == msg_name]
        ros2_msg = [
            m for m in ros2_msgs
            if m.package_name == 'builtin_interfaces' and m.message_name == msg_name]
        if ros1_msg and ros2_msg:
            mappings.append(Mapping(ros1_msg[0], ros2_msg[0]))

    msg_idx = MessageIndex()
    for ros1_msg, ros2_msg in message_pairs:
        msg_idx.ros1_put(ros1_msg)
        msg_idx.ros2_put(ros2_msg)

    for ros1_msg, ros2_msg in message_pairs:
        mapping = determine_field_mapping(ros1_msg, ros2_msg, mapping_rules, msg_idx)
        if mapping:
            mappings.append(mapping)

    # order mappings topologically to allow template specialization
    ordered_mappings = []
    while mappings:
        # pick first mapping without unsatisfied dependencies
        for m in mappings:
            if not m.depends_on_ros2_messages:
                break
        else:
            break
        # move mapping to ordered list
        mappings.remove(m)
        ordered_mappings.append(m)
        ros2_msg = m.ros2_msg
        # update unsatisfied dependencies of remaining mappings
        for m in mappings:
            if ros2_msg in m.depends_on_ros2_messages:
                m.depends_on_ros2_messages.remove(ros2_msg)

    if mappings:
        print('%d mappings can not be generated due to missing dependencies:' % len(mappings),
              file=sys.stderr)
        for m in mappings:
            print('- %s <-> %s:' %
                  ('%s/%s' % (m.ros1_msg.package_name, m.ros1_msg.message_name),
                   '%s/%s' % (m.ros2_msg.package_name, m.ros2_msg.message_name)), file=sys.stderr)
            for d in m.depends_on_ros2_messages:
                print('  -', '%s/%s' % (d.package_name, d.message_name), file=sys.stderr)
        print(file=sys.stderr)

    return {
        'ros1_msgs': [m.ros1_msg for m in ordered_mappings],
        'ros2_msgs': [m.ros2_msg for m in ordered_mappings],
        'mappings': ordered_mappings,
        'ros2_package_names_msg': ros2_package_names,
        'all_ros2_msgs': ros2_msgs,
    }


def generate_services(rospack=None, message_string_pairs=None):
    ros1_srvs = get_ros1_services(rospack=rospack)
    ros2_pkgs, ros2_srvs, mapping_rules = get_ros2_services()
    services = determine_common_services(
        ros1_srvs, ros2_srvs, mapping_rules,
        message_string_pairs=message_string_pairs)
    return {
        'services': services,
        'ros2_package_names_srv': ros2_pkgs,
        'all_ros2_srvs': ros2_srvs,
    }


def get_ros1_messages(rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()
    msgs = []
    pkgs = sorted(x for x in rosmsg.iterate_packages(rospack, rosmsg.MODE_MSG))
    for package_name, path in pkgs:
        for message_name in rosmsg._list_types(path, 'msg', rosmsg.MODE_MSG):
            msgs.append(Message(package_name, message_name, path))
    return msgs


def get_ros2_messages():
    pkgs = []
    msgs = []
    rules = []
    # get messages from packages
    resources = {
        key: (val, 'rosidl_interfaces') for key, val
        in ament_index_python.get_resources('rosidl_interfaces').items()
    }
    resources.update({
        key: (val, 'ros1_bridge_foreign_mapping') for key, val
        in ament_index_python.get_resources('ros1_bridge_foreign_mapping').items()
    })
    for package_name, val_tuple in resources.items():
        prefix_path, resource_type = val_tuple
        if resource_type == 'rosidl_interfaces':  # Required, otherwise linking fails
            pkgs.append(package_name)
        resource, _ = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        message_names = {
            i[4:-4]
            for i in interfaces
            if i.startswith('msg/') and i[-4:] in ('.idl', '.msg')}

        for message_name in sorted(message_names):
            msgs.append(Message(package_name, message_name, prefix_path))
        # check package manifest for mapping rules
        package_path = os.path.join(prefix_path, 'share', package_name)
        pkg = parse_package(package_path)
        for export in pkg.exports:
            if export.tagname != 'ros1_bridge':
                continue
            if 'mapping_rules' not in export.attributes:
                continue
            rule_file = os.path.join(package_path, export.attributes['mapping_rules'])
            with open(rule_file, 'r') as h:
                content = yaml.safe_load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('ros1_service_name', 'ros2_service_name')):
                    try:
                        rules.append(MessageMappingRule(data, package_name))
                    except Exception as e:  # noqa: B902
                        print('%s' % str(e), file=sys.stderr)
    return pkgs, msgs, rules


def get_ros1_services(rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()
    srvs = []
    pkgs = sorted(x for x in rosmsg.iterate_packages(rospack, rosmsg.MODE_SRV))
    for package_name, path in pkgs:
        for message_name in rosmsg._list_types(path, 'srv', rosmsg.MODE_SRV):
            srvs.append(Message(package_name, message_name, path))
    return srvs


def get_ros2_services():
    pkgs = []
    srvs = []
    rules = []
    resources = {
        key: (val, 'rosidl_interfaces') for key, val
        in ament_index_python.get_resources('rosidl_interfaces').items()
    }
    resources.update({
        key: (val, 'ros1_bridge_foreign_mapping') for key, val
        in ament_index_python.get_resources('ros1_bridge_foreign_mapping').items()
    })
    resource_type = 'rosidl_interfaces'
    for package_name, val_tuple in resources.items():
        prefix_path, resource_type = val_tuple
        if resource_type == 'rosidl_interfaces':  # Required, otherwise linking fails
            pkgs.append(package_name)
        resource, _ = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        service_names = {
            i[4:-4]
            for i in interfaces
            if i.startswith('srv/') and i[-4:] in ('.idl', '.srv')}

        for service_name in sorted(service_names):
            srvs.append(Message(package_name, service_name, prefix_path))
        # check package manifest for mapping rules
        package_path = os.path.join(prefix_path, 'share', package_name)
        pkg = parse_package(package_path)
        for export in pkg.exports:
            if export.tagname != 'ros1_bridge':
                continue
            if 'mapping_rules' not in export.attributes:
                continue
            rule_file = os.path.join(package_path, export.attributes['mapping_rules'])
            with open(rule_file, 'r') as h:
                content = yaml.safe_load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('ros1_message_name', 'ros2_message_name')):
                    try:
                        rules.append(ServiceMappingRule(data, package_name))
                    except Exception as e:  # noqa: B902
                        print('%s' % str(e), file=sys.stderr)
    return pkgs, srvs, rules


class Message:
    __slots__ = [
        'package_name',
        'message_name',
        'prefix_path'
    ]

    def __init__(self, package_name, message_name, prefix_path=None):
        self.package_name = package_name
        self.message_name = message_name
        self.prefix_path = prefix_path

    def __eq__(self, other):
        return self.package_name == other.package_name and \
            self.message_name == other.message_name

    def __hash__(self):
        return hash('%s/%s' % (self.package_name, self.message_name))

    def __str__(self):
        return self.prefix_path + ':' + self.package_name + ':' + self.message_name

    def __repr__(self):
        return self.__str__()


class MappingRule:
    __slots__ = [
        'ros1_package_name',
        'ros2_package_name',
        'package_mapping',
        'foreign_mapping'
    ]

    def __init__(self, data, expected_package_name):
        if all(n in data for n in ('ros1_package_name', 'ros2_package_name')):
            if (data['ros2_package_name'] != expected_package_name
                    and not data.get('enable_foreign_mappings')):
                raise Exception(
                    ('Ignoring rule which affects a different ROS 2 package (%s) '
                     'then the one it is defined in (%s)\n\n'
                     '(Please set `enable_foreign_mappings` to `true` if '
                     'you explicitly want the rule to apply.)') %
                    (data['ros2_package_name'], expected_package_name)
                )
            self.ros1_package_name = data['ros1_package_name']
            self.ros2_package_name = data['ros2_package_name']
            self.foreign_mapping = bool(data.get('enable_foreign_mappings'))
            self.package_mapping = (
                len(data) == (2 + int('enable_foreign_mappings' in data))
            )
        else:
            raise Exception('Ignoring a rule without a ros1_package_name and/or ros2_package_name')

    def is_package_mapping(self):
        return self.package_mapping

    def is_foreign_mapping(self):
        return self.foreign_mapping

    def __repr__(self):
        return self.__str__()


class MessageMappingRule(MappingRule):
    __slots__ = [
        'ros1_message_name',
        'ros2_message_name',
        'fields_1_to_2',
    ]

    def __init__(self, data, expected_package_name):
        super().__init__(data, expected_package_name)
        self.ros1_message_name = None
        self.ros2_message_name = None
        self.fields_1_to_2 = None
        if all(n in data for n in ('ros1_message_name', 'ros2_message_name')):
            self.ros1_message_name = data['ros1_message_name']
            self.ros2_message_name = data['ros2_message_name']
            if 'fields_1_to_2' or 'fields_2_to_1' in data:
                self.fields_1_to_2 = OrderedDict()
                if 'fields_1_to_2' in data:
                    for ros1_field_name, ros2_field_name in data['fields_1_to_2'].items():
                        self.fields_1_to_2[ros1_field_name] = ros2_field_name
                if 'fields_2_to_1' in data:
                    for ros2_field_name, ros1_field_name in data['fields_2_to_1'].items():
                        self.fields_1_to_2[ros1_field_name] = ros2_field_name
            elif len(data) > 4 + int('enable_foreign_mappings' in data):
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)
        elif len(data) > 2 + int('enable_foreign_mappings' in data):
            raise RuntimeError(
                'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)

    def is_message_mapping(self):
        return self.ros1_message_name is not None

    def is_field_mapping(self):
        return self.fields_1_to_2 is not None

    def __str__(self):
        return 'MessageMappingRule(%s <-> %s)' % (self.ros1_package_name, self.ros2_package_name)


class ServiceMappingRule(MappingRule):
    __slots__ = [
        'ros1_service_name',
        'ros2_service_name',
        'request_fields_1_to_2',
        'response_fields_1_to_2',
    ]

    def __init__(self, data, expected_package_name):
        super().__init__(data, expected_package_name)
        self.ros1_service_name = None
        self.ros2_service_name = None
        self.request_fields_1_to_2 = None
        self.response_fields_1_to_2 = None
        if all(n in data for n in ('ros1_service_name', 'ros2_service_name')):
            self.ros1_service_name = data['ros1_service_name']
            self.ros2_service_name = data['ros2_service_name']
            expected_keys = 4
            if 'request_fields_1_to_2' in data:
                self.request_fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['request_fields_1_to_2'].items():
                    self.request_fields_1_to_2[ros1_field_name] = ros2_field_name
                expected_keys += 1
            if 'response_fields_1_to_2' in data:
                self.response_fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['response_fields_1_to_2'].items():
                    self.response_fields_1_to_2[ros1_field_name] = ros2_field_name
                expected_keys += 1
            elif len(data) > expected_keys + int('enable_foreign_mappings' in data):
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)
        elif len(data) > 2 + int('enable_foreign_mappings' in data):
            raise RuntimeError(
                'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)

    def __str__(self):
        return 'ServiceMappingRule(%s <-> %s)' % (self.ros1_package_name, self.ros2_package_name)


def determine_package_pairs(ros1_msgs, ros2_msgs, mapping_rules):
    pairs = []
    # determine package names considered equal between ROS 1 and ROS 2
    ros1_suffix = '_msgs'
    ros2_suffixes = ['_msgs', '_interfaces']
    ros1_package_names = {m.package_name for m in ros1_msgs}
    ros2_package_names = {m.package_name for m in ros2_msgs}
    for ros1_package_name in ros1_package_names:
        if not ros1_package_name.endswith(ros1_suffix):
            continue
        ros1_package_basename = ros1_package_name[:-len(ros1_suffix)]

        for ros2_package_name in ros2_package_names:
            for ros2_suffix in ros2_suffixes:
                if ros2_package_name.endswith(ros2_suffix):
                    break
            else:
                continue
            ros2_package_basename = ros2_package_name[:-len(ros2_suffix)]
            if ros1_package_basename != ros2_package_basename:
                continue
            pairs.append((ros1_package_name, ros2_package_name))

    # add manual package mapping rules
    for rule in mapping_rules:
        if not rule.is_package_mapping():
            continue
        if rule.ros1_package_name not in ros1_package_names:
            continue
        if rule.ros2_package_name not in ros2_package_names:
            continue
        pair = (rule.ros1_package_name, rule.ros2_package_name)
        if pair not in pairs:
            pairs.append(pair)

    return pairs


def determine_message_pairs(ros1_msgs, ros2_msgs, package_pairs, mapping_rules):
    pairs = []
    # determine message names considered equal between ROS 1 and ROS 2
    for ros1_msg in ros1_msgs:
        for ros2_msg in ros2_msgs:
            package_pair = (ros1_msg.package_name, ros2_msg.package_name)
            if package_pair not in package_pairs:
                continue
            if ros1_msg.message_name != ros2_msg.message_name:
                continue
            pairs.append((ros1_msg, ros2_msg))

    # add manual message mapping rules
    for rule in mapping_rules:
        if not rule.is_message_mapping():
            continue
        for ros1_msg in ros1_msgs:
            if rule.ros1_package_name == ros1_msg.package_name and \
                    rule.ros1_message_name == ros1_msg.message_name:
                break
        else:
            # skip unknown messages
            continue
        for ros2_msg in ros2_msgs:
            if rule.ros2_package_name == ros2_msg.package_name and \
                    rule.ros2_message_name == ros2_msg.message_name:
                break
        else:
            # skip unknown messages
            continue

        pair = (ros1_msg, ros2_msg)
        if pair not in pairs:
            pairs.append(pair)

    return pairs


def determine_common_services(
    ros1_srvs, ros2_srvs, mapping_rules, message_string_pairs=None
):
    if message_string_pairs is None:
        message_string_pairs = set()

    pairs = []
    services = []
    for ros1_srv in ros1_srvs:
        for ros2_srv in ros2_srvs:
            if ros1_srv.package_name == ros2_srv.package_name:
                if ros1_srv.message_name == ros2_srv.message_name:
                    pairs.append((ros1_srv, ros2_srv))

    for rule in mapping_rules:
        for ros1_srv in ros1_srvs:
            for ros2_srv in ros2_srvs:
                pair = (ros1_srv, ros2_srv)
                if pair in pairs:
                    continue
                if rule.ros1_package_name == ros1_srv.package_name and \
                   rule.ros2_package_name == ros2_srv.package_name:
                    if rule.ros1_service_name is None and rule.ros2_service_name is None:
                        if ros1_srv.message_name == ros2_srv.message_name:
                            pairs.append(pair)
                    else:
                        if (
                            rule.ros1_service_name == ros1_srv.message_name and
                            rule.ros2_service_name == ros2_srv.message_name
                        ):
                            pairs.append(pair)

    for pair in pairs:
        ros1_spec = load_ros1_service(pair[0])
        ros2_spec = load_ros2_service(pair[1])
        ros1_fields = {
            'request': ros1_spec.request.fields(),
            'response': ros1_spec.response.fields()
        }
        ros2_fields = {
            'request': ros2_spec.request.fields,
            'response': ros2_spec.response.fields
        }
        output = {
            'request': [],
            'response': []
        }
        match = True
        for direction in ['request', 'response']:
            if len(ros1_fields[direction]) != len(ros2_fields[direction]):
                match = False
                break
            for i, ros1_field in enumerate(ros1_fields[direction]):
                ros1_type = ros1_field[0]
                ros2_type = str(ros2_fields[direction][i].type)
                ros1_name = ros1_field[1]
                ros2_name = ros2_fields[direction][i].name
                if ros1_type != ros2_type or ros1_name != ros2_name:
                    # if the message types have a custom mapping their names
                    # might not be equal, therefore check the message pairs
                    if (ros1_type, ros2_type) not in message_string_pairs:
                        match = False
                        break
                output[direction].append({
                    'basic': False if '/' in ros1_type else True,
                    'array': True if '[]' in ros1_type else False,
                    'ros1': {
                        'name': ros1_name,
                        'type': ros1_type.rstrip('[]'),
                        'cpptype': ros1_type.rstrip('[]').replace('/', '::')
                    },
                    'ros2': {
                        'name': ros2_name,
                        'type': ros2_type.rstrip('[]'),
                        'cpptype': ros2_type.rstrip('[]').replace('/', '::msg::')
                    }
                })
        if match:
            services.append({
                'ros1_name': pair[0].message_name,
                'ros2_name': pair[1].message_name,
                'ros1_package': pair[0].package_name,
                'ros2_package': pair[1].package_name,
                'fields': output
            })
    return services


def update_ros1_field_information(ros1_field, package_name):
    parts = ros1_field.base_type.split('/')
    assert len(parts) in [1, 2]
    if len(parts) == 1:
        ros1_field.pkg_name = package_name
        ros1_field.msg_name = parts[0]
    else:
        ros1_field.pkg_name = parts[0]
        ros1_field.msg_name = parts[1]


def get_ros1_selected_fields(ros1_field_selection, parent_ros1_spec, msg_idx):
    """
    Get a tuple of fields corresponding to a field selection on a ROS 1 message.

    :param ros1_field_selection: a string with message field names separated by `.`
    :param parent_ros1_spec: a genmsg.MsgSpec for a message that contains the first field
    in ros1_field_selection
    :type msg_idx: MessageIndex

    :return: a tuple of genmsg.msgs.Field objets with additional attributes `pkg_name`
    and `msg_name` as defined by `update_ros1_field_information`, corresponding to
    traversing `parent_ros1_spec` recursively following `ros1_field_selection`

    :throws: IndexError in case some expected field is not found while traversing
    `parent_ros1_spec` recursively following `ros1_field_selection`
    """
    selected_fields = []

    def consume_field(field):
        update_ros1_field_information(field, parent_ros1_spec.package)
        selected_fields.append(field)

    fields = ros1_field_selection.split('.')
    current_field = [f for f in parent_ros1_spec.parsed_fields() if f.name == fields[0]][0]
    consume_field(current_field)
    for field in fields[1:]:
        parent_ros1_spec = load_ros1_message(msg_idx.ros1_get_from_field(current_field))
        current_field = [f for f in parent_ros1_spec.parsed_fields() if f.name == field][0]
        consume_field(current_field)

    return tuple(selected_fields)


def get_ros2_selected_fields(ros2_field_selection, parent_ros2_spec, msg_idx):
    selected_fields = []
    fields = ros2_field_selection.split('.')
    current_field = [
        member for member in parent_ros2_spec.structure.members
        if member.name == fields[0]
    ][0]
    selected_fields.append(current_field)
    for field in fields[1:]:
        parent_ros2_spec = load_ros2_message(msg_idx.ros2_get_from_field(current_field))
        current_field = [
            member for member in parent_ros2_spec.structure.members
            if member.name == field
        ][0]
        selected_fields.append(current_field)
    return tuple(selected_fields)


def determine_field_mapping(ros1_msg, ros2_msg, mapping_rules, msg_idx):
    """
    Return the first mapping object for ros1_msg and ros2_msg found in mapping_rules.

    If not found in mapping_rules otherwise defined implicitly, or None if no mapping is found.

    :type ros1_msg: Message
    :type ros2_msg: Message
    :type mapping_rules: list of MessageMappingRule
    :type msg_idx: MessageIndex
    """
    ros1_spec = load_ros1_message(ros1_msg)
    if not ros1_spec:
        return None
    ros2_spec = load_ros2_message(ros2_msg)
    if not ros2_spec:
        return None

    mapping = Mapping(ros1_msg, ros2_msg)

    # check for manual field mapping rules first
    for rule in mapping_rules:
        if not rule.is_field_mapping():
            continue
        if rule.ros1_package_name != ros1_msg.package_name or \
                rule.ros1_message_name != ros1_msg.message_name:
            continue
        if rule.ros2_package_name != ros2_msg.package_name or \
                rule.ros2_message_name != ros2_msg.message_name:
            continue

        for ros1_field_selection, ros2_field_selection in rule.fields_1_to_2.items():
            try:
                ros1_selected_fields = \
                    get_ros1_selected_fields(ros1_field_selection, ros1_spec, msg_idx)
            except IndexError:
                print(
                    "A manual mapping refers to an invalid field '%s' " % ros1_field_selection +
                    "in the ROS 1 message '%s/%s'" %
                    (rule.ros1_package_name, rule.ros1_message_name),
                    file=sys.stderr)
                continue
            try:
                ros2_selected_fields = \
                    get_ros2_selected_fields(ros2_field_selection, ros2_spec, msg_idx)
            except IndexError:
                print(
                    "A manual mapping refers to an invalid field '%s' " % ros2_field_selection +
                    "in the ROS 2 message '%s/%s'" %
                    (rule.ros2_package_name, rule.ros2_message_name),
                    file=sys.stderr)
                continue
            mapping.add_field_pair(ros1_selected_fields, ros2_selected_fields)

    # apply name based mapping of fields
    ros1_field_missing_in_ros2 = False

    ros1_fields_not_mapped = []
    for ros1_field in ros1_spec.parsed_fields():
        for ros2_member in ros2_spec.structure.members:
            if ros1_field.name.lower() == ros2_member.name:
                # get package name and message name from ROS 1 field type
                update_ros1_field_information(ros1_field, ros1_msg.package_name)
                mapping.add_field_pair(ros1_field, ros2_member)
                break

        ros1_fields_mapped_to_a_ros2_member = [field[0].name
                                               for field
                                               in mapping.fields_1_to_2.keys()]
        if ros1_field.name not in ros1_fields_mapped_to_a_ros2_member:
            # this allows fields to exist in ROS 1 but not in ROS 2
            ros1_fields_not_mapped += [ros1_field]

    ros1_field_missing_in_ros2 = any(ros1_fields_not_mapped)

    mapping.ros1_field_missing_in_ros2 = ros1_field_missing_in_ros2

    if ros1_field_missing_in_ros2:
        # if some fields exist in ROS 1 but not in ROS 2
        # check that no fields exist in ROS 2 but not in ROS 1
        # since then it might be the case that those have been renamed and should be mapped
        for ros2_member in ros2_spec.structure.members:
            for ros1_field in ros1_spec.parsed_fields():
                if ros1_field.name.lower() == ros2_member.name:
                    break
            else:
                # if fields from both sides are not mappable the whole message is not mappable
                return None

    return mapping


def load_ros1_message(ros1_msg):
    msg_context = genmsg.MsgContext.create_default()
    message_path = os.path.join(ros1_msg.prefix_path, ros1_msg.message_name + '.msg')
    try:
        spec = genmsg.msg_loader.load_msg_from_file(
            msg_context, message_path, '%s/%s' % (ros1_msg.package_name, ros1_msg.message_name))
    except genmsg.InvalidMsgSpec:
        return None
    return spec


def load_ros1_service(ros1_srv):
    srv_context = genmsg.MsgContext.create_default()
    srv_path = os.path.join(ros1_srv.prefix_path, ros1_srv.message_name + '.srv')
    srv_name = '%s/%s' % (ros1_srv.package_name, ros1_srv.message_name)
    try:
        spec = genmsg.msg_loader.load_srv_from_file(srv_context, srv_path, srv_name)
    except genmsg.InvalidMsgSpec:
        return None
    return spec


def load_ros2_message(ros2_msg):
    message_basepath = os.path.join(ros2_msg.prefix_path, 'share')
    message_relative_path = \
        os.path.join(ros2_msg.package_name, 'msg', ros2_msg.message_name)
    message_path = os.path.join(message_basepath, message_relative_path)
    # Check to see if the message is defined as a .msg file or an .idl file,
    # but preferring '.idl' if both exist.
    if os.path.exists(message_path + '.idl'):
        message_path += '.idl'
        message_relative_path += '.idl'
    elif os.path.exists(message_path + '.msg'):
        message_path += '.msg'
        message_relative_path += '.msg'
    else:
        raise RuntimeError(
            f"message '{ros2_msg.package_name}/msg/{ros2_msg.message_name}' "
            f"was not found in prefix '{ros2_msg.prefix_path}' with either "
            f"file extension '.msg' or '.idl'")
    # We don't support .msg files, but that shouldn't be a problem since an .idl
    # version should have been created when the package was built by rosidl_adapter.
    if message_path.endswith('.msg'):
        raise RuntimeError(
            "ros1_bridge cannot process ROS 2 message definitions that lack a '.idl' version, "
            "which normally does not occur as rosidl_adapter should create a '.idl' version "
            "when building the message package which contains the original '.msg' file."
        )
    if not message_path.endswith('.idl'):
        raise RuntimeError(
            f"message_path '{message_path}' unexpectedly does not end with '.idl'"
        )
    idl_locator = \
        rosidl_parser.definition.IdlLocator(message_basepath, message_relative_path)
    spec = rosidl_parser.parser.parse_idl_file(idl_locator)
    messages = spec.content.get_elements_of_type(rosidl_parser.definition.Message)
    if len(messages) != 1:
        raise RuntimeError(
            'unexpectedly found multiple message definitions when processing '
            f"message '{ros2_msg.package_name}/msg/{ros2_msg.message_name}'"
        )
    return messages[0]


def load_ros2_service(ros2_srv):
    srv_path = os.path.join(
        ros2_srv.prefix_path, 'share', ros2_srv.package_name, 'srv',
        ros2_srv.message_name + '.srv')
    try:
        spec = rosidl_adapter.parser.parse_service_file(ros2_srv.package_name, srv_path)
    except rosidl_adapter.parser.InvalidSpecification:
        return None
    return spec


# make field types hashable
def FieldHash(self):
    return self.name.__hash__()


genmsg.msgs.Field.__hash__ = FieldHash
rosidl_adapter.parser.Field.__hash__ = FieldHash


class Mapping:
    __slots__ = [
        'ros1_msg',
        'ros2_msg',
        'fields_1_to_2',
        'fields_2_to_1',
        'depends_on_ros2_messages',
        'ros1_field_missing_in_ros2',
    ]

    def __init__(self, ros1_msg, ros2_msg):
        self.ros1_msg = ros1_msg
        self.ros2_msg = ros2_msg
        self.fields_1_to_2 = OrderedDict()
        self.fields_2_to_1 = OrderedDict()
        self.depends_on_ros2_messages = set()
        self.ros1_field_missing_in_ros2 = False

    def add_field_pair(self, ros1_fields, ros2_members):
        """
        Add a new mapping for a pair of ROS 1 and ROS 2 messages.

        :type ros1_fields: either a genmsg.msgs.Field object with additional attributes `pkg_name`
        and `msg_name` as defined by `update_ros1_field_information`, or a tuple of objects of
        that type
        :type ros2_members: a single, or list of, rosidl_parser.definition.Member object(s)
        """
        if not isinstance(ros1_fields, tuple):
            ros1_fields = (ros1_fields,)
        if not isinstance(ros2_members, tuple):
            ros2_members = (ros2_members, )
        self.fields_1_to_2[ros1_fields] = ros2_members
        self.fields_2_to_1[ros2_members] = ros1_fields
        for ros2_member in ros2_members:
            # If the member is not a namespaced type, skip.
            if not isinstance(ros2_member.type, rosidl_parser.definition.NamespacedType):
                continue
            # If it is, then the type will have a namespaced name, e.g. (std_msgs, msg, String)
            # If it is not of the standard ('<package name>', 'msg', '<type>'), skip it
            if len(ros2_member.type.namespaces) != 2 or ros2_member.type.namespaces[1] != 'msg':
                continue
            # Extract the package name and message name
            pkg_name = ros2_member.type.namespaces[0]
            msg_name = ros2_member.type.name
            if pkg_name != 'builtin_interfaces':
                self.depends_on_ros2_messages.add(Message(pkg_name, msg_name))


def camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is not followed by another upper case letter
    value = re.sub('(.)([A-Z][a-z]+)', '\\1_\\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', '\\1_\\2', value)
    return value.lower()


class MessageIndex:
    """
    Index from package and message names to Message objects.

    Maintains 2 indices from (package_name, message_name) to Message,
    one for ROS 1 messages and another for ROS 2 messages
    """

    def __init__(self):
        self._ros1_idx = {}
        self._ros2_idx = {}

    def ros1_put(self, msg):
        """Add msg to the ROS1 index."""
        self._ros1_idx[(msg.package_name, msg.message_name)] = msg

    def ros2_put(self, msg):
        """Add msg to the ROS2 index."""
        self._ros2_idx[(msg.package_name, msg.message_name)] = msg

    def ros1_get_from_field(self, field):
        """
        Get Message from ROS 1 index.

        :type field: genmsg.msgs.Field with additional fields `pkg_name`
        and `msg_name` as added by `update_ros1_field_information`
        :return: the message indexed for the fields `pkg_name` and
        `msg_name` of `field`
        """
        return self._ros1_idx[(field.pkg_name, field.msg_name)]

    def ros2_get_from_field(self, field):
        """
        Get Message from ROS 2 index.

        :type field: rosidl_parser.definition.NamespacedType
        :return: the message indexed for the fields `type.namespaces[0]` and
        `type.name` of `field`
        """
        return self._ros2_idx[(field.type.namespaces[0], field.type.name)]
