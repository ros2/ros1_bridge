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

from rosidl_cmake import expand_template
import rosidl_parser

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
    data.update(generate_services(rospack))

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
        for extension in ['cpp', 'hpp']:
            data_pkg = {
                'ros2_package_name': ros2_package_name,
                'mappings': [
                    m for m in data['mappings']
                    if m.ros2_msg.package_name == ros2_package_name],
                'services': [
                    s for s in data['services']
                    if s['ros2_package'] == ros2_package_name]
            }
            if extension == 'hpp':
                data_pkg.update({
                    'ros1_msgs': [
                        m.ros1_msg for m in data['mappings']
                        if m.ros2_msg.package_name == ros2_package_name],
                    'ros2_msgs': [
                        m.ros2_msg for m in data['mappings']
                        if m.ros2_msg.package_name == ros2_package_name],
                })
            template_file = os.path.join(template_dir, 'pkg_factories.%s.em' % extension)
            output_file = os.path.join(
                output_path, '%s_factories.%s' % (ros2_package_name, extension))
            expand_template(template_file, data_pkg, output_file)


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

    for ros1_msg, ros2_msg in message_pairs:
        mapping = determine_field_mapping(ros1_msg, ros2_msg, mapping_rules)
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
        'ros2_package_names_msg': ros2_package_names
    }


def generate_services(rospack=None):
    ros1_srvs = get_ros1_services(rospack=rospack)
    ros2_pkgs, ros2_srvs, mapping_rules = get_ros2_services()
    services = determine_common_services(ros1_srvs, ros2_srvs, mapping_rules)
    return {
        'services': services,
        'ros2_package_names_srv': ros2_pkgs
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
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
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
                content = yaml.load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('ros1_service_name', 'ros2_service_name')):
                    try:
                        rules.append(MessageMappingRule(data, package_name))
                    except Exception as e:
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
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
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
                content = yaml.load(h)
            if not isinstance(content, list):
                print(
                    "The content of the mapping rules in '%s' is not a list" % rule_file,
                    file=sys.stderr)
                continue
            for data in content:
                if all(n not in data for n in ('ros1_message_name', 'ros2_message_name')):
                    try:
                        rules.append(ServiceMappingRule(data, package_name))
                    except Exception as e:
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
        'package_mapping'
    ]

    def __init__(self, data, expected_package_name):
        if all(n in data for n in ('ros1_package_name', 'ros2_package_name')):
            if data['ros2_package_name'] != expected_package_name:
                raise Exception(
                    ('Ignoring rule which affects a different ROS 2 package (%s) '
                     'then the one it is defined in (%s)') %
                    (data['ros2_package_name'], expected_package_name))
            self.ros1_package_name = data['ros1_package_name']
            self.ros2_package_name = data['ros2_package_name']
            self.package_mapping = (len(data) == 2)
        else:
            raise Exception('Ignoring a rule without a ros1_package_name and/or ros2_package_name')

    def is_package_mapping(self):
        return self.package_mapping

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
            if 'fields_1_to_2' in data:
                self.fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['fields_1_to_2'].items():
                    self.fields_1_to_2[ros1_field_name] = ros2_field_name
            elif len(data) > 4:
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)
        elif len(data) > 2:
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
            elif len(data) > expected_keys:
                raise RuntimeError(
                    'Mapping for package %s contains unknown field(s)' % self.ros2_package_name)
        elif len(data) > 2:
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


def determine_common_services(ros1_srvs, ros2_srvs, mapping_rules):
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
                if rule.ros1_package_name == ros1_srv.package_name and \
                   rule.ros2_package_name == ros2_srv.package_name:
                    if rule.ros1_service_name is None and rule.ros2_service_name is None:
                        if ros1_srv.message_name == ros2_srv.message_name:
                            pairs.append((ros1_srv, ros2_srv))
                    else:
                        if (
                            rule.ros1_service_name == ros1_srv.message_name and
                            rule.ros2_service_name == ros2_srv.message_name
                        ):
                            pairs.append((ros1_srv, ros2_srv))

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


def determine_field_mapping(ros1_msg, ros2_msg, mapping_rules):
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

        for ros1_field_name, ros2_field_name in rule.fields_1_to_2.items():
            try:
                ros1_field = \
                    [f for f in ros1_spec.parsed_fields() if f.name == ros1_field_name][0]
            except IndexError:
                print(
                    "A manual mapping refers to an invalid field '%s' " % ros1_field_name +
                    "in the ROS 1 message '%s/%s'" %
                    (rule.ros1_package_name, rule.ros1_message_name),
                    file=sys.stderr)
                continue
            try:
                ros2_field = \
                    [f for f in ros2_spec.fields if f.name == ros2_field_name][0]
            except IndexError:
                print(
                    "A manual mapping refers to an invalid field '%s' " % ros2_field_name +
                    "in the ROS 2 message '%s/%s'" %
                    (rule.ros2_package_name, rule.ros2_message_name),
                    file=sys.stderr)
                continue
            update_ros1_field_information(ros1_field, ros1_msg.package_name)
            mapping.add_field_pair(ros1_field, ros2_field)
        return mapping

    # apply name based mapping of fields
    ros1_field_missing_in_ros2 = False

    for ros1_field in ros1_spec.parsed_fields():
        for ros2_field in ros2_spec.fields:
            if ros1_field.name.lower() == ros2_field.name:
                # get package name and message name from ROS 1 field type
                if ros2_field.type.pkg_name:
                    update_ros1_field_information(ros1_field, ros1_msg.package_name)
                mapping.add_field_pair(ros1_field, ros2_field)
                break
        else:
            # this allows fields to exist in ROS 1 but not in ROS 2
            ros1_field_missing_in_ros2 = True

    if ros1_field_missing_in_ros2:
        # if some fields exist in ROS 1 but not in ROS 2
        # check that no fields exist in ROS 2 but not in ROS 1
        # since then it might be the case that those have been renamed and should be mapped
        for ros2_field in ros2_spec.fields:
            for ros1_field in ros1_spec.parsed_fields():
                if ros1_field.name.lower() == ros2_field.name:
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
    message_path = os.path.join(
        ros2_msg.prefix_path, 'share', ros2_msg.package_name, 'msg',
        ros2_msg.message_name + '.msg')
    try:
        spec = rosidl_parser.parse_message_file(ros2_msg.package_name, message_path)
    except rosidl_parser.InvalidSpecification:
        return None
    return spec


def load_ros2_service(ros2_srv):
    srv_path = os.path.join(
        ros2_srv.prefix_path, 'share', ros2_srv.package_name, 'srv',
        ros2_srv.message_name + '.srv')
    try:
        spec = rosidl_parser.parse_service_file(ros2_srv.package_name, srv_path)
    except rosidl_parser.InvalidSpecification:
        return None
    return spec


# make field types hashable
def FieldHash(self):
    return self.name.__hash__()


genmsg.msgs.Field.__hash__ = FieldHash
rosidl_parser.Field.__hash__ = FieldHash


class Mapping:
    __slots__ = [
        'ros1_msg',
        'ros2_msg',
        'fields_1_to_2',
        'fields_2_to_1',
        'depends_on_ros2_messages'
    ]

    def __init__(self, ros1_msg, ros2_msg):
        self.ros1_msg = ros1_msg
        self.ros2_msg = ros2_msg
        self.fields_1_to_2 = OrderedDict()
        self.fields_2_to_1 = OrderedDict()
        self.depends_on_ros2_messages = set()

    def add_field_pair(self, ros1_field, ros2_field):
        self.fields_1_to_2[ros1_field] = ros2_field
        self.fields_2_to_1[ros2_field] = ros1_field
        if ros2_field.type.pkg_name and ros2_field.type.pkg_name != 'builtin_interfaces':
            self.depends_on_ros2_messages.add(
                Message(ros2_field.type.pkg_name, ros2_field.type.type))


def camel_case_to_lower_case_underscore(value):
    # insert an underscore before any upper case letter
    # which is not followed by another upper case letter
    value = re.sub('(.)([A-Z][a-z]+)', '\\1_\\2', value)
    # insert an underscore before any upper case letter
    # which is preseded by a lower case letter or number
    value = re.sub('([a-z0-9])([A-Z])', '\\1_\\2', value)
    return value.lower()
