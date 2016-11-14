from collections import OrderedDict
import os
import re
import sys
import yaml

import ament_index_python
from ament_package import parse_package
from rosidl_cmake import expand_template
import rosidl_parser

# ROS 1 imports
import genmsg
import genmsg.msg_loader

# import catkin_pkg and rospkg which are required by rosmsg
# and likely only available for Python 2
try:
    import catkin_pkg
except ImportError:
    from importlib.machinery import SourceFileLoader
    import subprocess
    for python_executable in ['python2', 'python2.7']:
        try:
            catkin_pkg_path = subprocess.check_output(
                [python_executable, '-c', 'import catkin_pkg; print(catkin_pkg.__file__)'])
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue
        catkin_pkg_path = catkin_pkg_path.decode().strip()
        if catkin_pkg_path.endswith('.pyc'):
            catkin_pkg_path = catkin_pkg_path[:-1]
        catkin_pkg = SourceFileLoader('catkin_pkg', catkin_pkg_path).load_module()
    if not catkin_pkg:
        raise

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
import rosmsg


def generate_cpp(output_file, template_dir):
    data1 = generate_messages()
    data2 = generate_services()
    data1.update(data2);
    template_file = os.path.join(template_dir, 'generated_factories.cpp.em')
    expand_template(template_file, data1, output_file)


def generate_messages():
    rospack = rospkg.RosPack()
    ros1_msgs = get_ros1_messages(rospack=rospack)
    ros2_msgs, mapping_rules = get_ros2_messages()

    package_pairs = determine_package_pairs(ros1_msgs, ros2_msgs, mapping_rules)
    message_pairs = determine_message_pairs(ros1_msgs, ros2_msgs, package_pairs, mapping_rules)

    mappings = []
    for ros1_msg, ros2_msg in message_pairs:
        mapping = determine_field_mapping(ros1_msg, ros2_msg, mapping_rules, rospack=rospack)
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
    }


def generate_services():
    ros1_srvs = get_ros1_services()
    ros2_srvs = get_ros2_services()
    services = determine_common_services(ros1_srvs, ros2_srvs)
    return { "services": services }


def get_ros1_messages(rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()
    msgs = []
    pkgs = sorted([x for x in rosmsg.iterate_packages(rospack, rosmsg.MODE_MSG)])
    for package_name, path in pkgs:
        for message_name in rosmsg._list_types(path, 'msg', rosmsg.MODE_MSG):
            msgs.append(Message(package_name, message_name, path))
    return msgs


def get_ros2_messages():
    msgs = []
    rules = []
    # get messages from packages
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
        resource, _ = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        message_names = [i[:-4] for i in interfaces if i.endswith('.msg')]
        for message_name in message_names:
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
            rules += read_mapping_rules(rule_file, package_name)
    return msgs, rules


def get_ros1_services(rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()
    srvs = []
    pkgs = sorted([x for x in rosmsg.iterate_packages(rospack, rosmsg.MODE_SRV)])
    for package_name, path in pkgs:
        for message_name in rosmsg._list_types(path, 'srv', rosmsg.MODE_SRV):
            srvs.append(Message(package_name, message_name, path))
    return srvs


def get_ros2_services():
    srvs = []
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
        resource = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        service_names = [i[:-4] for i in interfaces if i.endswith('.srv')]
        for service_name in service_names:
            srvs.append(Message(package_name, service_name, prefix_path))
    return srvs


def read_mapping_rules(rule_file, package_name):
    rules = []
    with open(rule_file, 'r') as h:
        rules_data = yaml.load(h)
        for rule_data in rules_data:
            for field_name in ['ros1_package_name', 'ros2_package_name']:
                if field_name not in rule_data:
                    print("Ignoring rule without a '%s'" % field_name, file=sys.stderr)
                    continue
            if rule_data['ros2_package_name'] != package_name:
                print(
                    ("Ignoring rule in '%s' which affects a different ROS 2 package (%s) "
                     'then the one it is defined in (%s)') %
                    (rule_file, rule_data['ros2_package_name'], package_name), file=sys.stderr)
                continue
            rules.append(MappingRule(rule_data))
    return rules


class Message(object):
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
        return self.prefix_path + ":" + self.package_name + ":" + self.message_name

    def __repr__(self):
        return self.__str__()


class MappingRule(object):
    __slots__ = [
        'ros1_package_name',
        'ros1_message_name',
        'ros2_package_name',
        'ros2_message_name',
        'fields_1_to_2',
    ]

    def __init__(self, data):
        self.ros1_package_name = data['ros1_package_name']
        self.ros2_package_name = data['ros2_package_name']
        self.ros1_message_name = None
        self.ros2_message_name = None
        self.fields_1_to_2 = None
        if 'ros1_message_name' in data:
            self.ros1_message_name = data['ros1_message_name']
            self.ros2_message_name = data['ros2_message_name']
            if 'fields_1_to_2' in data:
                self.fields_1_to_2 = OrderedDict()
                for ros1_field_name, ros2_field_name in data['fields_1_to_2'].items():
                    self.fields_1_to_2[ros1_field_name] = ros2_field_name

    def is_package_mapping(self):
        return self.ros1_message_name is None

    def is_message_mapping(self):
        return self.ros1_message_name is not None and self.fields_1_to_2 is None

    def is_field_mapping(self):
        return self.fields_1_to_2 is not None


def determine_package_pairs(ros1_msgs, ros2_msgs, mapping_rules):
    pairs = []
    # determine package names considered equal between ROS 1 and ROS 2
    ros1_suffix = '_msgs'
    ros2_suffixes = ['_msgs', '_interfaces']
    ros1_package_names = set([m.package_name for m in ros1_msgs])
    ros2_package_names = set([m.package_name for m in ros2_msgs])
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
        if not rule.is_package_mapping:
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
        if not rule.is_message_mapping:
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


def determine_common_services(ros1_srvs, ros2_srvs):
    pairs = []
    services = []
    for ros1_srv in ros1_srvs:
        for ros2_srv in ros2_srvs:
            if (ros1_srv.package_name == ros2_srv.package_name):
                if (ros1_srv.message_name == ros2_srv.message_name):
                    pairs.append((ros1_srv, ros2_srv))
    for pair in pairs:
        ros1_spec = load_ros1_service(pair[0])
        ros2_spec = load_ros2_service(pair[1])
        ros1_fields = {
            "request": ros1_spec.request.fields(),
            "response": ros1_spec.response.fields()
        }
        ros2_fields = {
            "request": ros2_spec.request.fields,
            "response": ros2_spec.response.fields
        }
        output = {
            "request": [],
            "response": []
        }
        match = True
        for direction in ["request", "response"]:
            if len(ros1_fields[direction]) != len(ros2_fields[direction]):
                match = False
                break
            for i in range(0, len(ros1_fields[direction])):
                ros1_type = ros1_fields[direction][i][0]
                ros2_type = str(ros2_fields[direction][i].type)
                ros1_name = ros1_fields[direction][i][1]
                ros2_name = ros2_fields[direction][i].name
                if (ros1_type != ros2_type or ros1_name != ros2_name):
                    match = False
                    break
                output[direction].append({
                    "basic": False if "/" in ros1_type else True,
                    "array": True if "[]" in ros1_type else False,
                    "ros1": {
                        "type": ros1_type.rstrip("[]"),
                        "name": ros1_name,
                    },
                    "ros2": {
                        "type": ros2_type.rstrip("[]"),
                        "name": ros2_name,
                    }
                })
        if match:
            services.append({
                "ros1_name": pair[0].message_name,
                "ros2_name": pair[1].message_name,
                "ros1_package": pair[0].package_name,
                "ros2_package": pair[1].package_name,
                "fields": output
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


def determine_field_mapping(ros1_msg, ros2_msg, mapping_rules, rospack=None):
    ros1_spec = load_ros1_message(ros1_msg, rospack=rospack)
    if not ros1_spec:
        return None
    ros2_spec = load_ros2_message(ros2_msg)
    if not ros2_spec:
        return None

    mapping = Mapping(ros1_msg, ros2_msg)

    # check for manual field mapping rules first
    for rule in mapping_rules:
        if not rule.is_field_mapping:
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


def load_ros1_message(ros1_msg, rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()

    msg_context = genmsg.MsgContext.create_default()
    message_path = os.path.join(ros1_msg.prefix_path, ros1_msg.message_name + '.msg')
    try:
        spec = genmsg.msg_loader.load_msg_from_file(
            msg_context, message_path, '%s/%s' % (ros1_msg.package_name, ros1_msg.message_name))
    except genmsg.InvalidMsgSpec:
        return None
    return spec


def load_ros1_service(ros1_srv, rospack=None):
    if not rospack:
        rospack = rospkg.RosPack()

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


class Mapping(object):
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
        self.depends_on_ros2_messages = set([])

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
