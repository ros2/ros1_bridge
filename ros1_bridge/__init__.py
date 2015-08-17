from collections import OrderedDict
import os
import re
import sys
import ament_index_python
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
import rosmsg


def generate_cpp(output_file, template_dir):
    rospack = rospkg.RosPack()
    ros1_msgs = get_ros1_messages(rospack=rospack)
    ros2_msgs = get_ros2_messages()

    message_pairs = determine_message_pairs(ros1_msgs, ros2_msgs)

    mappings = []
    for ros1_msg, ros2_msg in message_pairs:
        mapping = determine_field_mapping(ros1_msg, ros2_msg, rospack=rospack)
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

    data = {
        'ros1_msgs': [m.ros1_msg for m in ordered_mappings],
        'ros2_msgs': [m.ros2_msg for m in ordered_mappings],
        'mappings': ordered_mappings,
    }

    template_file = os.path.join(template_dir, 'generated_factories.cpp.template')
    expand_template(template_file, data, output_file)


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
    resource_type = 'rosidl_interfaces'
    resources = ament_index_python.get_resources(resource_type)
    for package_name, prefix_path in resources.items():
        resource = ament_index_python.get_resource(resource_type, package_name)
        interfaces = resource.splitlines()
        message_names = [i[:-4] for i in interfaces if i.endswith('.msg')]
        for message_name in message_names:
            msgs.append(Message(package_name, message_name, prefix_path))
    return msgs


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


def determine_message_pairs(ros1_msgs, ros2_msgs):
    pairs = []
    ros1_suffix = '_msgs'
    ros2_suffixes = ['_msgs', '_interfaces']
    for ros1_msg in ros1_msgs:
        if not ros1_msg.package_name.endswith(ros1_suffix):
            continue
        ros1_package_basename = ros1_msg.package_name[:-len(ros1_suffix)]

        for ros2_msg in ros2_msgs:
            for ros2_suffix in ros2_suffixes:
                if ros2_msg.package_name.endswith(ros2_suffix):
                    break
            else:
                continue
            ros2_package_basename = ros2_msg.package_name[:-len(ros2_suffix)]
            if ros1_package_basename != ros2_package_basename:
                continue
            if ros1_msg.message_name != ros2_msg.message_name:
                continue
            pairs.append((ros1_msg, ros2_msg))

    return pairs


def determine_field_mapping(ros1_msg, ros2_msg, rospack=None):
    ros1_spec = load_ros1_message(ros1_msg, rospack=rospack)
    if not ros1_spec:
        return None
    ros2_spec = load_ros2_message(ros2_msg)
    if not ros2_spec:
        return None

    # ROS 1 fields
    #   name
    #   type
    #   base_type
    #   is_array
    #   array_len
    #   is_builtin
    #   is_header

    ros1_field_missing_in_ros2 = False

    mapping = Mapping(ros1_msg, ros2_msg)
    for ros1_field in ros1_spec.parsed_fields():
        # TODO handle header correctly
        if ros1_field.is_header:
            continue
        for ros2_field in ros2_spec.fields:
            if ros1_field.name.lower() == ros2_field.name:
                # get package name and message name from ROS 1 field type
                if ros2_field.type.pkg_name:
                    parts = ros1_field.base_type.split('/')
                    assert len(parts) in [1, 2]
                    if len(parts) == 1:
                        ros1_field.pkg_name = ros1_msg.package_name
                        ros1_field.msg_name = parts[0]
                    else:
                        ros1_field.pkg_name = parts[0]
                        ros1_field.msg_name = parts[1]
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


def load_ros2_message(ros2_msg):
    message_path = os.path.join(
        ros2_msg.prefix_path, 'share', ros2_msg.package_name, 'msg',
        ros2_msg.message_name + '.msg')
    try:
        spec = rosidl_parser.parse_message_file(ros2_msg.package_name, message_path)
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
