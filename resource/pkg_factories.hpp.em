// generated from ros1_bridge/resource/pkg_factories.hpp.em

@###############################################
@#
@# Factory template specializations based on
@# message types of a single ROS 2 package
@#
@# EmPy template for generating <pkgname>_factories.hpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_name (str)
@#    The ROS 2 package name of this file
@#  - mappings (list of ros1_bridge.Mapping)
@#    Mapping between messages as well as their fields
@#  - ros1_msgs (list of ros1_bridge.Message)
@#    ROS 1 messages
@#  - ros2_msgs (list of ros1_bridge.Message)
@#    ROS 2 messages
@###############################################
@
@{
from ros1_bridge import camel_case_to_lower_case_underscore
}@
#include <memory>
#include <string>

#include <ros1_bridge/factory.hpp>
#include <ros1_bridge/action_factory.hpp>

// include ROS 1 messages
@[for ros1_msg in mapped_ros1_msgs]@
#include <@(ros1_msg.package_name)/@(ros1_msg.message_name).h>
@[end for]@

// include ROS 2 messages
@[for ros2_msg in mapped_ros2_msgs]@
#include <@(ros2_msg.package_name)/msg/@(camel_case_to_lower_case_underscore(ros2_msg.message_name)).hpp>
@[end for]@

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)(const std::string & ros1_type_name, const std::string & ros2_type_name);
@[for m in ros2_msg_types]@

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)__msg__@(m.message_name)(const std::string & ros1_type_name, const std::string & ros2_type_name);
@[end for]@

std::unique_ptr<ServiceFactoryInterface>
get_service_factory_@(ros2_package_name)(const std::string & ros_id, const std::string & package_name, const std::string & service_name);
@[for s in ros2_srv_types]@

std::unique_ptr<ServiceFactoryInterface>
get_service_factory_@(ros2_package_name)__srv__@(s.message_name)(const std::string & ros_id, const std::string & package_name, const std::string & service_name);
@[end for]@

std::unique_ptr<ActionFactoryInterface>
get_action_factory_@(ros2_package_name)(const std::string & ros_id, const std::string & package_name, const std::string & action_name);
@[for a in ros2_action_types]@

std::unique_ptr<ActionFactoryInterface>
get_action_factory_@(ros2_package_name)__action__@(a.message_name)(const std::string & ros_id, const std::string & package_name, const std::string & action_name);
@[end for]@

// conversion functions for available interfaces
@[for m in mappings]@

template<>
void
Factory<
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
>::convert_1_to_2(
  const @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name) & ros1_msg,
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name) & ros2_msg);

template<>
void
Factory<
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
>::convert_2_to_1(
  const @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name) & ros2_msg,
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name) & ros1_msg);

@[end for]@
}  // namespace ros1_bridge
