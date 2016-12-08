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
#include <ros1_bridge/factory.hpp>

// include ROS 1 messages
@[for ros1_msg in ros1_msgs]@
#include <@(ros1_msg.package_name)/@(ros1_msg.message_name).h>
@[end for]@

// include ROS 1 services
@[for ros1_srv in ros1_srvs]@
#include <@(ros1_srv.package_name)/@(ros1_srv.service_name).h>
@[end for]@

// include ROS 2 messages
@[for ros2_msg in ros2_msgs]@
#include <@(ros2_msg.package_name)/msg/@(camel_case_to_lower_case_underscore(ros2_msg.message_name)).hpp>
@[end for]@

// include ROS 2 services
@[for ros2_srv in ros2_srvs]@
#include <@(ros2_srv.package_name)/srv/@(camel_case_to_lower_case_underscore(ros2_srv.service_name)).hpp>
@[end for]@

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)(const std::string & ros1_type_name, const std::string & ros2_type_name);

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
