// generated from ros1_bridge/resource/pkg_factories.cpp.em

@###############################################
@#
@# Factory template specializations based on
@# message types of a single ROS 2 package
@#
@# EmPy template for generating <pkgname>_factories.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_name (str)
@#    The ROS 2 package name of this file
@#  - mappings (list of ros1_bridge.Mapping)
@#    Mapping between messages as well as their fields
@###############################################
@
#include "@(ros2_package_name)_factories.hpp"

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
@[if not ros2_msg_types]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[else]@
  std::shared_ptr<FactoryInterface> factory;
@[end if]@
@[for m in ros2_msg_types]@
  factory = get_factory_@(ros2_package_name)__msg__@(m.message_name)(ros1_type_name, ros2_type_name);
  if (factory) {
    return factory;
  }
@[end for]@
  return std::shared_ptr<FactoryInterface>();
}

std::unique_ptr<ServiceFactoryInterface>
get_service_factory_@(ros2_package_name)(const std::string & ros_id, const std::string & package_name, const std::string & service_name)
{
@[if not ros2_srv_types]@
  (void)ros_id;
  (void)package_name;
  (void)service_name;
@[else]@
  std::unique_ptr<ServiceFactoryInterface> factory;
@[end if]@
@[for s in ros2_srv_types]@
  factory = get_service_factory_@(ros2_package_name)__srv__@(s.message_name)(ros_id, package_name, service_name);
  if (factory) {
    return factory;
  }
@[end for]@
  return nullptr;
}

std::unique_ptr<ActionFactoryInterface>
get_action_factory_@(ros2_package_name)(const std::string & ros_id, const std::string & package_name, const std::string & action_name)
{
@[if not ros2_action_types]@
  (void)ros_id;
  (void)package_name;
  (void)action_name;
@[else]@
  std::unique_ptr<ActionFactoryInterface> factory;
@[end if]@
@[for a in ros2_action_types]@
  factory = get_action_factory_@(ros2_package_name)__action__@(a.message_name)(ros_id, package_name, action_name);
  if (factory) {
    return factory;
  }
@[end for]@
  return nullptr;
}
}  // namespace ros1_bridge
