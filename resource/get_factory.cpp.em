// generated from ros1_bridge/resource/get_factory.cpp.em

@###############################################
@#
@# Factory for creating publisher / subscribers
@# based on message names
@#
@# EmPy template for generating get_factory.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_package_names (list of str)
@#    ROS 2 package names
@###############################################
@
@{
from ros1_bridge import camel_case_to_lower_case_underscore
}@
#include "ros1_bridge/factory.hpp"
#include "ros1_bridge/builtin_interfaces_factories.hpp"

@[for ros2_package_name in sorted(ros2_package_names)]@
#include "@(ros2_package_name)_factories.hpp"
@[end for]@

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
@[if not ros2_package_names]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[else]@
  std::shared_ptr<FactoryInterface> factory;
@[end if]@
  factory = get_factory_builtin_interfaces(ros1_type_name, ros2_type_name);
  if (factory) {
    return factory;
  }
@[for ros2_package_name in sorted(ros2_package_names)]@
  factory = get_factory_@(ros2_package_name)(ros1_type_name, ros2_type_name);
  if (factory) {
    return factory;
  }
@[end for]@
  throw std::runtime_error("No template specialization for the pair");
}

std::unique_ptr<ServiceFactoryInterface> get_service_factory(const std::string & ros_id, const std::string & package_name, const std::string & service_name)
{
@[if not ros2_package_names]@
  (void)ros_id;
  (void)package_name;
  (void)service_name;
@[else]@
  std::unique_ptr<ServiceFactoryInterface> factory;
@[end if]@
@[for ros2_package_name in sorted(ros2_package_names)]@
  factory = get_service_factory_@(ros2_package_name)(ros_id, package_name, service_name);
  if (factory) {
    return factory;
  }
@[end for]@
  return factory;
}

std::unique_ptr<ActionFactoryInterface> get_action_factory(const std::string & ros_id, const std::string & package_name, const std::string & action_name)
{
@[if not ros2_package_names]@
  (void)ros_id;
  (void)package_name;
  (void)action_name;
@[else]@
  std::unique_ptr<ActionFactoryInterface> factory;
@[end if]@
@[for ros2_package_name in sorted(ros2_package_names)]@
  factory = get_action_factory_@(ros2_package_name)(ros_id, package_name, action_name);
  if (factory) {
    return factory;
  }
@[end for]@
  return nullptr;
}

}  // namespace ros1_bridge
