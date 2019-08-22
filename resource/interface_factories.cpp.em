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
@{
from ros1_bridge import camel_case_to_lower_case_underscore
}@
#include "rclcpp/rclcpp.hpp"
#include "@(ros2_package_name)_factories.hpp"

// include builtin interfaces
#include <ros1_bridge/convert_builtin_interfaces.hpp>

// include ROS 1 services
@[for service in services]@
#include <@(service["ros1_package"])/@(service["ros1_name"]).h>
@[end for]@

// include ROS 2 services
@[for service in services]@
#include <@(service["ros2_package"])/srv/@(camel_case_to_lower_case_underscore(service["ros2_name"])).hpp>
@[end for]@

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
@[if not mappings]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[end if]@
  // mapping from string to specialized template
@[for m in mappings]@
  if (
    (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)" ||
     ros1_type_name == "") &&
    ros2_type_name == "@(m.ros2_msg.package_name)/@(m.ros2_msg.message_name)")
  {
    return std::make_shared<
      Factory<
        @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
        @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
      >
    >("@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)", ros2_type_name);
  }
@[end for]@
  return std::shared_ptr<FactoryInterface>();
}

// conversion functions for available interfaces
@[for m in mappings]@

template<>
void
Factory<
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
>::convert_1_to_2(
  const @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name) & ros1_msg,
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name) & ros2_msg)
{
@[  if not m.fields_1_to_2]@
  (void)ros1_msg;
  (void)ros2_msg;
@[  end if]@
@[  for ros1_field, ros2_field in m.fields_1_to_2.items()]@
@[    if not ros2_field.type.is_array]@
  // convert non-array field
@[      if not ros2_field.type.pkg_name]@
  // convert primitive field
  ros2_msg.@(ros2_field.name) = ros1_msg.@(ros1_field.name);
@[      elif ros2_field.type.pkg_name == 'builtin_interfaces']@
  // convert builtin field
  ros1_bridge::convert_1_to_2(ros1_msg.@(ros1_field.name), ros2_msg.@(ros2_field.name));
@[      else]@
  // convert sub message field
  Factory<
    @(ros1_field.pkg_name)::@(ros1_field.msg_name),
    @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
  >::convert_1_to_2(
    ros1_msg.@(ros1_field.name), ros2_msg.@(ros2_field.name));
@[      end if]@
@[    else]@
  // convert array field
@[      if not ros2_field.type.array_size or ros2_field.type.is_upper_bound]@
  // ensure array size
@[        if ros2_field.type.is_upper_bound]@
  // check boundary
  assert(ros1_msg.@(ros1_field.name).size() <= @(ros2_field.type.array_size));
@[        end if]@
  // dynamic arrays must be resized
  ros2_msg.@(ros2_field.name).resize(ros1_msg.@(ros1_field.name).size());
@[      end if]@
@[      if not ros2_field.type.pkg_name]@
  // convert primitive array elements
  std::copy(
    ros1_msg.@(ros1_field.name).begin(),
    ros1_msg.@(ros1_field.name).end(),
    ros2_msg.@(ros2_field.name).begin());
@[      else]@
  // copy element wise since the type is different
  {
    auto ros1_it = ros1_msg.@(ros1_field.name).begin();
    auto ros2_it = ros2_msg.@(ros2_field.name).begin();
    for (
      ;
      ros1_it != ros1_msg.@(ros1_field.name).end() &&
      ros2_it != ros2_msg.@(ros2_field.name).end();
      ++ros1_it, ++ros2_it
    )
    {
      // convert sub message element
@[        if ros2_field.type.pkg_name == 'builtin_interfaces']@
      ros1_bridge::convert_1_to_2(*ros1_it, *ros2_it);
@[        else]@
      Factory<
        @(ros1_field.pkg_name)::@(ros1_field.msg_name),
        @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
      >::convert_1_to_2(
        *ros1_it, *ros2_it);
@[        end if]@
    }
  }
@[      end if]@
@[    end if]@
@[  end for]@
}

template<>
void
Factory<
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name),
  @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name)
>::convert_2_to_1(
  const @(m.ros2_msg.package_name)::msg::@(m.ros2_msg.message_name) & ros2_msg,
  @(m.ros1_msg.package_name)::@(m.ros1_msg.message_name) & ros1_msg)
{
@[  if not m.fields_2_to_1]@
  (void)ros2_msg;
  (void)ros1_msg;
@[  end if]@
@[  for ros2_field, ros1_field in m.fields_2_to_1.items()]@
@[    if not ros2_field.type.is_array]@
  // convert non-array field
@[      if not ros2_field.type.pkg_name]@
  // convert primitive field
  ros1_msg.@(ros1_field.name) = ros2_msg.@(ros2_field.name);
@[      elif ros2_field.type.pkg_name == 'builtin_interfaces']@
  // convert builtin field
  ros1_bridge::convert_2_to_1(ros2_msg.@(ros2_field.name), ros1_msg.@(ros1_field.name));
@[      else]@
  // convert sub message field
  Factory<
    @(ros1_field.pkg_name)::@(ros1_field.msg_name),
    @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
  >::convert_2_to_1(
    ros2_msg.@(ros2_field.name), ros1_msg.@(ros1_field.name));
@[      end if]@
@[    else]@
  // convert array field
@[      if not ros2_field.type.array_size or ros2_field.type.is_upper_bound]@
  // ensure array size
  // dynamic arrays must be resized
  ros1_msg.@(ros1_field.name).resize(ros2_msg.@(ros2_field.name).size());
@[      end if]@
@[      if not ros2_field.type.pkg_name]@
  // convert primitive array elements
  std::copy(
    ros2_msg.@(ros2_field.name).begin(),
    ros2_msg.@(ros2_field.name).end(),
    ros1_msg.@(ros1_field.name).begin());
@[      else]@
  // copy element wise since the type is different
  {
    auto ros2_it = ros2_msg.@(ros2_field.name).begin();
    auto ros1_it = ros1_msg.@(ros1_field.name).begin();
    for (
      ;
      ros2_it != ros2_msg.@(ros2_field.name).end() &&
      ros1_it != ros1_msg.@(ros1_field.name).end();
      ++ros2_it, ++ros1_it
    )
    {
      // convert sub message element
@[        if ros2_field.type.pkg_name == 'builtin_interfaces']@
      ros1_bridge::convert_2_to_1(*ros2_it, *ros1_it);
@[        else]@
      Factory<
        @(ros1_field.pkg_name)::@(ros1_field.msg_name),
        @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
      >::convert_2_to_1(
        *ros2_it, *ros1_it);
@[        end if]@
    }
  }
@[      end if]@
@[    end if]@
@[  end for]@
}

@[end for]@

@[for service in services]@
@[  for frm, to in [("1", "2"), ("2", "1")]]@
@[    for type in ["Request", "Response"]]@
template <>
void ServiceFactory<
  @(service["ros1_package"])::@(service["ros1_name"]),
  @(service["ros2_package"])::srv::@(service["ros2_name"])
>::translate_@(frm)_to_@(to)(
@[      if frm == "1"]@
  const @(service["ros1_package"])::@(service["ros1_name"])::@(type)& req1,
  @(service["ros2_package"])::srv::@(service["ros2_name"])::@(type)& req2
@[      else]@
  const @(service["ros2_package"])::srv::@(service["ros2_name"])::@(type)& req2,
  @(service["ros1_package"])::@(service["ros1_name"])::@(type)& req1
@[      end if]@
) {
@[      for field in service["fields"][type.lower()]]@
@[        if field["array"]]@
  req@(to).@(field["ros" + to]["name"]).resize(req@(frm).@(field["ros" + frm]["name"]).size());
  auto @(field["ros1"]["name"])1_it = req1.@(field["ros1"]["name"]).begin();
  auto @(field["ros2"]["name"])2_it = req2.@(field["ros2"]["name"]).begin();
  while (
    @(field["ros1"]["name"])1_it != req1.@(field["ros1"]["name"]).end() &&
    @(field["ros2"]["name"])2_it != req2.@(field["ros2"]["name"]).end()
  ) {
    auto & @(field["ros1"]["name"])1 = *(@(field["ros1"]["name"])1_it++);
    auto & @(field["ros2"]["name"])2 = *(@(field["ros2"]["name"])2_it++);
@[      else]@
  auto & @(field["ros1"]["name"])1 = req1.@(field["ros1"]["name"]);
  auto & @(field["ros2"]["name"])2 = req2.@(field["ros2"]["name"]);
@[        end if]@
@[        if field["basic"]]@
  @(field["ros2"]["name"])@(to) = @(field["ros1"]["name"])@(frm);
@[        else]@
  Factory<@(field["ros1"]["cpptype"]),@(field["ros2"]["cpptype"])>::convert_@(frm)_to_@(to)(@
@(field["ros2"]["name"])@(frm), @(field["ros1"]["name"])@(to));
@[        end if]@
@[        if field["array"]]@
  }
@[        end if]@
@[      end for]@
}

@[    end for]@
@[  end for]@
@[end for]@

std::unique_ptr<ServiceFactoryInterface>
get_service_factory_@(ros2_package_name)(const std::string & ros_id, const std::string & package_name, const std::string & service_name)
{
@[if not services]@
  (void)ros_id;
  (void)package_name;
  (void)service_name;
@[end if]@
@[for service in services]@
  if (
    (
      ros_id == "ros1" &&
      package_name == "@(service["ros1_package"])" &&
      service_name == "@(service["ros1_name"])"
    ) || (
      ros_id == "ros2" &&
      package_name == "@(service["ros2_package"])" &&
      service_name == "@(service["ros2_name"])"
    )
  ) {
    return std::unique_ptr<ServiceFactoryInterface>(new ServiceFactory<
      @(service["ros1_package"])::@(service["ros1_name"]),
      @(service["ros2_package"])::srv::@(service["ros2_name"])
    >);
  }
@[end for]@
  return nullptr;
}
}  // namespace ros1_bridge
