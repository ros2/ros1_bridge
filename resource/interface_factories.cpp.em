// generated from ros1_bridge/resource/interface_factories.cpp.em

@###############################################
@#
@# Factory template specializations for a
@# specific message type
@#
@# EmPy template for generating
@# <pkgname>__<type>__<interfacename>__factories.cpp
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
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import NamespacedType
}@
#include "@(ros2_package_name)_factories.hpp"

#include <algorithm>

#include "rclcpp/rclcpp.hpp"

// include builtin interfaces
#include <ros1_bridge/convert_builtin_interfaces.hpp>

// include ROS 1 services
@[for service in mapped_services]@
#include <@(service["ros1_package"])/@(service["ros1_name"]).h>
@[end for]@

// include ROS 2 services
@[for service in mapped_services]@
#include <@(service["ros2_package"])/srv/@(camel_case_to_lower_case_underscore(service["ros2_name"])).hpp>
@[end for]@

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_@(ros2_package_name)__@(interface_type)__@(interface.message_name)(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
@[if not mapped_msgs]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[else]@
  // mapping from string to specialized template
@[end if]@
@[for m in mapped_msgs]@
  if (
    (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)" ||
     ros1_type_name == "") &&
    ros2_type_name == "@(m.ros2_msg.package_name)/msg/@(m.ros2_msg.message_name)")
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

std::unique_ptr<ServiceFactoryInterface>
get_service_factory_@(ros2_package_name)__@(interface_type)__@(interface.message_name)(const std::string & ros_id, const std::string & package_name, const std::string & service_name)
{
@[if not mapped_services]@
  (void)ros_id;
  (void)package_name;
  (void)service_name;
@[end if]@
@[for service in mapped_services]@
  if (
    (
      ros_id == "ros1" &&
      package_name == "@(service["ros1_package"])" &&
      service_name == "@(service["ros1_name"])"
    ) || (
      ros_id == "ros2" &&
      package_name == "@(service["ros2_package"])" &&
      service_name == "srv/@(service["ros2_name"])"
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
@
// conversion functions for available interfaces
@[for m in mapped_msgs]@

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
@[  for ros1_fields, ros2_fields in m.fields_1_to_2.items()]@
@{
ros1_field_selection = '.'.join((str(field.name) for field in ros1_fields))
ros2_field_selection = '.'.join((str(field.name) for field in ros2_fields))

if isinstance(ros2_fields[-1].type, NamespacedType):
    namespaces = ros2_fields[-1].type.namespaces
    assert len(namespaces) == 2 and namespaces[1] == 'msg', \
      "messages not using the '<pkg_name>, msg, <type_name>' triplet are not supported"
}
@[    if not isinstance(ros2_fields[-1].type, AbstractNestedType)]@
  // convert non-array field
@[      if not isinstance(ros2_fields[-1].type, NamespacedType)]@
  // convert primitive field
  ros2_msg.@(ros2_field_selection) = ros1_msg.@(ros1_field_selection);
@[      elif ros2_fields[-1].type.namespaces[0] == 'builtin_interfaces']@
  // convert builtin field
  ros1_bridge::convert_1_to_2(ros1_msg.@(ros1_field_selection), ros2_msg.@(ros2_field_selection));
@[      else]@
  // convert sub message field
  Factory<
    @(ros1_fields[-1].pkg_name)::@(ros1_fields[-1].msg_name),
    @(ros2_fields[-1].type.namespaces[0])::msg::@(ros2_fields[-1].type.name)
  >::convert_1_to_2(
    ros1_msg.@(ros1_field_selection), ros2_msg.@(ros2_field_selection));
@[      end if]@
@[    else]@
  // convert array or sequence field
@[      if isinstance(ros2_fields[-1].type, AbstractSequence)]@
  // dynamically sized sequence, ensure destination sequence/vector size is large enough
@[        if isinstance(ros2_fields[-1].type, BoundedSequence)]@
  // bounded size sequence, check that the ros 1 vector size is not larger than the upper bound for the target
  assert(ros1_msg.@(ros1_field_selection).size() <= @(ros2_fields[-1].type.maximum_size));
@[        end if]@
  // resize ros2 field to match the ros1 field
  ros2_msg.@(ros2_field_selection).resize(ros1_msg.@(ros1_field_selection).size());
@[      else]@
  // statically sized array
  static_assert(
    (ros2_msg.@(ros2_field_selection).size()) >= (ros1_msg.@(ros1_field_selection).size()),
    "destination array not large enough for source array"
  );
@[      end if]@
@[      if not isinstance(ros2_fields[-1].type.value_type, NamespacedType)]@
  // convert primitive array elements
  std::copy(
    ros1_msg.@(ros1_field_selection).begin(),
    ros1_msg.@(ros1_field_selection).end(),
    ros2_msg.@(ros2_field_selection).begin());
@[      else]@
  // copy element wise since the type is different
  {
    auto ros1_it = ros1_msg.@(ros1_field_selection).cbegin();
    auto ros2_it = ros2_msg.@(ros2_field_selection).begin();
    for (
      ;
      ros1_it != ros1_msg.@(ros1_field_selection).cend() &&
      ros2_it != ros2_msg.@(ros2_field_selection).end();
      ++ros1_it, ++ros2_it
    )
    {
      // convert sub message element
@[        if ros2_fields[-1].type.value_type.namespaces[0] == 'builtin_interfaces']@
      ros1_bridge::convert_1_to_2(*ros1_it, *ros2_it);
@[        else]@
      Factory<
        @(ros1_fields[-1].pkg_name)::@(ros1_fields[-1].msg_name),
        @(ros2_fields[-1].type.value_type.namespaces[0])::msg::@(ros2_fields[-1].type.value_type.name)
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
@[  for ros2_fields, ros1_fields in m.fields_2_to_1.items()]@
@{
ros2_field_selection = '.'.join((str(field.name) for field in ros2_fields))
ros1_field_selection = '.'.join((str(field.name) for field in ros1_fields))

if isinstance(ros2_fields[-1].type, NamespacedType):
    namespaces = ros2_fields[-1].type.namespaces
    assert len(namespaces) == 2 and namespaces[1] == 'msg', \
      "messages not using the '<pkg_name>, msg, <type_name>' triplet are not supported"
}
@[    if not isinstance(ros2_fields[-1].type, AbstractNestedType)]@
  // convert non-array field
@[      if not isinstance(ros2_fields[-1].type, NamespacedType)]@
  // convert primitive field
  ros1_msg.@(ros1_field_selection) = ros2_msg.@(ros2_field_selection);
@[      elif ros2_fields[-1].type.namespaces[0] == 'builtin_interfaces']@
  // convert builtin field
  ros1_bridge::convert_2_to_1(ros2_msg.@(ros2_field_selection), ros1_msg.@(ros1_field_selection));
@[      else]@
  // convert sub message field
  Factory<
    @(ros1_fields[-1].pkg_name)::@(ros1_fields[-1].msg_name),
    @(ros2_fields[-1].type.namespaces[0])::msg::@(ros2_fields[-1].type.name)
  >::convert_2_to_1(
    ros2_msg.@(ros2_field_selection), ros1_msg.@(ros1_field_selection));
@[      end if]@
@[    else]@
  // convert array or sequence field
@[      if isinstance(ros2_fields[-1].type, AbstractSequence)]@
  // dynamically sized sequence, ensure destination vector size is large enough
  // resize ros1 field to match the ros2 field
  ros1_msg.@(ros1_field_selection).resize(ros2_msg.@(ros2_field_selection).size());
@[      else]@
  // statically sized array
  static_assert(
    (ros1_msg.@(ros1_field_selection).size()) >= (ros2_msg.@(ros2_field_selection).size()),
    "destination array not large enough for source array"
  );
@[      end if]@
@[      if not isinstance(ros2_fields[-1].type.value_type, NamespacedType)]@
  // convert primitive array elements
  std::copy(
    ros2_msg.@(ros2_field_selection).begin(),
    ros2_msg.@(ros2_field_selection).end(),
    ros1_msg.@(ros1_field_selection).begin());
@[      else]@
  // copy element wise since the type is different
  {
    auto ros2_it = ros2_msg.@(ros2_field_selection).cbegin();
    auto ros1_it = ros1_msg.@(ros1_field_selection).begin();
    for (
      ;
      ros2_it != ros2_msg.@(ros2_field_selection).cend() &&
      ros1_it != ros1_msg.@(ros1_field_selection).end();
      ++ros2_it, ++ros1_it
    )
    {
      // convert sub message element
@[        if ros2_fields[-1].type.value_type.namespaces[0] == 'builtin_interfaces']@
      ros1_bridge::convert_2_to_1(*ros2_it, *ros1_it);
@[        else]@
      Factory<
        @(ros1_fields[-1].pkg_name)::@(ros1_fields[-1].msg_name),
        @(ros2_fields[-1].type.value_type.namespaces[0])::msg::@(ros2_fields[-1].type.value_type.name)
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
@
@[for service in mapped_services]@

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
}  // namespace ros1_bridge
