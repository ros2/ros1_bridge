// generated from ros1_bridge/resource/generated_factories.cpp.em

@###############################################
@#
@# Factory for creating publisher / subscribers
@# based on message names
@#
@# EmPy template for generating generated_factories.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros1_msgs (list of ros1_bridge.Message)
@#    ROS 1 messages
@#  - ros2_msgs (list of ros1_bridge.Message)
@#    ROS 2 messages
@#  - mappings (list of ros1_bridge.Mapping)
@#    Mapping between messages as well as their fields
@###############################################
@
@{
from ros1_bridge import camel_case_to_lower_case_underscore
}@
#include <ros1_bridge/factory.hpp>

// include ROS 1 builtin messages
#include <ros/duration.h>
#include <ros/time.h>

// include ROS 2 builtin messages
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

// include ROS 1 messages
@[for ros1_msg in ros1_msgs]@
#include <@(ros1_msg.package_name)/@(ros1_msg.message_name).h>
@[end for]@

// include ROS 2 messages
@[for ros2_msg in ros2_msgs]@
#include <@(ros2_msg.package_name)/msg/@(camel_case_to_lower_case_underscore(ros2_msg.message_name)).hpp>
@[end for]@


namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros1_type_name, const std::string & ros2_type_name)
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
    >();
  }
@[end for]@
  throw std::runtime_error("No template specialization for the pair");
}


// conversion functions for builtin interfaces

template<typename ROS1_T, typename ROS2_T>
void
convert_1_to_2(
  const ROS1_T & ros1_msg,
  ROS2_T & ros2_msg);

template<typename ROS1_T, typename ROS2_T>
void
convert_2_to_1(
  const ROS2_T & ros2_msg,
  ROS1_T & ros1_msg);


template<>
void
convert_1_to_2(
  const ros::Duration & ros1_msg,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros2_msg.sec = ros1_msg.sec;
  ros2_msg.nanosec = ros1_msg.nsec;
}

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  ros::Duration & ros1_msg)
{
  ros1_msg.sec = ros2_msg.sec;
  ros1_msg.nsec = ros2_msg.nanosec;
}


template<>
void
convert_1_to_2(
  const ros::Time & ros1_msg,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros2_msg.sec = ros1_msg.sec;
  ros2_msg.nanosec = ros1_msg.nsec;
}

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  ros::Time & ros1_msg)
{
  ros1_msg.sec = ros2_msg.sec;
  ros1_msg.nsec = ros2_msg.nanosec;
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
@[if not m.fields_1_to_2]@
  (void)ros1_msg;
  (void)ros2_msg;
@[end if]@
@[for ros1_field, ros2_field in m.fields_1_to_2.items()]@
@
@ @[if not ros2_field.type.is_array]@
@ @ @# convert non-array field
@
@ @ @[if not ros2_field.type.pkg_name]@
@ @ @ @# convert primitive field
  ros2_msg.@(ros2_field.name) = ros1_msg.@(ros1_field.name);
@
@ @ @[elif ros2_field.type.pkg_name == 'builtin_interfaces']@
@ @ @ @# convert builtin field
  ros1_bridge::convert_1_to_2(ros1_msg.@(ros1_field.name), ros2_msg.@(ros2_field.name));
@
@ @ @[else]@
@ @ @ @# convert sub message field
  Factory<
    @(ros1_field.pkg_name)::@(ros1_field.msg_name),
    @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
  >::convert_1_to_2(
    ros1_msg.@(ros1_field.name), ros2_msg.@(ros2_field.name));
@ @ @[end if]@
@
@ @[else]@
@ @ @# convert array field
@
@ @ @[if not ros2_field.type.array_size or ros2_field.type.is_upper_bound]@
@ @ @ @# ensure array size
@
@ @ @ @[if ros2_field.type.is_upper_bound]@
@ @ @ @ @# check boundary
  assert(ros1_msg.@(ros1_field.name).size() <= @(ros2_field.type.array_size));
@ @ @ @[end if]@
@
@ @ @ @# dynamic arrays must be resized
  ros2_msg.@(ros2_field.name).resize(ros1_msg.@(ros1_field.name).size());
@ @ @[end if]@
@
@ @ @[if not ros2_field.type.pkg_name]@
@ @ @ @# convert primitive array elements
  std::copy(
    ros1_msg.@(ros1_field.name).begin(),
    ros1_msg.@(ros1_field.name).end(),
    ros2_msg.@(ros2_field.name).begin());
@
@ @ @[else]@
@ @ @ @# copy element wise since the type is different
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
@ @ @ @# convert sub message element
@ @ @ @[if ros2_field.type.pkg_name == 'builtin_interfaces']@
      ros1_bridge::convert_1_to_2(*ros1_it, *ros2_it);
@ @ @ @[else]@
      Factory<
        @(ros1_field.pkg_name)::@(ros1_field.msg_name),
        @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
      >::convert_1_to_2(
        *ros1_it, *ros2_it);
@ @ @ @[end if]@
    }
  }
@ @ @[end if]@
@ @[end if]@
@[end for]@
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
@[if not m.fields_2_to_1]@
  (void)ros2_msg;
  (void)ros1_msg;
@[end if]@
@[for ros2_field, ros1_field in m.fields_2_to_1.items()]@
@
@ @[if not ros2_field.type.is_array]@
@ @ @# convert non-array field
@
@ @ @[if not ros2_field.type.pkg_name]@
@ @ @ @# convert primitive field
  ros1_msg.@(ros1_field.name) = ros2_msg.@(ros2_field.name);
@
@ @ @[elif ros2_field.type.pkg_name == 'builtin_interfaces']@
@ @ @ @# convert builtin field
  ros1_bridge::convert_2_to_1(ros2_msg.@(ros2_field.name), ros1_msg.@(ros1_field.name));
@
@ @ @[else]@
@ @ @ @# convert sub message field
  Factory<
    @(ros1_field.pkg_name)::@(ros1_field.msg_name),
    @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
  >::convert_2_to_1(
    ros2_msg.@(ros2_field.name), ros1_msg.@(ros1_field.name));
@ @ @[end if]@
@
@ @[else]@
@ @ @# convert array field
@
@ @ @[if not ros2_field.type.array_size or ros2_field.type.is_upper_bound]@
@ @ @ @# ensure array size
@
@ @ @ @# dynamic arrays must be resized
  ros1_msg.@(ros1_field.name).resize(ros2_msg.@(ros2_field.name).size());
@ @ @[end if]@
@
@ @ @[if not ros2_field.type.pkg_name]@
@ @ @ @# convert primitive array elements
  std::copy(
    ros2_msg.@(ros2_field.name).begin(),
    ros2_msg.@(ros2_field.name).end(),
    ros1_msg.@(ros1_field.name).begin());
@
@ @ @[else]@
@ @ @ @# copy element wise since the type is different
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
@ @ @ @# convert sub message element
@ @ @ @[if ros2_field.type.pkg_name == 'builtin_interfaces']@
      ros1_bridge::convert_2_to_1(*ros2_it, *ros1_it);
@ @ @ @[else]@
      Factory<
        @(ros1_field.pkg_name)::@(ros1_field.msg_name),
        @(ros2_field.type.pkg_name)::msg::@(ros2_field.type.type)
      >::convert_2_to_1(
        *ros2_it, *ros1_it);
@ @ @ @[end if]@
    }
  }
@ @ @[end if]@
@ @[end if]@
@[end for]@
}

@[end for]@
}  // namespace ros1_bridge
