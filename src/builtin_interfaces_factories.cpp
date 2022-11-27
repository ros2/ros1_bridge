// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// include builtin interfaces
#include "ros1_bridge/builtin_interfaces_factories.hpp"
#include "ros1_bridge/convert_builtin_interfaces.hpp"

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros1_type_name,
  const std::string & ros2_type_name)
{
  // mapping from string to specialized template
  if (
    (ros1_type_name == "std_msgs/Duration" || ros1_type_name == "") &&
    ros2_type_name == "builtin_interfaces/msg/Duration")
  {
    return std::make_shared<
      Factory<
        std_msgs::Duration,
        builtin_interfaces::msg::Duration
      >
    >("std_msgs/Duration", ros2_type_name);
  }
  if (
    (ros1_type_name == "std_msgs/Time" || ros1_type_name == "") &&
    ros2_type_name == "builtin_interfaces/msg/Time")
  {
    return std::make_shared<
      Factory<
        std_msgs::Time,
        builtin_interfaces::msg::Time
      >
    >("std_msgs/Time", ros2_type_name);
  }
  return std::shared_ptr<FactoryInterface>();
}

// conversion functions for available interfaces
template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_1_to_2(
  const std_msgs::Duration & ros1_msg,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros1_bridge::convert_1_to_2(ros1_msg.data, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  std_msgs::Duration & ros1_msg)
{
  ros1_bridge::convert_2_to_1(ros2_msg, ros1_msg.data);
}

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const builtin_interfaces::msg::Duration & ros2_msg)
{
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const builtin_interfaces::msg::Duration & ros2_msg)
{
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  ros::serialization::OStream & out_stream)
{
  internal_stream_translate_helper(out_stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_1_to_2(
  ros::serialization::IStream & in_stream,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  internal_stream_translate_helper(in_stream, ros2_msg);
}

template<>
uint32_t
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::length_2_as_1_stream(const builtin_interfaces::msg::Duration & ros2_msg)
{
  ros::serialization::LStream len_stream;
  internal_stream_translate_helper(len_stream, ros2_msg);
  return len_stream.getLength();
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_1_to_2(
  const std_msgs::Time & ros1_msg,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros1_bridge::convert_1_to_2(ros1_msg.data, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  std_msgs::Time & ros1_msg)
{
  ros1_bridge::convert_2_to_1(ros2_msg, ros1_msg.data);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const builtin_interfaces::msg::Time & ros2_msg)
{
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const builtin_interfaces::msg::Time & ros2_msg)
{
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  ros::serialization::OStream & out_stream)
{
  internal_stream_translate_helper(out_stream, ros2_msg);
}

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_1_to_2(
  ros::serialization::IStream & in_stream,
  builtin_interfaces::msg::Time & ros2_msg)
{
  internal_stream_translate_helper(in_stream, ros2_msg);
}

template<>
uint32_t
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::length_2_as_1_stream(const builtin_interfaces::msg::Time & ros2_msg)
{
  ros::serialization::LStream len_stream;
  internal_stream_translate_helper(len_stream, ros2_msg);
  return len_stream.getLength();
}

template<>
void
Factory<
  std_msgs::Header,
  std_msgs::msg::Header
>::internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const std_msgs::msg::Header & ros2_msg)
{
  // ROS2 Header is missing seq, provide a fake one so stream matches
  uint32_t seq = 0;
  stream.next(seq);

  // write non-array field
  // write builtin field
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg.stamp);

  // write non-array field
  // write primitive field
  stream.next(ros2_msg.frame_id);
}

template<>
void
Factory<
  std_msgs::Header,
  std_msgs::msg::Header
>::internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  std_msgs::msg::Header & ros2_msg)
{
  // ROS2 Header is missing seq, provide a fake one so stream matches
  uint32_t seq = 0;
  stream.next(seq);

  // write non-array field
  // write builtin field
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg.stamp);

  // write non-array field
  // write primitive field
  stream.next(ros2_msg.frame_id);
}

template<>
void
Factory<
  std_msgs::Header,
  std_msgs::msg::Header
>::internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const std_msgs::msg::Header & ros2_msg)
{
  // ROS2 Header is missing seq, provide a fake one so stream matches
  uint32_t seq = 0;
  stream.next(seq);

  // write non-array field
  // write builtin field
  ros1_bridge::internal_stream_translate_helper(stream, ros2_msg.stamp);

  // write non-array field
  // write primitive field
  stream.next(ros2_msg.frame_id);
}

template<>
void
Factory<
  std_msgs::Header,
  std_msgs::msg::Header
>::convert_2_to_1(
  const std_msgs::msg::Header & ros2_msg,
  ros::serialization::OStream & out_stream)
{
  internal_stream_translate_helper(out_stream, ros2_msg);
}


template<>
void
Factory<
  std_msgs::Header,
  std_msgs::msg::Header
>::convert_1_to_2(
  ros::serialization::IStream & in_stream,
  std_msgs::msg::Header & ros2_msg)
{
  internal_stream_translate_helper(in_stream, ros2_msg);
}

template<>
uint32_t
Factory<
  std_msgs::Header,
  std_msgs::msg::Header
>::length_2_as_1_stream(const std_msgs::msg::Header & ros2_msg)
{
  ros::serialization::LStream len_stream;
  internal_stream_translate_helper(len_stream, ros2_msg);
  return len_stream.getLength();
}

}  // namespace ros1_bridge
