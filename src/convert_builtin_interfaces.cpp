// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "ros1_bridge/convert_builtin_interfaces.hpp"

#include "ros/serialization.h"

namespace ros1_bridge
{

template<>
void
convert_1_to_2(
  const ros::Duration & ros1_type,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  ros2_msg.sec = ros1_type.sec;
  ros2_msg.nanosec = ros1_type.nsec;
}

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  ros::Duration & ros1_type)
{
  ros1_type.sec = ros2_msg.sec;
  ros1_type.nsec = ros2_msg.nanosec;
}

template<>
void
internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const builtin_interfaces::msg::Duration & ros2_msg)
{
  stream.next(ros2_msg.sec);
  stream.next(ros2_msg.nanosec);
}

template<>
void
internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  builtin_interfaces::msg::Duration & ros2_msg)
{
  stream.next(ros2_msg.sec);
  stream.next(ros2_msg.nanosec);
}

template<>
void
internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const builtin_interfaces::msg::Duration & ros2_msg)
{
  stream.next(ros2_msg.sec);
  stream.next(ros2_msg.nanosec);
}

template<>
void
convert_1_to_2(
  const ros::Time & ros1_type,
  builtin_interfaces::msg::Time & ros2_msg)
{
  ros2_msg.sec = ros1_type.sec;
  ros2_msg.nanosec = ros1_type.nsec;
}

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  ros::Time & ros1_type)
{
  ros1_type.sec = ros2_msg.sec;
  ros1_type.nsec = ros2_msg.nanosec;
}

template<>
void
internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const builtin_interfaces::msg::Time & ros2_msg)
{
  stream.next(ros2_msg.sec);
  stream.next(ros2_msg.nanosec);
}

template<>
void
internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  builtin_interfaces::msg::Time & ros2_msg)
{
  stream.next(ros2_msg.sec);
  stream.next(ros2_msg.nanosec);
}

template<>
void
internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const builtin_interfaces::msg::Time & ros2_msg)
{
  stream.next(ros2_msg.sec);
  stream.next(ros2_msg.nanosec);
}

}  // namespace ros1_bridge
