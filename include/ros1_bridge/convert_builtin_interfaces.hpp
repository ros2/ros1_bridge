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

#ifndef ROS1_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_
#define ROS1_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_

#include "ros1_bridge/convert_decl.hpp"

// include ROS 1 builtin messages
#include "ros/duration.h"
#include "ros/time.h"

// include ROS 2 builtin messages
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace ros1_bridge
{

template<>
void
convert_1_to_2(
  const ros::Duration & ros1_type,
  builtin_interfaces::msg::Duration & ros2_msg);

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  ros::Duration & ros1_type);

template<typename STREAM_T>
void
internal_stream_translate_helper(
  STREAM_T & stream,
  const builtin_interfaces::msg::Duration & msg);

template<typename STREAM_T>
void
internal_stream_translate_helper(
  STREAM_T & stream,
  builtin_interfaces::msg::Duration & msg);


template<>
void
convert_1_to_2(
  const ros::Time & ros1_type,
  builtin_interfaces::msg::Time & ros2_msg);

template<>
void
convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  ros::Time & ros1_type);

template<typename STREAM_T>
void
internal_stream_translate_helper(
  STREAM_T & stream,
  const builtin_interfaces::msg::Time & msg);

template<typename STREAM_T>
void
internal_stream_translate_helper(
  STREAM_T & stream,
  builtin_interfaces::msg::Time & msg);

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_
