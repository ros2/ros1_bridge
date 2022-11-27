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

#ifndef ROS1_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
#define ROS1_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_

#include <ros/serialization.h>

// include ROS 1 messages
#include <std_msgs/Duration.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>

#include <memory>
#include <string>

// include ROS 2 messages
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>

#include "ros1_bridge/factory.hpp"

namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros1_type_name,
  const std::string & ros2_type_name);

// conversion functions for available interfaces

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_1_to_2(
  const std_msgs::Duration & ros1_msg,
  builtin_interfaces::msg::Duration & ros2_msg);

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::convert_2_to_1(
  const builtin_interfaces::msg::Duration & ros2_msg,
  std_msgs::Duration & ros1_msg);

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const builtin_interfaces::msg::Duration & msg);

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  builtin_interfaces::msg::Duration & msg);

template<>
void
Factory<
  std_msgs::Duration,
  builtin_interfaces::msg::Duration
>::internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const builtin_interfaces::msg::Duration & msg);

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_1_to_2(
  const std_msgs::Time & ros1_msg,
  builtin_interfaces::msg::Time & ros2_msg);

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::convert_2_to_1(
  const builtin_interfaces::msg::Time & ros2_msg,
  std_msgs::Time & ros1_msg);

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::internal_stream_translate_helper(
  ros::serialization::OStream & stream,
  const builtin_interfaces::msg::Time & msg);

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::internal_stream_translate_helper(
  ros::serialization::IStream & stream,
  builtin_interfaces::msg::Time & msg);

template<>
void
Factory<
  std_msgs::Time,
  builtin_interfaces::msg::Time
>::internal_stream_translate_helper(
  ros::serialization::LStream & stream,
  const builtin_interfaces::msg::Time & msg);

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
