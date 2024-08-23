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

#ifndef ROS1_BRIDGE__COMMAND_PARSER_UTILS_HPP_
#define ROS1_BRIDGE__COMMAND_PARSER_UTILS_HPP_

#include <string>
#include <vector>

namespace ros1_bridge
{

bool find_command_option(
  const std::vector<const char *> & args,
  const std::string & option);

bool get_option_value(
  std::vector<const char *> & args,
  const std::string & option,
  const char * & value,
  bool remove = false);

bool get_option_values(
  std::vector<const char *> & args, const std::string & option,
  std::vector<const char *> & available_options,
  std::vector<const char *> & values, bool remove = false);

bool get_option_flag(
  std::vector<const char *> & args,
  const std::string & option,
  bool remove = false);

void split_ros1_ros2_args(
  const std::vector<const char *> & args,
  std::vector<const char *> & ros1_args,
  std::vector<const char *> & ros2_args);

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__COMMAND_PARSER_UTILS_HPP_
