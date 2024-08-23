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

#include "ros1_bridge/command_parser_utils.hpp"

#include <algorithm>
#include <cstring>

namespace ros1_bridge
{

bool find_command_option(const std::vector<const char *> & args, const std::string & option)
{
  auto it = std::find_if(
    args.begin(), args.end(), [&option](const char * const & element) {
      return strcmp(element, option.c_str()) == 0;
    });

  return it != args.end();
}

bool get_option_value(
  std::vector<const char *> & args, const std::string & option,
  const char * & value, bool remove)
{
  auto it = std::find_if(
    args.begin(), args.end(), [&option](const char * const & element) {
      return strcmp(element, option.c_str()) == 0;
    });

  if (it != args.end()) {
    auto value_it = std::next(it);

    if (value_it != args.end()) {
      value = *value_it;

      if (remove) {
        args.erase(it);  // Remove option
        args.erase(it);  // Remove value
      }

      return true;
    }
  }

  return false;
}

bool get_option_values(
  std::vector<const char *> & args, const std::string & option,
  std::vector<const char *> & available_options,
  std::vector<const char *> & values, bool remove)
{
  auto it = std::find_if(
    args.begin(), args.end(), [&option](const char * const & element) {
      return strcmp(element, option.c_str()) == 0;
    });

  if (it != args.end()) {
    auto value_it = std::next(it);

    while (value_it != args.end() &&
      std::none_of(
        available_options.begin(), available_options.end(),
        [value_it](const char * available_option) {
          return strcmp(*value_it, available_option) == 0;
        }))
    {
      values.push_back(*value_it);

      if (remove) {
        args.erase(value_it);
      } else {
        ++value_it;
      }
    }

    if (remove) {
      args.erase(it);  // Remove option
    }

    return true;
  }

  return false;
}

bool get_option_flag(std::vector<const char *> & args, const std::string & option, bool remove)
{
  auto it = std::find_if(
    args.begin(), args.end(), [&option](const char * const & element) {
      return strcmp(element, option.c_str()) == 0;
    });

  if (it != args.end()) {
    if (remove) {
      args.erase(it);
    }
    return true;
  }

  return false;
}

void split_ros1_ros2_args(
  const std::vector<const char *> & args, std::vector<const char *> & ros1_args,
  std::vector<const char *> & ros2_args)
{
  // Start iterating from the second argument, since the first argument is the executable name
  auto it = std::find_if(
    args.begin() + 1, args.end(), [](const char * const & element) {
      return strcmp(element, "--ros-args") == 0;
    });

  if (it != args.end()) {
    ros1_args = std::vector<const char *>(args.begin(), it);
    ros2_args = std::vector<const char *>(it, args.end());
  } else {
    ros1_args = args;
    ros2_args = {};
  }

  ros2_args.insert(ros2_args.begin(), args.at(0));
}

}  // namespace ros1_bridge
