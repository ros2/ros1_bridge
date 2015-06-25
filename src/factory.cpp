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

#include <ros1_bridge/factory.hpp>

// include ROS 1 messages
#include <std_msgs/String.h>

// include ROS 2 messages
#include <std_interfaces/msg/string.hpp>


namespace ros1_bridge
{

std::shared_ptr<FactoryInterface>
get_factory(const std::string & ros1_type_name, const std::string & ros2_type_name)
{
  // mapping from string to specialized template
  //REGISTER_TYPE_PAIR(std_msgs, String, std_interfaces, String)
  if (ros1_type_name == "std_msgs/String" && ros2_type_name == "std_interfaces/String") {
    return std::make_shared<Factory<std_msgs::String, std_interfaces::msg::String>>();
  }
  throw std::runtime_error("No template specialization for the pair");
}

template<>
void
Factory<std_msgs::String, std_interfaces::msg::String>::convert_1_to_2(
  std_msgs::String::ConstPtr ros1_msg,
  std_interfaces::msg::String::SharedPtr ros2_msg)
{
  ros2_msg->data = ros1_msg->data;
}

template<>
void
Factory<std_msgs::String, std_interfaces::msg::String>::convert_2_to_1(
  std_interfaces::msg::String::ConstSharedPtr ros2_msg,
  std_msgs::String & ros1_msg)
{
  ros1_msg.data = ros2_msg->data;
}

}  // namespace ros1_bridge
