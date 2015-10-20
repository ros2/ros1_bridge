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

#ifndef  ROS1_BRIDGE__FACTORY_INTERFACE_HPP_
#define  ROS1_BRIDGE__FACTORY_INTERFACE_HPP_

#include <string>

// include ROS 1
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"

// include ROS 2
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

namespace ros1_bridge
{

class FactoryInterface
{
public:
  virtual
  ros::Publisher
  create_ros1_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  rclcpp::publisher::Publisher::SharedPtr
  create_ros2_publisher(
    rclcpp::node::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::publisher::Publisher::SharedPtr ros2_pub) = 0;

  virtual
  rclcpp::subscription::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::node::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size,
    ros::Publisher ros1_pub) = 0;
};

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__FACTORY_INTERFACE_HPP_
