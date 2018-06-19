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

#ifndef ROS1_BRIDGE__BRIDGE_HPP_
#define ROS1_BRIDGE__BRIDGE_HPP_

#include <map>
#include <memory>
#include <string>

// include ROS 1
#include "ros/node_handle.h"

// include ROS 2
#include "rclcpp/node.hpp"

#include "ros1_bridge/factory_interface.hpp"

namespace ros1_bridge
{

struct Bridge1to2Handles
{
  ros::Subscriber ros1_subscriber;
  rclcpp::PublisherBase::SharedPtr ros2_publisher;
};

struct Bridge2to1Handles
{
  rclcpp::SubscriptionBase::SharedPtr ros2_subscriber;
  ros::Publisher ros1_publisher;
};

struct BridgeHandles
{
  Bridge1to2Handles bridge1to2;
  Bridge2to1Handles bridge2to1;
};

bool
get_1to2_mapping(
  const std::string & ros1_type_name,
  std::string & ros2_type_name);

bool
get_2to1_mapping(
  const std::string & ros2_type_name,
  std::string & ros1_type_name);

std::map<std::string, std::string>
get_all_message_mappings_2to1();

std::map<std::string, std::string>
get_all_service_mappings_2to1();

std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros1_type_name,
  const std::string & ros2_type_name);

Bridge1to2Handles
create_bridge_from_1_to_2(
  ros::NodeHandle ros1_node,
  rclcpp::Node::SharedPtr ros2_node,
  const std::string & ros1_type_name,
  const std::string & ros1_topic_name,
  size_t subscriber_queue_size,
  const std::string & ros2_type_name,
  const std::string & ros2_topic_name,
  size_t publisher_queue_size)
{
  auto factory = get_factory(ros1_type_name, ros2_type_name);
  auto ros2_pub = factory->create_ros2_publisher(
    ros2_node, ros2_topic_name, publisher_queue_size);

  auto ros1_sub = factory->create_ros1_subscriber(
    ros1_node, ros1_topic_name, subscriber_queue_size, ros2_pub);

  Bridge1to2Handles handles;
  handles.ros1_subscriber = ros1_sub;
  handles.ros2_publisher = ros2_pub;
  return handles;
}

Bridge2to1Handles
create_bridge_from_2_to_1(
  rclcpp::Node::SharedPtr ros2_node,
  ros::NodeHandle ros1_node,
  const std::string & ros2_type_name,
  const std::string & ros2_topic_name,
  size_t subscriber_queue_size,
  const std::string & ros1_type_name,
  const std::string & ros1_topic_name,
  size_t publisher_queue_size,
  rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr)
{
  auto factory = get_factory(ros1_type_name, ros2_type_name);
  auto ros1_pub = factory->create_ros1_publisher(
    ros1_node, ros1_topic_name, publisher_queue_size);

  auto ros2_sub = factory->create_ros2_subscriber(
    ros2_node, ros2_topic_name, subscriber_queue_size, ros1_pub, ros2_pub);

  Bridge2to1Handles handles;
  handles.ros2_subscriber = ros2_sub;
  handles.ros1_publisher = ros1_pub;
  return handles;
}

BridgeHandles
create_bidirectional_bridge(
  ros::NodeHandle ros1_node,
  rclcpp::Node::SharedPtr ros2_node,
  const std::string & ros1_type_name,
  const std::string & ros2_type_name,
  const std::string & topic_name,
  size_t queue_size = 10)
{
  printf("create bidirectional bridge for topic [%s]\n", topic_name.c_str());
  BridgeHandles handles;
  handles.bridge1to2 = create_bridge_from_1_to_2(
    ros1_node, ros2_node,
    ros1_type_name, topic_name, queue_size, ros2_type_name, topic_name, queue_size);
  handles.bridge2to1 = create_bridge_from_2_to_1(
    ros2_node, ros1_node,
    ros2_type_name, topic_name, queue_size, ros1_type_name, topic_name, queue_size,
    handles.bridge1to2.ros2_publisher);
  return handles;
}

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__BRIDGE_HPP_
