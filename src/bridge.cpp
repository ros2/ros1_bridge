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

#include <string>

#include "ros1_bridge/bridge.hpp"


namespace ros1_bridge
{

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
  return create_bridge_from_1_to_2(
    ros1_node,
    ros2_node,
    ros1_type_name,
    ros1_topic_name,
    subscriber_queue_size,
    ros2_type_name,
    ros2_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(publisher_queue_size)));
}

Bridge1to2Handles
create_bridge_from_1_to_2(
  ros::NodeHandle ros1_node,
  rclcpp::Node::SharedPtr ros2_node,
  const std::string & ros1_type_name,
  const std::string & ros1_topic_name,
  size_t subscriber_queue_size,
  const std::string & ros2_type_name,
  const std::string & ros2_topic_name,
  const rclcpp::QoS & publisher_qos)
{
  auto factory = get_factory(ros1_type_name, ros2_type_name);
  auto ros2_pub = factory->create_ros2_publisher(
    ros2_node, ros2_topic_name, publisher_qos);

  auto ros1_sub = factory->create_ros1_subscriber(
    ros1_node, ros1_topic_name, subscriber_queue_size, ros2_pub, ros2_node->get_logger());

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
  rclcpp::PublisherBase::SharedPtr ros2_pub)
{
  auto subscriber_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(subscriber_queue_size));
  return create_bridge_from_2_to_1(
    ros2_node,
    ros1_node,
    ros2_type_name,
    ros2_topic_name,
    subscriber_qos,
    ros1_type_name,
    ros1_topic_name,
    publisher_queue_size,
    ros2_pub);
}

Bridge2to1Handles
create_bridge_from_2_to_1(
  rclcpp::Node::SharedPtr ros2_node,
  ros::NodeHandle ros1_node,
  const std::string & ros2_type_name,
  const std::string & ros2_topic_name,
  const rclcpp::QoS & subscriber_qos,
  const std::string & ros1_type_name,
  const std::string & ros1_topic_name,
  size_t publisher_queue_size,
  rclcpp::PublisherBase::SharedPtr ros2_pub)
{
  auto factory = get_factory(ros1_type_name, ros2_type_name);
  auto ros1_pub = factory->create_ros1_publisher(
    ros1_node, ros1_topic_name, publisher_queue_size);

  auto ros2_sub = factory->create_ros2_subscriber(
    ros2_node, ros2_topic_name, subscriber_qos, ros1_pub, ros2_pub);

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
  size_t queue_size)
{
  RCLCPP_INFO(
    ros2_node->get_logger(), "create bidirectional bridge for topic %s",
    topic_name.c_str());
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

BridgeHandles
create_bidirectional_bridge(
  ros::NodeHandle ros1_node,
  rclcpp::Node::SharedPtr ros2_node,
  const std::string & ros1_type_name,
  const std::string & ros2_type_name,
  const std::string & topic_name,
  size_t queue_size,
  const rclcpp::QoS & publisher_qos)
{
  RCLCPP_INFO(
    ros2_node->get_logger(), "create bidirectional bridge for topic %s",
    topic_name.c_str());
  BridgeHandles handles;
  handles.bridge1to2 = create_bridge_from_1_to_2(
    ros1_node, ros2_node,
    ros1_type_name, topic_name, queue_size, ros2_type_name, topic_name, publisher_qos);
  handles.bridge2to1 = create_bridge_from_2_to_1(
    ros2_node, ros1_node,
    ros2_type_name, topic_name, queue_size, ros1_type_name, topic_name, queue_size,
    handles.bridge1to2.ros2_publisher);
  return handles;
}

}  // namespace ros1_bridge
