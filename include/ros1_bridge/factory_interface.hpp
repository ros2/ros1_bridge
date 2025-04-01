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
#include <vector>

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

struct ServiceBridge1to2
{
  ros::ServiceServer server;
  rclcpp::ClientBase::SharedPtr client;
};

struct ServiceBridge2to1
{
  rclcpp::ServiceBase::SharedPtr server;
  ros::ServiceClient client;
};

class FactoryInterface
{
public:
  virtual
  ros::Publisher
  create_ros1_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    bool latch = false) = 0;

  virtual
  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size) = 0;

  virtual
  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile) = 0;

  virtual
  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rclcpp::QoS & qos) = 0;

  virtual
  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::PublisherBase::SharedPtr ros2_pub,
    rclcpp::Logger logger) = 0;

  virtual
  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size,
    ros::Publisher ros1_pub,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr) = 0;

  virtual
  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile,
    ros::Publisher ros1_pub,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr) = 0;

  virtual
  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    ros::Publisher ros1_pub,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr) = 0;

  virtual
  void
  convert_1_to_2(const void * ros1_msg, void * ros2_msg) const = 0;

  virtual
  void
  convert_2_to_1(const void * ros2_msg, void * ros1_msg) const = 0;

  virtual
  bool convert_2_to_1_generic(
    const rclcpp::SerializedMessage & ros2_msg,
    std::vector<uint8_t> & ros1_msg) const = 0;

  virtual
  bool convert_1_to_2_generic(
    const std::vector<uint8_t> & ros1_msg,
    rclcpp::SerializedMessage & ros2_msg) const = 0;

  virtual const char * get_ros1_md5sum() const = 0;
  virtual const char * get_ros1_data_type() const = 0;
  virtual const char * get_ros1_message_definition() const = 0;
};

class ServiceFactoryInterface
{
public:
  virtual ServiceBridge1to2 service_bridge_1_to_2(
    ros::NodeHandle &, rclcpp::Node::SharedPtr, const std::string &, int) = 0;

  virtual ServiceBridge2to1 service_bridge_2_to_1(
    ros::NodeHandle &, rclcpp::Node::SharedPtr, const std::string &) = 0;
};

class ActionFactoryInterface
{
public:
  virtual void create_server_client(
    ros::NodeHandle ros1_node,
    rclcpp::Node::SharedPtr ros2_node,
    const std::string action_name) = 0;

  virtual void shutdown() = 0;
};

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__FACTORY_INTERFACE_HPP_
