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

#ifndef  ROS1_BRIDGE__FACTORY_HPP_
#define  ROS1_BRIDGE__FACTORY_HPP_

#include <functional>
#include <string>

// include ROS 1 message event
#include "ros/message.h"

#include "ros1_bridge/factory_interface.hpp"

namespace ros1_bridge
{

template<typename ROS1_T, typename ROS2_T>
class Factory : public FactoryInterface
{
public:
  ros::Publisher
  create_ros1_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size)
  {
    return node.advertise<ROS1_T>(topic_name, queue_size);
  }

  rclcpp::publisher::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::node::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size)
  {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = queue_size;
    return node->create_publisher<ROS2_T>(topic_name, custom_qos_profile);
  }

  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::publisher::PublisherBase::SharedPtr ros2_pub)
  {
    // workaround for https://github.com/ros/roscpp_core/issues/22 to get the connection header
    ros::SubscribeOptions ops;
    ops.topic = topic_name;
    ops.queue_size = queue_size;
    ops.md5sum = ros::message_traits::md5sum<ROS1_T>();
    ops.datatype = ros::message_traits::datatype<ROS1_T>();
    ops.helper = ros::SubscriptionCallbackHelperPtr(
      new ros::SubscriptionCallbackHelperT<const ros::MessageEvent<ROS1_T const>&>(
        boost::bind(&Factory<ROS1_T, ROS2_T>::ros1_callback, _1, ros2_pub)
      )
    );
    return node.subscribe(ops);
  }

  rclcpp::subscription::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::node::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size,
    ros::Publisher ros1_pub)
  {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = queue_size;
    // TODO(wjwwood): use a lambda until create_subscription supports std/boost::bind.
    auto callback = [this, ros1_pub](const typename ROS2_T::SharedPtr msg) {
      return this->ros2_callback(msg, ros1_pub);
    };
    return node->create_subscription<ROS2_T>(
      topic_name, callback, custom_qos_profile, nullptr, true);
  }

protected:
  static
  void ros1_callback(
    const ros::MessageEvent<ROS1_T const> & ros1_msg_event,
    rclcpp::publisher::PublisherBase::SharedPtr ros2_pub
    )
  {
    typename rclcpp::publisher::Publisher<ROS2_T>::SharedPtr typed_ros2_pub;
    typed_ros2_pub =
      std::dynamic_pointer_cast<typename rclcpp::publisher::Publisher<ROS2_T>>(ros2_pub);

    if (!typed_ros2_pub) {
      throw std::runtime_error("Invalid type for publisher");
    }

    const boost::shared_ptr<ros::M_string> & connection_header =
      ros1_msg_event.getConnectionHeaderPtr();
    if (!connection_header) {
      printf("  dropping message without connection header\n");
      return;
    }

    std::string key = "callerid";
    if (connection_header->find(key) != connection_header->end()) {
      if (connection_header->at(key) == "/ros_bridge") {
        return;
      }
    }

    const boost::shared_ptr<ROS1_T const> & ros1_msg = ros1_msg_event.getConstMessage();

    auto ros2_msg = std::make_shared<ROS2_T>();
    convert_1_to_2(*ros1_msg, *ros2_msg);
    printf("  Passing message from ROS 1 to ROS 2\n");
    typed_ros2_pub->publish(ros2_msg);
  }

  static
  void ros2_callback(
    typename ROS2_T::SharedPtr ros2_msg,
    ros::Publisher ros1_pub
    )
  {
    ROS1_T ros1_msg;
    convert_2_to_1(*ros2_msg, ros1_msg);
    printf("  Passing message from ROS 2 to ROS 1\n");
    ros1_pub.publish(ros1_msg);
  }

// since convert functions call each other for sub messages they must be public
public:
  // defined outside of the class
  static
  void
  convert_1_to_2(
    const ROS1_T & ros1_msg,
    ROS2_T & ros2_msg);
  static
  void
  convert_2_to_1(
    const ROS2_T & ros2_msg,
    ROS1_T & ros1_msg);
};

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__FACTORY_HPP_
