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

#ifndef __ros1_bridge__factory__hpp__
#define __ros1_bridge__factory__hpp__

// include ROS 1 message event
#include <ros/message.h>

#include <ros1_bridge/factory_interface.hpp>

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

  rclcpp::publisher::Publisher::SharedPtr
  create_ros2_publisher(
    rclcpp::node::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size)
  {
    return node->create_publisher<ROS2_T>(topic_name, queue_size);
  }

  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::publisher::Publisher::SharedPtr ros2_pub)
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
    auto callback = boost::bind(&Factory<ROS1_T, ROS2_T>::ros2_callback, _1, ros1_pub);
    return node->create_subscription<ROS2_T>(topic_name, queue_size, callback, nullptr, true);
  }

protected:

  static
  void ros1_callback(
    const ros::MessageEvent<ROS1_T const> & ros1_msg_event,
    rclcpp::publisher::Publisher::SharedPtr ros2_pub
    )
  {
    const boost::shared_ptr<ros::M_string> & connection_header = ros1_msg_event.getConnectionHeaderPtr();
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
    convert_1_to_2(ros1_msg, ros2_msg);
    printf("Passing message from ROS 1 to ROS 2\n");
    ros2_pub->publish(ros2_msg);
  }

  static
  void ros2_callback(
    typename ROS2_T::ConstSharedPtr ros2_msg,
    ros::Publisher ros1_pub
    )
  {
    ROS1_T ros1_msg;
    convert_2_to_1(ros2_msg, ros1_msg);
    printf("Passing message from ROS 2 to ROS 1\n");
    ros1_pub.publish(ros1_msg);
  }

  // defined outside of the class
  static
  void
  convert_1_to_2(
    typename ROS1_T::ConstPtr ros1_msg,
    typename ROS2_T::SharedPtr ros2_msg);
  static
  void
  convert_2_to_1(
    typename ROS2_T::ConstSharedPtr ros2_msg,
    ROS1_T & ros1_msg);
};

}  // namespace ros1_bridge

#endif // __ros1_bridge__factory__hpp__
