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

#include <iostream>
#include <memory>
#include <string>

// include ROS 1
#include "ros/message.h"
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "std_msgs/String.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


ros::Publisher ros1_pub;

void ros2ChatterCallback(const std_msgs::msg::String::SharedPtr ros2_msg)
{
  printf("  I heard from ROS 2: [%s]\n", ros2_msg->data.c_str());

  std_msgs::String ros1_msg;
  ros1_msg.data = ros2_msg->data;
  printf("  Passing along to ROS 1: [%s]\n", ros1_msg.data.c_str());
  ros1_pub.publish(ros1_msg);
}


rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros2_pub;

void ros1ChatterCallback(const ros::MessageEvent<std_msgs::String const> & ros1_msg_event)
{
  const boost::shared_ptr<ros::M_string> & connection_header =
    ros1_msg_event.getConnectionHeaderPtr();
  std::string key = "callerid";
  if (connection_header->find(key) != connection_header->end()) {
    if (connection_header->at(key) == "/ros_bridge") {
      printf("    I heard from ROS 1 from myself\n");
      return;
    }
    printf("I heard from ROS 1 from: [%s]\n", connection_header->at(key).c_str());
  }

  const boost::shared_ptr<std_msgs::String const> & ros1_msg = ros1_msg_event.getConstMessage();
  printf("I heard from ROS 1: [%s]\n", ros1_msg->data.c_str());

  std_msgs::msg::String ros2_msg;
  ros2_msg.data = ros1_msg->data;
  printf("Passing along to ROS 2: [%s]\n", ros2_msg.data.c_str());
  ros2_pub->publish(ros2_msg);
}


int main(int argc, char * argv[])
{
  // ROS 1 node and publisher
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;
  ros1_pub = ros1_node.advertise<std_msgs::String>("chatter", 10);

  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");
  ros2_pub = ros2_node->create_publisher<std_msgs::msg::String>(
    "chatter", 10);

  // ROS 1 subscriber
  ros::Subscriber ros1_sub = ros1_node.subscribe(
    "chatter", 10, ros1ChatterCallback);

  // ROS 2 subscriber
  rclcpp::SubscriptionOptions options;
  options.ignore_local_publications = true;
  auto ros2_sub = ros2_node->create_subscription<std_msgs::msg::String>(
    "chatter", rclcpp::SensorDataQoS(), ros2ChatterCallback, options);

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
