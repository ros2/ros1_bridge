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

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#endif
#include "ros/ros.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "ros1_bridge/bridge.hpp"

int main(int argc, char * argv[])
{
  // ROS 2 node
  // must be before ROS1, because ros::init consumes args like __name and we cannot remap the node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  std::string dir = argv[1];
  std::string package = argv[2];
  std::string type = argv[3];
  std::string name = argv[4];

  std::cout << dir << " " << package << " " << type << " " << name << std::endl;


  auto factory = ros1_bridge::get_action_factory(dir, package, type);
  if (factory) {
    printf("created action factory\n");
    try {
      factory->create_server_client(ros1_node, ros2_node, name);
    } catch (std::runtime_error & e) {
      fprintf(stderr, "Failed to created a bridge: %s\n", e.what());
    }
  } else {
    fprintf(stderr, "Failed to create a factory\n");
  }

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
