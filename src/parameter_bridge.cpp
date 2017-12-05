// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <list>
#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  std::list<ros1_bridge::BridgeHandles> all_handles;

  // bridge all topics listed in a ROS 1 parameter
  // the parameter needs to be an array
  // and each item needs to be a dictionary with the following keys;
  // topic: the name of the topic to bridge
  // type: the type of the topic to bridge
  // queue_size: the queue size to use (default: 100)
  const char * parameter_name = "topics";
  if (argc > 1) {
    parameter_name = argv[1];
  }

  XmlRpc::XmlRpcValue topics;
  if (
    ros1_node.getParam(parameter_name, topics) &&
    topics.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < static_cast<size_t>(topics.size()); ++i) {
      std::string topic_name = static_cast<std::string>(topics[i]["topic"]);
      std::string type_name = static_cast<std::string>(topics[i]["type"]);
      size_t queue_size = static_cast<int>(topics[i]["queue_size"]);
      if (!queue_size) {
        queue_size = 100;
      }
      printf(
        "Trying to create bidirectional bridge for topic ''%s' "
        "with ROS 1 type '%s' and ROS 2 type '%s'\n",
        topic_name.c_str(), type_name.c_str(), type_name.c_str());

      try {
        ros1_bridge::BridgeHandles handles = ros1_bridge::create_bidirectional_bridge(
          ros1_node, ros2_node, type_name, type_name, topic_name, queue_size);
        all_handles.push_back(handles);
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to create bidirectional bridge for topic '%s' "
          "with ROS 1 type '%s' and ROS 2 type '%s': %s\n",
          topic_name.c_str(), type_name.c_str(), type_name.c_str(), e.what());
      }
    }
  } else {
    fprintf(
      stderr, "The parameter '%s' either doesn't exist or isn't an array\n", parameter_name);
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
