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

// include ROS 1
#include <ros/ros.h>
#include <ros/this_node.h>

// include ROS 2
#include <rclcpp/rclcpp.hpp>

#include <ros1_bridge/bridge.hpp>

// TODO hack until rclcpp has been refactored into a library
#include "generated_factories.cpp"


std::mutex g_bridge_mutex;


struct BridgeHandlesAndMessageTypes {
  ros1_bridge::BridgeHandles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};


void update_bridge(
  ros::NodeHandle & ros1_node,
  rclcpp::node::Node::SharedPtr ros2_node,
  const std::map<std::string, std::string> & ros1_topics,
  const std::map<std::string, std::string> & ros2_topics,
  std::map<std::string, BridgeHandlesAndMessageTypes> & bridges)
{
  std::lock_guard<std::mutex> lock(g_bridge_mutex);

  for (auto ros1_topic : ros1_topics) {
    // identify topics available in ROS 1 as well as ROS 2
    auto topic_name = ros1_topic.first;
    auto ros2_topic = ros2_topics.find(topic_name);
    if (ros2_topic == ros2_topics.end()) {
      continue;
    }

    std::string ros1_type_name = ros1_topic.second;
    std::string ros2_type_name = ros2_topic->second;

    // check if bridge for the same topic exists
    if (bridges.find(topic_name) != bridges.end()) {
      auto bridge = bridges.find(topic_name)->second;
      if (bridge.ros1_type_name == ros1_type_name && bridge.ros2_type_name == ros2_type_name) {
        // skip if bridge with correct types is already in place
        continue;
      }
      // remove existing bridge with previous types
      bridges.erase(topic_name);
      printf("removed bridge for topic '%s'\n", topic_name.c_str());
    }

    BridgeHandlesAndMessageTypes bridge;
    bridge.ros1_type_name = ros1_type_name;
    bridge.ros2_type_name = ros2_type_name;

    try {
      bridge.bridge_handles = ros1_bridge::create_bidirectional_bridge(
        ros1_node, ros2_node, bridge.ros1_type_name, bridge.ros2_type_name, topic_name, 10);
    } catch (std::runtime_error & e) {
      fprintf(
        stderr,
        "failed to create bridge for topic '%s' with ROS 1 type '%s' and ROS 2 type '%s': %s\n",
        topic_name.c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str(), e.what());
      continue;
    }

    bridges[topic_name] = bridge;
    printf(
      "created bridge for topic '%s' with ROS 1 type '%s' and ROS 2 type '%s'\n",
      topic_name.c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str());
  }

  // remove obsolete bridges
  std::vector<std::string> to_be_removed;
  for (auto it : bridges) {
    std::string topic_name = it.first;
    if (
      ros1_topics.find(topic_name) == ros1_topics.end() ||
      ros2_topics.find(topic_name) == ros2_topics.end()
    )
    {
      to_be_removed.push_back(topic_name);
    }
  }
  for (auto topic_name : to_be_removed) {
    bridges.erase(topic_name);
    printf("removed bridge for topic '%s'\n", topic_name.c_str());
  }
}

int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::node::Node::make_shared("ros_bridge");

  // mapping of available topic names to type names
  std::map<std::string, std::string> ros1_topics;
  std::map<std::string, std::string> ros2_topics;

  // TODO hard coded test topic until ROS 2 can query the current topics
  ros2_topics["chatter"] = "std_interfaces/String";

  std::map<std::string, BridgeHandlesAndMessageTypes> bridges;

  // setup polling of ROS 1 master
  auto ros1_poll = [&ros1_node, ros2_node, &ros1_topics, &ros2_topics, &bridges](
    const ros::TimerEvent & timer_event
  ) -> void
  {
    // collect all topics names which have at least one publisher or subscriber beside this bridge
    std::set<std::string> active_topic_names;

    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
      fprintf(stderr, "failed to get system state from ROS 1 master\n");
      return;
    }
    // check publishers
    if (payload.size() >= 1) {
      for (int j = 0; j < payload[0].size(); ++j) {
        std::string topic_name = payload[0][j][0];
        for (int k = 0; k < payload[0][j][1].size(); ++k){
          std::string node_name = payload[0][j][1][k];
          if (node_name == ros::this_node::getName()) {
            continue;
          }
          active_topic_names.insert(topic_name);
          break;
        }
      }
    }
    // check subscribers
    if (payload.size() >= 2) {
      for (int j = 0; j < payload[1].size(); ++j) {
        std::string topic_name = payload[1][j][0];
        for (int k = 0; k < payload[1][j][1].size(); ++k){
          std::string node_name = payload[1][j][1][k];
          if (node_name == ros::this_node::getName()) {
            continue;
          }
          active_topic_names.insert(topic_name);
          break;
        }
      }
    }

    // get message types for all topics
    ros::master::V_TopicInfo topics;
    bool success = ros::master::getTopics(topics);
    if (!success) {
      fprintf(stderr, "failed to poll ROS 1 master\n");
      return;
    }

    ros1_topics.clear();
    for (auto topic : topics) {
      auto topic_name = topic.name;
      if (active_topic_names.find(topic_name) == active_topic_names.end()) {
        // skip inactive topics
        continue;
      }
      if (topic_name.compare(0, 1, "/") == 0) {
        topic_name = topic_name.substr(1);
      }
      ros1_topics[topic_name] = topic.datatype;
    }

    update_bridge(ros1_node, ros2_node, ros1_topics, ros2_topics, bridges);
  };

  auto ros1_poll_timer = ros1_node.createTimer(ros::Duration(1.0), ros1_poll);

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::utilities::ok()) {
    executor.spin_node_once(ros2_node, true);
  }

  return 0;
}
