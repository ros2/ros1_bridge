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

#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <boost/algorithm/string/predicate.hpp>   // NOLINT

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif
#include "ros/this_node.h"
#include "ros/header.h"
#include "ros/service_manager.h"
#include "ros/transport/transport_tcp.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/scope_exit.hpp"

#include "ros1_bridge/bridge.hpp"
#include "ros1_bridge/command_parser_utils.hpp"
#include "ros1_bridge/action_factory.hpp"


std::mutex g_bridge_mutex;

struct Bridge1to2HandlesAndMessageTypes
{
  ros1_bridge::Bridge1to2Handles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};

struct Bridge2to1HandlesAndMessageTypes
{
  ros1_bridge::Bridge2to1Handles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};

bool parse_command_options(
  int argc, char ** argv, std::vector<const char *> & ros1_args,
  std::vector<const char *> & ros2_args, bool & output_topic_introspection,
  bool & bridge_all_1to2_topics, bool & bridge_all_2to1_topics)
{
  std::vector<const char *> args(argv, argv + argc);

  std::vector<const char *> available_options = {
    "-h", "--help",
    "--show-introspection",
    "--print-pairs",
    "--bridge-all-topics",
    "--bridge-all-1to2-topics",
    "--bridge-all-2to1-topics",
    "--ros1-args",
    "--ros2-args",
  };

  if (ros1_bridge::find_command_option(args, "-h") ||
    ros1_bridge::find_command_option(args, "--help"))
  {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << "ros2 run ros1_bridge dynamic_bridge [Bridge specific options] \\" << std::endl;
    ss << "    [--ros1-args [ROS1 arguments]] [--ros2-args [ROS2 arguments]]" << std::endl;
    ss << std::endl;
    ss << "Options:" << std::endl;
    ss << " -h, --help: This message." << std::endl;
    ss << " --show-introspection: Print output of introspection of both sides of the bridge.";
    ss << std::endl;
    ss << " --print-pairs: Print a list of the supported ROS 2 <=> ROS 1 conversion pairs.";
    ss << std::endl;
    ss << " --bridge-all-topics: Bridge all topics in both directions, whether or not there is ";
    ss << "a matching subscriber." << std::endl;
    ss << " --bridge-all-1to2-topics: Bridge all ROS 1 topics to ROS 2, whether or not there is ";
    ss << "a matching subscriber." << std::endl;
    ss << " --bridge-all-2to1-topics: Bridge all ROS 2 topics to ROS 1, whether or not there is ";
    ss << "a matching subscriber." << std::endl;
    ss << " --ros1-args: Arguments to pass to the ROS 1 bridge node." << std::endl;
    ss << " --ros2-args: Arguments to pass to the ROS 2 bridge node." << std::endl;
    std::cout << ss.str();
    return false;
  }

  if (ros1_bridge::get_option_flag(args, "--print-pairs")) {
    auto mappings_2to1 = ros1_bridge::get_all_message_mappings_2to1();
    if (mappings_2to1.size() > 0) {
      printf("Supported ROS 2 <=> ROS 1 message type conversion pairs:\n");
      for (auto & pair : mappings_2to1) {
        printf("  - '%s' (ROS 2) <=> '%s' (ROS 1)\n", pair.first.c_str(), pair.second.c_str());
      }
    } else {
      printf("No message type conversion pairs supported.\n");
    }
    mappings_2to1 = ros1_bridge::get_all_service_mappings_2to1();
    if (mappings_2to1.size() > 0) {
      printf("Supported ROS 2 <=> ROS 1 service type conversion pairs:\n");
      for (auto & pair : mappings_2to1) {
        printf("  - '%s' (ROS 2) <=> '%s' (ROS 1)\n", pair.first.c_str(), pair.second.c_str());
      }
    } else {
      printf("No service type conversion pairs supported.\n");
    }
    mappings_2to1 = ros1_bridge::get_all_action_mappings_2to1();
    if (mappings_2to1.size() > 0) {
      printf("Supported ROS 2 <=> ROS 1 action type conversion pairs:\n");
      for (auto & pair : mappings_2to1) {
        printf("  - '%s' (ROS 2) <=> '%s' (ROS 1)\n", pair.first.c_str(), pair.second.c_str());
      }
    } else {
      printf("No action type conversion pairs supported.\n");
    }
    return false;
  }

  output_topic_introspection = ros1_bridge::get_option_flag(args, "--show-introspection", true);

  bool bridge_all_topics = ros1_bridge::get_option_flag(args, "--bridge-all-topics", true);
  bridge_all_1to2_topics = bridge_all_topics ||
    ros1_bridge::get_option_flag(args, "--bridge-all-1to2-topics", true);
  bridge_all_2to1_topics = bridge_all_topics ||
    ros1_bridge::get_option_flag(args, "--bridge-all-2to1-topics", true);

  auto logger = rclcpp::get_logger("ros1_bridge");

  // Get ROS1 arguments
  if (ros1_bridge::get_option_values(args, "--ros1-args", available_options, ros1_args, true)) {
    if (ros1_args.size() == 0) {
      RCLCPP_ERROR(logger, "Error: --ros1-args specified but no arguments provided.");
      return false;
    }
  }

  ros1_args.insert(ros1_args.begin(), args.at(0));

  // Get ROS2 arguments
  if (ros1_bridge::get_option_values(args, "--ros2-args", available_options, ros2_args, true)) {
    if (ros2_args.size() == 0) {
      RCLCPP_ERROR(logger, "Error: --ros2-args specified but no arguments provided.");
      return false;
    }

    ros2_args.insert(ros2_args.begin(), "--ros-args");
  }

  ros2_args.insert(ros2_args.begin(), args.at(0));

  if (ros1_bridge::find_command_option(args, "--ros-args") || args.size() > 1) {
    RCLCPP_WARN(
      logger, "Warning: passing the ROS node arguments directly to the node is "
      "deprecated, use --ros1-args and --ros2-args instead.");

    ros1_bridge::split_ros1_ros2_args(args, ros1_args, ros2_args);
  }

  return true;
}

void update_bridge(
  ros::NodeHandle & ros1_node,
  rclcpp::Node::SharedPtr ros2_node,
  const std::map<std::string, std::string> & ros1_publishers,
  const std::map<std::string, std::string> & ros1_subscribers,
  const std::map<std::string, std::string> & ros2_publishers,
  const std::map<std::string, std::string> & ros2_subscribers,
  const std::map<std::string, std::map<std::string, std::string>> & ros1_services,
  const std::map<std::string, std::map<std::string, std::string>> & ros2_services,
  const std::map<std::string, std::map<std::string, std::string>> & ros1_action_servers,
  const std::map<std::string, std::map<std::string, std::string>> & ros2_action_servers,
  std::map<std::string, Bridge1to2HandlesAndMessageTypes> & bridges_1to2,
  std::map<std::string, Bridge2to1HandlesAndMessageTypes> & bridges_2to1,
  std::map<std::string, ros1_bridge::ServiceBridge1to2> & service_bridges_1_to_2,
  std::map<std::string, ros1_bridge::ServiceBridge2to1> & service_bridges_2_to_1,
  std::map<std::string,
  std::unique_ptr<ros1_bridge::ActionFactoryInterface>> & action_bridges_1_to_2,
  std::map<std::string,
  std::unique_ptr<ros1_bridge::ActionFactoryInterface>> & action_bridges_2_to_1,
  bool bridge_all_1to2_topics, bool bridge_all_2to1_topics)
{
  std::lock_guard<std::mutex> lock(g_bridge_mutex);

  // create 1to2 bridges
  for (auto ros1_publisher : ros1_publishers) {
    // identify topics available as ROS 1 publishers as well as ROS 2 subscribers
    auto topic_name = ros1_publisher.first;
    std::string ros1_type_name = ros1_publisher.second;
    std::string ros2_type_name;

    auto ros2_subscriber = ros2_subscribers.find(topic_name);
    if (ros2_subscriber == ros2_subscribers.end()) {
      if (!bridge_all_1to2_topics) {
        continue;
      }
      // update the ROS 2 type name to be that of the anticipated bridged type
      // TODO(dhood): support non 1-1 "bridge-all" mappings
      bool mapping_found = ros1_bridge::get_1to2_mapping(ros1_type_name, ros2_type_name);
      if (!mapping_found) {
        // printf("No known mapping for ROS 1 type '%s'\n", ros1_type_name.c_str());
        continue;
      }
      // printf("topic name '%s' has ROS 2 publishers\n", topic_name.c_str());
    } else {
      ros2_type_name = ros2_subscriber->second;
      // printf("topic name '%s' has ROS 1 publishers and ROS 2 subscribers\n", topic_name.c_str());
    }

    // check if 1to2 bridge for the topic exists
    if (bridges_1to2.find(topic_name) != bridges_1to2.end()) {
      auto bridge = bridges_1to2.find(topic_name)->second;
      if (bridge.ros1_type_name == ros1_type_name && bridge.ros2_type_name == ros2_type_name) {
        // skip if bridge with correct types is already in place
        continue;
      }
      // remove existing bridge with previous types
      bridges_1to2.erase(topic_name);
      printf("replace 1to2 bridge for topic '%s'\n", topic_name.c_str());
    }

    Bridge1to2HandlesAndMessageTypes bridge;
    bridge.ros1_type_name = ros1_type_name;
    bridge.ros2_type_name = ros2_type_name;

    auto ros2_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    if (topic_name == "/tf_static") {
      ros2_publisher_qos.keep_all();
      ros2_publisher_qos.transient_local();
    }
    try {
      bridge.bridge_handles = ros1_bridge::create_bridge_from_1_to_2(
        ros1_node, ros2_node,
        bridge.ros1_type_name, topic_name, 10,
        bridge.ros2_type_name, topic_name, ros2_publisher_qos);
    } catch (std::runtime_error & e) {
      // fprintf(
      //   stderr,
      //   "failed to create 1to2 bridge for topic '%s' "
      //   "with ROS 1 type '%s' and ROS 2 type '%s': %s\n",
      //   topic_name.c_str(), bridge.ros1_type_name.c_str(),
      //   bridge.ros2_type_name.c_str(), e.what());
      // if (std::string(e.what()).find("No template specialization") != std::string::npos) {
      //   fprintf(stderr, "check the list of supported pairs with the `--print-pairs` option\n");
      // }
      continue;
    }

    bridges_1to2[topic_name] = bridge;
    printf(
      "created 1to2 bridge for topic '%s' with ROS 1 type '%s' and ROS 2 type '%s'\n",
      topic_name.c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str());
  }

  // create 2to1 bridges
  for (auto ros2_publisher : ros2_publishers) {
    // identify topics available as ROS 1 subscribers as well as ROS 2 publishers
    auto topic_name = ros2_publisher.first;
    std::string ros2_type_name = ros2_publisher.second;
    std::string ros1_type_name;

    auto ros1_subscriber = ros1_subscribers.find(topic_name);
    if (ros1_subscriber == ros1_subscribers.end()) {
      if (!bridge_all_2to1_topics) {
        continue;
      }
      // update the ROS 1 type name to be that of the anticipated bridged type
      // TODO(dhood): support non 1-1 "bridge-all" mappings
      bool mapping_found = ros1_bridge::get_2to1_mapping(ros2_type_name, ros1_type_name);
      if (!mapping_found) {
        // printf("No known mapping for ROS 2 type '%s'\n", ros2_type_name.c_str());
        continue;
      }
      // printf("topic name '%s' has ROS 2 publishers\n", topic_name.c_str());
    } else {
      ros1_type_name = ros1_subscriber->second;
      // printf("topic name '%s' has ROS 1 subscribers and ROS 2 publishers\n", topic_name.c_str());
    }

    // check if 2to1 bridge for the topic exists
    if (bridges_2to1.find(topic_name) != bridges_2to1.end()) {
      auto bridge = bridges_2to1.find(topic_name)->second;
      if ((bridge.ros1_type_name == ros1_type_name || bridge.ros1_type_name == "") &&
        bridge.ros2_type_name == ros2_type_name)
      {
        // skip if bridge with correct types is already in place
        continue;
      }
      // remove existing bridge with previous types
      bridges_2to1.erase(topic_name);
      printf("replace 2to1 bridge for topic '%s'\n", topic_name.c_str());
    }

    Bridge2to1HandlesAndMessageTypes bridge;
    bridge.ros1_type_name = ros1_type_name;
    bridge.ros2_type_name = ros2_type_name;

    try {
      bridge.bridge_handles = ros1_bridge::create_bridge_from_2_to_1(
        ros2_node, ros1_node,
        bridge.ros2_type_name, topic_name, 10,
        bridge.ros1_type_name, topic_name, 10);
    } catch (std::runtime_error & e) {
      // fprintf(
      //   stderr,
      //   "failed to create 2to1 bridge for topic '%s' "
      //   "with ROS 2 type '%s' and ROS 1 type '%s': %s\n",
      //   topic_name.c_str(), bridge.ros2_type_name.c_str(),
      //   bridge.ros1_type_name.c_str(), e.what());
      // if (std::string(e.what()).find("No template specialization") != std::string::npos) {
      //   fprintf(stderr, "check the list of supported pairs with the `--print-pairs` option\n");
      // }
      continue;
    }

    bridges_2to1[topic_name] = bridge;
    printf(
      "created 2to1 bridge for topic '%s' with ROS 2 type '%s' and ROS 1 type '%s'\n",
      topic_name.c_str(), bridge.ros2_type_name.c_str(), bridge.ros1_type_name.c_str());
  }

  // remove obsolete bridges
  std::vector<std::string> to_be_removed_1to2;
  for (auto it : bridges_1to2) {
    std::string topic_name = it.first;
    if (
      ros1_publishers.find(topic_name) == ros1_publishers.end() ||
      (!bridge_all_1to2_topics && ros2_subscribers.find(topic_name) == ros2_subscribers.end()))
    {
      to_be_removed_1to2.push_back(topic_name);
    }
  }
  for (auto topic_name : to_be_removed_1to2) {
    bridges_1to2.erase(topic_name);
    printf("removed 1to2 bridge for topic '%s'\n", topic_name.c_str());
  }

  std::vector<std::string> to_be_removed_2to1;
  for (auto it : bridges_2to1) {
    std::string topic_name = it.first;
    if (
      (!bridge_all_2to1_topics && ros1_subscribers.find(topic_name) == ros1_subscribers.end()) ||
      ros2_publishers.find(topic_name) == ros2_publishers.end())
    {
      to_be_removed_2to1.push_back(topic_name);
    }
  }
  for (auto topic_name : to_be_removed_2to1) {
    bridges_2to1.erase(topic_name);
    printf("removed 2to1 bridge for topic '%s'\n", topic_name.c_str());
  }

  // create bridges for ros1 services
  for (auto & service : ros1_services) {
    auto & name = service.first;
    auto & details = service.second;
    if (
      service_bridges_2_to_1.find(name) == service_bridges_2_to_1.end() &&
      service_bridges_1_to_2.find(name) == service_bridges_1_to_2.end())
    {
      auto factory = ros1_bridge::get_service_factory(
        "ros1", details.at("package"), details.at("name"));
      if (factory) {
        try {
          service_bridges_2_to_1[name] = factory->service_bridge_2_to_1(ros1_node, ros2_node, name);
          printf("Created 2 to 1 bridge for service %s\n", name.data());
        } catch (std::runtime_error & e) {
          fprintf(stderr, "Failed to created a bridge: %s\n", e.what());
        }
      }
    }
  }

  int service_execution_timeout{5};
  ros1_node.getParamCached(
    "ros1_bridge/dynamic_bridge/service_execution_timeout", service_execution_timeout);

  // create bridges for ros2 services
  for (auto & service : ros2_services) {
    auto & name = service.first;
    auto & details = service.second;
    if (
      service_bridges_1_to_2.find(name) == service_bridges_1_to_2.end() &&
      service_bridges_2_to_1.find(name) == service_bridges_2_to_1.end())
    {
      auto factory = ros1_bridge::get_service_factory(
        "ros2", details.at("package"), details.at("name"));
      if (factory) {
        try {
          service_bridges_1_to_2[name] = factory->service_bridge_1_to_2(
            ros1_node, ros2_node, name, service_execution_timeout);
          printf("Created 1 to 2 bridge for service %s\n", name.data());
        } catch (std::runtime_error & e) {
          fprintf(stderr, "Failed to created a bridge: %s\n", e.what());
        }
      }
    }
  }

  // remove obsolete ros1 services
  for (auto it = service_bridges_2_to_1.begin(); it != service_bridges_2_to_1.end(); ) {
    if (ros1_services.find(it->first) == ros1_services.end()) {
      printf("Removed 2 to 1 bridge for service %s\n", it->first.data());
      try {
        it = service_bridges_2_to_1.erase(it);
      } catch (std::runtime_error & e) {
        fprintf(stderr, "There was an error while removing 2 to 1 bridge: %s\n", e.what());
      }
    } else {
      ++it;
    }
  }

  // remove obsolete ros2 services
  for (auto it = service_bridges_1_to_2.begin(); it != service_bridges_1_to_2.end(); ) {
    if (ros2_services.find(it->first) == ros2_services.end()) {
      printf("Removed 1 to 2 bridge for service %s\n", it->first.data());
      try {
        it->second.server.shutdown();
        it = service_bridges_1_to_2.erase(it);
      } catch (std::runtime_error & e) {
        fprintf(stderr, "There was an error while removing 1 to 2 bridge: %s\n", e.what());
      }
    } else {
      ++it;
    }
  }

  // create bridges for ros1 actions
  for (auto & ros1_action : ros1_action_servers) {
    auto & name = ros1_action.first;
    auto & details = ros1_action.second;
    if (
      action_bridges_1_to_2.find(name) == action_bridges_1_to_2.end() &&
      action_bridges_2_to_1.find(name) == action_bridges_2_to_1.end())
    {
      auto factory = ros1_bridge::get_action_factory(
        "ros1", details.at("package"), details.at("type"));
      if (factory) {
        try {
          factory->create_server_client(ros1_node, ros2_node, name);
          action_bridges_2_to_1[name] = std::move(factory);
          printf("Created 2 to 1 bridge for action %s\n", name.data());
        } catch (std::runtime_error & e) {
          fprintf(stderr, "Failed to created a bridge: %s\n", e.what());
        }
      }
    }
  }

  // create bridges for ros2 actions
  for (auto & ros2_action : ros2_action_servers) {
    auto & name = ros2_action.first;
    auto & details = ros2_action.second;
    if (
      action_bridges_1_to_2.find(name) == action_bridges_1_to_2.end() &&
      action_bridges_2_to_1.find(name) == action_bridges_2_to_1.end())
    {
      auto factory = ros1_bridge::get_action_factory(
        "ros2", details.at("package"), details.at("type"));
      if (factory) {
        try {
          factory->create_server_client(ros1_node, ros2_node, name);
          action_bridges_1_to_2[name] = std::move(factory);
          printf("Created 1 to 2 bridge for action %s\n", name.data());
        } catch (std::runtime_error & e) {
          fprintf(stderr, "Failed to created a bridge: %s\n", e.what());
        }
      }
    }
  }

  // remove obsolete ros1 actions
  for (auto it = action_bridges_2_to_1.begin(); it != action_bridges_2_to_1.end(); ) {
    if (ros1_action_servers.find(it->first) == ros1_action_servers.end()) {
      printf("Removed 2 to 1 bridge for action %s\n", it->first.data());
      try {
        it->second->shutdown();
        it->second.reset();
        it = action_bridges_2_to_1.erase(it);
      } catch (std::runtime_error & e) {
        fprintf(stderr, "There was an error while removing 2 to 1 bridge: %s\n", e.what());
      }
    } else {
      ++it;
    }
  }

  // remove obsolete ros2 actions
  for (auto it = action_bridges_1_to_2.begin(); it != action_bridges_1_to_2.end(); ) {
    if (ros2_action_servers.find(it->first) == ros2_action_servers.end()) {
      printf("Removed 1 to 2 bridge for action %s\n", it->first.data());
      try {
        // it->second.server.shutdown();
        it->second->shutdown();
        it->second.reset();
        it = action_bridges_1_to_2.erase(it);
      } catch (std::runtime_error & e) {
        fprintf(stderr, "There was an error while removing 1 to 2 bridge: %s\n", e.what());
      }
    } else {
      ++it;
    }
  }
}

void get_ros1_service_info(
  const std::string name, std::map<std::string, std::map<std::string, std::string>> & ros1_services)
{
  // NOTE(rkozik):
  // I tried to use Connection class but could not make it work
  // auto callback = [](const ros::ConnectionPtr&, const ros::Header&)
  //                 { printf("Callback\n"); return true; };
  // ros::HeaderReceivedFunc f(callback);
  // ros::ConnectionPtr connection(new ros::Connection);
  // connection->initialize(transport, false, ros::HeaderReceivedFunc());
  ros::ServiceManager manager;
  std::string host;
  std::uint32_t port;
  if (!manager.lookupService(name, host, port)) {
    fprintf(stderr, "Failed to look up %s\n", name.data());
    return;
  }
  ros::TransportTCPPtr transport(new ros::TransportTCP(nullptr, ros::TransportTCP::SYNCHRONOUS));
  auto transport_exit = rcpputils::make_scope_exit(
    [transport]() {
      transport->close();
    });
  if (!transport->connect(host, port)) {
    fprintf(stderr, "Failed to connect to %s (%s:%d)\n", name.data(), host.data(), port);
    return;
  }
  ros::M_string header_out;
  header_out["probe"] = "1";
  header_out["md5sum"] = "*";
  header_out["service"] = name;
  header_out["callerid"] = ros::this_node::getName();
  boost::shared_array<uint8_t> buffer;
  uint32_t len;
  ros::Header::write(header_out, buffer, len);
  std::vector<uint8_t> message(len + 4);
  std::memcpy(&message[0], &len, 4);
  std::memcpy(&message[4], buffer.get(), len);
  transport->write(message.data(), message.size());
  uint32_t length;
  auto read = transport->read(reinterpret_cast<uint8_t *>(&length), 4);
  if (read != 4) {
    fprintf(stderr, "Failed to read a response from a service server\n");
    return;
  }
  std::vector<uint8_t> response(length);
  read = transport->read(response.data(), length);
  if (read < 0 || static_cast<uint32_t>(read) != length) {
    fprintf(stderr, "Failed to read a response from a service server\n");
    return;
  }
  std::string key = name;
  ros1_services[key] = std::map<std::string, std::string>();
  ros::Header header_in;
  std::string error;
  auto success = header_in.parse(response.data(), length, error);
  if (!success) {
    fprintf(stderr, "%s\n", error.data());
    return;
  }
  for (std::string field : {"type"}) {
    std::string value;
    auto success = header_in.getValue(field, value);
    if (!success) {
      fprintf(stderr, "Failed to read '%s' from a header for '%s'\n", field.data(), key.c_str());
      ros1_services.erase(key);
      return;
    }
    ros1_services[key][field] = value;
  }
  std::string t = ros1_services[key]["type"];
  ros1_services[key]["package"] = std::string(t.begin(), t.begin() + t.find("/"));
  ros1_services[key]["name"] = std::string(t.begin() + t.find("/") + 1, t.end());
}

inline bool is_action_topic(
  std::map<std::string, std::map<std::string, std::string>> & actions,
  std::map<std::string, uint8_t> & action_nums, const bool is_action_type,
  const std::string topic_name, const std::string topic_name_ends_with,
  const std::string type, const std::string type_ends_with, bool is_ros2 = false)
{
  // check if the topic name and topic types are as expected
  if (boost::algorithm::ends_with(topic_name.c_str(), topic_name_ends_with.c_str()) &&
    boost::algorithm::ends_with(type.c_str(), type_ends_with.c_str()))
  {
    // extract action name from topic name
    std::string name = topic_name.substr(0, topic_name.find(topic_name_ends_with.c_str()));
    if (actions.find(name) == actions.end()) {
      actions[name]["package"] = "";
      actions[name]["type"] = "";
      action_nums[name] = 0;
    }

    // e.g.: topic type of '/fibonacci/goal' is 'actionlib_tutorials/FibonacciActionGoal'
    // Thus, package name is action type is 'actionlib_tutorials' and
    // action type is 'Fibonacci'
    if (!type.empty() && is_action_type) {
      std::string pkg_name = type.substr(0, type.find("/"));
      std::string action_type =
        type.substr(
        type.find_last_of("/") + 1,
        type.length() - (type.find_last_of("/") + type_ends_with.length() + 1));
      actions[name]["package"] = pkg_name;
      if (is_ros2) {
        actions[name]["type"] = "action/" + action_type;
      } else {
        actions[name]["type"] = action_type;
      }
    }

    action_nums[name] += 1;

    return true;
  }
  return false;
}

// if topics 'goal' with type 'ActionGoal' and 'cancel' with type 'GoalID' are pubs, then it is an
// action client
// equivalent ROS2 action pkg and type can be retrieved from get_mappings.cpp
void get_active_ros1_actions(
  std::map<std::string, std::string> publishers,
  std::map<std::string, std::string> subscribers,
  std::map<std::string, std::map<std::string, std::string>> & active_ros1_action_servers,
  std::map<std::string, std::map<std::string, std::string>> & active_ros1_action_clients)
{
  // check if the topics end with 'goal', 'result', 'cancel', 'status'

  // find topics that end with goal and cancel, find corresponding result, status and feedback
  // in the other map
  std::map<std::string, std::string>::iterator it;
  std::map<std::string, uint8_t>::iterator it_num;
  // store count of pubs and subs for each action
  std::map<std::string, uint8_t> action_server_nums, action_client_nums;

  for (it = publishers.begin(); it != publishers.end(); it++) {
    // check for action client
    if (
      is_action_topic(
        active_ros1_action_clients, action_client_nums, false,
        it->first.c_str(), "/cancel", it->second.c_str(), "/GoalID"))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros1_action_clients, action_client_nums, true,
        it->first.c_str(), "/goal", it->second.c_str(), "ActionGoal"))
    {
      continue;
    } else if (   // NOLINT // check for action server
      is_action_topic(
        active_ros1_action_servers, action_server_nums, true,
        it->first.c_str(), "/feedback", it->second.c_str(),
        "ActionFeedback"))
    {
      continue;
    }
    if (
      is_action_topic(
        active_ros1_action_servers, action_server_nums, false,
        it->first.c_str(), "/result", it->second.c_str(), "ActionResult"))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros1_action_servers, action_server_nums, false,
        it->first.c_str(), "/status", it->second.c_str(),
        "GoalStatusArray"))
    {
      continue;
    }
  }

  // subscribers do not report their types, but use it to confirm action
  for (it = subscribers.begin(); it != subscribers.end(); it++) {
    // check for action server
    if (
      is_action_topic(
        active_ros1_action_servers, action_server_nums, false,
        it->first.c_str(), "/cancel", it->second.c_str(), ""))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros1_action_servers, action_server_nums, false,
        it->first.c_str(), "/goal", it->second.c_str(), ""))
    {
      continue;
    } else if (   // NOLINT   // check for action client
      is_action_topic(
        active_ros1_action_clients, action_client_nums, false,
        it->first.c_str(), "/feedback", it->second.c_str(), ""))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros1_action_clients, action_client_nums, false,
        it->first.c_str(), "/result", it->second.c_str(), ""))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros1_action_clients, action_client_nums, false,
        it->first.c_str(), "/status", it->second.c_str(), ""))
    {
      continue;
    }
  }

  for (it_num = action_client_nums.begin(); it_num != action_client_nums.end(); it_num++) {
    if (it_num->second != 5) {
      active_ros1_action_clients.erase(it_num->first);
    }
  }
  for (it_num = action_server_nums.begin(); it_num != action_server_nums.end(); it_num++) {
    if (it_num->second != 5) {
      active_ros1_action_servers.erase(it_num->first);
    }
  }
}

// how does ros2 action list determine active interfaces?
// ref: opt/ros/foxy/lib/python3.8/site-packages/ros2action/verb/list.py
// https://github.com/ros2/rcl/blob/master/rcl_action/src/rcl_action/graph.c
void get_active_ros2_actions(
  const std::map<std::string, std::string> active_ros2_publishers,
  const std::map<std::string, std::string> active_ros2_subscribers,
  std::map<std::string, std::map<std::string, std::string>> & active_ros2_action_servers,
  std::map<std::string, std::map<std::string, std::string>> & active_ros2_action_clients)
{
  std::map<std::string, std::string>::const_iterator it;
  std::map<std::string, uint8_t>::iterator it_num;
  std::map<std::string, uint8_t> action_server_nums, action_client_nums;
  for (it = active_ros2_publishers.begin(); it != active_ros2_publishers.end(); it++) {
    if (
      is_action_topic(
        active_ros2_action_servers, action_server_nums, true, it->first.c_str(),
        "/_action/feedback", it->second.c_str(), "_FeedbackMessage", true))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros2_action_servers, action_server_nums, false,
        it->first.c_str(), "/_action/status", it->second.c_str(),
        "GoalStatusArray"), true)
    {
      continue;
    }
  }
  for (it = active_ros2_subscribers.begin(); it != active_ros2_subscribers.end(); it++) {
    if (
      is_action_topic(
        active_ros2_action_clients, action_client_nums, true,
        it->first.c_str(), "/_action/feedback", it->second.c_str(),
        "_FeedbackMessage", true))
    {
      continue;
    } else if (   // NOLINT
      is_action_topic(
        active_ros2_action_clients, action_client_nums, false,
        it->first.c_str(), "/_action/status", it->second.c_str(),
        "GoalStatusArray", true))
    {
      continue;
    }
  }
  for (it_num = action_client_nums.begin(); it_num != action_client_nums.end(); it_num++) {
    if (it_num->second != 2) {
      active_ros2_action_clients.erase(it_num->first);
    }
  }
  for (it_num = action_server_nums.begin(); it_num != action_server_nums.end(); it_num++) {
    if (it_num->second != 2) {
      active_ros2_action_servers.erase(it_num->first);
    }
  }
}

int main(int argc, char * argv[])
{
  bool output_topic_introspection;
  bool bridge_all_1to2_topics;
  bool bridge_all_2to1_topics;

  std::vector<const char *> ros1_args;
  std::vector<const char *> ros2_args;

  if (!parse_command_options(
      argc, argv, ros1_args, ros2_args, output_topic_introspection,
      bridge_all_1to2_topics, bridge_all_2to1_topics))
  {
    return 0;
  }

  // ROS 2 node
  rclcpp::init(ros2_args.size(), ros2_args.data());

  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  // ROS 1 node
  int argc_ros1 = ros1_args.size();
  ros::init(argc_ros1, const_cast<char **>(ros1_args.data()), "ros_bridge");
  ros::NodeHandle ros1_node;

  // mapping of available topic names to type names
  std::map<std::string, std::string> ros1_publishers;
  std::map<std::string, std::string> ros1_subscribers;
  std::map<std::string, std::string> ros2_publishers;
  std::map<std::string, std::string> ros2_subscribers;
  std::map<std::string, std::map<std::string, std::string>> ros1_services;
  std::map<std::string, std::map<std::string, std::string>> ros2_services;
  std::map<std::string, std::map<std::string, std::string>> ros1_action_servers;
  std::map<std::string, std::map<std::string, std::string>> ros1_action_clients;
  std::map<std::string, std::map<std::string, std::string>> ros2_action_servers;
  std::map<std::string, std::map<std::string, std::string>> ros2_action_clients;
  std::map<std::string, Bridge1to2HandlesAndMessageTypes> bridges_1to2;
  std::map<std::string, Bridge2to1HandlesAndMessageTypes> bridges_2to1;
  std::map<std::string, ros1_bridge::ServiceBridge1to2> service_bridges_1_to_2;
  std::map<std::string, ros1_bridge::ServiceBridge2to1> service_bridges_2_to_1;
  std::map<std::string, std::unique_ptr<ros1_bridge::ActionFactoryInterface>> action_bridges_1_to_2;
  std::map<std::string, std::unique_ptr<ros1_bridge::ActionFactoryInterface>> action_bridges_2_to_1;

  // setup polling of ROS 1 master
  auto ros1_poll = [
    &ros1_node, ros2_node,
    &ros1_publishers, &ros1_subscribers,
    &ros2_publishers, &ros2_subscribers,
    &bridges_1to2, &bridges_2to1,
    &ros1_services, &ros2_services,
    &ros1_action_servers, &ros1_action_clients,
    &ros2_action_servers, &ros2_action_clients,
    &service_bridges_1_to_2, &service_bridges_2_to_1,
    &action_bridges_1_to_2, &action_bridges_2_to_1,
    &output_topic_introspection,
    &bridge_all_1to2_topics, &bridge_all_2to1_topics
    ](const ros::TimerEvent &) -> void
    {
      // collect all topics names which have at least one publisher or subscriber beside this bridge
      std::set<std::string> active_publishers;
      std::set<std::string> active_subscribers;

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
          for (int k = 0; k < payload[0][j][1].size(); ++k) {
            std::string node_name = payload[0][j][1][k];
            // ignore publishers from the bridge itself
            if (node_name == ros::this_node::getName()) {
              continue;
            }
            active_publishers.insert(topic_name);
            break;
          }
        }
      }
      // check subscribers
      if (payload.size() >= 2) {
        for (int j = 0; j < payload[1].size(); ++j) {
          std::string topic_name = payload[1][j][0];
          for (int k = 0; k < payload[1][j][1].size(); ++k) {
            std::string node_name = payload[1][j][1][k];
            // ignore subscribers from the bridge itself
            if (node_name == ros::this_node::getName()) {
              continue;
            }
            active_subscribers.insert(topic_name);
            break;
          }
        }
      }

      // check services
      std::map<std::string, std::map<std::string, std::string>> active_ros1_services;
      if (payload.size() >= 3) {
        for (int j = 0; j < payload[2].size(); ++j) {
          if (payload[2][j][0].getType() == XmlRpc::XmlRpcValue::TypeString) {
            std::string name = payload[2][j][0];
            get_ros1_service_info(name, active_ros1_services);
          }
        }
      }
      {
        std::lock_guard<std::mutex> lock(g_bridge_mutex);
        ros1_services = active_ros1_services;
      }

      // get message types for all topics
      ros::master::V_TopicInfo topics;
      bool success = ros::master::getTopics(topics);
      if (!success) {
        fprintf(stderr, "failed to poll ROS 1 master\n");
        return;
      }

      std::map<std::string, std::string> current_ros1_publishers;
      std::map<std::string, std::string> current_ros1_subscribers;
      for (auto topic : topics) {
        auto topic_name = topic.name;
        bool has_publisher = active_publishers.find(topic_name) != active_publishers.end();
        bool has_subscriber = active_subscribers.find(topic_name) != active_subscribers.end();
        if (!has_publisher && !has_subscriber) {
          // skip inactive topics
          continue;
        }
        if (has_publisher) {
          current_ros1_publishers[topic_name] = topic.datatype;
        }
        if (has_subscriber) {
          current_ros1_subscribers[topic_name] = topic.datatype;
        }
        if (output_topic_introspection) {
          printf(
            "  ROS 1: %s (%s) [%s pubs, %s subs]\n",
            topic_name.c_str(), topic.datatype.c_str(),
            has_publisher ? ">0" : "0", has_subscriber ? ">0" : "0");
        }
      }

      // since ROS 1 subscribers don't report their type they must be added anyway
      for (auto active_subscriber : active_subscribers) {
        if (current_ros1_subscribers.find(active_subscriber) == current_ros1_subscribers.end()) {
          current_ros1_subscribers[active_subscriber] = "";
          if (output_topic_introspection) {
            printf("  ROS 1: %s (<unknown>) sub++\n", active_subscriber.c_str());
          }
        }
      }

      // check actions
      std::map<std::string, std::map<std::string, std::string>> active_ros1_action_servers,
        active_ros1_action_clients;
      get_active_ros1_actions(
        current_ros1_publishers, current_ros1_subscribers,
        active_ros1_action_servers, active_ros1_action_clients);

      {
        std::lock_guard<std::mutex> lock(g_bridge_mutex);
        ros1_services = active_ros1_services;
        ros1_action_servers = active_ros1_action_servers;
        ros1_action_clients = active_ros1_action_clients;
      }

      if (output_topic_introspection) {
        printf("\n");
      }

      {
        std::lock_guard<std::mutex> lock(g_bridge_mutex);
        ros1_publishers = current_ros1_publishers;
        ros1_subscribers = current_ros1_subscribers;
      }

      update_bridge(
        ros1_node, ros2_node,
        ros1_publishers, ros1_subscribers,
        ros2_publishers, ros2_subscribers,
        ros1_services, ros2_services,
        ros1_action_servers, ros2_action_servers,
        bridges_1to2, bridges_2to1,
        service_bridges_1_to_2, service_bridges_2_to_1,
        action_bridges_1_to_2, action_bridges_2_to_1,
        bridge_all_1to2_topics, bridge_all_2to1_topics);
    };

  auto ros1_poll_timer = ros1_node.createTimer(ros::Duration(1.0), ros1_poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // setup polling of ROS 2
  std::set<std::string> already_ignored_topics;
  std::set<std::string> already_ignored_services;
  auto ros2_poll = [
    &ros1_node, ros2_node,
    &ros1_publishers, &ros1_subscribers,
    &ros2_publishers, &ros2_subscribers,
    &ros1_services, &ros2_services,
    &ros1_action_servers, &ros1_action_clients,
    &ros2_action_servers, &ros2_action_clients,
    &bridges_1to2, &bridges_2to1,
    &service_bridges_1_to_2, &service_bridges_2_to_1,
    &action_bridges_1_to_2, &action_bridges_2_to_1,
    &output_topic_introspection,
    &bridge_all_1to2_topics, &bridge_all_2to1_topics,
    &already_ignored_topics, &already_ignored_services
    ]() -> void
    {
      auto ros2_topics = ros2_node->get_topic_names_and_types();

      std::set<std::string> ignored_topics;
      ignored_topics.insert("parameter_events");

      std::map<std::string, std::string> current_ros2_publishers;
      std::map<std::string, std::string> current_ros2_subscribers;
      for (auto topic_and_types : ros2_topics) {
        // ignore some common ROS 2 specific topics
        if (ignored_topics.find(topic_and_types.first) != ignored_topics.end()) {
          continue;
        }

        auto & topic_name = topic_and_types.first;
        auto & topic_type = topic_and_types.second[0];  // explicitly take the first

        // explicitly avoid topics with more than one type
        if (topic_and_types.second.size() > 1) {
          if (already_ignored_topics.count(topic_name) == 0) {
            std::string types = "";
            for (auto type : topic_and_types.second) {
              types += type + ", ";
            }
            fprintf(
              stderr,
              "warning: ignoring topic '%s', which has more than one type: [%s]\n",
              topic_name.c_str(),
              types.substr(0, types.length() - 2).c_str()
            );
            already_ignored_topics.insert(topic_name);
          }
          continue;
        }

        auto publisher_count = ros2_node->count_publishers(topic_name);
        auto subscriber_count = ros2_node->count_subscribers(topic_name);

        // ignore publishers from the bridge itself
        if (bridges_1to2.find(topic_name) != bridges_1to2.end()) {
          if (publisher_count > 0) {
            --publisher_count;
          }
        }
        // ignore subscribers from the bridge itself
        if (bridges_2to1.find(topic_name) != bridges_2to1.end()) {
          if (subscriber_count > 0) {
            --subscriber_count;
          }
        }

        if (publisher_count) {
          current_ros2_publishers[topic_name] = topic_type;
        }

        if (subscriber_count) {
          current_ros2_subscribers[topic_name] = topic_type;
        }

        if (output_topic_introspection) {
          printf(
            "  ROS 2: %s (%s) [%zu pubs, %zu subs]\n",
            topic_name.c_str(), topic_type.c_str(), publisher_count, subscriber_count);
        }
      }

      // collect available services (not clients)
      std::set<std::string> service_names;
      std::vector<std::pair<std::string, std::string>> node_names_and_namespaces =
        ros2_node->get_node_graph_interface()->get_node_names_and_namespaces();
      for (auto & pair : node_names_and_namespaces) {
        if (pair.first == ros2_node->get_name() && pair.second == ros2_node->get_namespace()) {
          continue;
        }
        std::map<std::string, std::vector<std::string>> services_and_types =
          ros2_node->get_service_names_and_types_by_node(pair.first, pair.second);
        for (auto & it : services_and_types) {
          service_names.insert(it.first);
        }
      }

      auto ros2_services_and_types = ros2_node->get_service_names_and_types();
      std::map<std::string, std::map<std::string, std::string>> active_ros2_services;
      for (const auto & service_and_types : ros2_services_and_types) {
        auto & service_name = service_and_types.first;
        auto & service_type = service_and_types.second[0];  // explicitly take the first

        // explicitly avoid services with more than one type
        if (service_and_types.second.size() > 1) {
          if (already_ignored_services.count(service_name) == 0) {
            std::string types = "";
            for (auto type : service_and_types.second) {
              types += type + ", ";
            }
            fprintf(
              stderr,
              "warning: ignoring service '%s', which has more than one type: [%s]\n",
              service_name.c_str(),
              types.substr(0, types.length() - 2).c_str()
            );
            already_ignored_services.insert(service_name);
          }
          continue;
        }

        // TODO(wjwwood): this should be common functionality in the C++ rosidl package
        size_t separator_position = service_type.find('/');
        if (separator_position == std::string::npos) {
          fprintf(stderr, "invalid service type '%s', skipping...\n", service_type.c_str());
          continue;
        }

        // only bridge if there is a service, not for a client
        if (service_names.find(service_name) != service_names.end()) {
          auto service_type_package_name = service_type.substr(0, separator_position);
          auto service_type_srv_name = service_type.substr(separator_position + 1);
          active_ros2_services[service_name]["package"] = service_type_package_name;
          active_ros2_services[service_name]["name"] = service_type_srv_name;
        }
      }

      std::map<std::string, std::map<std::string, std::string>> active_ros2_action_servers,
        active_ros2_action_clients;
      get_active_ros2_actions(
        current_ros2_publishers, current_ros2_subscribers,
        active_ros2_action_servers, active_ros2_action_clients);

      {
        std::lock_guard<std::mutex> lock(g_bridge_mutex);
        ros2_services = active_ros2_services;
        ros2_action_servers = active_ros2_action_servers;
        ros2_action_clients = active_ros2_action_clients;
      }

      if (output_topic_introspection) {
        printf("\n");
      }

      {
        std::lock_guard<std::mutex> lock(g_bridge_mutex);
        ros2_publishers = current_ros2_publishers;
        ros2_subscribers = current_ros2_subscribers;
      }

      update_bridge(
        ros1_node, ros2_node,
        ros1_publishers, ros1_subscribers,
        ros2_publishers, ros2_subscribers,
        ros1_services, ros2_services,
        ros1_action_servers, ros2_action_servers,
        bridges_1to2, bridges_2to1,
        service_bridges_1_to_2, service_bridges_2_to_1,
        action_bridges_1_to_2, action_bridges_2_to_1,
        bridge_all_1to2_topics, bridge_all_2to1_topics);
    };

  auto ros2_poll_timer = ros2_node->create_wall_timer(
    std::chrono::seconds(1), ros2_poll);


  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node);
  }

  return 0;
}
