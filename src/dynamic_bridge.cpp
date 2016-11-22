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

#include <map>
#include <set>
#include <string>
#include <vector>

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

#include "ros1_bridge/bridge.hpp"


std::mutex g_bridge_mutex;

namespace ros1_bridge {
std::unique_ptr<ros1_bridge::ServiceFactoryInterface> get_service_factory(std::string, std::string, std::string);
}

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


void update_bridge(
  ros::NodeHandle & ros1_node,
  rclcpp::node::Node::SharedPtr ros2_node,
  const std::map<std::string, std::string> & ros1_publishers,
  const std::map<std::string, std::string> & ros1_subscribers,
  const std::map<std::string, std::string> & ros2_publishers,
  const std::map<std::string, std::string> & ros2_subscribers,
  const std::map<std::string, std::map<std::string, std::string>> & ros1_services,
  const std::map<std::string, std::map<std::string, std::string>> & ros2_services,
  std::map<std::string, Bridge1to2HandlesAndMessageTypes> & bridges_1to2,
  std::map<std::string, Bridge2to1HandlesAndMessageTypes> & bridges_2to1,
  std::map<std::string, ros1_bridge::ServiceBridge1to2>& service_bridges_1_to_2,
  std::map<std::string, ros1_bridge::ServiceBridge2to1>& service_bridges_2_to_1)
{
  std::lock_guard<std::mutex> lock(g_bridge_mutex);

  // create 1to2 bridges
  for (auto ros1_publisher : ros1_publishers) {
    // identify topics available as ROS 1 publishers as well as ROS 2 subscribers
    auto topic_name = ros1_publisher.first;
    auto ros2_subscriber = ros2_subscribers.find(topic_name);
    if (ros2_subscriber == ros2_subscribers.end()) {
      continue;
    }

    std::string ros1_type_name = ros1_publisher.second;
    std::string ros2_type_name = ros2_subscriber->second;
    // printf("topic name '%s' has ROS 1 publishers and ROS 2 subscribers\n", topic_name.c_str());

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

    try {
      bridge.bridge_handles = ros1_bridge::create_bridge_from_1_to_2(
        ros1_node, ros2_node,
        bridge.ros1_type_name, topic_name, 10,
        bridge.ros2_type_name, topic_name, 10);
    } catch (std::runtime_error & e) {
      fprintf(
        stderr,
        "failed to create 1to2 bridge for topic '%s' "
        "with ROS 1 type '%s' and ROS 2 type '%s': %s\n",
        topic_name.c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str(), e.what());
      continue;
    }

    bridges_1to2[topic_name] = bridge;
    printf(
      "created 1to2 bridge for topic '%s' with ROS 1 type '%s' and ROS 2 type '%s'\n",
      topic_name.c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str());
  }

  // create 2to1 bridges
  for (auto ros1_subscriber : ros1_subscribers) {
    // identify topics available as ROS 1 subscribers as well as ROS 2 publishers
    auto topic_name = ros1_subscriber.first;
    auto ros2_publisher = ros2_publishers.find(topic_name);
    if (ros2_publisher == ros2_publishers.end()) {
      continue;
    }

    std::string ros1_type_name = ros1_subscriber.second;
    std::string ros2_type_name = ros2_publisher->second;
    // printf("topic name '%s' has ROS 1 subscribers and ROS 2 publishers\n", topic_name.c_str());

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
      fprintf(
        stderr,
        "failed to create 2to1 bridge for topic '%s' "
        "with ROS 2 type '%s' and ROS 1 type '%s': %s\n",
        topic_name.c_str(), bridge.ros2_type_name.c_str(), bridge.ros1_type_name.c_str(), e.what());
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
      ros2_subscribers.find(topic_name) == ros2_subscribers.end()
    )
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
      ros1_subscribers.find(topic_name) == ros1_subscribers.end() ||
      ros2_publishers.find(topic_name) == ros2_publishers.end()
    )
    {
      to_be_removed_2to1.push_back(topic_name);
    }
  }
  for (auto topic_name : to_be_removed_2to1) {
    bridges_2to1.erase(topic_name);
    printf("removed 2to1 bridge for topic '%s'\n", topic_name.c_str());
  }

  // create bridges for ros1 services
  for (auto& service : ros1_services) {
    auto& name = service.first;
    auto& details = service.second;
    if (
      service_bridges_2_to_1.find(name) == service_bridges_2_to_1.end() &&
      service_bridges_1_to_2.find(name) == service_bridges_1_to_2.end())
    {
      auto factory = ros1_bridge::get_service_factory("ros1", details.at("package"), details.at("name"));
      service_bridges_2_to_1[name] = factory->service_bridge_2_to_1(ros1_node, ros2_node, name);
      // printf("Created 2 to 1 bridge for service %s\n", name.data());
    }
  }

  // create bridges for ros2 services
  for (auto& service : ros2_services) {
    auto& name = service.first;
    auto& details = service.second;
    if (
      service_bridges_1_to_2.find(name) == service_bridges_1_to_2.end() &&
      service_bridges_2_to_1.find(name) == service_bridges_2_to_1.end())
    {
      auto factory = ros1_bridge::get_service_factory("ros2", details.at("package"), details.at("name"));
      service_bridges_1_to_2[name] = factory->service_bridge_1_to_2(ros1_node, ros2_node, name);
      // printf("Created 1 to 2 bridge for service %s\n", name.data());
    }
  }

  // remove obsolete ros1 services
  for (auto it = service_bridges_2_to_1.begin(); it != service_bridges_2_to_1.end();) {
    if (ros1_services.find(it->first) == ros1_services.end()) {
      it = service_bridges_2_to_1.erase(it);
      // printf("Removed 2 to 1 bridge for service %s\n", service.first.data());
    }
    else {
      ++it;
    }
  }

  // remove obsolete ros2 services
  for (auto it = service_bridges_1_to_2.begin(); it != service_bridges_1_to_2.end();) {
    if (ros2_services.find(it->first) == ros2_services.end()) {
      it->second.server.shutdown();
      it = service_bridges_1_to_2.erase(it);
      // printf("Removed 1 to 2 bridge for service %s\n", service.first.data());
    }
    else {
      ++it;
    }
  }
}

void get_ros1_service_info(
  const std::string name, std::map<std::string, std::map<std::string,std::string>>& ros1_services)
{
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
    printf("Failed to look up %s", name.data());
    return;
  }
  ros::TransportTCPPtr transport(new ros::TransportTCP(nullptr, ros::TransportTCP::SYNCHRONOUS));
  if (!transport->connect(host, port)) {
    printf("Failed to connect to %s:%d", host.data(), port);
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
  auto read = transport->read(reinterpret_cast<uint8_t*>(&length), 4);
  if (read != 4) {
    printf("Failed to read a response from a service server\n");
    return;
  }
  std::vector<uint8_t> response(length);
  read = transport->read(response.data(), length);
  if (read < 0 || static_cast<uint32_t>(read) != length) {
    printf("Failed to read a response from a service server\n");
    return;
  }
  std::string key(name.begin() + 1, name.end());
  ros1_services[key] = std::map<std::string,std::string>();
  ros::Header header_in;
  std::string error;
  auto success = header_in.parse(response.data(), length, error);
  if (!success) {
    printf("%s", error.data());
    return;
  }
  for (std::string field : { "type", "request_type", "response_type" }) {
    std::string value;
    auto success = header_in.getValue(field, value);
    if (!success)
    {
      printf("Failed to read %s from a header", field.data());
      return;
    }
    ros1_services[key][field] = value;
  }
  std::string t = ros1_services[key]["type"];
  ros1_services[key]["package"] = std::string(t.begin(), t.begin() + t.find("/"));
  ros1_services[key]["name"] = std::string(t.begin() + t.find("/") + 1, t.end());
}

int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::node::Node::make_shared("ros_bridge");

  bool output_topic_introspection = (2 == argc && std::string("--show-introspection") == argv[1]);

  // mapping of available topic names to type names
  std::map<std::string, std::string> ros1_publishers;
  std::map<std::string, std::string> ros1_subscribers;
  std::map<std::string, std::string> ros2_publishers;
  std::map<std::string, std::string> ros2_subscribers;
  std::map<std::string, std::map<std::string,std::string>> ros1_services;
  std::map<std::string, std::map<std::string,std::string>> ros2_services;

  std::map<std::string, Bridge1to2HandlesAndMessageTypes> bridges_1to2;
  std::map<std::string, Bridge2to1HandlesAndMessageTypes> bridges_2to1;
  std::map<std::string, ros1_bridge::ServiceBridge1to2> service_bridges_1_to_2;
  std::map<std::string, ros1_bridge::ServiceBridge2to1> service_bridges_2_to_1;

  // setup polling of ROS 1 master
  auto ros1_poll = [
    &ros1_node, ros2_node,
    &ros1_publishers, &ros1_subscribers,
    &ros2_publishers, &ros2_subscribers,
    &bridges_1to2, &bridges_2to1,
    &ros1_services, &ros2_services,
    &service_bridges_1_to_2, &service_bridges_2_to_1,
    &output_topic_introspection
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
          if (topic_name.compare(0, 1, "/") == 0) {
            topic_name = topic_name.substr(1);
          }
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
          if (topic_name.compare(0, 1, "/") == 0) {
            topic_name = topic_name.substr(1);
          }
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
      std::map<std::string, std::map<std::string,std::string>> active_ros1_services;
      if (payload.size() >= 3) {
        for (int j = 0; j < payload[2].size(); ++j) {
          if (payload[2][j][0].getType() == XmlRpc::XmlRpcValue::TypeString) {
            std::string name = payload[2][j][0];
            if (name.find("/", 1) == std::string::npos) {
              get_ros1_service_info(name, active_ros1_services);
            }
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

      ros1_publishers.clear();
      ros1_subscribers.clear();
      for (auto topic : topics) {
        auto topic_name = topic.name;
        if (topic_name.compare(0, 1, "/") == 0) {
          topic_name = topic_name.substr(1);
        }
        bool has_publisher = active_publishers.find(topic_name) != active_publishers.end();
        bool has_subscriber = active_subscribers.find(topic_name) != active_subscribers.end();
        if (!has_publisher && !has_subscriber) {
          // skip inactive topics
          continue;
        }
        if (has_publisher) {
          ros1_publishers[topic_name] = topic.datatype;
        }
        if (has_subscriber) {
          ros1_subscribers[topic_name] = topic.datatype;
        }
        if (output_topic_introspection) {
          printf("  ROS 1: %s (%s) [%s pubs, %s subs]\n",
            topic_name.c_str(), topic.datatype.c_str(),
            has_publisher ? ">0" : "0", has_subscriber ? ">0" : "0");
        }
      }

      // since ROS 1 subscribers don't report their type they must be added anyway
      for (auto active_subscriber : active_subscribers) {
        if (ros1_subscribers.find(active_subscriber) == ros1_subscribers.end()) {
          ros1_subscribers[active_subscriber] = "";
          if (output_topic_introspection) {
            printf("  ROS 1: %s (<unknown>) sub++\n", active_subscriber.c_str());
          }
        }
      }

      if (output_topic_introspection) {
        printf("\n");
      }

      update_bridge(
        ros1_node, ros2_node,
        ros1_publishers, ros1_subscribers,
        ros2_publishers, ros2_subscribers,
        ros1_services, ros2_services,
        bridges_1to2, bridges_2to1,
        service_bridges_1_to_2, service_bridges_2_to_1);
    };

  auto ros1_poll_timer = ros1_node.createTimer(ros::Duration(1.0), ros1_poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // setup polling of ROS 2
  auto ros2_poll = [
    &ros1_node, ros2_node,
    &ros1_publishers, &ros1_subscribers,
    &ros2_publishers, &ros2_subscribers,
    &ros1_services, &ros2_services,
    &bridges_1to2, &bridges_2to1,
    &service_bridges_1_to_2, &service_bridges_2_to_1,
    &output_topic_introspection
    ]() -> void
    {
      auto ros2_topics = ros2_node->get_topic_names_and_types();

      std::set<std::string> ignored_topics;
      ignored_topics.insert("CMParticipant");
      ignored_topics.insert("DCPSCandMCommand");
      ignored_topics.insert("DCPSDelivery");
      ignored_topics.insert("DCPSHeartbeat");
      ignored_topics.insert("DCPSParticipant");
      ignored_topics.insert("DCPSPublication");
      ignored_topics.insert("DCPSSubscription");
      ignored_topics.insert("DCPSTopic");
      ignored_topics.insert("d_deleteData");
      ignored_topics.insert("d_groupsRequest");
      ignored_topics.insert("d_nameSpaces");
      ignored_topics.insert("d_nameSpacesRequest");
      ignored_topics.insert("d_newGroup");
      ignored_topics.insert("d_sampleChain");
      ignored_topics.insert("d_sampleRequest");
      ignored_topics.insert("d_status");
      ignored_topics.insert("d_statusRequest");
      ignored_topics.insert("parameter_events");
      ignored_topics.insert("q_bubble");

      ros2_publishers.clear();
      ros2_subscribers.clear();
      std::map<std::string, std::map<std::string,std::string>> active_ros2_services;
      for (auto it : ros2_topics) {
        // ignore builtin DDS topics
        if (ignored_topics.find(it.first) != ignored_topics.end()) {
          continue;
        }
        auto publisher_count = ros2_node->count_publishers(it.first);
        auto subscriber_count = ros2_node->count_subscribers(it.first);

        // ignore publishers from the bridge itself
        if (bridges_1to2.find(it.first) != bridges_1to2.end()) {
          if (publisher_count > 0) {
            --publisher_count;
          }
        }
        // ignore subscribers from the bridge itself
        if (bridges_2to1.find(it.first) != bridges_2to1.end()) {
          if (subscriber_count > 0) {
            --subscriber_count;
          }
        }

        if (publisher_count) {
          std::string suffix("Reply");
          if (it.first.rfind(suffix) == it.first.size() - suffix.size()) {
            std::string& t = it.second;
            std::string name(it.first.begin(), it.first.end() - suffix.size());
            if (active_ros2_services.find(name) == active_ros2_services.end()) {
              active_ros2_services[name] = std::map<std::string,std::string>();
            }
            std::string srv(t.begin() + t.rfind("::") + 2, t.begin() + t.rfind("_Response_"));
            std::string pkg(t.begin(), t.begin() + t.find("::"));
            active_ros2_services[name]["package"] = pkg;
            active_ros2_services[name]["name"] = srv;
            active_ros2_services[name]["response_topic"] = it.first;
            active_ros2_services[name]["response_type"] = t;
          } else {
            ros2_publishers[it.first] = it.second;
          }
        }

        if (subscriber_count) {
          std::string suffix("Request");
          if (it.first.rfind(suffix) == it.first.size() - suffix.size()) {
            std::string& t = it.second;
            std::string name(it.first.begin(), it.first.end() - suffix.size());
            if (active_ros2_services.find(name) == active_ros2_services.end()) {
              active_ros2_services[name] = std::map<std::string,std::string>();
            }
            active_ros2_services[name]["request_topic"] = it.first;
            active_ros2_services[name]["request_type"] = t;
          } else {
            ros2_subscribers[it.first] = it.second;
          }
        }

        if (output_topic_introspection) {
          printf("  ROS 2: %s (%s) [%ld pubs, %ld subs]\n",
            it.first.c_str(), it.second.c_str(), publisher_count, subscriber_count);
        }
      }

      for (auto it = active_ros2_services.begin(); it != active_ros2_services.end();) {
        if (it->second.size() != 6) {
          it = active_ros2_services.erase(it);
        }
        else {
          ++it;
        }
      }
      {
        std::lock_guard<std::mutex> lock(g_bridge_mutex);
        ros2_services = active_ros2_services;
      }

      if (output_topic_introspection) {
        printf("\n");
      }

      update_bridge(
        ros1_node, ros2_node,
        ros1_publishers, ros1_subscribers,
        ros2_publishers, ros2_subscribers,
        ros1_services, ros2_services,
        bridges_1to2, bridges_2to1,
        service_bridges_1_to_2, service_bridges_2_to_1);
    };

  auto ros2_poll_timer = ros2_node->create_wall_timer(
    std::chrono::seconds(1), ros2_poll);


  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::utilities::ok()) {
    executor.spin_node_once(ros2_node);
  }

  return 0;
}
