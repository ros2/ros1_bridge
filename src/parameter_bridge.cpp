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

#include <xmlrpcpp/XmlRpcException.h>

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

rclcpp::QoS qos_from_params(XmlRpc::XmlRpcValue qos_params)
{
  auto ros2_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10));

  printf("Qos(");

  if (qos_params.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    if (qos_params.hasMember("history")) {
      auto history = static_cast<std::string>(qos_params["history"]);
      printf("history: ");
      if (history == "keep_all") {
        ros2_publisher_qos.keep_all();
        printf("keep_all, ");
      } else if (history == "keep_last") {
        if (qos_params.hasMember("depth")) {
          auto depth = static_cast<int>(qos_params["depth"]);
          ros2_publisher_qos.keep_last(depth);
          printf("keep_last(%i), ", depth);
        } else {
          fprintf(
            stderr,
            "history: keep_last requires that also a depth is set\n");
        }
      } else {
        fprintf(
          stderr,
          "invalid value for 'history': '%s', allowed values are 'keep_all',"
          "'keep_last' (also requires 'depth' to be set)\n",
          history.c_str());
      }
    }

    if (qos_params.hasMember("reliability")) {
      auto reliability = static_cast<std::string>(qos_params["reliability"]);
      printf("reliability: ");
      if (reliability == "best_effort") {
        ros2_publisher_qos.best_effort();
        printf("best_effort, ");
      } else if (reliability == "reliable") {
        ros2_publisher_qos.reliable();
        printf("reliable, ");
      } else {
        fprintf(
          stderr,
          "invalid value for 'reliability': '%s', allowed values are 'best_effort', 'reliable'\n",
          reliability.c_str());
      }
    }

    if (qos_params.hasMember("durability")) {
      auto durability = static_cast<std::string>(qos_params["durability"]);
      printf("durability: ");
      if (durability == "transient_local") {
        ros2_publisher_qos.transient_local();
        printf("transient_local, ");
      } else if (durability == "volatile") {
        ros2_publisher_qos.durability_volatile();
        printf("volatile, ");
      } else {
        fprintf(
          stderr,
          "invalid value for 'durability': '%s', allowed values are 'best_effort', 'volatile'\n",
          durability.c_str());
      }
    }

    if (qos_params.hasMember("deadline")) {
      try {
        rclcpp::Duration dur = rclcpp::Duration(
          static_cast<int>(qos_params["deadline"]["secs"]),
          static_cast<int>(qos_params["deadline"]["nsecs"]));
        ros2_publisher_qos.deadline(dur);
        printf("deadline: Duration(nsecs: %ld), ", dur.nanoseconds());
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to parse deadline: '%s'\n",
          e.what());
      } catch (XmlRpc::XmlRpcException & e) {
        fprintf(
          stderr,
          "failed to parse deadline: '%s'\n",
          e.getMessage().c_str());
      }
    }

    if (qos_params.hasMember("lifespan")) {
      try {
        rclcpp::Duration dur = rclcpp::Duration(
          static_cast<int>(qos_params["lifespan"]["secs"]),
          static_cast<int>(qos_params["lifespan"]["nsecs"]));
        ros2_publisher_qos.lifespan(dur);
        printf("lifespan: Duration(nsecs: %ld), ", dur.nanoseconds());
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to parse lifespan: '%s'\n",
          e.what());
      } catch (XmlRpc::XmlRpcException & e) {
        fprintf(
          stderr,
          "failed to parse lifespan: '%s'\n",
          e.getMessage().c_str());
      }
    }

    if (qos_params.hasMember("liveliness")) {
      if (qos_params["liveliness"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        try {
          auto liveliness = static_cast<int>(qos_params["liveliness"]);
          ros2_publisher_qos.liveliness(static_cast<rmw_qos_liveliness_policy_t>(liveliness));
          printf("liveliness: %i, ", static_cast<int>(liveliness));
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.what());
        } catch (XmlRpc::XmlRpcException & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.getMessage().c_str());
        }
      } else if (qos_params["liveliness"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        try {
          rmw_qos_liveliness_policy_t liveliness =
            rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
          auto liveliness_str = static_cast<std::string>(qos_params["liveliness"]);
          if (liveliness_str == "LIVELINESS_SYSTEM_DEFAULT" ||
            liveliness_str == "liveliness_system_default")
          {
            liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
          } else if (liveliness_str == "LIVELINESS_AUTOMATIC" ||  // NOLINT
            liveliness_str == "liveliness_automatic")
          {
            liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
          } else if (liveliness_str == "LIVELINESS_MANUAL_BY_TOPIC" ||  // NOLINT
            liveliness_str == "liveliness_manual_by_topic")
          {
            liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
          } else {
            fprintf(
              stderr,
              "invalid value for 'liveliness': '%s', allowed values are "
              "LIVELINESS_{SYSTEM_DEFAULT, AUTOMATIC, MANUAL_BY_TOPIC}, upper or lower case\n",
              liveliness_str.c_str());
          }

          ros2_publisher_qos.liveliness(liveliness);
          printf("liveliness: %s, ", liveliness_str.c_str());
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.what());
        } catch (XmlRpc::XmlRpcException & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.getMessage().c_str());
        }
      } else {
        fprintf(
          stderr,
          "failed to parse liveliness, parameter was not a string or int \n");
      }
    }

    if (qos_params.hasMember("liveliness_lease_duration")) {
      try {
        rclcpp::Duration dur = rclcpp::Duration(
          static_cast<int>(qos_params["liveliness_lease_duration"]["secs"]),
          static_cast<int>(qos_params["liveliness_lease_duration"]["nsecs"]));
        ros2_publisher_qos.liveliness_lease_duration(dur);
        printf("liveliness_lease_duration: Duration(nsecs: %ld), ", dur.nanoseconds());
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to parse liveliness_lease_duration: '%s'\n",
          e.what());
      } catch (XmlRpc::XmlRpcException & e) {
        fprintf(
          stderr,
          "failed to parse liveliness_lease_duration: '%s'\n",
          e.getMessage().c_str());
      }
    }
  } else {
    fprintf(
      stderr,
      "QoS parameters could not be read\n");
  }

  printf(")");
  return ros2_publisher_qos;
}

int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  std::list<ros1_bridge::BridgeHandles> all_handles;
  std::list<ros1_bridge::ServiceBridge1to2> service_bridges_1_to_2;
  std::list<ros1_bridge::ServiceBridge2to1> service_bridges_2_to_1;

  // bridge all topics listed in a ROS 1 parameter
  // the topics parameter needs to be an array
  // and each item needs to be a dictionary with the following keys;
  // topic: the name of the topic to bridge (e.g. '/topic_name')
  // type: the type of the topic to bridge (e.g. 'pkgname/msg/MsgName')
  // queue_size: the queue size to use (default: 100)
  const char * topics_parameter_name = "topics";
  // the services parameters need to be arrays
  // and each item needs to be a dictionary with the following keys;
  // service: the name of the service to bridge (e.g. '/service_name')
  // type: the type of the service to bridge (e.g. 'pkgname/srv/SrvName')
  const char * services_1_to_2_parameter_name = "services_1_to_2";
  const char * services_2_to_1_parameter_name = "services_2_to_1";
  const char * service_execution_timeout_parameter_name =
    "ros1_bridge/parameter_bridge/service_execution_timeout";
  if (argc > 1) {
    topics_parameter_name = argv[1];
  }
  if (argc > 2) {
    services_1_to_2_parameter_name = argv[2];
  }
  if (argc > 3) {
    services_2_to_1_parameter_name = argv[3];
  }

  // Topics
  XmlRpc::XmlRpcValue topics;
  if (
    ros1_node.getParam(topics_parameter_name, topics) &&
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
        "Trying to create bidirectional bridge for topic '%s' "
        "with ROS 2 type '%s'\n",
        topic_name.c_str(), type_name.c_str());

      try {
        if (topics[i].hasMember("qos")) {
          printf("Setting up QoS for '%s': ", topic_name.c_str());
          auto qos_settings = qos_from_params(topics[i]["qos"]);
          printf("\n");
          ros1_bridge::BridgeHandles handles = ros1_bridge::create_bidirectional_bridge(
            ros1_node, ros2_node, "", type_name, topic_name, queue_size, qos_settings);
          all_handles.push_back(handles);
        } else {
          ros1_bridge::BridgeHandles handles = ros1_bridge::create_bidirectional_bridge(
            ros1_node, ros2_node, "", type_name, topic_name, queue_size);
          all_handles.push_back(handles);
        }
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to create bidirectional bridge for topic '%s' "
          "with ROS 2 type '%s': %s\n",
          topic_name.c_str(), type_name.c_str(), e.what());
      }
    }
  } else {
    fprintf(
      stderr,
      "The parameter '%s' either doesn't exist or isn't an array\n", topics_parameter_name);
  }

  // ROS 1 Services in ROS 2
  XmlRpc::XmlRpcValue services_1_to_2;
  if (
    ros1_node.getParam(services_1_to_2_parameter_name, services_1_to_2) &&
    services_1_to_2.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    int service_execution_timeout{5};
    ros1_node.getParamCached(
      service_execution_timeout_parameter_name, service_execution_timeout);
    for (size_t i = 0; i < static_cast<size_t>(services_1_to_2.size()); ++i) {
      std::string service_name = static_cast<std::string>(services_1_to_2[i]["service"]);
      std::string type_name = static_cast<std::string>(services_1_to_2[i]["type"]);
      {
        // for backward compatibility
        std::string package_name = static_cast<std::string>(services_1_to_2[i]["package"]);
        if (!package_name.empty()) {
          fprintf(
            stderr,
            "The service '%s' uses the key 'package' which is deprecated for "
            "services. Instead prepend the 'type' value with '<package>/'.\n",
            service_name.c_str());
          type_name = package_name + "/" + type_name;
        }
      }
      printf(
        "Trying to create bridge for ROS 2 service '%s' with type '%s'\n",
        service_name.c_str(), type_name.c_str());

      const size_t index = type_name.find("/");
      if (index == std::string::npos) {
        fprintf(
          stderr,
          "the service '%s' has a type '%s' without a slash.\n",
          service_name.c_str(), type_name.c_str());
        continue;
      }
      auto factory = ros1_bridge::get_service_factory(
        "ros2", type_name.substr(0, index), type_name.substr(index + 1));
      if (factory) {
        try {
          service_bridges_1_to_2.push_back(
            factory->service_bridge_1_to_2(
              ros1_node, ros2_node, service_name, service_execution_timeout));
          printf("Created 1 to 2 bridge for service %s\n", service_name.c_str());
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to create bridge ROS 1 service '%s' with type '%s': %s\n",
            service_name.c_str(), type_name.c_str(), e.what());
        }
      } else {
        fprintf(
          stderr,
          "failed to create bridge ROS 1 service '%s' no conversion for type '%s'\n",
          service_name.c_str(), type_name.c_str());
      }
    }

  } else {
    fprintf(
      stderr,
      "The parameter '%s' either doesn't exist or isn't an array\n",
      services_1_to_2_parameter_name);
  }

  // ROS 2 Services in ROS 1
  XmlRpc::XmlRpcValue services_2_to_1;
  if (
    ros1_node.getParam(services_2_to_1_parameter_name, services_2_to_1) &&
    services_2_to_1.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < static_cast<size_t>(services_2_to_1.size()); ++i) {
      std::string service_name = static_cast<std::string>(services_2_to_1[i]["service"]);
      std::string type_name = static_cast<std::string>(services_2_to_1[i]["type"]);
      {
        // for backward compatibility
        std::string package_name = static_cast<std::string>(services_2_to_1[i]["package"]);
        if (!package_name.empty()) {
          fprintf(
            stderr,
            "The service '%s' uses the key 'package' which is deprecated for "
            "services. Instead prepend the 'type' value with '<package>/'.\n",
            service_name.c_str());
          type_name = package_name + "/" + type_name;
        }
      }
      printf(
        "Trying to create bridge for ROS 1 service '%s' with type '%s'\n",
        service_name.c_str(), type_name.c_str());

      const size_t index = type_name.find("/");
      if (index == std::string::npos) {
        fprintf(
          stderr,
          "the service '%s' has a type '%s' without a slash.\n",
          service_name.c_str(), type_name.c_str());
        continue;
      }

      auto factory = ros1_bridge::get_service_factory(
        "ros1", type_name.substr(0, index), type_name.substr(index + 1));
      if (factory) {
        try {
          service_bridges_2_to_1.push_back(
            factory->service_bridge_2_to_1(ros1_node, ros2_node, service_name));
          printf("Created 2 to 1 bridge for service %s\n", service_name.c_str());
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to create bridge ROS 2 service '%s' with type '%s': %s\n",
            service_name.c_str(), type_name.c_str(), e.what());
        }
      } else {
        fprintf(
          stderr,
          "failed to create bridge ROS 2 service '%s' no conversion for type '%s'\n",
          service_name.c_str(), type_name.c_str());
      }
    }

  } else {
    fprintf(
      stderr,
      "The parameter '%s' either doesn't exist or isn't an array\n",
      services_2_to_1_parameter_name);
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
