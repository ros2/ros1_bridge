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

  std::list<ros1_bridge::ServiceBridge1to2> service_bridges_1_to_2;
  std::list<ros1_bridge::ServiceBridge2to1> service_bridges_2_to_1;

  
  // bridge one example topic
  std::string topic_name = "chatter";
  std::string ros1_type_name = "std_msgs/String";
  std::string ros2_type_name = "std_msgs/msg/String";
  size_t queue_size = 0;

  auto handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, ros1_type_name, ros2_type_name, topic_name, queue_size);

  // bridge joint_states topic
  std::string joint_states_topic_name = "joint_states";
  std::string joint_states_ros1_type_name = "sensor_msgs/JointState";
  std::string joint_states_ros2_type_name = "sensor_msgs/msg/JointState";

  auto joint_states_handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, joint_states_ros1_type_name, joint_states_ros2_type_name, joint_states_topic_name, queue_size);
  
  // bridge tf topic
  std::string tf_topic_name = "tf";
  std::string tf_ros1_type_name = "tf2_msgs/TFMessage";
  std::string tf_ros2_type_name = "tf2_msgs/msg/TFMessage";

  auto tf_handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, tf_ros1_type_name, tf_ros2_type_name, tf_topic_name, queue_size);
  
  // bridge ensenso/point_cloud topic
  std::string point_cloud_topic_name = "ensenso/point_cloud";
  std::string point_cloud_ros1_type_name = "sensor_msgs/PointCloud2";
  std::string point_cloud_ros2_type_name = "sensor_msgs/msg/PointCloud2";

  auto point_cloud_handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, point_cloud_ros1_type_name, point_cloud_ros2_type_name, point_cloud_topic_name, queue_size);

  // bridge ensenso/rectified/left/image topic
  std::string ensenso_image_topic_name = "ensenso/rectified/left/image";
  std::string ensenso_image_ros1_type_name = "sensor_msgs/Image";
  std::string ensenso_image_ros2_type_name = "sensor_msgs/msg/Image";

  auto ensenso_image_handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, ensenso_image_ros1_type_name, ensenso_image_ros2_type_name, ensenso_image_topic_name, queue_size);


  // bridge pylon_camera_node/image_raw topic
  std::string pylon_image_topic_name = "pylon_camera_node/image_raw";
  std::string pylon_image_ros1_type_name = "sensor_msgs/Image";
  std::string pylon_image_ros2_type_name = "sensor_msgs/msg/Image";

  auto pylon_image_handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, pylon_image_ros1_type_name, pylon_image_ros2_type_name, pylon_image_topic_name, queue_size);

  
//  // create bridges for ros1 services
//  auto request_data_relay_2to1_factory = ros1_bridge::get_service_factory(
//    "ros1", "std_srvs", "Trigger");
//  if (request_data_relay_2to1_factory) {
//    try {
//      ros1_bridge::ServiceBridge2to1 request_data_relay_2to1 = request_data_relay_2to1_factory->service_bridge_2_to_1(ros1_node, ros2_node, "request_data_relay");
//      printf("Created 2 to 1 bridge for service %s\n", "/request_data_relay");
//    } catch (std::runtime_error & e) {
//      fprintf(stderr, "Failed to created a bridge: %s\n", "/request_data_relay");
//    }
//  }

  // create bridges for ros2 services
//  auto request_data_relay_1to2_factory = ros1_bridge::get_service_factory(
//    "ros2", "std_srvs", "srv/Trigger");
//  if (request_data_relay_1to2_factory) {
//    try {
//      ros1_bridge::ServiceBridge1to2 request_data_relay_1to2 = request_data_relay_1to2_factory->service_bridge_1_to_2(ros1_node, ros2_node, "request_data_relay");
//      printf("Created 1 to 2 bridge for service %s\n", "/request_data_relay");
//    } catch (std::runtime_error & e) {
//      fprintf(stderr, "Failed to created a bridge: %s\n", "/request_data_relay");
//    }
//  }
  

  std::string request_data_relay_service_name = "/request_data_relay";
  std::string request_data_relay_package_name = "std_srvs";
  std::string request_data_relay_type_name = "Trigger";
  printf(
    "Trying to create bridge for ROS 1 service '%s' "
    "with package '%s' and type '%s'\n",
    request_data_relay_service_name.c_str(), request_data_relay_package_name.c_str(), request_data_relay_type_name.c_str());

  auto request_data_relay_factory = ros1_bridge::get_service_factory(
    "ros1", request_data_relay_package_name, request_data_relay_type_name);
  if (request_data_relay_factory) {
    try {
      service_bridges_2_to_1.push_back(
        request_data_relay_factory->service_bridge_2_to_1(ros1_node, ros2_node, request_data_relay_service_name));
      printf("Created 2 to 1 bridge for service %s\n", request_data_relay_service_name.c_str());
    } catch (std::runtime_error & e) {
      fprintf(
        stderr,
        "failed to create bridge ROS 2 service '%s' "
        "with package '%s' and type '%s': %s\n",
        request_data_relay_service_name.c_str(), request_data_relay_type_name.c_str(), request_data_relay_type_name.c_str(), e.what());
    }
  } else {
    fprintf(
      stderr,
      "failed to create bridge ROS 2 service '%s' "
      "no conversion for package '%s' and type '%s'\n",
      request_data_relay_service_name.c_str(), request_data_relay_package_name.c_str(), request_data_relay_type_name.c_str());
  }

  std::string set_camera_info_service_name = "/pylon_camera_node/set_camera_info";
  std::string set_camera_info_package_name = "std_srvs";
  std::string set_camera_info_type_name = "SetCameraInfo";
  printf(
    "Trying to create bridge for ROS 1 service '%s' "
    "with package '%s' and type '%s'\n",
    set_camera_info_service_name.c_str(), set_camera_info_package_name.c_str(), set_camera_info_type_name.c_str());

  auto set_camera_info_factory = ros1_bridge::get_service_factory(
    "ros1", set_camera_info_package_name, set_camera_info_type_name);
  if (set_camera_info_factory) {
    try {
      service_bridges_2_to_1.push_back(
        set_camera_info_factory->service_bridge_2_to_1(ros1_node, ros2_node, set_camera_info_service_name));
      printf("Created 2 to 1 bridge for service %s\n", set_camera_info_service_name.c_str());
    } catch (std::runtime_error & e) {
      fprintf(
        stderr,
        "failed to create bridge ROS 2 service '%s' "
        "with package '%s' and type '%s': %s\n",
        set_camera_info_service_name.c_str(), set_camera_info_type_name.c_str(), set_camera_info_type_name.c_str(), e.what());
    }
  } else {
    fprintf(
      stderr,
      "failed to create bridge ROS 2 service '%s' "
      "no conversion for package '%s' and type '%s'\n",
      set_camera_info_service_name.c_str(), set_camera_info_package_name.c_str(), set_camera_info_type_name.c_str());
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
