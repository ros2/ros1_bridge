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
#include <memory>
#include <string>

#include "rmw/rmw.h"

// include ROS 1 message event
#include "ros/message.h"

#include "rcutils/logging_macros.h"

#include "ros1_bridge/factory_interface.hpp"

namespace ros1_bridge
{

template<typename ROS1_T, typename ROS2_T>
class Factory : public FactoryInterface
{
public:
  Factory(
    const std::string & ros1_type_name, const std::string & ros2_type_name)
  : ros1_type_name_(ros1_type_name),
    ros2_type_name_(ros2_type_name)
  {}

  ros::Publisher
  create_ros1_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size)
  {
    return node.advertise<ROS1_T>(topic_name, queue_size);
  }

  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
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
    rclcpp::PublisherBase::SharedPtr ros2_pub)
  {
    // workaround for https://github.com/ros/roscpp_core/issues/22 to get the connection header
    ros::SubscribeOptions ops;
    ops.topic = topic_name;
    ops.queue_size = queue_size;
    ops.md5sum = ros::message_traits::md5sum<ROS1_T>();
    ops.datatype = ros::message_traits::datatype<ROS1_T>();
    ops.helper = ros::SubscriptionCallbackHelperPtr(
      new ros::SubscriptionCallbackHelperT<const ros::MessageEvent<ROS1_T const> &>(
        boost::bind(
          &Factory<ROS1_T, ROS2_T>::ros1_callback,
          _1, ros2_pub, ros1_type_name_, ros2_type_name_)));
    return node.subscribe(ops);
  }

  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size,
    ros::Publisher ros1_pub,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr)
  {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = queue_size;
    const std::string & ros1_type_name = ros1_type_name_;
    const std::string & ros2_type_name = ros2_type_name_;
    // TODO(wjwwood): use a lambda until create_subscription supports std/boost::bind.
    auto callback =
      [this, ros1_pub, ros1_type_name, ros2_type_name,
      ros2_pub](const typename ROS2_T::SharedPtr msg, const rmw_message_info_t & msg_info) {
        return this->ros2_callback(
          msg, msg_info, ros1_pub, ros1_type_name, ros2_type_name, ros2_pub);
      };
    return node->create_subscription<ROS2_T>(
      topic_name, callback, custom_qos_profile, nullptr, true);
  }

protected:
  static
  void ros1_callback(
    const ros::MessageEvent<ROS1_T const> & ros1_msg_event,
    rclcpp::PublisherBase::SharedPtr ros2_pub,
    const std::string & ros1_type_name,
    const std::string & ros2_type_name)
  {
    typename rclcpp::Publisher<ROS2_T>::SharedPtr typed_ros2_pub;
    typed_ros2_pub =
      std::dynamic_pointer_cast<typename rclcpp::Publisher<ROS2_T>>(ros2_pub);

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
    RCUTILS_LOG_INFO_ONCE_NAMED(
      "ros1_bridge",
      "Passing message from ROS 1 %s to ROS 2 %s (showing msg only once per type)",
      ros1_type_name.c_str(), ros2_type_name.c_str());
    typed_ros2_pub->publish(ros2_msg);
  }

  static
  void ros2_callback(
    typename ROS2_T::SharedPtr ros2_msg,
    const rmw_message_info_t & msg_info,
    ros::Publisher ros1_pub,
    const std::string & ros1_type_name,
    const std::string & ros2_type_name,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr)
  {
    if (ros2_pub) {
      bool result = false;
      auto ret = rmw_compare_gids_equal(&msg_info.publisher_gid, &ros2_pub->get_gid(), &result);
      if (ret == RMW_RET_OK) {
        if (result) {  // message GID equals to bridge's ROS2 publisher GID
          return;  // do not publish messages from bridge itself
        }
      } else {
        auto msg = std::string("Failed to compare gids: ") + rmw_get_error_string();
        rmw_reset_error();
        throw std::runtime_error(msg);
      }
    }

    ROS1_T ros1_msg;
    convert_2_to_1(*ros2_msg, ros1_msg);
    RCUTILS_LOG_INFO_ONCE_NAMED(
      "ros1_bridge",
      "Passing message from ROS 2 %s to ROS 1 %s (showing msg only once per type)",
      ros1_type_name.c_str(), ros2_type_name.c_str());
    ros1_pub.publish(ros1_msg);
  }

public:
  // since convert functions call each other for sub messages they must be public
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

  std::string ros1_type_name_;
  std::string ros2_type_name_;
};

template<class ROS1_T, class ROS2_T>
class ServiceFactory : public ServiceFactoryInterface
{
public:
  using ROS1Request = typename ROS1_T::Request;
  using ROS2Request = typename ROS2_T::Request;
  using ROS1Response = typename ROS1_T::Response;
  using ROS2Response = typename ROS2_T::Response;

  void forward_2_to_1(
    ros::ServiceClient client, const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<ROS2Request> request, std::shared_ptr<ROS2Response> response)
  {
    ROS1_T srv;
    translate_2_to_1(*request, srv.request);
    if (client.call(srv)) {
      translate_1_to_2(srv.response, *response);
    } else {
      throw std::runtime_error("Failed to get response from ROS service");
    }
  }

  bool forward_1_to_2(
    rclcpp::ClientBase::SharedPtr cli,
    const ROS1Request & request1, ROS1Response & response1)
  {
    auto client = std::dynamic_pointer_cast<rclcpp::Client<ROS2_T>>(cli);
    if (!client) {
      fprintf(stderr, "Failed to get the client.\n");
      return false;
    }
    auto request2 = std::make_shared<ROS2Request>();
    translate_1_to_2(request1, *request2);
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        fprintf(stderr, "Client was interrupted while waiting for the service. Exiting.\n");
        return false;
      }
    }
    auto timeout = std::chrono::seconds(5);
    auto future = client->async_send_request(request2);
    auto status = future.wait_for(timeout);
    if (status == std::future_status::ready) {
      auto response2 = future.get();
      translate_2_to_1(*response2, response1);
    } else {
      fprintf(stderr, "Failed to get response from ROS2 service.\n");
      return false;
    }
    return true;
  }

  ServiceBridge1to2 service_bridge_1_to_2(
    ros::NodeHandle & ros1_node, rclcpp::Node::SharedPtr ros2_node, const std::string & name)
  {
    ServiceBridge1to2 bridge;
    bridge.client = ros2_node->create_client<ROS2_T>(name);
    auto m = &ServiceFactory<ROS1_T, ROS2_T>::forward_1_to_2;
    auto f = std::bind(m, this, bridge.client, std::placeholders::_1, std::placeholders::_2);
    bridge.server = ros1_node.advertiseService<ROS1Request, ROS1Response>(name, f);
    return bridge;
  }

  ServiceBridge2to1 service_bridge_2_to_1(
    ros::NodeHandle & ros1_node, rclcpp::Node::SharedPtr ros2_node, const std::string & name)
  {
    ServiceBridge2to1 bridge;
    bridge.client = ros1_node.serviceClient<ROS1_T>(name);
    auto m = &ServiceFactory<ROS1_T, ROS2_T>::forward_2_to_1;
    std::function<
      void(
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<ROS2Request>,
        std::shared_ptr<ROS2Response>)> f;
    f = std::bind(
      m, this, bridge.client, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    bridge.server = ros2_node->create_service<ROS2_T>(name, f);
    return bridge;
  }

private:
  void translate_1_to_2(const ROS1Request &, ROS2Request &);
  void translate_1_to_2(const ROS1Response &, ROS2Response &);
  void translate_2_to_1(const ROS2Request &, ROS1Request &);
  void translate_2_to_1(const ROS2Response &, ROS1Response &);
};

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__FACTORY_HPP_
