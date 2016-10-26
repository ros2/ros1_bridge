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
#include <string>

// include ROS 1 message event
#include "ros/message.h"

#include "ros1_bridge/factory_interface.hpp"

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

  rclcpp::publisher::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::node::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size)
  {
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = queue_size;
    return node->create_publisher<ROS2_T>(topic_name, custom_qos_profile);
  }

  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::publisher::PublisherBase::SharedPtr ros2_pub)
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
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = queue_size;
    // TODO(wjwwood): use a lambda until create_subscription supports std/boost::bind.
    auto callback = [this, ros1_pub](const typename ROS2_T::SharedPtr msg) {
      return this->ros2_callback(msg, ros1_pub);
    };
    return node->create_subscription<ROS2_T>(
      topic_name, callback, custom_qos_profile, nullptr, true);
  }

protected:
  static
  void ros1_callback(
    const ros::MessageEvent<ROS1_T const> & ros1_msg_event,
    rclcpp::publisher::PublisherBase::SharedPtr ros2_pub
    )
  {
    typename rclcpp::publisher::Publisher<ROS2_T>::SharedPtr typed_ros2_pub;
    typed_ros2_pub =
      std::dynamic_pointer_cast<typename rclcpp::publisher::Publisher<ROS2_T>>(ros2_pub);

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
    printf("  Passing message from ROS 1 to ROS 2\n");
    typed_ros2_pub->publish(ros2_msg);
  }

  static
  void ros2_callback(
    typename ROS2_T::SharedPtr ros2_msg,
    ros::Publisher ros1_pub
    )
  {
    ROS1_T ros1_msg;
    convert_2_to_1(*ros2_msg, ros1_msg);
    printf("  Passing message from ROS 2 to ROS 1\n");
    ros1_pub.publish(ros1_msg);
  }

// since convert functions call each other for sub messages they must be public
public:
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
};

template <class R1_TYPE, class R1_REQ, class R1_RES, class R2_TYPE, class R2_REQ, class R2_RES>
class ServiceFactory : public ServiceFactoryInterface
{
private:

  void translate_1_to_2(const R1_REQ&, R2_REQ&);
  void translate_1_to_2(const R1_RES&, R2_RES&);
  void translate_2_to_1(R1_REQ&, const R2_REQ&);
  void translate_2_to_1(R1_RES&, const R2_RES&);

public:
  void forward_2_to_1(
    ros::ServiceClient client, const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<R2_REQ> request, std::shared_ptr<R2_RES> response)
  {
    R1_TYPE srv;
    translate_2_to_1(srv.request, *request);
    if (client.call(srv))
    {
      translate_1_to_2(srv.response, *response);
    }
    else
    {
      throw std::runtime_error("Failed to get response from ROS2 service");
    }
  }

  bool forward_1_to_2(
    rclcpp::client::ClientBase::SharedPtr cli, const R1_REQ& request1, R1_RES& response1)
  {
    auto client = std::dynamic_pointer_cast<rclcpp::client::Client<R2_TYPE>>(cli);
    if (!client)
    {
      printf("Failed to get the client.\n");
      return false;
    }
    auto request2 = std::make_shared<R2_REQ>();
    translate_1_to_2(request1, *request2);
    while (!client->wait_for_service(1_s))
    {
      if (!rclcpp::ok())
      {
        printf("Client was interrupted while waiting for the service. Exiting.\n");
        return false;
      }
    }
    auto timeout = std::chrono::seconds(5);
    auto future = client->async_send_request(request2);
    auto status = future.wait_for(timeout);
    if (status == std::future_status::ready)
    {
      auto response2 = future.get();
      translate_2_to_1(response1, *response2);
    }
    else
    {
        printf("Failed to get response from ROS2 service.\n");
        return false;
    }
    return true;
  }

  ServiceBridge1to2 service_bridge_1_to_2(
    ros::NodeHandle& ros1_node, rclcpp::Node::SharedPtr ros2_node, const std::string name)
  {
    ServiceBridge1to2 bridge;
    bridge.client = ros2_node->create_client<R2_TYPE>(name);
    auto m = &ServiceFactory<R1_TYPE,R1_REQ,R1_RES,R2_TYPE,R2_REQ,R2_RES>::forward_1_to_2;
    auto f = boost::bind(m, this, bridge.client, _1, _2);
    bridge.server = ros1_node.advertiseService<R1_REQ,R1_RES>(name, f);
    return bridge;
  }

  ServiceBridge2to1 service_bridge_2_to_1(
     ros::NodeHandle& ros1_node, rclcpp::Node::SharedPtr ros2_node, const std::string name)
  {
    ServiceBridge2to1 bridge;
    bridge.client = ros1_node.serviceClient<R1_TYPE>(name);
    auto m = &ServiceFactory<R1_TYPE,R1_REQ,R1_RES,R2_TYPE,R2_REQ,R2_RES>::forward_2_to_1;
    boost::function<void(const std::shared_ptr<rmw_request_id_t>,const std::shared_ptr<R2_REQ>,std::shared_ptr<R2_RES>)> f;
    f = boost::bind(m, this, bridge.client, _1, _2, _3);
    bridge.server = ros2_node->create_service<R2_TYPE>(name, f);
    return bridge;
  }
};

}  // namespace ros1_bridge

#endif  // ROS1_BRIDGE__FACTORY_HPP_
