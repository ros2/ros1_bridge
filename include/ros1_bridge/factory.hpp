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
#include <utility>
#include <vector>

#include "rmw/rmw.h"
#include "rclcpp/rclcpp.hpp"

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
  {
    ts_lib_ = rclcpp::get_typesupport_library(ros2_type_name, "rosidl_typesupport_cpp");
    if (static_cast<bool>(ts_lib_)) {
      type_support_ = rclcpp::get_typesupport_handle(
        ros2_type_name, "rosidl_typesupport_cpp",
        *ts_lib_);
    }
  }

  ros::Publisher
  create_ros1_publisher(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    bool latch = false)
  {
    return node.advertise<ROS1_T>(topic_name, queue_size, latch);
  }

  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    size_t queue_size)
  {
    return node->create_publisher<ROS2_T>(topic_name, rclcpp::QoS(rclcpp::KeepLast(queue_size)));
  }

  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepAll());
    qos.get_rmw_qos_profile() = qos_profile;
    return create_ros2_publisher(node, topic_name, qos);
  }

  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rclcpp::QoS & qos)
  {
    return node->create_publisher<ROS2_T>(topic_name, qos);
  }

  ros::Subscriber
  create_ros1_subscriber(
    ros::NodeHandle node,
    const std::string & topic_name,
    size_t queue_size,
    rclcpp::PublisherBase::SharedPtr ros2_pub,
    rclcpp::Logger logger)
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
          boost::placeholders::_1, ros2_pub, ros1_type_name_, ros2_type_name_, logger)));
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
    auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
    return create_ros2_subscriber(node, topic_name, qos, ros1_pub, ros2_pub);
  }

  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rmw_qos_profile_t & qos,
    ros::Publisher ros1_pub,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr)
  {
    auto rclcpp_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos));
    rclcpp_qos.get_rmw_qos_profile() = qos;
    return create_ros2_subscriber(
      node, topic_name, rclcpp_qos, ros1_pub, ros2_pub);
  }

  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    ros::Publisher ros1_pub,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr)
  {
    std::function<
      void(const typename ROS2_T::SharedPtr msg, const rclcpp::MessageInfo & msg_info)> callback;
    callback = std::bind(
      &Factory<ROS1_T, ROS2_T>::ros2_callback, std::placeholders::_1, std::placeholders::_2,
      ros1_pub, ros1_type_name_, ros2_type_name_, node->get_logger(), ros2_pub);
    rclcpp::SubscriptionOptions options;
    options.ignore_local_publications = true;
    return node->create_subscription<ROS2_T>(
      topic_name, qos, callback, options);
  }

  void convert_1_to_2(const void * ros1_msg, void * ros2_msg) const override
  {
    auto typed_ros1_msg = static_cast<const ROS1_T *>(ros1_msg);
    auto typed_ros2_msg = static_cast<ROS2_T *>(ros2_msg);
    convert_1_to_2(*typed_ros1_msg, *typed_ros2_msg);
  }

  void convert_2_to_1(const void * ros2_msg, void * ros1_msg) const override
  {
    auto typed_ros2_msg = static_cast<const ROS2_T *>(ros2_msg);
    auto typed_ros1_msg = static_cast<ROS1_T *>(ros1_msg);
    convert_2_to_1(*typed_ros2_msg, *typed_ros1_msg);
  }

protected:
  static
  void ros1_callback(
    const ros::MessageEvent<ROS1_T const> & ros1_msg_event,
    rclcpp::PublisherBase::SharedPtr ros2_pub,
    const std::string & ros1_type_name,
    const std::string & ros2_type_name,
    rclcpp::Logger logger)
  {
    typename rclcpp::Publisher<ROS2_T>::SharedPtr typed_ros2_pub;
    typed_ros2_pub =
      std::dynamic_pointer_cast<typename rclcpp::Publisher<ROS2_T>>(ros2_pub);

    if (!typed_ros2_pub) {
      throw std::runtime_error(
              "Invalid type " + ros2_type_name + " for ROS 2 publisher " +
              ros2_pub->get_topic_name());
    }

    const boost::shared_ptr<ros::M_string> & connection_header =
      ros1_msg_event.getConnectionHeaderPtr();
    if (!connection_header) {
      RCLCPP_WARN(
        logger, "Dropping ROS 1 message %s without connection header", ros1_type_name.c_str());
      return;
    }

    std::string key = "callerid";
    if (connection_header->find(key) != connection_header->end()) {
      if (connection_header->at(key) == "/ros_bridge") {
        return;
      }
    }

    const boost::shared_ptr<ROS1_T const> & ros1_msg = ros1_msg_event.getConstMessage();

    auto ros2_msg = std::make_unique<ROS2_T>();
    convert_1_to_2(*ros1_msg, *ros2_msg);
    RCLCPP_INFO_ONCE(
      logger, "Passing message from ROS 1 %s to ROS 2 %s (showing msg only once per type)",
      ros1_type_name.c_str(), ros2_type_name.c_str());
    typed_ros2_pub->publish(std::move(ros2_msg));
  }

  static
  void ros2_callback(
    typename ROS2_T::SharedPtr ros2_msg,
    const rclcpp::MessageInfo & msg_info,
    ros::Publisher ros1_pub,
    const std::string & ros1_type_name,
    const std::string & ros2_type_name,
    rclcpp::Logger logger,
    rclcpp::PublisherBase::SharedPtr ros2_pub = nullptr)
  {
    if (ros2_pub) {
      bool result = false;
      auto ret = rmw_compare_gids_equal(
        &msg_info.get_rmw_message_info().publisher_gid,
        &ros2_pub->get_gid(),
        &result);
      if (ret == RMW_RET_OK) {
        if (result) {  // message GID equals to bridge's ROS2 publisher GID
          return;  // do not publish messages from bridge itself
        }
      } else {
        auto msg = std::string("Failed to compare gids: ") + rmw_get_error_string().str;
        rmw_reset_error();
        throw std::runtime_error(msg);
      }
    }

    void * ptr = ros1_pub;
    if (ptr == 0) {
      RCLCPP_WARN_ONCE(
        logger,
        "Message from ROS 2 %s failed to be passed to ROS 1 %s because the "
        "ROS 1 publisher is invalid (showing msg only once per type)",
        ros2_type_name.c_str(), ros1_type_name.c_str());
      return;
    }

    ROS1_T ros1_msg;
    convert_2_to_1(*ros2_msg, ros1_msg);
    RCLCPP_INFO_ONCE(
      logger, "Passing message from ROS 2 %s to ROS 1 %s (showing msg only once per type)",
      ros2_type_name.c_str(), ros1_type_name.c_str());
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


  const char * get_ros1_md5sum() const override
  {
    return ros::message_traits::MD5Sum<ROS1_T>::value();
  }

  const char * get_ros1_data_type() const override
  {
    return ros::message_traits::DataType<ROS1_T>::value();
  }

  const char * get_ros1_message_definition() const override
  {
    return ros::message_traits::Definition<ROS1_T>::value();
  }

  bool convert_2_to_1_generic(
    const rclcpp::SerializedMessage & ros2_msg,
    std::vector<uint8_t> & ros1_msg) const override
  {
    if (type_support_ == nullptr) {
      return false;
    }

    // Deserialize to a ROS2 message
    ROS2_T ros2_typed_msg;
    if (rmw_deserialize(
        &ros2_msg.get_rcl_serialized_message(), type_support_,
        &ros2_typed_msg) != RMW_RET_OK)
    {
      return false;
    }

    // Call convert_2_to_1
    ROS1_T ros1_typed_msg;
    convert_2_to_1(&ros2_typed_msg, &ros1_typed_msg);

    // Serialize the ROS1 message into a buffer
    uint32_t length = ros::serialization::serializationLength(ros1_typed_msg);
    ros1_msg.resize(length);
    ros::serialization::OStream out_stream(ros1_msg.data(), length);
    ros::serialization::serialize(out_stream, ros1_typed_msg);

    return true;
  }

  bool convert_1_to_2_generic(
    const std::vector<uint8_t> & ros1_msg,
    rclcpp::SerializedMessage & ros2_msg) const override
  {
    if (type_support_ == nullptr) {
      return false;
    }

    // Deserialize to a ROS1 message
    ROS1_T ros1_typed_msg;
    // Both IStream and OStream inherits their functionality from Stream
    // So IStream needs a non-const data reference to data
    // However deserialization function probably shouldn't modify data they are serializing from
    uint8_t * ros1_msg_data = const_cast<uint8_t *>(ros1_msg.data());
    ros::serialization::IStream in_stream(ros1_msg_data, ros1_msg.size());
    ros::serialization::deserialize(in_stream, ros1_typed_msg);

    // Call convert_1_to_2
    ROS2_T ros2_typed_msg;
    convert_1_to_2(&ros1_typed_msg, &ros2_typed_msg);

    // Serialize ROS2 message
    if (rmw_serialize(
        &ros2_typed_msg, type_support_,
        &ros2_msg.get_rcl_serialized_message()) != RMW_RET_OK)
    {
      return false;
    }

    return true;
  }

  /**
   * @brief Writes (serializes) a ROS2 class directly to a ROS1 stream
   */
  static void convert_2_to_1(const ROS2_T & msg, ros::serialization::OStream & out_stream);

  /**
   * @brief Reads (deserializes) a ROS2 class directly from a ROS1 stream
   */
  static void convert_1_to_2(ros::serialization::IStream & in_stream, ROS2_T & msg);

  /**
   * @brief Determines the length of a ROS2 class if it was serialized to a ROS1 stream
   */
  static uint32_t length_2_as_1_stream(const ROS2_T & msg);

  /**
   * @brief Internal helper functions conversion for ROS2 message types to/from ROS1 streams
   *
   * This function is not meant to be used externally. However, since this the internal helper
   * functions call each other for sub messages they must be public.
   */
  static void internal_stream_translate_helper(
    ros::serialization::OStream & stream,
    const ROS2_T & msg);
  static void internal_stream_translate_helper(
    ros::serialization::IStream & stream,
    ROS2_T & msg);
  static void internal_stream_translate_helper(
    ros::serialization::LStream & stream,
    const ROS2_T & msg);

  std::string ros1_type_name_;
  std::string ros2_type_name_;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  const rosidl_message_type_support_t * type_support_ = nullptr;
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
    ros::ServiceClient client, rclcpp::Logger logger, const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<ROS2Request> request, std::shared_ptr<ROS2Response> response)
  {
    ROS1_T srv;
    translate_2_to_1(*request, srv.request);
    if (client.call(srv)) {
      translate_1_to_2(srv.response, *response);
    } else {
      throw std::runtime_error("Failed to get response from ROS 1 service " + client.getService());
    }
  }

  bool forward_1_to_2(
    rclcpp::ClientBase::SharedPtr cli, rclcpp::Logger logger,
    const ROS1Request & request1, ROS1Response & response1,
    int service_execution_timeout)
  {
    auto client = std::dynamic_pointer_cast<rclcpp::Client<ROS2_T>>(cli);
    if (!client) {
      RCLCPP_ERROR(logger, "Failed to get ROS 2 client %s", cli->get_service_name());
      return false;
    }
    auto request2 = std::make_shared<ROS2Request>();
    translate_1_to_2(request1, *request2);
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          logger, "Interrupted while waiting for ROS 2 service %s", cli->get_service_name());
        return false;
      }
      RCLCPP_WARN(logger, "Waiting for ROS 2 service %s...", cli->get_service_name());
    }
    auto timeout = std::chrono::seconds(service_execution_timeout);
    auto future = client->async_send_request(request2);
    auto status = future.wait_for(timeout);
    if (status == std::future_status::ready) {
      auto response2 = future.get();
      translate_2_to_1(*response2, response1);
    } else {
      RCLCPP_ERROR(
        logger, "Failed to get response from ROS 2 service %s within %d seconds",
        cli->get_service_name(), service_execution_timeout);
      return false;
    }
    return true;
  }

  ServiceBridge1to2 service_bridge_1_to_2(
    ros::NodeHandle & ros1_node, rclcpp::Node::SharedPtr ros2_node, const std::string & name,
    int service_execution_timeout)
  {
    ServiceBridge1to2 bridge;
    bridge.client = ros2_node->create_client<ROS2_T>(name);
    auto m = &ServiceFactory<ROS1_T, ROS2_T>::forward_1_to_2;
    auto f = std::bind(
      m, this, bridge.client, ros2_node->get_logger(), std::placeholders::_1,
      std::placeholders::_2, service_execution_timeout);
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
      m, this, bridge.client, ros2_node->get_logger(), std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3);
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
