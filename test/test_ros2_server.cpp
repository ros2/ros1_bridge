// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "diagnostic_msgs/srv/self_test.hpp"
#include "rclcpp/rclcpp.hpp"

void handler(
  const std::shared_ptr<rmw_request_id_t>/* request_header */,
  const std::shared_ptr<diagnostic_msgs::srv::SelfTest::Request>/* request */,
  std::shared_ptr<diagnostic_msgs::srv::SelfTest::Response> response)
{
  response->id = "ros2";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros1_bridge_test_server");
  auto server = node->create_service<diagnostic_msgs::srv::SelfTest>("ros1_bridge_test", handler);
  rclcpp::spin(node);
  return 0;
}
