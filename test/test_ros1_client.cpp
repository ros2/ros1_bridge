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

#include <chrono>
#include <thread>

#include "diagnostic_msgs/SelfTest.h"
#include "ros/ros.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ros1_bridge_test_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<diagnostic_msgs::SelfTest>(
    "ros1_bridge_test");
  client.waitForExistence();
  diagnostic_msgs::SelfTest request;
  if (client.call(request)) {
    if (request.response.id != "ros2") {
      throw std::runtime_error("Expected a response from ROS2");
    }
  } else {
    throw std::runtime_error("Failed to call service ros1_bridge_test");
  }
  return 0;
}
