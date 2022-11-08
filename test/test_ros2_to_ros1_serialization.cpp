// Copyright 2022 Open Source Robotics Foundation, Inc.
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

// GTest
#include <gtest/gtest.h>

// ROS1 Messsages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

// C++ Standard Library
#include <string>
#include <type_traits>
#include <tuple>

// ros1_bridge
#include "ros1_bridge/bridge.hpp"
#include "ros1_bridge/factory.hpp"

// RCLCPP
#include "rclcpp/serialized_message.hpp"

// ROS2 Messsages
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>


template<typename ROS1_T_, typename ROS2_T_>
struct GenericTestBase
{
  using ROS1_T = ROS1_T_;
  using ROS2_T = ROS2_T_;
  using FACTORY_T = ros1_bridge::Factory<ROS1_T, ROS2_T>;

  FACTORY_T factory;
  ROS1_T ros1_msg;
  ROS2_T ros2_msg;
  GenericTestBase(const std::string & ros1_type_name, const std::string & ros2_type_name)
  : factory(ros1_type_name, ros2_type_name) {}

  // Generate serialized buffer from ROS1 message
  static std::vector<uint8_t> generateRos1SerializedMessage(const ROS1_T & ros1_msg)
  {
    // Serialize ROS1 message
    const uint32_t length = ros::serialization::serializationLength(ros1_msg);
    std::vector<uint8_t> buffer(length);
    ros::serialization::OStream out_stream(buffer.data(), length);
    ros::serialization::serialize(out_stream, ros1_msg);
    return buffer;
  }

  // Generate SerializedMessage from a ROS2 message
  rclcpp::SerializedMessage generateRos2SerializedMessage(const ROS2_T & ros2_msg)
  {
    // Directly serialize ROS2 message
    rclcpp::SerializedMessage serialized_msg;
    auto ret = rmw_serialize(
      &ros2_msg, factory.type_support_,
      &serialized_msg.get_rcl_serialized_message());
    EXPECT_EQ(RMW_RET_OK, ret);
    return serialized_msg;
  }

  static bool compare(const ROS2_T &, const ROS2_T &)
  {
    return false;
  }
};

template<typename T>
bool compareXYZ(const T & a, const T & b)
{
  return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}

template<typename T>
bool compareXYZW(const T & a, const T & b)
{
  return (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && (a.w == b.w);
}

bool compareTime(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b)
{
  return (a.sec == b.sec) && (a.nanosec == b.nanosec);
}

bool compareDuration(
  const builtin_interfaces::msg::Duration & a,
  const builtin_interfaces::msg::Duration & b)
{
  return (a.sec == b.sec) && (a.nanosec == b.nanosec);
}

bool compareHeader(
  const std_msgs::msg::Header & a,
  const std_msgs::msg::Header & b)
{
  return compareTime(a.stamp, b.stamp) && (a.frame_id == b.frame_id);
}

bool comparePose(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  return compareXYZ(a.position, b.position) && compareXYZW(a.orientation, b.orientation);
}

bool comparePoseArray(
  const geometry_msgs::msg::PoseArray & a,
  const geometry_msgs::msg::PoseArray & b)
{
  if (!compareHeader(a.header, b.header)) {
    return false;
  }
  if (a.poses.size() != b.poses.size()) {
    std::cerr << "a.poses.size() != b.poses.size()" << a.poses.size() << " != " << b.poses.size() <<
      std::endl;
    return false;
  }
  for (size_t idx = 0; idx < a.poses.size(); ++idx) {
    if (!comparePose(a.poses.at(idx), b.poses.at(idx))) {
      return false;
    }
  }
  return true;
}

struct Vector3Test : public GenericTestBase<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>
{
  Vector3Test()
  : GenericTestBase("geometry_msgs/Vector3", "geometry_msgs/msg/Vector3")
  {
    ros1_msg.x = ros2_msg.x = 1.1;
    ros1_msg.y = ros2_msg.y = 2.2;
    ros1_msg.z = ros2_msg.z = 3.3;
  }

  static bool compare(
    const geometry_msgs::msg::Vector3 & a,
    const geometry_msgs::msg::Vector3 & b)
  {
    return compareXYZ(a, b);
  }
};


struct StringTestEmpty : public GenericTestBase<std_msgs::String,
    std_msgs::msg::String>
{
  StringTestEmpty()
  : GenericTestBase("std_msgs/String", "std_msgs/msg/String") {}

  static bool compare(
    const std_msgs::msg::String & a,
    const std_msgs::msg::String & b)
  {
    return a.data == b.data;
  }
};


struct StringTestHello : public StringTestEmpty
{
  StringTestHello()
  {
    ros1_msg.data = ros2_msg.data = "hello";
  }
};

struct TimeTest : public GenericTestBase<
    std_msgs::Time,
    builtin_interfaces::msg::Time>
{
  TimeTest()
  : GenericTestBase("std_msgs/Time", "builtin_interfaces/msg/Time")
  {
    ros1_msg.data.sec = ros2_msg.sec = 1000 * 2000;
    ros1_msg.data.nsec = ros2_msg.nanosec = 1000 * 1000 * 1000;
  }

  static bool compare(
    const builtin_interfaces::msg::Time & a,
    const builtin_interfaces::msg::Time & b)
  {
    return compareTime(a, b);
  }
};

struct DurationTest : public GenericTestBase<
    std_msgs::Duration,
    builtin_interfaces::msg::Duration>
{
  DurationTest()
  : GenericTestBase("std_msgs/Duration", "builtin_interfaces/msg/Duration")
  {
    ros1_msg.data.sec = ros2_msg.sec = 1001 * 2001;
    ros1_msg.data.nsec = ros2_msg.nanosec = 1002 * 1003 * 1004;
  }

  static bool compare(
    const builtin_interfaces::msg::Duration & a,
    const builtin_interfaces::msg::Duration & b)
  {
    return compareDuration(a, b);
  }
};

struct HeaderTestEmpty : public GenericTestBase<
    std_msgs::Header,
    std_msgs::msg::Header>
{
  HeaderTestEmpty()
  : GenericTestBase("std_msgs/Header", "std_msgs/msg/Header")
  {
    ros1_msg.stamp.sec = ros2_msg.stamp.sec = 100 * 100;
    ros1_msg.stamp.nsec = ros2_msg.stamp.nanosec = 100 * 200 * 300;
    ros1_msg.seq = 0;
    // Leave header.seq as zero, ros2_msg header does not have seq number so generic
    // serialization function will always write a zero to output stream
  }

  static bool compare(
    const std_msgs::msg::Header & a,
    const std_msgs::msg::Header & b)
  {
    return compareHeader(a, b);
  }
};

struct HeaderTestBaseLink : public HeaderTestEmpty
{
  HeaderTestBaseLink()
  {
    ros1_msg.frame_id = ros2_msg.frame_id = "base_link";
  }

  static bool compare(
    const std_msgs::msg::Header & a,
    const std_msgs::msg::Header & b)
  {
    return compareHeader(a, b);
  }
};

struct PoseTest : public GenericTestBase<
    geometry_msgs::Pose,
    geometry_msgs::msg::Pose>
{
  PoseTest()
  : GenericTestBase("geometry_msgs/Pose", "geometry_msgs/msg/Pose")
  {
    ros1_msg.position.x = ros2_msg.position.x = 1.0;
    ros1_msg.position.y = ros2_msg.position.y = 2.0;
    ros1_msg.position.z = ros2_msg.position.z = 3.0;
    ros1_msg.orientation.x = ros2_msg.orientation.x = 4.0;
    ros1_msg.orientation.y = ros2_msg.orientation.y = 5.0;
    ros1_msg.orientation.z = ros2_msg.orientation.z = 6.0;
    ros1_msg.orientation.w = ros2_msg.orientation.w = 7.0;
  }

  static bool compare(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b)
  {
    return comparePose(a, b);
  }
};

struct PoseArrayTestEmpty : public GenericTestBase<
    geometry_msgs::PoseArray,
    geometry_msgs::msg::PoseArray>
{
  PoseArrayTestEmpty()
  : GenericTestBase("geometry_msgs/PoseArray", "geometry_msgs/msg/PoseArray") {}

  static bool compare(
    const geometry_msgs::msg::PoseArray & a,
    const geometry_msgs::msg::PoseArray & b)
  {
    return comparePoseArray(a, b);
  }
};

struct PoseArrayTest : public PoseArrayTestEmpty
{
  PoseArrayTest()
  {
    ros1_msg.header.frame_id = ros2_msg.header.frame_id = "base_link";
    ros1_msg.header.stamp.sec = ros2_msg.header.stamp.sec = 0x12345678;
    ros1_msg.header.stamp.nsec = ros2_msg.header.stamp.nanosec = 0x76543210;
    const size_t size = 10;
    ros1_msg.poses.resize(size);
    ros2_msg.poses.resize(size);
    for (size_t ii = 0; ii < size; ++ii) {
      auto & pose1 = ros1_msg.poses.at(ii);
      auto & pose2 = ros2_msg.poses.at(ii);
      pose1.orientation.x = pose2.orientation.x = 0.1 * ii;
      // By default ROS2 message orientation is 1.0, so need to set this
      pose1.orientation.w = pose2.orientation.w = 0.9 * ii;
    }
  }
};


template<typename TEST_T_>
class Ros2ToRos1SerializationTest : public testing::Test
{
public:
  using TEST_T = TEST_T_;
  using ROS1_T = typename TEST_T::ROS1_T;
  using ROS2_T = typename TEST_T::ROS2_T;
  using FACTORY_T = typename TEST_T::FACTORY_T;

  TEST_T test;

  void TestBody() {}
};


using Ros2ToRos1SerializationTypes = ::testing::Types<
  Vector3Test,  // 0
  StringTestEmpty,  // 1
  StringTestHello,  // 2
  TimeTest,  // 3
  DurationTest,  // 4
  HeaderTestEmpty,  // 5
  HeaderTestBaseLink,  // 6
  PoseTest,  // 7
  PoseArrayTestEmpty,  // 8
  PoseArrayTest  // 9
>;
TYPED_TEST_SUITE(Ros2ToRos1SerializationTest, Ros2ToRos1SerializationTypes);


// cppcheck-suppress syntaxError
TYPED_TEST(Ros2ToRos1SerializationTest, test_length)
{
  using FACTORY_T = typename TestFixture::FACTORY_T;
  using ROS1_T = typename TestFixture::ROS1_T;
  using ROS2_T = typename TestFixture::ROS2_T;
  const ROS1_T & ros1_msg = this->test.ros1_msg;
  const ROS2_T & ros2_msg = this->test.ros2_msg;
  uint32_t ros1_len = ros::serialization::serializationLength(ros1_msg);
  uint32_t ros2_len = FACTORY_T::length_2_as_1_stream(ros2_msg);
  EXPECT_EQ(ros1_len, ros2_len);
}


// cppcheck-suppress syntaxError
TYPED_TEST(Ros2ToRos1SerializationTest, test_get_factory)
{
  // Make sure get_factory still works with Header and other types
  const std::string & ros1_type_name = this->test.factory.ros1_type_name_;
  const std::string & ros2_type_name = this->test.factory.ros2_type_name_;
  std::shared_ptr<ros1_bridge::FactoryInterface> factory;
  EXPECT_NO_THROW(factory = ros1_bridge::get_factory(ros1_type_name, ros2_type_name));
  ASSERT_TRUE(static_cast<bool>(factory));
}


// cppcheck-suppress syntaxError
TYPED_TEST(Ros2ToRos1SerializationTest, test_deserialization)
{
  using FACTORY_T = typename TestFixture::FACTORY_T;
  using ROS1_T = typename TestFixture::ROS1_T;
  using ROS2_T = typename TestFixture::ROS2_T;
  const ROS1_T & ros1_msg = this->test.ros1_msg;
  const ROS2_T & ros2_msg = this->test.ros2_msg;
  const uint32_t ros1_len = ros::serialization::serializationLength(ros1_msg);
  std::vector<uint8_t> buffer(ros1_len);
  ros::serialization::OStream out_stream(buffer.data(), ros1_len);
  FACTORY_T::convert_2_to_1(ros2_msg, out_stream);
  ros::serialization::IStream in_stream(buffer.data(), ros1_len);
  ROS2_T ros2_msg2;
  FACTORY_T::convert_1_to_2(in_stream, ros2_msg2);
  EXPECT_TRUE(TestFixture::TEST_T::compare(ros2_msg, ros2_msg2));
}


// cppcheck-suppress syntaxError
TYPED_TEST(Ros2ToRos1SerializationTest, test_serialization)
{
  using FACTORY_T = typename TestFixture::FACTORY_T;
  using ROS1_T = typename TestFixture::ROS1_T;
  using ROS2_T = typename TestFixture::ROS2_T;
  const ROS1_T & ros1_msg = this->test.ros1_msg;
  const ROS2_T & ros2_msg = this->test.ros2_msg;
  const uint32_t ros1_len = ros::serialization::serializationLength(ros1_msg);
  const uint32_t ros2_len = FACTORY_T::length_2_as_1_stream(ros2_msg);
  ASSERT_EQ(ros1_len, ros2_len);
  const uint32_t len = ros1_len;

  // Serialize both streams and check byte-by-byte the value match
  const uint8_t guard_value = 0x42;

  std::vector<uint8_t> buffer1(len + 1);
  {
    buffer1.at(len) = guard_value;
    ros::serialization::OStream stream(buffer1.data(), len);
    EXPECT_EQ(stream.getLength(), len);
    ros::serialization::serialize(stream, ros1_msg);
    // after serializating all the "capacity" should be used up
    EXPECT_EQ(stream.getLength(), 0ul);
    // Hacky check to make sure serialization didn't run over the end of the buffer
    EXPECT_EQ(buffer1.at(len), guard_value);
  }

  std::vector<uint8_t> buffer2(len + 1);
  {
    buffer2.at(len) = guard_value;
    ros::serialization::OStream stream(buffer2.data(), len);
    EXPECT_EQ(stream.getLength(), len);
    FACTORY_T::convert_2_to_1(ros2_msg, stream);
    EXPECT_EQ(stream.getLength(), 0ul);
    EXPECT_EQ(buffer2.at(len), guard_value);
  }

  unsigned mismatch_count = 0;
  const unsigned max_mismatch_count = 0;
  for (size_t idx = 0; idx < len; ++idx) {
    unsigned val1 = buffer1.at(idx);
    unsigned val2 = buffer2.at(idx);
    EXPECT_EQ(val1, val2) << " idx=" << idx << " of " << len;
    if (val1 != val2) {
      ++mismatch_count;
      ASSERT_LE(mismatch_count, max_mismatch_count);
    }
  }
  ASSERT_EQ(mismatch_count, 0u);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
