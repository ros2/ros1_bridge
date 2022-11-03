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
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>

// C++ Standard Library
#include <string>
#include <type_traits>
#include <tuple>

// ros1_bridge
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
};

struct Vector3Test : public GenericTestBase<geometry_msgs::Vector3, geometry_msgs::msg::Vector3>
{
  Vector3Test()
  : GenericTestBase("geometry_msgs/Vector3", "geometry_msgs/msg/Vector3")
  {
    ros1_msg.x = ros2_msg.x = 1.1;
    ros1_msg.y = ros2_msg.y = 2.2;
    ros1_msg.z = ros2_msg.z = 3.3;
  }
};


struct StringTestEmpty : public GenericTestBase<std_msgs::String,
    std_msgs::msg::String>
{
  StringTestEmpty()
  : GenericTestBase("std_msgs/String", "std_msgs/msg/String") {}
};


struct StringTestHello : public StringTestEmpty
{
  StringTestHello()
  {
    ros1_msg.data = ros2_msg.data = "hello";
  }
};

struct TimeTest : public GenericTestBase<std_msgs::Time,
    builtin_interfaces::msg::Time>
{
  TimeTest()
  : GenericTestBase("std_msgs/Time", "builtin_interfaces/msg/Time")
  {
    ros1_msg.data.sec = ros2_msg.sec = 1000 * 2000;
    ros1_msg.data.nsec = ros2_msg.nanosec = 1000 * 1000 * 1000;
  }
};

struct HeaderTestEmpty : public GenericTestBase<std_msgs::Header,
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
};

struct HeaderTestBaseLink : public HeaderTestEmpty
{
  HeaderTestBaseLink()
  {
    ros1_msg.frame_id = ros2_msg.frame_id = "base_link";
  }
};


struct PoseTest : public GenericTestBase<geometry_msgs::Pose,
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
};

struct PoseArrayTestEmpty : public GenericTestBase<geometry_msgs::PoseArray,
    geometry_msgs::msg::PoseArray>
{
  PoseArrayTestEmpty()
  : GenericTestBase("geometry_msgs/PoseArray", "geometry_msgs/msg/PoseArray") {}
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
class ConvertGenericTest : public testing::Test
{
public:
  using TEST_T = TEST_T_;
  using ROS1_T = typename TEST_T::ROS1_T;
  using ROS2_T = typename TEST_T::ROS2_T;
  using FACTORY_T = typename TEST_T::FACTORY_T;

  TEST_T test;

  void TestBody()
  {
    // nothing
  }
};


using ConvertGenericTypes = ::testing::Types<
  Vector3Test,  // 0
  StringTestEmpty,  // 1
  StringTestHello,  // 2
  TimeTest,  // 3
  HeaderTestEmpty,  // 4
  HeaderTestBaseLink,  // 5
  PoseTest,  // 6
  PoseArrayTestEmpty,  // 7
  PoseArrayTest  // 8
>;
TYPED_TEST_SUITE(ConvertGenericTest, ConvertGenericTypes);


// cppcheck-suppress syntaxError
TYPED_TEST(ConvertGenericTest, test_factory_md5)
{
  TestFixture fixture;
  using ROS1_T = typename TestFixture::ROS1_T;
  EXPECT_EQ(
    std::string(fixture.test.factory.get_ros1_md5sum()),
    std::string(ros::message_traits::MD5Sum<ROS1_T>::value()));
}


// cppcheck-suppress syntaxError
TYPED_TEST(ConvertGenericTest, test_factory_data_type)
{
  TestFixture fixture;
  using ROS1_T = typename TestFixture::ROS1_T;
  EXPECT_EQ(
    std::string(fixture.test.factory.get_ros1_data_type()),
    std::string(ros::message_traits::DataType<ROS1_T>::value()));
}


TYPED_TEST(ConvertGenericTest, test_factory_msg_def)
{
  TestFixture fixture;
  using ROS1_T = typename TestFixture::ROS1_T;
  EXPECT_EQ(
    std::string(fixture.test.factory.get_ros1_message_definition()),
    std::string(ros::message_traits::Definition<ROS1_T>::value()));
}

// cppcheck-suppress syntaxError
TYPED_TEST(ConvertGenericTest, test_convert_2_to_1)
{
  // Directly serialize ROS2 message
  rclcpp::SerializedMessage serialized_msg =
    this->test.generateRos2SerializedMessage(this->test.ros2_msg);

  // Convert ROS2 SerializedMessage into ROS1 buffer
  // This is the function-under-test
  std::vector<uint8_t> buffer2;
  bool success = this->test.factory.convert_2_to_1_generic(serialized_msg, buffer2);
  ASSERT_TRUE(success);

  // Write ROS1 message (which has same fields values) into a different stream
  std::vector<uint8_t> buffer1 =
    TestFixture::TEST_T::generateRos1SerializedMessage(this->test.ros1_msg);

  // Buffer1 and Buffer2 should match in size and contents
  // ROS1 serialization this should always be true because there is no padding or empty space
  // left in any output buffers
  ASSERT_EQ(buffer1.size(), buffer2.size());

  // The Gtest output from comparing buffers directly is a little hard to
  // understand when there is a few mismatching value
  // Instead use custom loop to make each mismatched byte easier to understand
  // ASSERT_EQ(buffer1, buffer2);
  unsigned mismatch_count = 0;
  const unsigned mismatch_count_limit = 10;
  for (size_t idx = 0; idx < buffer1.size(); ++idx) {
    int val1 = buffer1.at(idx);
    int val2 = buffer2.at(idx);
    EXPECT_EQ(val1, val2) << " idx=" << idx << " of " << buffer1.size();
    if (val1 != val2) {
      ++mismatch_count;
    }
    ASSERT_LE(
      mismatch_count,
      mismatch_count_limit) << " stopping comparison after " << mismatch_count_limit <<
      " mismatches";
  }
  ASSERT_EQ(mismatch_count, 0u) << " the output buffers should be exactly the same";
}


// cppcheck-suppress syntaxError
TYPED_TEST(ConvertGenericTest, test_convert_1_to_2_to_1)
{
  // Serialize ROS1 message into a ShapeShifter
  std::vector<uint8_t> buffer1 =
    TestFixture::TEST_T::generateRos1SerializedMessage(this->test.ros1_msg);

  // Convert ROS1 shape-shifter into ROS2 SerializedMessage
  // This is the function-under-test
  rclcpp::SerializedMessage serialized_msg;
  bool success = this->test.factory.convert_1_to_2_generic(buffer1, serialized_msg);
  ASSERT_TRUE(success);

  // The serialized data from matching ROS2 message seems to contain padding between
  // values.  This padding might have random values so the serialized data
  // might different even though the message fields all have the same value.
  // However ROS1 messages are "packed" so there is no space for garbage data.
  // Since convert_1_to_2_generic is tested indepedently elsewere, use it to
  // to test convert_1_to_2_generic
  std::vector<uint8_t> buffer2;
  success = this->test.factory.convert_2_to_1_generic(serialized_msg, buffer2);
  ASSERT_TRUE(success);

  // Buffer1 and Buffer2 should match in size and contents
  // ROS1 serialization this should always be true because there is no padding or empty space
  // left in any output buffers
  ASSERT_EQ(buffer1.size(), buffer2.size());

  unsigned mismatch_count = 0;
  const unsigned mismatch_count_limit = 10;
  for (size_t idx = 0; idx < buffer1.size(); ++idx) {
    int val1 = buffer1.at(idx);
    int val2 = buffer2.at(idx);
    EXPECT_EQ(val1, val2) << " idx=" << idx << " of " << buffer1.size();
    if (val1 != val2) {
      ++mismatch_count;
    }
    ASSERT_LE(
      mismatch_count,
      mismatch_count_limit) << " stopping comparison after " << mismatch_count_limit <<
      " mismatches";
  }
  ASSERT_EQ(mismatch_count, 0u) << " the output buffers should be exactly the same";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
