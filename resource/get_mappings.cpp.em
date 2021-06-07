// generated from ros1_bridge/resource/get_mappings.cpp.em

@###############################################
@#
@# Methods for determing mappings between
@# ROS 1 and ROS 2 interfaces
@#
@# EmPy template for generating get_mappings.cpp
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - mappings (list of ros1_bridge.Mapping)
@#    Mapping between messages as well as their fields
@#  - services (list of dictionaries)
@#    Mapping between services as well as their fields
@###############################################
@
#include <map>
#include <string>

namespace ros1_bridge
{

bool
get_1to2_mapping(const std::string & ros1_type_name, std::string & ros2_type_name)
{
@[if not mappings]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[end if]@

@[for m in mappings]@
  if (ros1_type_name == "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)")
  {
    ros2_type_name = "@(m.ros2_msg.package_name)/msg/@(m.ros2_msg.message_name)";
    return true;
  }
@[end for]@

  return false;
}

bool
get_2to1_mapping(const std::string & ros2_type_name, std::string & ros1_type_name)
{
@[if not mappings]@
  (void)ros1_type_name;
  (void)ros2_type_name;
@[end if]@

@[for m in mappings]@
  if (ros2_type_name == "@(m.ros2_msg.package_name)/msg/@(m.ros2_msg.message_name)")
  {
    ros1_type_name = "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)";
    return true;
  }
@[end for]@

  return false;
}

std::multimap<std::string, std::string>
get_all_message_mappings_2to1()
{
  static std::multimap<std::string, std::string> mappings = {
@[for m in mappings]@
    {
      "@(m.ros2_msg.package_name)/msg/@(m.ros2_msg.message_name)",  // ROS 2
      "@(m.ros1_msg.package_name)/@(m.ros1_msg.message_name)"   // ROS 1
    },
@[end for]@
  };
  return mappings;
}

std::multimap<std::string, std::string>
get_all_service_mappings_2to1()
{
  static std::multimap<std::string, std::string> mappings = {
@[for s in services]@
    {
      "@(s['ros2_package'])/srv/@(s['ros2_name'])",  // ROS 2
      "@(s['ros1_package'])/@(s['ros1_name'])"   // ROS 1
    },
@[end for]@
  };
  return mappings;
}

std::multimap<std::string, std::string>
get_all_action_mappings_2to1()
{
  static std::multimap<std::string, std::string> mappings = {
@[for a in actions]@
    {
      "@(a['ros2_package'])/action/@(a['ros2_name'])",  // ROS 2
      "@(a['ros1_package'])/@(a['ros1_name'])"   // ROS 1
    },
@[end for]@
  };
  return mappings;
}

}  // namespace ros1_bridge
