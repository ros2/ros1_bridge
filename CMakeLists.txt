cmake_minimum_required(VERSION 2.8.3)

project(ros1_bridge)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(simple_msgs REQUIRED)

find_package(roscpp)
find_package(std_msgs)

ament_package()

# Get the implementations ahead of time
get_available_rmw_implementations(middleware_implementations)
foreach(middleware_impl ${middleware_implementations})
  find_package("${middleware_impl}" REQUIRED)
endforeach()

function(build_executable executable)
  cmake_parse_arguments(ARG "ROS1_DEPENDENCIES" "" "" ${ARGN})
  # Build the normal executable
  add_executable(${executable} ${ARG_UNPARSED_ARGUMENTS})
  ament_target_dependencies(${executable}
    "rclcpp"
    "simple_msgs"
    "rmw_implementation")
  if(ARG_ROS1_DEPENDENCIES)
    ament_target_dependencies(${executable}
      "roscpp"
      "std_msgs")
  endif()

  install(TARGETS ${executable} DESTINATION bin)

  # Build an executable for each ROS middleware implementation
  foreach(middleware_impl_tmp ${middleware_implementations})
    add_executable(${executable}__${middleware_impl_tmp} ${ARG_UNPARSED_ARGUMENTS})
    ament_target_dependencies(${executable}__${middleware_impl_tmp}
      "rclcpp"
      "simple_msgs"
      "${middleware_impl_tmp}")
    if(ARG_ROS1_DEPENDENCIES)
      ament_target_dependencies(${executable}__${middleware_impl_tmp}
        "roscpp"
        "std_msgs")
    endif()

    install(TARGETS ${executable}__${middleware_impl_tmp} DESTINATION bin)
  endforeach()
endfunction()

build_executable(bridge_1_to_2 src/bridge_1_to_2.cpp ROS1_DEPENDENCIES)
build_executable(bridge_2_to_1 src/bridge_2_to_1.cpp ROS1_DEPENDENCIES)
build_executable(ros1_bridge_talker src/talker.cpp)
build_executable(ros1_bridge_listener src/listener.cpp)
