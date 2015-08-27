cmake_minimum_required(VERSION 2.8.3)

project(ros1_bridge)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)

# find ROS 1 packages
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(STATUS "Failed to find PkgConfig, skipping...")
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(STATUS "Failed to find ROS 1 roscpp, skipping...")
  return()
endif()

find_ros1_package(std_msgs REQUIRED)

ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

ament_package()

# Get the implementations ahead of time
get_available_rmw_implementations(middleware_implementations)
foreach(middleware_impl ${middleware_implementations})
  find_package("${middleware_impl}" REQUIRED)
endforeach()

include_directories(include)

function(build_executable executable)
  cmake_parse_arguments(ARG "ROS1_DEPENDENCIES" "DEPENDENCIES" "TARGET_DEPENDENCIES" ${ARGN})

  set(target_dependencies
    ${ARG_TARGET_DEPENDENCIES}
    "rclcpp")
  if(ARG_ROS1_DEPENDENCIES)
    list(APPEND target_dependencies
      "ros1_roscpp"
      "ros1_std_msgs")
  endif()

  add_executable_for_each_rmw_implementations(${executable}
    ${ARG_UNPARSED_ARGUMENTS}
    TARGET_DEPENDENCIES ${target_dependencies}
    INSTALL
    ALL_TARGET_NAMES_VAR targets)
  foreach(target ${targets})
    if(ARG_DEPENDENCIES)
      add_dependencies(${target} ${ARG_DEPENDENCIES})
    endif()
  endforeach()
endfunction()

build_executable(ros1_bridge_talker src/talker.cpp
  TARGET_DEPENDENCIES "std_msgs")
build_executable(ros1_bridge_listener src/listener.cpp
  TARGET_DEPENDENCIES "std_msgs")

build_executable(simple_bridge_1_to_2 src/simple_bridge_1_to_2.cpp
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES "std_msgs")
build_executable(simple_bridge_2_to_1 src/simple_bridge_2_to_1.cpp
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES "std_msgs")

build_executable(simple_bridge src/simple_bridge.cpp
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES "std_msgs")

# TODO until the generated file can be passed to add_executable
# this must be a custom target instead of a custom command
add_custom_target(generate_factories
  COMMAND ${PYTHON_EXECUTABLE} bin/ros1_bridge_generate_factories
    --output-file "${CMAKE_BINARY_DIR}/generated/generated_factories.cpp" --template-dir resource
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# to find the generated factories
include_directories(${CMAKE_BINARY_DIR}/generated)

ament_index_get_resources(message_packages "rosidl_interfaces")
foreach(message_package ${message_packages})
  find_package(${message_package} REQUIRED)
endforeach()

build_executable(static_bridge src/static_bridge.cpp ROS1_DEPENDENCIES
  DEPENDENCIES generate_factories
  TARGET_DEPENDENCIES ${message_packages})

build_executable(dynamic_bridge src/dynamic_bridge.cpp ROS1_DEPENDENCIES
  DEPENDENCIES generate_factories
  TARGET_DEPENDENCIES ${message_packages})

install(
  PROGRAMS bin/ros1_bridge_generate_factories
  DESTINATION lib/ros1_bridge_generate_factories
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)
