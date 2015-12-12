cmake_minimum_required(VERSION 2.8.3)

project(ros1_bridge)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(rmw REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# find ROS 1 packages
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(STATUS "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid being treated as a plain CMake package from ament_tools
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(STATUS "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid being treated as a plain CMake package from ament_tools
  ament_package()
  return()
endif()

find_ros1_package(std_msgs REQUIRED)

ament_export_include_directories(include)

ament_python_install_package(${PROJECT_NAME})

ament_package()

add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/generated/generated_factories.cpp"
  COMMAND ${PYTHON_EXECUTABLE} bin/ros1_bridge_generate_factories
    --output-file "${CMAKE_BINARY_DIR}/generated/generated_factories.cpp" --template-dir resource
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT WIN32)
  # ignore warning in ROS 1 message headers
  set_source_files_properties("${CMAKE_BINARY_DIR}/generated/generated_factories.cpp"
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter")
endif()

ament_index_get_resources(message_packages "rosidl_interfaces")
foreach(message_package ${message_packages})
  find_package(${message_package} REQUIRED)
endforeach()

include_directories(include)

function(custom_executable target)
  cmake_parse_arguments(
    ARG "ROS1_DEPENDENCIES" "" "TARGET_DEPENDENCIES" ${ARGN})

  add_executable(${target}${target_suffix}
    ${ARG_UNPARSED_ARGUMENTS})
  ament_target_dependencies(${target}${target_suffix}
    ${ARG_TARGET_DEPENDENCIES}
    "rclcpp${target_suffix}")
  if(ARG_ROS1_DEPENDENCIES)
    ament_target_dependencies(${target}${target_suffix}
      "ros1_roscpp"
      "ros1_std_msgs")
  endif()
  if(ARG_DEPENDENCIES)
    add_dependencies(${target}${target_suffix} ${ARG_DEPENDENCIES})
  endif()

  install(TARGETS ${target}${target_suffix}
    DESTINATION bin)
endfunction()

macro(targets)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")
  endif()

  custom_executable(ros1_bridge_talker "src/talker.cpp"
    TARGET_DEPENDENCIES "std_msgs")
  custom_executable(ros1_bridge_listener "src/listener.cpp"
    TARGET_DEPENDENCIES "std_msgs")

  custom_executable(simple_bridge_1_to_2 "src/simple_bridge_1_to_2.cpp"
    ROS1_DEPENDENCIES
    TARGET_DEPENDENCIES "std_msgs")
  custom_executable(simple_bridge_2_to_1 "src/simple_bridge_2_to_1.cpp"
    ROS1_DEPENDENCIES
    TARGET_DEPENDENCIES "std_msgs")

  custom_executable(simple_bridge "src/simple_bridge.cpp"
    ROS1_DEPENDENCIES
    TARGET_DEPENDENCIES "std_msgs")

  add_library(${PROJECT_NAME}${target_suffix} SHARED
    "${CMAKE_BINARY_DIR}/generated/generated_factories.cpp")
  ament_target_dependencies(${PROJECT_NAME}${target_suffix}
    ${message_packages}
    "rclcpp${target_suffix}"
    "ros1_roscpp"
    "ros1_std_msgs")

  install(TARGETS ${PROJECT_NAME}${target_suffix}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  custom_executable(static_bridge
    "src/static_bridge.cpp"
    ROS1_DEPENDENCIES
    TARGET_DEPENDENCIES ${message_packages})
  target_link_libraries(static_bridge${target_suffix}
    ${PROJECT_NAME}${target_suffix})

  custom_executable(dynamic_bridge
    "src/dynamic_bridge.cpp"
    ROS1_DEPENDENCIES
    TARGET_DEPENDENCIES ${message_packages})
  target_link_libraries(dynamic_bridge${target_suffix}
    ${PROJECT_NAME}${target_suffix})
endmacro()

call_for_each_rmw_implementation(targets GENERATE_DEFAULT)

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
