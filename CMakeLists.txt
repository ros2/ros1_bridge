cmake_minimum_required(VERSION 3.5)

project(ros1_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
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
  message(WARNING "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
if(NOT ros1_roscpp_FOUND)
  message(WARNING "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(std_msgs REQUIRED)

# find ROS 1 packages with messages / services
include(cmake/find_ros1_interface_packages.cmake)
find_ros1_interface_packages(ros1_message_packages)

set(prefixed_ros1_message_packages "")
foreach(ros1_message_package ${ros1_message_packages})
  find_ros1_package(${ros1_message_package} REQUIRED)
  list(APPEND prefixed_ros1_message_packages "ros1_${ros1_message_package}")
endforeach()

set(TEST_ROS1_BRIDGE FALSE)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_ros1_package(diagnostic_msgs)
  find_ros1_package(roslaunch)
  ament_lint_auto_find_test_dependencies()
  if(ros1_diagnostic_msgs_FOUND AND ros1_roslaunch_FOUND)
    set(TEST_ROS1_BRIDGE TRUE)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/get_factory.cpp")
list(APPEND generated_files "${generated_path}/get_mappings.cpp")

# generate per package compilation units to keep the memory usage low
ament_index_get_resources(ros2_message_packages "rosidl_interfaces")
foreach(message_package ${ros2_message_packages})
  find_package(${message_package} QUIET REQUIRED)
  message(STATUS "Found ${message_package}: ${${message_package}_VERSION} (${${message_package}_DIR})")
  if(NOT "${message_package}" STREQUAL "builtin_interfaces")
    list(APPEND generated_files "${generated_path}/${message_package}_factories.cpp")
  endif()
endforeach()

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND ${PYTHON_EXECUTABLE} bin/ros1_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT WIN32)
  # ignore warning in ROS 1 message headers
  set_source_files_properties(${generated_files}
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter")
endif()

include_directories(include ${generated_path})

function(custom_executable target)
  cmake_parse_arguments(
    ARG "ROS1_DEPENDENCIES" "" "TARGET_DEPENDENCIES" ${ARGN})

  add_executable(${target}
    ${ARG_UNPARSED_ARGUMENTS})
  ament_target_dependencies(${target}
    "rclcpp"
    ${ARG_TARGET_DEPENDENCIES})
  if(ARG_ROS1_DEPENDENCIES)
    ament_target_dependencies(${target}
      "ros1_roscpp"
      "ros1_std_msgs")
  endif()
  if(ARG_DEPENDENCIES)
    add_dependencies(${target} ${ARG_DEPENDENCIES})
  endif()

  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(simple_bridge_1_to_2 "src/simple_bridge_1_to_2.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES "std_msgs")
custom_executable(simple_bridge_2_to_1 "src/simple_bridge_2_to_1.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES "std_msgs")

custom_executable(simple_bridge "src/simple_bridge.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES "std_msgs")

add_library(${PROJECT_NAME} SHARED
  "src/builtin_interfaces_factories.cpp"
  "src/convert_builtin_interfaces.cpp"
  ${generated_files})
ament_target_dependencies(${PROJECT_NAME}
  ${prefixed_ros1_message_packages}
  ${ros2_message_packages}
  "rclcpp"
  "ros1_roscpp"
  "ros1_std_msgs")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

custom_executable(static_bridge
  "src/static_bridge.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${ros2_message_packages})
target_link_libraries(static_bridge
  ${PROJECT_NAME})

custom_executable(parameter_bridge
  "src/parameter_bridge.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${message_packages})
target_link_libraries(parameter_bridge
  ${PROJECT_NAME})

custom_executable(dynamic_bridge
  "src/dynamic_bridge.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${ros2_message_packages})
target_link_libraries(dynamic_bridge
  ${PROJECT_NAME})

if(TEST_ROS1_BRIDGE)
  custom_executable(test_ros2_client_cpp "test/test_ros2_client.cpp")
  ament_target_dependencies("test_ros2_client_cpp${target_suffix}" "ros1_roscpp" "diagnostic_msgs")
  custom_executable(test_ros2_server_cpp "test/test_ros2_server.cpp")
  ament_target_dependencies("test_ros2_server_cpp${target_suffix}" "ros1_roscpp" "diagnostic_msgs")

  set(TEST_BRIDGE_DYNAMIC_BRIDGE "$<TARGET_FILE:dynamic_bridge>")
  set(TEST_BRIDGE_ROS2_CLIENT "$<TARGET_FILE:test_ros2_client_cpp>")
  set(TEST_BRIDGE_ROS2_SERVER "$<TARGET_FILE:test_ros2_server_cpp>")
  set(TEST_BRIDGE_RMW ${rmw_implementation})
endif()

macro(targets)
  configure_file(
    test/test_dynamic_bridge.py.in
    test_dynamic_bridge${target_suffix}.py.genexp
    @ONLY
  )
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_dynamic_bridge${target_suffix}_$<CONFIG>.py"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_dynamic_bridge${target_suffix}.py.genexp"
  )
  ament_add_pytest_test(test_dynamic_bridge${target_suffix}
    "${CMAKE_CURRENT_BINARY_DIR}/test_dynamic_bridge${target_suffix}_$<CONFIG>.py"
    ENV RMW_IMPLEMENTAION=${rmw_implementaion}
    TIMEOUT 60)
endmacro()

if(TEST_ROS1_BRIDGE)
  find_package(ament_cmake_pytest REQUIRED)

  add_executable(test_ros1_client "test/test_ros1_client.cpp")
  ament_target_dependencies(test_ros1_client "ros1_roscpp")
  add_executable(test_ros1_server "test/test_ros1_server.cpp")
  ament_target_dependencies(test_ros1_server "ros1_roscpp")
  set(TEST_BRIDGE_ROS1_ENV "${ros1_roslaunch_PREFIX}/env.sh")
  set(TEST_BRIDGE_ROSCORE "${ros1_roslaunch_PREFIX}/bin/roscore")
  set(TEST_BRIDGE_ROS1_CLIENT "$<TARGET_FILE:test_ros1_client>")
  set(TEST_BRIDGE_ROS1_SERVER "$<TARGET_FILE:test_ros1_server>")

  call_for_each_rmw_implementation(targets)
endif()

install(
  PROGRAMS bin/ros1_bridge_generate_factories
  DESTINATION lib/${PROJECT_NAME}/generate_factories
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
