# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

function(find_ros1_interface_packages var)
  # rosmsg/rossrv require the ROS 1 packages to be first in the PYTHONPATH
  # therefore we temporarily remove every ROS 2 path from the PYTHONPATH
  file(TO_CMAKE_PATH "$ENV{AMENT_PREFIX_PATH}" AMENT_PREFIX_PATH)
  set(PYTHONPATH "$ENV{PYTHONPATH}")
  file(TO_CMAKE_PATH "${PYTHONPATH}" CMAKE_PYTHONPATH)
  set(PYTHONPATH_WITHOUT_ROS2 "")
  foreach(python_path IN LISTS CMAKE_PYTHONPATH)
    set(match FALSE)
    foreach(ament_prefix_path IN LISTS AMENT_PREFIX_PATH)
      if(python_path MATCHES "^${ament_prefix_path}/")
        set(match TRUE)
        break()
      endif()
    endforeach()
    if(NOT match)
      list(APPEND PYTHONPATH_WITHOUT_ROS2 "${python_path}")
    endif()
  endforeach()
  file(TO_NATIVE_PATH "${PYTHONPATH_WITHOUT_ROS2}" PYTHONPATH_WITHOUT_ROS2)
  if(NOT WIN32)
    string(REPLACE ";" ":" PYTHONPATH_WITHOUT_ROS2 "${PYTHONPATH_WITHOUT_ROS2}")
  endif()
  set(ENV{PYTHONPATH} "${PYTHONPATH_WITHOUT_ROS2}")

  # find all known ROS1 message/service packages
  execute_process(
      COMMAND "rosmsg" "list"
      OUTPUT_VARIABLE rosmsg_output
      ERROR_VARIABLE rosmsg_error
  )
  execute_process(
      COMMAND "rossrv" "list"
      OUTPUT_VARIABLE rossrv_output
      ERROR_VARIABLE rossrv_error
  )

  # restore PYTHONPATH
  set(ENV{PYTHONPATH} ${PYTHONPATH})

  if(NOT rosmsg_error STREQUAL "" OR NOT rossrv_error STREQUAL "")
    message(FATAL_ERROR "${rosmsg_error}\n${rossrv_error}\nFailed to call rosmsg/rossrv")
  endif()

  set(ros1_message_packages "")
  string(REPLACE "\n" ";" ros1_interfaces "${rosmsg_output}\n${rossrv_output}")
  foreach(interface_package ${ros1_interfaces})
    string(REGEX REPLACE "/.*$" "" interface_package "${interface_package}")
    list_append_unique(ros1_message_packages ${interface_package})
  endforeach()
  set(${var} ${ros1_message_packages} PARENT_SCOPE)
endfunction()
