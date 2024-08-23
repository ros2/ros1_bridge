# Copyright (C) 2024, Locus Robotics. All rights reserved.
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

function(filter_packages)
  set(options "")
  set(oneValueArgs PKG_LIST)
  set(multiValueArgs INCLUDE_PKGS IGNORE_PKGS)

  cmake_parse_arguments(BRIDGE "${options}"
                        "${oneValueArgs}" "${multiValueArgs}" "${ARGN}")

  set(ros2_packages ${${BRIDGE_PKG_LIST}})
  if(BRIDGE_INCLUDE_PKGS)
    foreach(pkg ${ros2_packages})
      if(NOT ${pkg} IN_LIST BRIDGE_INCLUDE_PKGS)
        list(REMOVE_ITEM ros2_packages ${pkg})
      endif()
    endforeach()
  elseif(BRIDGE_IGNORE_PKGS)
    foreach(pkg ${ros2_packages})
      if(${pkg} IN_LIST BRIDGE_IGNORE_PKGS)
        list(REMOVE_ITEM ros2_packages ${pkg})
      endif()
    endforeach()
  endif()

  set(${BRIDGE_PKG_LIST} "${ros2_packages}" PARENT_SCOPE)
endfunction()
