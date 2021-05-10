# Module/Package resp. message ignores and includes 
#
# Finds lists of ROS modules/packages resp. messages that shall
# be included resp. ignored by the ROS bridge according to the
# following rules:
# - Include lists over ignore lists. I.e. if an include list is
#   present, the ignore list will be ... well, ignored.)

set(USE_PKG_IGNORE OFF)
set(USE_PKG_INCLUDE OFF)
set(USE_MSG_IGNORE OFF)
set(USE_MSG_INCLUDE OFF)

set(BRIDGE_IGNORE_PKGS "")
set(BRIDGE_IGNORE_MSGS "")
set(BRIDGE_INCLUDE_PKGS "")
set(BRIDGE_INCLUDE_MSGS "")

# File with packages to ignore/include in ROS 1 Bridge
set(BRIDGE_PACKAGE_IGNORE "${CMAKE_SOURCE_DIR}/../../../BridgePackageIgnore.List")
set(BRIDGE_PACKAGE_INCLUDE "${CMAKE_SOURCE_DIR}/../../../BridgePackageInclude.List")

# File with messages to ignore/include in ROS 1 Bridge
set(BRIDGE_MSG_IGNORE "${CMAKE_SOURCE_DIR}/../../../BridgePackageIgnore.List")
set(BRIDGE_MSG_INCLUDE "${CMAKE_SOURCE_DIR}/../../../BridgePackageInclude.List")

if(EXISTS "${BRIDGE_PACKAGE_INCLUDE}")
  set(USE_PKG_INCLUDE ON)
else() # EXISTS "${BRIDGE_PACKAGE_INCLUDE}"
  if(EXISTS "${BRIDGE_PACKAGE_IGNORE}")
    set(USE_PKG_IGNORE ON)
  endif() # EXISTS "${BRIDGE_PACKAGE_IGNORE}"
endif() # EXISTS "${BRIDGE_PACKAGE_INCLUDE}"

if(${USE_PKG_INCLUDE})
  FILE(READ "${BRIDGE_PACKAGE_INCLUDE}" BRIDGE_INCLUDE_PKGS)
  STRING(REGEX REPLACE ";" "\\\\;" BRIDGE_INCLUDE_PKGS "${BRIDGE_INCLUDE_PKGS}")
  STRING(REGEX REPLACE "\n" ";" BRIDGE_INCLUDE_PKGS "${BRIDGE_INCLUDE_PKGS}")
  list(REMOVE_DUPLICATES BRIDGE_INCLUDE_PKGS)
  message(STATUS "Found package include list, including: ${BRIDGE_INCLUDE_PKGS}")
endif() # USE_PKG_INCLUDE

if(${USE_PKG_IGNORE})
  FILE(READ "${BRIDGE_PACKAGE_IGNORE}" BRIDGE_IGNORE_PKGS)
  STRING(REGEX REPLACE ";" "\\\\;" BRIDGE_IGNORE_PKGS "${BRIDGE_IGNORE_PKGS}")
  STRING(REGEX REPLACE "\n" ";" BRIDGE_IGNORE_PKGS "${BRIDGE_IGNORE_PKGS}")
  list(REMOVE_DUPLICATES BRIDGE_IGNORE_PKGS)
  message(STATUS "Found package ignore list, ignoring: ${BRIDGE_IGNORE_PKGS}")
endif() # USE_PKG_IGNORE

function(filter_packages pkg_list)
  set(ros2_packages ${${pkg_list}})
  if(BRIDGE_INCLUDE_PKGS)
    foreach(pkg ${ros2_packages})
      if(NOT ${pkg} IN_LIST BRIDGE_INCLUDE_PKGS)
        list(REMOVE_ITEM ros2_packages ${pkg})
      endif(NOT ${pkg} IN_LIST BRIDGE_INCLUDE_PKGS)
    endforeach()
  elseif(BRIDGE_IGNORE_PKGS)
    foreach(pkg ${ros2_packages})
      if(${pkg} IN_LIST BRIDGE_IGNORE_PKGS)
        list(REMOVE_ITEM ros2_packages ${pkg})
      endif(${pkg} IN_LIST BRIDGE_IGNORE_PKGS)
    endforeach()
  endif(BRIDGE_INCLUDE_PKGS)
  
  set(${pkg_list} "${ros2_packages}" PARENT_SCOPE)
endfunction(filter_packages ros2_packages)
