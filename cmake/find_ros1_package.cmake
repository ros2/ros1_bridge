# Copyright 2015 Open Source Robotics Foundation, Inc.
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

macro(find_ros1_package name)
  cmake_parse_arguments(_ARG "REQUIRED" "" "" ${ARGN})
  if(_ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "find_ros1_package() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # the error message of pkg_check_modules for not found required modules
  # doesn't include the module name which is not helpful
  pkg_check_modules(ros1_${name} ${name})
  if(NOT ros1_${name}_FOUND)
    if(_ARG_REQUIRED)
      message(FATAL_ERROR
        "find_ros1_package() failed to find '${name}' using "
        "pkg_check_modules()")
    endif()
  else()
    set(_libraries "${ros1_${name}_LIBRARIES}")
    # Prior to catkin 0.7.7, dependent libraries in the pkg-config file were always generated
    # in the form "-l:/absolute/path/to/library".  Because of the leading "-l", this made
    # them show up in ${ros1_${name}_LIBRARIES}.  catkin 0.7.7 and later has changed this to
    # just be an absolute path (of the form "/absolute/path/to/library"), so we now have to
    # look through the LDFLAGS_OTHER and add those to the libraries that we need to link with.
    foreach(_flag ${ros1_${name}_LDFLAGS_OTHER})
      if(IS_ABSOLUTE ${_flag})
        list(APPEND _libraries "${_flag}")
      endif()
    endforeach()
    set(_library_dirs "${ros1_${name}_LIBRARY_DIRS}")
    set(ros1_${name}_LIBRARIES "")
    set(ros1_${name}_LIBRARY_DIRS "")
    foreach(_library ${_libraries})
      string(SUBSTRING "${_library}" 0 1 _prefix)
      if("${_prefix} " STREQUAL ": ")
        string(SUBSTRING "${_library}" 1 -1 _rest)
        list(APPEND ros1_${name}_LIBRARIES ${_rest})
      elseif(IS_ABSOLUTE ${_library})
        list(APPEND ros1_${name}_LIBRARIES ${_library})
      else()
        set(_lib "${_library}-NOTFOUND")
        set(_lib_path "")
        # since the path where the library is found is not returned
        # this has to be done for each path separately
        foreach(_path ${_library_dirs})
          find_library(_lib ${_library}
            PATHS ${_path}
            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
          if(_lib)
            set(_lib_path ${_path})
            break()
          endif()
        endforeach()
        if(_lib)
          list(APPEND ros1_${name}_LIBRARIES ${_lib})
          list_append_unique(ros1_${name}_LIBRARY_DIRS ${_lib_path})
        else()
          # as a fall back try to search globally
          find_library(_lib ${_library})
          if(NOT _lib)
            message(FATAL_ERROR "pkg-config module '${name}' failed to find library '${_library}'")
          endif()
          list(APPEND ros1_${name}_LIBRARIES ${_lib})
        endif()
      endif()
    endforeach()
  endif()
endmacro()
