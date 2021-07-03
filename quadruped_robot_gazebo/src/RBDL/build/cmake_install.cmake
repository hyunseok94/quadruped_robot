# Install script for directory: /home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.so.2.6.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/build/librbdl.so.2.6.0"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/build/librbdl.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.so.2.6.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rbdl" TYPE FILE FILES
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Body.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Constraints.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Dynamics.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Joint.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Kinematics.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Logging.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Model.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/Quaternion.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/SpatialAlgebraOperators.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/compileassert.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/rbdl.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/rbdl_eigenmath.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/rbdl_math.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/rbdl_mathutils.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/include/rbdl/rbdl_utils.h"
    "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/build/include/rbdl/rbdl_config.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/build/rbdl.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/build/addons/urdfreader/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hyunseok/catkin_ws/src/quadruped_robot/quadruped_robot_gazebo/src/RBDL/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
