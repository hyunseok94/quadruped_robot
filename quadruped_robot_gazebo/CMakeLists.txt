cmake_minimum_required(VERSION 3.0.2)
project(quadruped_robot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(gazebo REQUIRED)
FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES quadruped_robot_gazebo
   DEPENDS gazebo_ros roscpp std_msgs
#  DEPENDS system_lib
)

LINK_LIBRARIES(rbdl)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${Boost_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
${RBDL_INCLUDE_DIR} 
${EIGEN3_INCLUDE_DIR}
${PCL_LIBRARY_DIRS}
)

LINK_DIRECTORIES(
${GAZEBO_LIBRARY_DIRS}
/usr/local/lib
)

list(
APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}"
)
SET(SRC src/plugin.cpp src/QRobot.cpp src/QuadProgpp/Array.cc src/QuadProgpp/QuadProg++.cc)

add_library(plugin SHARED ${SRC})

target_link_libraries(plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})


