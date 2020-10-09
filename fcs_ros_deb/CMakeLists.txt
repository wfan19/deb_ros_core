cmake_minimum_required(VERSION 3.0.2)
project(fcs_ros_deb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  lib/fcs_deb/src/fcs
)

file(GLOB SRC_fcs_deb lib/fcs_deb/src/fcs/*.cpp)

add_library(fcs_deb
  ${SRC_fcs_deb}
)

add_executable(fcs_ros_deb
  src/fcs_ros_deb.cpp
)

target_link_libraries(fcs_ros_deb
  ${catkin_LIBRARIES}
  fcs_deb
)

