cmake_minimum_required(VERSION 3.0.2)
project(fcs_ros_deb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  msgs_ros_deb
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    msgs_ros_deb
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  lib/fcs_deb/src/fcs
)

file(GLOB SRC_fcs_deb lib/fcs_deb/src/fcs/*.cpp)

add_library(fcs_deb
  ${SRC_fcs_deb}
)

add_executable(fcs_node
  src/fcs_node.cpp
  src/FCSROSControl.cpp
)

target_link_libraries(fcs_node
  ${catkin_LIBRARIES}
  fcs_deb
)

add_dependencies(fcs_node ${catkin_EXPORTED_TARGETS})
