cmake_minimum_required(VERSION 2.8.3)
project(pid_ros_deb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  msgs_ros_deb
)

generate_dynamic_reconfigure_options(
  dynamic_reconfigure/ControllerConfig.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    msgs_ros_deb
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller_node
  src/controller/controller_node.cpp
  src/controller/FoilboatController.cpp
  src/controller/PIDWrapper.cpp
  src/controller/PIDFF.cpp
)
target_link_libraries(controller_node
  ${catkin_LIBRARIES}
)

add_dependencies(controller_node
  ${catkin_EXPORTED_TARGETS}
)