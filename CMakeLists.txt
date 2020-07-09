cmake_minimum_required(VERSION 2.8.3)
project(fcs_ros_deb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(
  FILES
  FoilboatTarget.msg
  FoilboatControl.msg
  FoilboatState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  dynamic_reconfigure/ControllerGains.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
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

add_executable(range_sensor_wrapper_node
  src/range_sensor_wrapper/range_sensor_wrapper_node.cpp
  src/range_sensor_wrapper/RangeSensorProcessor.cpp
)
target_link_libraries(range_sensor_wrapper_node
  ${catkin_LIBRARIES}
)

add_executable(minimaestro_llc_node
  src/low_level_control/minimaestro_llc_node.cpp
)
target_link_libraries(minimaestro_llc_node ${catkin_LIBRARIES})