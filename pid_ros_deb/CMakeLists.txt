cmake_minimum_required(VERSION 2.8.3)
project(pid_ros_deb)

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
  dynamic_reconfigure/ControllerConfig.cfg
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

add_dependencies(controller_node
  msgs_ros_deb_generate_message_cpp
  pid_ros_deb_gencfg
)

add_executable(range_sensor_wrapper_node
  src/range_sensor_wrapper/range_sensor_wrapper_node.cpp
  src/range_sensor_wrapper/RangeSensorProcessor.cpp
)
target_link_libraries(range_sensor_wrapper_node
  ${catkin_LIBRARIES}
)

add_dependencies(range_sensor_wrapper_node msgs_ros_deb_generate_message_cpp)

add_executable(minimaestro_llc_node
  src/low_level_control/minimaestro_llc_node.cpp
)
target_link_libraries(minimaestro_llc_node ${catkin_LIBRARIES})

add_dependencies(minimaestro_llc_node msgs_ros_deb_generate_message_cpp)

add_executable(clearpath_llc_node
  src/low_level_control/clearpath/clearpath_llc_node.cpp
  src/low_level_control/clearpath/ClearpathLLC.cpp
)
target_link_libraries(clearpath_llc_node
  ${catkin_LIBRARIES}
)

add_dependencies(clearpath_llc_node msgs_ros_deb_generate_message_cpp)