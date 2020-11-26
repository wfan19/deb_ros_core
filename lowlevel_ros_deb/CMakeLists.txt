cmake_minimum_required(VERSION 2.8.3)
project(lowlevel_ros_deb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  msgs_ros_deb
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

##########
# Range sensor wrapper node
##########
add_executable(range_sensor_wrapper_node
  src/range_sensor_wrapper/range_sensor_wrapper_node.cpp
  src/range_sensor_wrapper/RangeSensorProcessor.cpp
  )
target_link_libraries(range_sensor_wrapper_node
  ${catkin_LIBRARIES}
  )
add_dependencies(range_sensor_wrapper_node ${catkin_EXPORTED_TARGETS})

##########
# Minimaestro (PWM servo board) low level control node
##########
add_executable(minimaestro_llc_node
  src/low_level_control/minimaestro_llc_node.cpp
  )
target_link_libraries(minimaestro_llc_node ${catkin_LIBRARIES})

add_dependencies(minimaestro_llc_node ${catkin_EXPORTED_TARGETS})

##########
# Clearpath low level control node
##########
add_executable(clearpath_llc_node
  src/low_level_control/clearpath/clearpath_llc_node.cpp
  src/low_level_control/clearpath/ClearpathLLC.cpp
  )
target_link_libraries(clearpath_llc_node
  ${catkin_LIBRARIES}
  )

add_dependencies(clearpath_llc_node ${catkin_EXPORTED_TARGETS})