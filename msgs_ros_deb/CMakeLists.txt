cmake_minimum_required(VERSION 2.8.3)
project(msgs_ros_deb)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
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

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)