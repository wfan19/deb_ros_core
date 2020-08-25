cmake_minimum_required(VERSION 3.0.2)
project(clearpath_sc_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ServoConfig.msg
  ServoState.msg
)

add_service_files(
  FILES
  GetConfig.srv
  HomeAxis.srv
  ClearAlerts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES clearpath_sc_ros
  CATKIN_DEPENDS
    roscpp
    message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  lib/sFoundation/inc
  lib/sFoundation/inc/inc-pub
  lib/sFoundation/inc/inc-private
  lib/sFoundation/inc/inc-private/linux
  lib/sFoundation/inc/inc-private/sFound
  lib/sFoundation/LibINI/inc
  lib/sFoundation/LibLinuxOS/inc
  lib/sFoundation/LibXML/inc
)

file(GLOB sFoundationSRC lib/sFoundation/*/src/*.cpp)
file(GLOB sFoundationLinuxSRC lib/sFoundation/sFoundation/src-linux/*.cpp)

add_library(libSFoundation ${sFoundationSRC} ${sFoundationLinuxSRC})

add_executable(clearpath_driver_node
  src/clearpath_driver_node.cpp
  src/ClearpathDriver.cpp
  src/Axis.cpp
)

target_link_libraries(clearpath_driver_node
  ${catkin_LIBRARIES}
  libSFoundation
)

add_dependencies(clearpath_driver_node clearpath_sc_ros_generate_messages_cpp)