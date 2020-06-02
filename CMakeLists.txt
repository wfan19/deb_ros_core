cmake_minimum_required(VERSION 2.8.3)
project(toughsonic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(Toughsonic src/Toughsonic.cpp)
target_link_libraries(Toughsonic serial)

add_executable(toughsonic_node_main src/main.cpp)
target_link_libraries(toughsonic_node_main ${catkin_LIBRARIES})
target_link_libraries(toughsonic_node_main Toughsonic)
