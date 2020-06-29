cmake_minimum_required(VERSION 3.0.2)
project(clearpath_sc_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES clearpath_sc_ros
  CATKIN_DEPENDS roscpp
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
