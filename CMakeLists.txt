cmake_minimum_required(VERSION 2.8.3)
project(bag_reader)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -Wno-return-type -Wno-unused-result -Wno-unused-variable -Wall -Wno-maybe-uninitialized")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  nodelet
  nav_msgs
  mrs_msgs
  mrs_lib
  flocking
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BagReader
  CATKIN_DEPENDS roscpp rosbag nav_msgs mrs_msgs mrs_lib flocking
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(BagReader src/BagReader.cpp)
add_dependencies(BagReader ${catkin_EXPORTED_TARGETS})

target_link_libraries(BagReader
  ${catkin_LIBRARIES}
)