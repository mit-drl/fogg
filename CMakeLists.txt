cmake_minimum_required(VERSION 2.8.3)
project(fogg)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fopenmp ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  pcl_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs pcl_ros pcl_conversions pcl_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(fogg_node src/fogg_node.cpp src/clustering.cpp src/occupancy.cpp)
target_link_libraries(fogg_node ${catkin_LIBRARIES})
