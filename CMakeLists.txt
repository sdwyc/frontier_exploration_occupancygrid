cmake_minimum_required(VERSION 3.0.2)
project(frontier_exploration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  visualization_msgs
  message_generation
  actionlib
  actionlib_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PointArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  get_centroids.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   nav_msgs#   sensor_msgs#   vistualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frontier_exploration
 CATKIN_DEPENDS geometry_msgs move_base nav_msgs roscpp rospy sensor_msgs tf visualization_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(frontierDetector src/frontier_detector.cpp src/actuator.cpp)
add_executable(frontier_planner src/frontierMain.cpp
              src/frontier_detector.cpp
              src/actuator.cpp)

target_link_libraries(frontier_planner
  frontierDetector
  ${catkin_LIBRARIES}
)

