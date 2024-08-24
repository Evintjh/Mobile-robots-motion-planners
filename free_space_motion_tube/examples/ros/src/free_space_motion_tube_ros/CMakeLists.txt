cmake_minimum_required(VERSION 3.0.2)
project(free_space_motion_tube_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES free_space_motion_tube_ros
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${free_space_motion_tube_SOURCE_DIR}/include
)

## Add the free_space_motion_tube library
add_subdirectory(../../../../ bin/data_structure) # Target: free_space_motion_tube 

## Create a library for your project
add_library(${PROJECT_NAME}
  src/barn_motion_tube.cpp
)
add_dependencies(${PROJECT_NAME} free_space_motion_tube)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  free_space_motion_tube
)

## Add the barn_motion_tube_node executable
add_executable(barn_motion_tube_node nodes/barn_motion_tube_node.cpp)
add_dependencies(barn_motion_tube_node ${PROJECT_NAME})
target_link_libraries(barn_motion_tube_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}
  free_space_motion_tube
  /opt/ros/noetic/lib/libtf2_ros.so
)

## Add the old_working_node executable
add_executable(old_working_node nodes/old_working_node.cpp)
add_dependencies(old_working_node ${PROJECT_NAME})
target_link_libraries(old_working_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME} 
  free_space_motion_tube
)
