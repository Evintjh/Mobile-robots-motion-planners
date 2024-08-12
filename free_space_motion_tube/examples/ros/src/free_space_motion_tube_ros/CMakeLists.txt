cmake_minimum_required(VERSION 3.0.2)
project(free_space_motion_tube_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES free_space_motion_tube
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_subdirectory(../../../../ bin/data_structure) # target: free_space_motion_tube 
## Specify additional locations of header files
## Your package locations should be listed before other locations
add_library(${PROJECT_NAME}
  src/barn_motion_tube.cpp
)
add_dependencies(${PROJECT_NAME} free_space_motion_tube)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${free_space_motion_tube_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} 
  PRIVATE
    ${catkin_LIBRARIES}  
    free_space_motion_tube
)

add_executable(barn_motion_tube_node nodes/barn_motion_tube_node.cpp)
target_link_libraries(barn_motion_tube_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME} 
  free_space_motion_tube
)
add_dependencies(barn_motion_tube_node ${PROJECT_NAME})


add_executable(old_working_node nodes/old_working_node.cpp)
target_link_libraries(old_working_node 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME} 
  free_space_motion_tube
)
add_dependencies(old_working_node ${PROJECT_NAME})