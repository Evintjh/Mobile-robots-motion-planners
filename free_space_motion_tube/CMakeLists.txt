cmake_minimum_required(VERSION 3.10)
set(PROJECT_VERSION "0.0.2")
set(CMAKE_CXX_STANDARD 11)
project(free_space_motion_tube VERSION ${PROJECT_VERSION} LANGUAGES C CXX)

set(FREE_SPACE_MOTION_TUBE_VERSION_SO 0)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


##########################
##         Flags        ##
##########################
option(BUILD_GTESTS "Build unit tests" OFF)

##########################
##     Dependencies     ##
##########################
add_subdirectory(external/data_structure)


##########################
##        Build         ##
##########################
add_library(${PROJECT_NAME} SHARED
    src/core/basic.c
    src/core/motion_primitive.c
    src/core/motion_tube.c
    src/core/motion_tube_cartesian.c
    src/core/motion_tube_sensor_space.c
)
add_dependencies(${PROJECT_NAME} data_structure)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${data_structure_SOURCE_DIR}/include>
)
message(STATUS "------------------> " ${data_structure_SOURCE_DIR} )
target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        m
)

