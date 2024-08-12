cmake_minimum_required(VERSION 3.15)
set(PROJECT_VERSION "0.0.2")
set(CMAKE_CXX_STANDARD 11)
project(free_space_motion_tube_activity VERSION ${PROJECT_VERSION} LANGUAGES C CXX)

##########################
##     Dependencies     ##
##########################
set(five_c_FOUND false)
set(data_structure_FOUND false)
set(read_parameter_struct_FOUND false)

include(FetchContent)
FetchContent_Declare(five-c
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/algorithm_and_activity_component_architecture.git
    GIT_TAG master
    TIMEOUT 5
)
FetchContent_Declare(read-parameter-struct
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0141557/read_parameter_struct.git
    GIT_TAG main
    TIMEOUT 5
)
FetchContent_Declare(data-structure
    GIT_REPOSITORY git@gitlab.kuleuven.be:u0144428/data-structure-aacal.git
    GIT_TAG master
    TIMEOUT 5
)

FetchContent_MakeAvailable(read-parameter-struct five-c )

if(NOT ${five_c_FOUND})
    set(FIVE_C_INCLUDE_DIRS ${five_c_SOURCE_DIR}/include)
    set(FIVE_C_BUILD_DIRS ${five_c_BINARY_DIR})
    set(FIVE_C_LIBRARIES ${five_c_BINARY_DIR}/libfive_c.so)
endif()

if(NOT ${read_parameter_struct_FOUND})
    set(READ_PARAMETER_STRUCT_INCLUDE_DIRS ${read_parameter_struct_SOURCE_DIR}/include)
    set(READ_PARAMETER_STRUCT_BUILD_DIRS ${read_parameter_struct_BINARY_DIR})
    set(READ_PARAMETER_STRUCT_LIBRARIES ${read_parameter_struct_BINARY_DIR}/libread_parameter_struct.so)
endif()

if(NOT ${data_structure_FOUND})
    set(DATA_STRUCTURE_INCLUDE_DIRS ${data_structure_SOURCE_DIR}/include)
    set(DATA_STRUCTURE_BUILD_DIRS ${data_structure_BINARY_DIR})
    set(DATA_STRUCTURE_LIBRARIES ${data_structure_BINARY_DIR}/libdata_structure.so)
endif()

##########################
##        Build         ##
##########################

add_library(${PROJECT_NAME} SHARED 
    activity.c)
add_dependencies(${PROJECT_NAME} free_space_motion_tube read_parameter_struct five_c)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${DATA_STRUCTURE_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${FIVE_C_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${READ_PARAMETER_STRUCT_INCLUDE_DIRS}>
)

target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${READ_PARAMETER_STRUCT_BUILD_DIRS}
        ${DATA_STRUCTURE_BUILD_DIRS}
        ${FIVE_C_BUILD_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        pthread
        data_structure
        five_c
        free_space_motion_tube
        read_parameter_struct
)

set_target_properties( ${PROJECT_NAME}
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

##########################
##   Build (examples)   ##
##########################
add_executable(activity_demo examples/demo.c)
add_dependencies(activity_demo ${PROJECT_NAME})
target_link_libraries(activity_demo
    PRIVATE
        ${PROJECT_NAME}
)
set_target_properties(activity_demo
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
