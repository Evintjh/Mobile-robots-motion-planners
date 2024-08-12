cmake_minimum_required(VERSION 3.10.0)
project(data_structure VERSION 0.1.0 LANGUAGES C CXX)

message(STATUS "PROJECT_NAME: " ${PROJECT_NAME})

###########
## Build ##
###########
# ROB-KUL libraries
add_library(${PROJECT_NAME} SHARED
    src/data_structure/utils.c
)   

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
)


#############
## Install ##
#############
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS ${PROJECT_NAME}  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)


