cmake_minimum_required(VERSION 3.15)
set(PROJECT_VERSION "0.0.2")
project(free_space_motion_tube_test LANGUAGES C CXX)

# GTest libary
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
link_libraries(${GTEST_LIBRARIES}
    free_space_motion_tube
)

enable_testing()
add_executable(sample_line_segment_test 
    sample_line_segment_test.cpp)

add_executable(sample_motion_primitive_test
    sample_motion_primitive_test.cpp)

# add_executable(template_test
#     template_test.cpp)
# target_link_libraries(template_test
# ${GTEST_LIBRARIES}
# ${PROJECT_NAME})

include(GoogleTest)
gtest_discover_tests(sample_line_segment_test)
gtest_discover_tests(sample_motion_primitive_test)
# gtest_discover_tests(template_test)
