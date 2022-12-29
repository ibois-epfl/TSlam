cmake_minimum_required(VERSION 3.14)

project(tslam_reconstruct_tests)

set(CMAKE_CXX_STANDARD 17)

find_package(Open3D REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(CTest)
enable_testing()

#---------------------------------------------------------------------------------------------------------
# Tests
#---------------------------------------------------------------------------------------------------------

add_executable(test_gsolver test_gsolver.cc)
target_link_libraries(test_gsolver PRIVATE tslamrec)
target_include_directories(test_gsolver PRIVATE ${CMAKE_SOURCE_DIR})

add_test(test_gsolver_series test_gsolver)


# (2) Accuracy testing
# TODO: implement

#---------------------------------------------------------------------------------------------------------
# Post-build
#---------------------------------------------------------------------------------------------------------

# run all tests on build
add_custom_command(TARGET test_gsolver
                   POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
                   COMMAND ${CMAKE_CTEST_COMMAND} -j N)