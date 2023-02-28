cmake_minimum_required(VERSION 3.14)

project(tslam_reconstruct_tests)

set(CMAKE_CXX_STANDARD 17)

option(ENABLE_O3D_VISUAL_DEBUG_TEST "This option enable open3d visual for testing" OFF)

if(ENABLE_O3D_VISUAL_DEBUG_TEST)
    find_package(Open3D REQUIRED)
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(CTest)
enable_testing()

#---------------------------------------------------------------------------------------------------------
# Doc
#---------------------------------------------------------------------------------------------------------
set(TEST_EXE_NAME test_gsolver)

add_executable(${TEST_EXE_NAME} test_gsolver.cc)
target_link_libraries(${TEST_EXE_NAME} PRIVATE tslamrec)
target_include_directories(${TEST_EXE_NAME} PRIVATE ${CMAKE_SOURCE_DIR})

#---------------------------------------------------------------------------------------------------------
# Post-build for doctest
#---------------------------------------------------------------------------------------------------------

# run doctest series
add_custom_command(TARGET ${TEST_EXE_NAME}
                   POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                   COMMAND ${CMAKE_BINARY_DIR}/tests/${TEST_EXE_NAME})

