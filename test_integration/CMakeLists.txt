cmake_minimum_required(VERSION 3.14)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)


set(TEST_EXE_NAME main)

set(BUILD_TSLAMREC_API ON "Build the TSLAMREC API without testing/profiler")
add_subdirectory(${CMAKE_SOURCE_DIR}/reconstruction)
# set compile flags for subdirectory DTEST


add_executable(${TEST_EXE_NAME} main.cc)
target_include_directories(${TEST_EXE_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/reconstruction)
find_package(Open3D 0.16.1 REQUIRED)
# print version of open3d
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${TEST_EXE_NAME} PRIVATE tslamrec)