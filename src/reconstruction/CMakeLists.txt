cmake_minimum_required(VERSION 3.14)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)

#---------------------------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------------------------

option(ENABLE_REC_PROFILER  "Enable profiling for the TSlam reconstruction"                             ON)
option(ENABLE_REC_DEBUG     "Enable debug outputs (logs, visualizer) for the TSlam reconstruction"      ON)

option(ENABLE_TESTING       "Enable testing for the TSlam reconstruction"                              OFF)

#---------------------------------------------------------------------------------------------------------
# shared library for tslam reconstruction
#---------------------------------------------------------------------------------------------------------

# TODO: add properties and flags to library
# build shared lib
file(GLOB tslamrec_srcs_base "*.cc")
add_library(tslamrec SHARED)
target_sources(tslamrec PUBLIC ${tslamrec_srcs_base})
find_package(Open3D REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(tslamrec PUBLIC Open3D::Open3D)
target_link_libraries(tslamrec PUBLIC Eigen3::Eigen)

set(EXE_NAME reconstruct)                               # TODO: test exe
add_executable(${EXE_NAME} reconstruct_test.cc)         # TODO: test exe
target_link_libraries(${EXE_NAME} PRIVATE tslamrec)     # TODO: test exe

#---------------------------------------------------------------------------------------------------------
# Pre-processor definitions
#---------------------------------------------------------------------------------------------------------

if(ENABLE_REC_PROFILER)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TSLAM_REC_PROFILER=0)
endif()
if(ENABLE_REC_DEBUG)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TSLAM_REC_DEBUG=0)
endif()

#---------------------------------------------------------------------------------------------------------
# GTest
#---------------------------------------------------------------------------------------------------------

if(ENABLE_TESTING)
    add_subdirectory(tests)
endif()