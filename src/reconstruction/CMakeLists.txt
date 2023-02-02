cmake_minimum_required(VERSION 3.14)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)

#---------------------------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------------------------

# TODO: "BUILD_TSLAMREC_API" might not be necessary
option(BUILD_TSLAMREC_API   "Build the TSlam reconstruction API, *no profiler* "                        OFF)

option(ENABLE_REC_PROFILER  "Enable profiling for the TSlam reconstruction"                             OFF)

option(ENABLE_TESTING       "Enable testing for the TSlam reconstruction"                               ON)

#---------------------------------------------------------------------------------------------------------
# library for tslam reconstruction
#---------------------------------------------------------------------------------------------------------

# TODO: add properties and flags to library
# build shared lib
file(GLOB tslamrec_srcs_base "*.cc")
list(REMOVE_ITEM tslamrec_srcs_base "${CMAKE_CURRENT_SOURCE_DIR}/reconstruct_debug.cc")
add_library(tslamrec SHARED)  # SHARED or STATIC
target_sources(tslamrec PUBLIC ${tslamrec_srcs_base})
find_package(Open3D REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# add_subdirectory(CGAL)
# target_include_directories(tslamrec PUBLIC CGAL)
target_include_directories(tslamrec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/CGAL/include)
target_link_libraries(tslamrec PUBLIC Open3D::Open3D)
target_link_libraries(tslamrec PUBLIC Eigen3::Eigen)

#---------------------------------------------------------------------------------------------------------
# DocTests
#---------------------------------------------------------------------------------------------------------

if(ENABLE_TESTING)
    add_subdirectory(tests)
endif()

#---------------------------------------------------------------------------------------------------------
# Pre-processor definitions
#---------------------------------------------------------------------------------------------------------

if (BUILD_TSLAMREC_API)
    set(ENABLE_REC_PROFILER OFF)

    # TODO: prepare the library to be  integrated in the main tslam
    # set(TSLAMREC_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
    # set(TSLAMREC_LIBRARIES tslamrec)
endif()

if(ENABLE_REC_PROFILER)
    target_compile_definitions(tslamrec PUBLIC TSLAM_REC_PROFILER=0)
endif()