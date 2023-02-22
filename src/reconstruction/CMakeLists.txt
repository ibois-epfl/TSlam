cmake_minimum_required(VERSION 3.14)

project(tslam_reconstruct)

set(CMAKE_CXX_STANDARD 17)

#---------------------------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------------------------

# TODO: "BUILD_TSLAMREC_API" might not be necessary
option(BUILD_TSLAMREC_API   "Build the TSlam reconstruction API, *no profiler or testing* "             ON)
option(ENABLE_REC_PROFILER  "Enable profiling for the TSlam reconstruction"                             OFF)
option(ENABLE_TESTING       "Enable testing for the TSlam reconstruction"                               OFF)

#---------------------------------------------------------------------------------------------------------
# library for tslam reconstruction
#---------------------------------------------------------------------------------------------------------

# build shared lib
file(GLOB tslamrec_srcs_base "*.cc")
list(REMOVE_ITEM tslamrec_srcs_base "${CMAKE_CURRENT_SOURCE_DIR}/reconstruct_debug.cc")
add_library(tslamrec SHARED)  # SHARED or STATIC
target_sources(tslamrec PUBLIC ${tslamrec_srcs_base})
find_package(Open3D 0.16.1 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_include_directories(tslamrec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/CGAL/include)

target_link_libraries(tslamrec PUBLIC Open3D::Open3D)
target_link_libraries(tslamrec PUBLIC Eigen3::Eigen)

target_include_directories(tslamrec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cilantro/include)


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
    set(ENABLE_TESTING OFF)
endif()

if(ENABLE_REC_PROFILER)
    target_compile_definitions(tslamrec PUBLIC TSLAM_REC_PROFILER=0)
endif()