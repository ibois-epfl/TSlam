# NOTE: This cmake file is reserved for local debugging of tslam reconstruction plugin.
# The integration in the main TSlam is done by simply adding hh and cc.

cmake_minimum_required(VERSION 3.14)

project(tslam_reconstruct)

set(CMAKE_CXX_STANDARD 17)


#---------------------------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------------------------

# TODO: "BUILD_TSLAMREC_API" might not be necessary
option(ENABLE_O3D_VISUAL_DEBUG  "This option will find locally and link open3d to enable visual debugging"   OFF)
option(ENABLE_REC_PROFILER      "Enable profiling for the TSlam reconstruction"                              OFF)
option(ENABLE_TESTING           "Enable testing for the TSlam reconstruction"                                ON)


#---------------------------------------------------------------------------------------------------------
# library for tslam reconstruction
#---------------------------------------------------------------------------------------------------------

# build shared lib
file(GLOB tslamrec_srcs_base "*.cc")
list(REMOVE_ITEM tslamrec_srcs_base "${CMAKE_CURRENT_SOURCE_DIR}/reconstruct_debug.cc")
add_library(tslamrec SHARED)  # SHARED or STATIC
target_sources(tslamrec PUBLIC ${tslamrec_srcs_base})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_include_directories(tslamrec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/CGAL/include)

find_library(GMP_LIB NAMES "gmp" PATHS ${CMAKE_LIBRARY_PATH} )
find_library(MPFR_LIB NAMES "MPFR" PATHS ${CMAKE_LIBRARY_PATH} )

target_link_libraries(tslamrec PUBLIC
                      Eigen3::Eigen
                      gmp
                      mpfr
)

target_include_directories(tslamrec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cilantro/include)


#---------------------------------------------------------------------------------------------------------
# Pre-processor definitions
#---------------------------------------------------------------------------------------------------------

if(ENABLE_REC_PROFILER)
    target_compile_definitions(tslamrec PUBLIC TSLAM_REC_PROFILER=0)
endif()

if(ENABLE_O3D_VISUAL_DEBUG)
    find_package(Open3D 0.16.1 REQUIRED)
    target_link_libraries(tslamrec PUBLIC Open3D::Open3D)
    target_compile_definitions(tslamrec PUBLIC TSLAM_REC_O3D_VISUAL_DEBUG=0)
endif()

#---------------------------------------------------------------------------------------------------------
# DocTests
#---------------------------------------------------------------------------------------------------------

if(ENABLE_TESTING)
    if (ENABLE_O3D_VISUAL_DEBUG)
        set(ENABLE_O3D_VISUAL_DEBUG_TEST ON)
    else()
        set(ENABLE_O3D_VISUAL_DEBUG_TEST OFF)
    endif()
    add_subdirectory(tests)
endif()