cmake_minimum_required(VERSION 3.0)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)

file(GLOB SRC_FILES *.cc *.cpp)
list(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/reconstruct_test.cc")

set(EXE_NAME reconstruct)
add_executable(${EXE_NAME} reconstruct_test.cc ${SRC_FILES})


#------------------------------------------------------------------------------------------------------------
# Options
#------------------------------------------------------------------------------------------------------------

option(ENABLE_REC_PROFILER "Enable profiling for the TSlam reconstruction" ON)
option(ENABLE_REC_DEBUG "Enable debug outputs (logs, visualizer) for the TSlam reconstruction" ON)

if(ENABLE_REC_PROFILER)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TSLAM_REC_PROFILER=0)
endif()
if(ENABLE_REC_DEBUG)
    target_compile_definitions(${PROJECT_NAME} PRIVATE TSLAM_REC_DEBUG=0)
endif()


#------------------------------------------------------------------------------------------------------------
# Add dir and link libraries
#------------------------------------------------------------------------------------------------------------

find_package(Open3D REQUIRED)
message(STATUS "Open3D ${Open3D_VERSION} found at ${Open3D_DIR}")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

target_include_directories(reconstruct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(reconstruct PRIVATE 
    Open3D::Open3D
    Eigen3::Eigen
)