#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
add_library(tslam_g2o_config INTERFACE)
target_include_directories(tslam_g2o_config INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

add_subdirectory(stuff)
add_subdirectory(core)
