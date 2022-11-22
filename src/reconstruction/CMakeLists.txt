cmake_minimum_required(VERSION 3.0)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)

# file(GLOB PARTICLES_SRC ../*.cc)
# list(REMOVE_ITEM PARTICLES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../main.cc")

find_package(Open3D REQUIRED)
message(STATUS "Open3D ${Open3D_VERSION} found at ${Open3D_DIR}")

# FIXME: download and use the github fork of yaml-cpp: https://github.com/ibois-epfl/yaml-cpp-ibois
# for later integration insert the dependecy as submodule
find_package(yaml-cpp 0.7.0 REQUIRED)
message(STATUS "Yaml-cpp found at ${YAML_CPP_INCLUDE_DIR}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(reconstruct reconstruct_test.cc)

target_link_libraries(reconstruct PRIVATE Open3D::Open3D)
target_link_libraries(reconstruct PRIVATE Eigen3::Eigen)
target_link_libraries(reconstruct PRIVATE ${YAML_CPP_LIBRARIES})

target_include_directories(reconstruct PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(reconstruct PUBLIC ${YAML_CPP_INCLUDE_DIR})
# target_link_libraries(test gtest_main gtest)