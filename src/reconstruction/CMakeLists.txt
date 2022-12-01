cmake_minimum_required(VERSION 3.0)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)

# file(GLOB PARTICLES_SRC ../*.cc)
# list(REMOVE_ITEM PARTICLES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../main.cc")

find_package(Open3D REQUIRED)
message(STATUS "Open3D ${Open3D_VERSION} found at ${Open3D_DIR}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


file(GLOB SRC_FILES *.cc *.cpp)
list(REMOVE_ITEM SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/reconstruct_test.cc")


add_executable(reconstruct reconstruct_test.cc ${SRC_FILES})

target_include_directories(reconstruct PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


target_link_libraries(reconstruct PRIVATE 
    Open3D::Open3D
    Eigen3::Eigen
)