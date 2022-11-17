cmake_minimum_required(VERSION 3.0)

project(reconstruct)

set(CMAKE_CXX_STANDARD 17)

# file(GLOB PARTICLES_SRC ../*.cc)
# list(REMOVE_ITEM PARTICLES_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../main.cc")

add_executable(reconstruct reconstruct_test.cc ${PARTICLES_SRC})

target_include_directories(reconstruct PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(test gtest_main gtest)