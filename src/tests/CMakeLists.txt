cmake_minimum_required(VERSION 3.14)

project(smoke_t_runtime_trajectory)

set(CMAKE_CXX_STANDARD 17)

include(CTest)
enable_testing()

#---------------------------------------------------------------------------------------------------------
# Doc
#---------------------------------------------------------------------------------------------------------
set(TEST_EXE_NAME smoke_t_runtime_trajectory)

add_executable(${TEST_EXE_NAME} smoke_t_runtime_trajectory.cc)
if (BUILD_TESTS)
    target_link_libraries(${TEST_EXE_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/tests)
else()
    target_include_directories(${TEST_EXE_NAME} PRIVATE ${CMAKE_SOURCE_DIR})
endif()

message("DEBUG::: CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message("DEBUG::: CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

#---------------------------------------------------------------------------------------------------------
# Post-build for doctest
#---------------------------------------------------------------------------------------------------------

# # run doctest series
# add_custom_command(TARGET ${TEST_EXE_NAME}
#                    POST_BUILD
#                    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#                    COMMAND ${CMAKE_BINARY_DIR}/${TEST_EXE_NAME})