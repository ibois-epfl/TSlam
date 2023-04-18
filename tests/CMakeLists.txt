cmake_minimum_required(VERSION 3.14)

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/tests/example.map")
    message(STATUS "Downloading example.map from https://zenodo.org/record/7843165/files/example.map")
    file(DOWNLOAD "https://zenodo.org/record/7843165/files/example.map" "${PROJECT_SOURCE_DIR}/tests/example.map")
endif()

include(CTest)
enable_testing()

#---------------------------------------------------------------------------------------------------------
# Doc
#---------------------------------------------------------------------------------------------------------
set(TEST_EXE_NAME smoke_t_runtime_trajectory)

#if (BUILD_TESTS)
add_definitions(-DCMAKE_PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\" )
include_directories(${PROJECT_SOURCE_DIR}/src)
link_libraries(${PROJECT_NAME} ${TSLAM_REQUIRED_LIBRARIES})
add_executable(${TEST_EXE_NAME} smoke_t_runtime_trajectory.cc)
target_include_directories(${TEST_EXE_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/tests)

#---------------------------------------------------------------------------------------------------------
# Post-build for doctest
#---------------------------------------------------------------------------------------------------------

# run doctest series
message("executable test path: ${CMAKE_BINARY_DIR}/tests/${TEST_EXE_NAME}")
add_custom_command(TARGET ${TEST_EXE_NAME}
                   POST_BUILD
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                   COMMAND ${CMAKE_BINARY_DIR}/tests/${TEST_EXE_NAME})