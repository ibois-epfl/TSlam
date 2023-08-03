cmake_minimum_required(VERSION 3.14)

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/tests/example.map")
    message(STATUS "Downloading example.map from https://zenodo.org/record/7843165/files/example.map")
    file(DOWNLOAD "https://zenodo.org/record/7843165/files/example.map" "${PROJECT_SOURCE_DIR}/tests/example.map")
endif()

#---------------------------------------------------------------------------------------------------------
# Doc
#---------------------------------------------------------------------------------------------------------
add_executable(smoke_t_runtime_trajectory smoke_t_runtime_trajectory.cc)
#target_include_directories(smoke_t_runtime_trajectory PRIVATE ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(smoke_t_runtime_trajectory PRIVATE tslam)
target_compile_definitions(smoke_t_runtime_trajectory PRIVATE "CMAKE_PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")

add_test(NAME smoke_t_runtime_trajectory
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND smoke_t_runtime_trajectory)
