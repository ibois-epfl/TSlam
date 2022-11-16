cmake_minimum_required(VERSION 3.5)

project(reconstruction)

project(TSlam VERSION "1.1.0" LANGUAGES CXX)


add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/run_python.cpp ${SOURCES} ${HEADERS})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#include and link python3.9

set(Python3_FIND_STRATEGY LOCATION)
find_package(Python3 3.9 COMPONENTS Interpreter Development)
if(Python3_FOUND)
    message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
    message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
    message(STATUS "Python3_VERSION: ${Python3_VERSION}")
    include_directories(${Python3_INCLUDE_DIRS})
    link_directories(${Python3_LIBRARY_DIRS})
    target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})
endif()

# target_include_directories(${PROJECT_NAME} PUBLIC "/home/as/anaconda3/envs/aiac/include/")
# target_link_directories(${PROJECT_NAME} PUBLIC "/home/as/anaconda3/envs/aiac/include/python3.9")

# find_package(Python3 REQUIRED)
# target_link_libraries(${PROJECT_NAME} PRIVATE Python3)
