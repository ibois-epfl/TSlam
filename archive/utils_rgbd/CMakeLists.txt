INCLUDE_DIRECTORIES(../../src)
LINK_LIBRARIES(${PROJECT_NAME})

IF(${OPENNI2LIB_FOUND})
add_definitions(-DOPENNI2)
SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${LIBOPENNI2_LIBRARY})
ENDIF()

find_package(PCL REQUIRED COMPONENTS common registration)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(rgbdreader icp)
add_subdirectory(rgbdreader)
add_subdirectory(icp)

add_executable(rgbd_slam rgbd_slam.cpp)
target_link_libraries(rgbd_slam rgbdreader)

add_library(pointcloud pointcloud.cpp)
add_library(heightmap heightmap.cpp)
add_library(pointgrid pointgrid.cpp)
target_link_libraries(heightmap pointgrid)
add_library(pcd_io pcd_io.cpp)

add_executable(cloud_matcher cloud_matcher.cpp)
target_link_libraries(cloud_matcher pointcloud heightmap pcd_io icp)

#find_package(depthmaps REQUIRED)
add_executable(transform_finder transform_finder.cpp)
target_link_libraries(transform_finder pcd_io pointcloud icp heightmap)

add_library(sequence_optimizer sequenceoptimizer.cpp)

add_executable(rigid_registration rigid_registration.cpp)
target_link_libraries(rigid_registration ${REQUIRED_LIBRARIES} rgbdreader sequence_optimizer global_icp pointcloud heightmap pointgrid pcd_io ${depthmaps_LIBS})

add_executable(read_rgbd read_rgbd.cpp)
target_link_libraries(read_rgbd ${REQUIRED_LIBRARIES} rgbdreader)
