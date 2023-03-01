# CMake file for sba's demo program
include_directories(${PROJECT_SOURCE_DIR}/src )
link_libraries(${PROJECT_NAME} ${TSLAM_REQUIRED_LIBRARIES})

add_executable(tslam_minimal_example tslam_minimal_example.cpp)
add_executable(tslam_monocular monocular_slam.cpp inputreader.cpp inputreader.h)
add_executable(tslam_mapviewer mapviewer.cpp)
add_executable(tslam_map_export tslam_map_export.cpp)
add_executable(tslam_reconstruct tslam_reconstruct.cc)
add_executable(tslam_combine_map tslam_combine_map.cpp)

# add_executable(tslam_map_removeunusedkeypoint archive/tslam_map_removeunusedkeypoint.cpp)
# add_executable(tslam_fullba archive/tslam_fullba.cpp g2oba.h cvprojectpoint.h)
# add_executable(tslam_reduce_kf archive/tslam_reduce_kf.cpp g2oba.h cvprojectpoint.h)

#install(TARGETS tslam_monocular tslam_mapviewer RUNTIME DESTINATION bin)