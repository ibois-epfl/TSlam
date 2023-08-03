# CMake file for sba's demo program
add_executable(tslam_minimal_example tslam_minimal_example.cpp)
add_executable(tslam_monocular tslam_monocular.cpp input_reader.cpp input_reader.h)
add_executable(tslam_combine_map tslam_combine_map.cpp)
target_link_libraries(tslam_combine_map PRIVATE tslam_g2o)
add_executable(tslam_map_viewer tslam_map_viewer.cpp)
add_executable(tslam_map_export tslam_map_export.cpp)
add_executable(tslam_map_optimizer tslam_map_optimizer.cpp)
target_link_libraries(tslam_map_optimizer PRIVATE tslam_g2o tslam_g2o_stuff)
add_executable(tslam_reconstruct tslam_reconstruct.cc)

foreach(_tgt
    tslam_minimal_example tslam_monocular tslam_combine_map tslam_map_viewer tslam_map_export tslam_map_optimizer tslam_reconstruct)
  target_link_libraries(${_tgt} PRIVATE tslam)
endforeach()

#add_executable(tslam_fullba archive/tslam_fullba.cpp g2oba.h cvprojectpoint.h)
# add_executable(tslam_map_removeunusedkeypoint archive/tslam_map_removeunusedkeypoint.cpp)
# add_executable(tslam_reduce_kf archive/tslam_reduce_kf.cpp g2oba.h cvprojectpoint.h)
#install(TARGETS tslam_monocular tslam_mapviewer RUNTIME DESTINATION bin)
