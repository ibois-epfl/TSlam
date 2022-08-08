 # CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${TSLAM_REQUIRED_LIBRARIES})

# ADD_EXECUTABLE(tslam_stereo stereo_slam.cpp)
# ADD_EXECUTABLE(tslam_createinitialparamfile createinitialparamfile.cpp)
# ADD_EXECUTABLE(tslam_stereocalibrate tslam_stereocalibrate.cpp  )
ADD_EXECUTABLE(tslam_monocular monocular_slam.cpp   inputreader.cpp inputreader.h)
ADD_EXECUTABLE(tslam_mapviewer mapviewer.cpp  )
ADD_EXECUTABLE(tslam_map_removeunusedkeypoint tslam_map_removeunusedkeypoint.cpp  )
ADD_EXECUTABLE(tslam_map_export tslam_map_export.cpp  )

# ADD_EXECUTABLE(tslam_pmvs2 tslam_pmvs2.cpp  )

ADD_EXECUTABLE(tslam_fullba tslam_fullba.cpp g2oba.h cvprojectpoint.h)
ADD_EXECUTABLE(tslam_reduce_kf tslam_reduce_kf.cpp g2oba.h cvprojectpoint.h)
ADD_EXECUTABLE(tslam_combine_map tslam_combine_map.cpp g2oba.h cvprojectpoint.h)

ADD_EXECUTABLE(tslam_minimal_example tslam_minimal_example.cpp)
ADD_EXECUTABLE(tslam_mapping tslam_mapping.cpp   inputreader.cpp inputreader.h)


# INSTALL(TARGETS tslam_monocular tslam_mapviewer  tslam_stereo tslam_stereocalibrate tslam_pmvs2  tslam_fullba RUNTIME DESTINATION bin)
INSTALL(TARGETS tslam_monocular tslam_mapviewer tslam_fullba RUNTIME DESTINATION bin)

# IF(${OPENNI2LIB_FOUND})
#     ADD_EXECUTABLE(tslam_rgbd rgbd_slam.cpp cvni2.h )
#     TARGET_LINK_LIBRARIES(tslam_rgbd ${LIBOPENNI2_LIBRARY}  )
#     INSTALL(TARGETS tslam_rgbd   RUNTIME DESTINATION bin)
# ENDIF()
