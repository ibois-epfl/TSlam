 # CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${reslam_REQUIRED_LIBRARIES})

ADD_EXECUTABLE(reslam_stereo stereo_slam.cpp)
ADD_EXECUTABLE(reslam_monocular monocular_slam.cpp   inputreader.cpp inputreader.h)
ADD_EXECUTABLE(reslam_mapviewer mapviewer.cpp  )
ADD_EXECUTABLE(reslam_createinitialparamfile createinitialparamfile.cpp)
ADD_EXECUTABLE(reslam_map_removeunusedkeypoint reslam_map_removeunusedkeypoint.cpp  )
ADD_EXECUTABLE(reslam_stereocalibrate reslam_stereocalibrate.cpp  )
ADD_EXECUTABLE(reslam_map_export reslam_map_export.cpp  )

ADD_EXECUTABLE(reslam_pmvs2 reslam_pmvs2.cpp  )

ADD_EXECUTABLE(reslam_fullba reslam_fullba.cpp g2oba.h cvprojectpoint.h)


INSTALL(TARGETS reslam_monocular reslam_mapviewer  reslam_stereo reslam_stereocalibrate reslam_pmvs2  reslam_fullba RUNTIME DESTINATION bin)

IF(${OPENNI2LIB_FOUND})
    ADD_EXECUTABLE(reslam_rgbd rgbd_slam.cpp cvni2.h )
    TARGET_LINK_LIBRARIES(reslam_rgbd ${LIBOPENNI2_LIBRARY}  )
    INSTALL(TARGETS reslam_rgbd   RUNTIME DESTINATION bin)
ENDIF()
