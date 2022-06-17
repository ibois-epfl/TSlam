INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/utils  )
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES})
add_executable(dummy dummy.cpp)

IF(${OpenCV_VERSION} STRGREATER 3)

ADD_EXECUTABLE(leap_stereo leap_stereo.cpp ../utils/dirreader.h)
ADD_EXECUTABLE(calibration calibration.cpp  )
ADD_EXECUTABLE(fisheyecalibration fisheyecalibration.cpp  )
ADD_EXECUTABLE(stereo_match stereo_match.cpp )
ADD_EXECUTABLE(stereo_calibrate stereo_calibrate.cpp)
ADD_EXECUTABLE(imagelist_creator  imagelist_creator.cpp)
ADD_EXECUTABLE(videoflip  videoflip.cpp)
ADD_EXECUTABLE(videorecode  videorecode.cpp)
ADD_EXECUTABLE(videoundistort  videoundistort.cpp)
add_executable(keyframeselect4cnn keyframeselect4cnn.cpp)
add_executable(keyframeselectrange4cnn keyframeselectrange4cnn.cpp)
add_executable(stereo_watch stereo_watch.cpp)



ENDIF()
