 # CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${TSLAM_REQUIRED_LIBRARIES})

ADD_EXECUTABLE(tslam_array tslam_array.cpp )
ADD_EXECUTABLE(tslam_array_calibrate tslam_array_calibrate.cpp stereo_calibration.cpp stereo_calibration.h levmarq.h aruco_calibration_grid_board_a4.h)
ADD_EXECUTABLE(tslam_array_record tslam_array_record.cpp )



