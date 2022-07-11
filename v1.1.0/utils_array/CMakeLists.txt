 # CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${UCOSLAM_REQUIRED_LIBRARIES})

ADD_EXECUTABLE(ucoslam_array ucoslam_array.cpp )
ADD_EXECUTABLE(ucoslam_array_calibrate ucoslam_array_calibrate.cpp stereo_calibration.cpp stereo_calibration.h levmarq.h aruco_calibration_grid_board_a4.h)
ADD_EXECUTABLE(ucoslam_array_record ucoslam_array_record.cpp )



