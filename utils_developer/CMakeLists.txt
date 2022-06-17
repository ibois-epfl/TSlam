# CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${OPENCV_LIBS})
ADD_EXECUTABLE(video2framevideo video2framevideo.cpp videoframeio.cpp videoframeio.h)
#ADD_EXECUTABLE(monocular_slam_framevideoinput monocular_slam_framevideoinput.cpp  videoframeio.cpp videoframeio.h)

ADD_EXECUTABLE(map_global_optimization map_global_optimization.cpp)
ADD_EXECUTABLE(videorecord videorecord.cpp )
ADD_EXECUTABLE(orthophoto orthophoto.cpp )

