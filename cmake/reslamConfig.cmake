# ===================================================================================
#  reslam CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(reslam REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME )
#
#    This file will define the following variables:
#      - reslam_LIBS          : The list of libraries to links against.
#      - reslam_LIB_DIR       : The directory where lib files are. Calling LINK_DIRECTORIES
#                                with this path is NOT needed.
#      - reslam_VERSION       : The  version of this PROJECT_NAME build. Example: "1.2.0"
#      - reslam_VERSION_MAJOR : Major version part of VERSION. Example: "1"
#      - reslam_VERSION_MINOR : Minor version part of VERSION. Example: "2"
#      - reslam_VERSION_PATCH : Patch version part of VERSION. Example: "0"
#
# ===================================================================================
INCLUDE_DIRECTORIES("/usr/local/include")
INCLUDE_DIRECTORIES("/usr/local/include/reslam")
SET(reslam_INCLUDE_DIRS "/usr/local/include")

LINK_DIRECTORIES("/usr/local/lib")
SET(reslam_LIB_DIR "/usr/local/lib")
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(/usr/local/include/opencv4)

SET(reslam_LIBS opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;/usr/lib/gcc/x86_64-linux-gnu/9/libgomp.so;/usr/lib/x86_64-linux-gnu/libpthread.so;reslam_fbow;reslam_aruco;reslam_xflann;reslam_g2o_stuff;reslam_g2o_core reslam)

SET(reslam_FOUND YES)
SET(reslam_FOUND "YES")
SET(reslam_VERSION        1.2.4)
SET(reslam_VERSION_MAJOR  1)
SET(reslam_VERSION_MINOR  2)
SET(reslam_VERSION_PATCH  4)
