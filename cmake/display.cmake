
# ----------------------------------------------------------------------------
# display status message for important variables
# ----------------------------------------------------------------------------
message( STATUS )
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
message( STATUS "General configuration for ${PROJECT_NAME} ${PROJECT_VERSION}")
MESSAGE( STATUS "-------------------------------------------------------------------------------" )
message( STATUS )
message("    Built as dynamic libs?:" ${BUILD_SHARED_LIBS})
message("    Compiler:"                   "${CMAKE_COMPILER}"   "${CMAKE_CXX_COMPILER}")
message("    CMAKE_INSTALL_PREFIX:"       "${CMAKE_INSTALL_PREFIX}")


message( STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message( STATUS "C++ flags (Release):      ${CMAKE_CXX_FLAGS_RELEASE}")
message( STATUS "C++ flags (Debug):        ${CMAKE_CXX_FLAGS_DEBUG}")
message( STATUS "C++ flags (Relase+Debug): ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message( STATUS "CMAKE_BINARY_DIR:         ${CMAKE_BINARY_DIR}")

MESSAGE( STATUS )
message( STATUS "BUILD_4_API = ${BUILD_4_API}")
MESSAGE( STATUS "CMAKE_SYSTEM_PROCESSOR = ${CMAKE_SYSTEM_PROCESSOR}" )
MESSAGE( STATUS "BUILD_SHARED_LIBS = ${BUILD_SHARED_LIBS}" )
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}" )
MESSAGE( STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}" )
MESSAGE( STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}" )
MESSAGE( STATUS "BUILD_UTILS= ${BUILD_UTILS}" )
MESSAGE( STATUS "BUILD_TESTS= ${BUILD_TESTS}" )
MESSAGE( STATUS "OPENCV_DIR= ${OpenCV_DIR}" )
MESSAGE( STATUS "OPEMMP= ${OpenMP_FOUND}" )
MESSAGE( STATUS "USE_OWN_EIGEN3= ${USE_OWN_EIGEN3}" )
IF(NOT USE_OWN_EIGEN3)
MESSAGE( STATUS "    EIGEN= ${EIGEN3_INCLUDE_DIR}" )
ENDIF()
MESSAGE( STATUS "BUILD_OWN_ARUCO= ${BUILD_OWN_ARUCO}" )
MESSAGE( STATUS "BUILD_OWN_FBOW= ${BUILD_OWN_FBOW}" )
MESSAGE( STATUS "BUILD_OWN_CVBA= ${BUILD_OWN_CVBA}" )
MESSAGE( STATUS "BUILD_OWN_XFLANN= ${BUILD_OWN_XFLANN}" )
MESSAGE( STATUS "BUILD_OWN_G2O= ${BUILD_OWN_G2O}" )
MESSAGE( STATUS "BUILD_GUI= ${BUILD_GUI}" )
MESSAGE( STATUS "BUILD_DEBPACKAGE= ${BUILD_DEBPACKAGE}" )

MESSAGE(STATUS "TSLAM_REQUIRED_LIBRARIES=${TSLAM_REQUIRED_LIBRARIES}")

MESSAGE( STATUS )
MESSAGE( STATUS "OpenCV_LIB_DIR=${OpenCV_LIB_DIR}")
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}")

MESSAGE( STATUS )
MESSAGE( STATUS )
MESSAGE( STATUS "Change a value with: cmake -D<Variable>=<Value>" )
MESSAGE( STATUS )
