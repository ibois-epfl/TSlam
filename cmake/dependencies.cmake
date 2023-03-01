# ----------------------------------------------------------------------------
#   Find Dependencies
# ----------------------------------------------------------------------------
find_package(OpenCV 4.5.5 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(TSLAM_REQUIRED_LIBRARIES ${OpenCV_LIBS})

find_package(OpenMP)
if(OpenMP_FOUND)
    set (TSLAM_REQUIRED_LIBRARIES ${TSLAM_REQUIRED_LIBRARIES} ${OpenMP_CXX_LIBRARIES})
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_compile_options(-DUSE_OPENMP)
endif()
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
link_directories(${CMAKE_BINARY_DIR}/libs)

if(NOT USE_OWN_EIGEN3)
    find_package( Eigen3 REQUIRED )
else()
    set(EIGEN3_INCLUDE_DIR "3rdparty/eigen3/eigen3")
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "3rdparty/eigen3/eigen3")
endif()
include_directories( ${EIGEN3_INCLUDE_DIR} )


set(EXTRALIBNAME "${PROJECT_NAME}_")


add_definitions(-DXFLANN_OPENCV)
if(NOT BUILD_OWN_XFLANN)
    find_package(xflann   REQUIRED)
    set(TSLAM_XFLANN_LIBS ${xflann_LIBS})
else()
    option(XFLANN_OPENCV "Set on/off" ON)
    add_subdirectory(3rdparty/xflann/xflann)
    include_directories(3rdparty/xflann)
    set(TSLAM_XFLANN_LIBS ${EXTRALIBNAME}xflann)
endif()


if(NOT BUILD_OWN_G2O)
    link_directories(${G2O_DIR}/lib)
    include_directories(${G2O_DIR}/include)
else()
    include_directories(3rdparty/g2o/)
    ADD_SUBDIRECTORY(3rdparty/g2o/g2o)
endif()

set(G2O_LIBS    ${EXTRALIBNAME}g2o_stuff  ${EXTRALIBNAME}g2o_core       )

if(CHANGED_BUILD_TYPE STREQUAL "YES")
set(CMAKE_BUILD_TYPE "Debug")
endif()

if(NOT BUILD_OWN_FBOW)
    find_package(fbow   REQUIRED)
    set(TSLAM_FBOW_LIBS ${fbow_LIBS})
else()
    add_subdirectory(3rdparty/fbow/fbow)
    include_directories(3rdparty/fbow)
    set(TSLAM_FBOW_LIBS ${EXTRALIBNAME}fbow)
endif()

if(NOT BUILD_OWN_STAG)
    find_package(stag   REQUIRED)
    set(TSLAM_STAG_LIBS ${stag_LIBS})
else()
    add_subdirectory(3rdparty/stag/stag)
    include_directories(3rdparty/stag)
    set(TSLAM_STAG_LIBS ${EXTRALIBNAME}stag)
endif()

if(NOT BUILD_OWN_ARUCO)
    find_package(aruco   REQUIRED)
    set(TSLAM_ARUCO_LIBS ${aruco_LIBS})
else()
    add_subdirectory(3rdparty/aruco/aruco)
    include_directories(3rdparty/aruco)
    set(TSLAM_ARUCO_LIBS ${EXTRALIBNAME}aruco)
endif()

# FIXME: for tslam reconstruction integration
if(NOT USE_OWN_CGAL)
    find_package( CGAL 5.5.1 REQUIRED )
else()
    set(CGAL_INCLUDE_DIR "3rdparty/CGAL/include")
endif()
include_directories( ${CGAL_INCLUDE_DIR} )

if(NOT USE_OWN_CGAL)
    find_package( cilantro REQUIRED )
else()
    set(CILANTRO_INCLUDE_DIR "3rdparty/cilantro/include")
endif()
include_directories( ${CILANTRO_INCLUDE_DIR} )

if(NOT USE_OWN_GMP)
    find_library(GMP_LIB NAMES "gmp" PATHS ${CMAKE_LIBRARY_PATH}} )
    if(NOT GMP_LIB_FOUND)
        message(STATUS "GMP not found, trying to install it with apt-get...")
        execute_process(COMMAND sudo apt-get install libgmp-dev)
    endif()
else()
    set(GMP_INCLUDE_DIR "3rdparty/gmp/include")
endif()

if(NOT USE_OWN_MPFR)
    find_library(MPFR_LIB NAMES "MPFR" PATHS ${CMAKE_LIBRARY_PATH}} )
    if(NOT MPFR_LIB_FOUND)
        message(STATUS "MPFT not found, trying to install it with apt-get...")
        execute_process(COMMAND sudo apt-get install libmpfr-dev)
    endif()
else()
    set(MPFR_INCLUDE_DIR "3rdparty/mpfr/include")
endif()
link_libraries(gmp mpfr)

if(XFEATURES2D)
add_definitions(-DXFEATURES2D)
endif()

set(TSLAM_REQUIRED_LIBRARIES
    ${TSLAM_REQUIRED_LIBRARIES}
    ${TSLAM_FBOW_LIBS}
    ${TSLAM_ARUCO_LIBS}
    ${TSLAM_STAG_LIBS}
    ${TSLAM_XFLANN_LIBS}
    # ${TSLAM_O3D_LIBS} # FIXME: for tslam reconstruction integration
    ${G2O_LIBS})

#Find OpenNI2
### OPENNI 2
set(OPENNI2LIB_FOUND "NO" )

if(NOT WIN32)
  find_path(OpenNI2_INCLUDE  NAMES OpenNI2/OpenNI.h openni2/OpenNI.h)
  find_library(LIBOPENNI2_LIBRARY NAMES OpenNI2  )
else()
  find_path(OpenNI2_INCLUDE  NAMES OpenNI.h PATHS  ${OPENNI2_DIR}/Include/ )
  find_library(LIBOPENNI2_LIBRARY NAMES OpenNI2 PATHS ${OPENNI2_DIR}/Lib )
endif()

#message(FATAL_ERROR "NI=${OpenNI2_INCLUDE}")
if ( (OpenNI2_INCLUDE STREQUAL "OpenNI2_INCLUDE-NOTFOUND") OR (LIBOPENNI2_LIBRARY STREQUAL "LIBOPENNI2_LIBRARY-NOTFOUND"))
       message(STATUS_MESSAGE "OpenNi  not found inc=${OpenNI2_INCLUDE}")
  else()
    if (WIN32)
        include_directories(${OpenNI2_INCLUDE})
    else()
      include_directories(${OpenNI2_INCLUDE}/openni2)
    endif()
    message(STATUS  "OpenNI.h=${OpenNI2_INCLUDE} LIBOPENNI2_LIBRARY=${LIBOPENNI2_LIBRARY}")
    set(OPENNI2LIB_FOUND "YES" )
 endif()