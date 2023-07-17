# ----------------------------------------------------------------------------
#   Find Dependencies
# ----------------------------------------------------------------------------
set(TSLAM_PUBLIC_EXTERNAL_INCLUDE_DIRS)
set(TSLAM_PUBLIC_EXTERNAL_LIBRARIES)
set(TSLAM_PRIVATE_EXTERNAL_INCLUDE_DIRS)
set(TSLAM_PRIVATE_EXTERNAL_LIBRARIES)
set(TSLAM_PUBLIC_DEFINITIONS)
set(TSLAM_EXPORT_LIST)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

find_package(OpenCV 4.5.4 REQUIRED COMPONENTS )
#list(APPEND TSLAM_PUBLIC_EXTERNAL_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND TSLAM_PUBLIC_EXTERNAL_LIBRARIES ${OpenCV_LIBS})

find_package(OpenMP COMPONENTS CXX)
if(OpenMP_FOUND)
    list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES OpenMP::OpenMP_CXX)
    list(APPEND TSLAM_PUBLIC_DEFINITIONS USE_OPENMP)
endif()

find_package(TBB REQUIRED)
list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES TBB::tbb)

if(NOT TSLAM_USE_OWN_EIGEN3)
    find_package(Eigen3 REQUIRED)
else()
    add_library(eigen INTERFACE)
    target_include_directories(eigen INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty/eigen3/eigen3>)
    add_library(Eigen3::Eigen ALIAS eigen)
    list(APPEND TSLAM_EXPORT_LIST eigen)
endif()
list(APPEND TSLAM_PUBLIC_EXTERNAL_LIBRARIES Eigen3::Eigen)

# XLFANN (depends OpenCV)
set(XFLANN_OPENCV ON)
add_subdirectory(3rdparty/xflann/xflann)
list(APPEND TSLAM_PUBLIC_EXTERNAL_LIBRARIES tslam_xflann)
list(APPEND TSLAM_EXPORT_LIST tslam_xflann)

# g2o (depends OpenCV Eigen)
add_subdirectory(3rdparty/g2o/g2o)
list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES tslam_g2o_core)
list(APPEND TSLAM_EXPORT_LIST tslam_g2o_core tslam_g2o_stuff tslam_g2o_config)

if(CHANGED_BUILD_TYPE STREQUAL "YES")
    set(CMAKE_BUILD_TYPE "Debug")
endif()

add_subdirectory(3rdparty/fbow/fbow)
list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES tslam_fbow)
list(APPEND TSLAM_EXPORT_LIST tslam_fbow)

add_subdirectory(3rdparty/stag/stag)
list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES tslam_stag)
list(APPEND TSLAM_EXPORT_LIST tslam_stag)

add_subdirectory(3rdparty/aruco/aruco)
list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES tslam_aruco)
list(APPEND TSLAM_EXPORT_LIST tslam_aruco)

find_package(MPFR REQUIRED)
list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES MPFR::mpfr)

# FIXME: for tslam reconstruction integration
if(NOT TSLAM_USE_OWN_CGAL)
    find_package(CGAL QUIET COMPONENTS Core )
    if(CGAL_FOUND)
        #Don't let CGAL override flags
        set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")
        include(${CGAL_USE_FILE})
    endif()
else()
    set(CGAL_INCLUDE_DIR
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty/CGAL/include>)
endif()
list(APPEND TSLAM_PRIVATE_EXTERNAL_INCLUDE_DIRS ${CGAL_INCLUDE_DIR})

if(NOT TSLAM_USE_OWN_CILANTRO)
    find_package(cilantro REQUIRED)
else()
    set(CILANTRO_INCLUDE_DIR
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty/cilantro/include>)
endif()
list(APPEND TSLAM_PRIVATE_EXTERNAL_INCLUDE_DIRS ${CILANTRO_INCLUDE_DIR})

find_package(Boost REQUIRED)
list(APPEND TSLAM_PRIVATE_EXTERNAL_INCLUDE_DIRS ${Boost_INCLUDE_DIR})
