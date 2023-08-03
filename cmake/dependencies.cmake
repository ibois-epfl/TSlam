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
include(cmake/external_tools.cmake)

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
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty/eigen3/eigen3>
        $<INSTALL_INTERFACE:include/eigen3>)
    add_library(Eigen3::Eigen ALIAS eigen)

    set(_eigen_files_path ${PROJECT_SOURCE_DIR}/3rdparty/eigen3/eigen3/Eigen)
    file(GLOB _eigen_directory_files "${_eigen_files_path}/*")

    set(_eigen_directory_files_to_install)
    foreach(f ${_eigen_directory_files})
        if(NOT (IS_DIRECTORY "${f}" OR "${f}" MATCHES "\\.txt"))
            list(APPEND _eigen_directory_files_to_install ${f})
        endif()
    endforeach()

    include(GNUInstallDirs)
    install(FILES ${_eigen_directory_files_to_install}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/eigen3/Eigen
        COMPONENT Developpment)

    install(DIRECTORY ${_eigen_files_path}/src
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/eigen3/Eigen
        COMPONENT Devel
        FILES_MATCHING PATTERN "*.h")

    mark_as_advanced_prefix(EIGEN)
    list(APPEND TSLAM_EXPORT_LIST eigen)
endif()
list(APPEND TSLAM_PUBLIC_EXTERNAL_LIBRARIES Eigen3::Eigen)

# XLFANN (depends OpenCV)
set(XFLANN_OPENCV ON)
foreach(_deps xflann g2o fbow stag aruco)
    add_subdirectory(3rdparty/${_deps}/${_deps})
    list(APPEND TSLAM_PUBLIC_EXTERNAL_LIBRARIES tslam_${_deps})
    list(APPEND TSLAM_EXPORT_LIST tslam_${_deps})
endforeach()
list(APPEND TSLAM_EXPORT_LIST tslam_g2o_config tslam_g2o_stuff)


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

    find_package(MPFR REQUIRED)
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rdparty/CGAL/cmake/modules)
    list(APPEND TSLAM_PRIVATE_EXTERNAL_LIBRARIES MPFR::mpfr)
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
