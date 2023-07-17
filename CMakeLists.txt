#------------------------------------------------------------------------------------------------------------
#   Basic Configuration
#------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)


project(TSlam VERSION "1.1.0" LANGUAGES CXX)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
#set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11
#set(CMAKE_CXX_STANDARD 17) # C++11...

# Set the build type for API integration into third software or not
if(BUILD_4_API)
    set(TSLAM_BUILD_UTILS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
endif()

#------------------------------------------------------------------------------------------------------------
# Lib Names and Dirs
#------------------------------------------------------------------------------------------------------------

include(cmake/options.cmake)
include(cmake/compiler.cmake)
include(cmake/dependencies.cmake)

if(WIN32)
    # Postfix of DLLs:
    set(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    set(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION) 
endif()

add_subdirectory(src)

if(TSLAM_BUILD_UTILS)
    add_subdirectory(utils)
endif()

# install CGAL
install(DIRECTORY 3rdparty/CGAL/include/CGAL DESTINATION include)

#------------------------------------------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
#------------------------------------------------------------------------------------------------------------
#configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


#------------------------------------------------------------------------------------------------------------
# create configuration file from .in file (If you use windows take care with paths)
#------------------------------------------------------------------------------------------------------------

configure_file("${PROJECT_SOURCE_DIR}/cmake/config.cmake.in" "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" DESTINATION share/${PROJECT_NAME} )

include(cmake/display.cmake)

#------------------------------------------------------------------------------------------------------------
# Download tslam vocabulary file
#------------------------------------------------------------------------------------------------------------

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/assets/voc/orb.fbow")
    message(STATUS "Downloading vocabulary file from https://zenodo.org/record/7738721/files/orb.fbow")
    file(DOWNLOAD "https://zenodo.org/record/7738721/files/orb.fbow" "${PROJECT_SOURCE_DIR}/assets/voc/orb.fbow")
endif()


#------------------------------------------------------------------------------------------------------------
# Testing
#------------------------------------------------------------------------------------------------------------

if(TSLAM_BUILD_TESTS)
    add_subdirectory(tests)
endif()

#------------------------------------------------------------------------------------------------------------
# CPack packaging
#------------------------------------------------------------------------------------------------------------

if(TSLAM_BUILD_DEBPACKAGE)
    include(cmake/cpack.cmake)
endif()
