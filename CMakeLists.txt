# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)


project(tslam VERSION "1.1.0" LANGUAGES CXX)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11
set(CMAKE_CXX_STANDARD 11) # C++11...

if(NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE "Release" )
endif()

#------------------------------------------------------
# Lib Names and Dirs
#------------------------------------------------------
include(cmake/options.cmake)
include(cmake/compiler.cmake)
include(cmake/dependencies.cmake)


if(WIN32)
    # Postfix of DLLs:
    SET(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    SET(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION) 
endif()




add_subdirectory(src)
if(BUILD_UTILS)
add_subdirectory(utils)
endif()

# IF(BUILD_UTILS_RGBD)
# ADD_SUBDIRECTORY(utils_rgbd)
# ENDIF()
# IF (BUILD_UTILS_DEVELOPER)
#   ADD_SUBDIRECTORY(utils_developer)
# ENDIF()
# IF (BUILD_TESTS)
# ADD_SUBDIRECTORY(tests)
# #ADD_SUBDIRECTORY(old_tests)
# ENDIF()
# IF(BUILD_DEBUGTESTS)
# ADD_SUBDIRECTORY(debugtests)
# ENDIF()

# IF (BUILD_UTILS_ARRAY)
# ADD_SUBDIRECTORY(utils_array)
# ENDIF()

# IF (BUILD_UTILS_REALSENSE)
# ADD_SUBDIRECTORY(utils_realsense)
# ENDIF()


# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
# ----------------------------------------------------------------------------
# create configuration file from .in file (If you use windows take care with paths)
# ----------------------------------------------------------------------------
configure_file("${PROJECT_SOURCE_DIR}/cmake/config.cmake.in" "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" DESTINATION share/${PROJECT_NAME} )


include(cmake/display.cmake)


# Package Generator  #######################################################
if(BUILD_DEBPACKAGE)
include(cmake/cpack.cmake)
endif()
