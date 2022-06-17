# Install script for directory: /home/tpp/Downloads/trunk/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so.1.2.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so.1.2"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/tpp/Downloads/trunk/cmake/libs/libreslam.so.1.2.4"
    "/home/tpp/Downloads/trunk/cmake/libs/libreslam.so.1.2"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so.1.2.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so.1.2"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/tpp/Downloads/trunk/cmake/libs:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES "/home/tpp/Downloads/trunk/cmake/libs/libreslam.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so"
         OLD_RPATH "/home/tpp/Downloads/trunk/cmake/libs:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreslam.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/reslam" TYPE FILE FILES
    "/home/tpp/Downloads/trunk/src/imageparams.h"
    "/home/tpp/Downloads/trunk/src/map.h"
    "/home/tpp/Downloads/trunk/src/mapviewer.h"
    "/home/tpp/Downloads/trunk/src/reslam.h"
    "/home/tpp/Downloads/trunk/src/reslam_exports.h"
    "/home/tpp/Downloads/trunk/src/reslamtypes.h"
    "/home/tpp/Downloads/trunk/src/stereorectify.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/reslam/map_types" TYPE FILE FILES
    "/home/tpp/Downloads/trunk/src/map_types/covisgraph.h"
    "/home/tpp/Downloads/trunk/src/map_types/frame.h"
    "/home/tpp/Downloads/trunk/src/map_types/keyframedatabase.h"
    "/home/tpp/Downloads/trunk/src/map_types/mappoint.h"
    "/home/tpp/Downloads/trunk/src/map_types/marker.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/reslam/basictypes" TYPE FILE FILES
    "/home/tpp/Downloads/trunk/src/basictypes/cvversioning.h"
    "/home/tpp/Downloads/trunk/src/basictypes/debug.h"
    "/home/tpp/Downloads/trunk/src/basictypes/expansiblecontainer.h"
    "/home/tpp/Downloads/trunk/src/basictypes/fastmat.h"
    "/home/tpp/Downloads/trunk/src/basictypes/flag.h"
    "/home/tpp/Downloads/trunk/src/basictypes/hash.h"
    "/home/tpp/Downloads/trunk/src/basictypes/heap.h"
    "/home/tpp/Downloads/trunk/src/basictypes/io_utils.h"
    "/home/tpp/Downloads/trunk/src/basictypes/minmaxbags.h"
    "/home/tpp/Downloads/trunk/src/basictypes/misc.h"
    "/home/tpp/Downloads/trunk/src/basictypes/osadapter.h"
    "/home/tpp/Downloads/trunk/src/basictypes/picoflann.h"
    "/home/tpp/Downloads/trunk/src/basictypes/reusablecontainer.h"
    "/home/tpp/Downloads/trunk/src/basictypes/safemap.h"
    "/home/tpp/Downloads/trunk/src/basictypes/se3.h"
    "/home/tpp/Downloads/trunk/src/basictypes/se3transform.h"
    "/home/tpp/Downloads/trunk/src/basictypes/sgl.h"
    "/home/tpp/Downloads/trunk/src/basictypes/timers.h"
    "/home/tpp/Downloads/trunk/src/basictypes/tsqueue.h"
    )
endif()

