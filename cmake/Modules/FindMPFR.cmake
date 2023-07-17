
find_library(MPFR_LIBRARIES NAMES mpfr)
find_path(MPFR_INCLUDE_DIRS NAMES mpfr.h)

if(MPFR_INCLUDE_DIR)
  file(STRINGS ${SCOTCH_INCLUDE_DIR}/scotch.h _versions
    REGEX "^#define\ +MPFR_VERSION_(MAJOR|MINOR|PATCHLEVEL) .*")
  foreach(_ver ${_versions})
    string(REGEX MATCH "MPFR_VERSION_(MAJOR|MINOR|PATCHLEVEL) +([0-9.]+)" _tmp "${_ver}")
    set(_mpfr_${CMAKE_MATCH_1} ${CMAKE_MATCH_2})
  endforeach()
  set(MPFR_VERSION
    "${_mpfr_MAJOR}.${_mpfr_MINOR}.${_mpfr_PATCHLEVEL}"
    CACHE INTERNAL "")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MPFR
  REQUIRED_VARS MPFR_INCLUDE_DIRS MPFR_LIBRARIES
  VERSION_VAR MPFR_VERSION)
mark_as_advanced(MPFR_INCLUDES MPFR_LIBRARIES)

if(NOT TARGET mpfr)
  add_library(mpfr UNKNOWN IMPORTED GLOBAL)
endif()

set_target_properties(mpfr PROPERTIES
  IMPORTED_LOCATION                 "${MPFR_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES     "${MPFR_INCLUDE_DIR}"
  IMPORTED_LINK_INTERFACE_LANGUAGES "C")

add_library(MPFR::mpfr ALIAS mpfr)
