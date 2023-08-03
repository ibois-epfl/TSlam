# main TSLAM cmakefile

file(GLOB tslam_base_hdrs "*.h" )
file(GLOB tslam_base_srcs "*.cpp")

set(tslam_fextractors_hdrs featureextractors/ORBextractor.h featureextractors/feature2dserializable.h )
set(tslam_fextractors_srcs featureextractors/ORBextractor.cpp featureextractors/feature2dserializable.cpp)
IF(${OpenCV_VERSION} STRGREATER 3)
    set(tslam_fextractors_hdrs ${tslam_fextractors_hdrs} featureextractors/gridextractor.h )
    set(tslam_fextractors_srcs ${tslam_fextractors_srcs} featureextractors/gridextractor.cpp )
ENDIF()
file(GLOB tslam_opt_srcs "optimization/*.cpp" )
file(GLOB tslam_opt_hdrs "optimization/*.h" )
file(GLOB tslam_map_hdrs "map_types/*.h" )
file(GLOB tslam_map_srcs "map_types/*.cpp" )
file(GLOB tslam_utils_hdrs "utils/*.h" )
file(GLOB tslam_utils_srcs "utils/*.cpp" )
file(GLOB tslam_basictypes_hdrs "basictypes/*.h" )
file(GLOB tslam_basictypes_srcs "basictypes/*.cpp" )
file(GLOB tslam_reconstruction_hdrs "reconstruction/*.hh" )
file(GLOB tslam_reconstruction_srcs "reconstruction/*.cc" )

add_library(tslam
    ${tslam_base_srcs}
    ${tslam_basictypes_srcs}
    ${tslam_fextractors_srcs}
    ${tslam_map_srcs}
    ${tslam_opt_srcs}
    ${tslam_reconstruction_srcs}
    ${tslam_utils_srcs}
    ${tslam_base_hdrs}
    ${tslam_basictypes_hdrs}
    ${tslam_fextractors_hdrs}
    ${tslam_map_hdrs}
    ${tslam_opt_hdrs}
    ${tslam_utils_hdrs}
)

set_target_properties(tslam PROPERTIES
    DEFINE_SYMBOL TSLAM_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1
    OUTPUT_NAME "${PROJECT_NAME}"
)

target_link_libraries(tslam
    PRIVATE ${TSLAM_PRIVATE_EXTERNAL_LIBRARIES}
    PUBLIC ${TSLAM_PUBLIC_EXTERNAL_LIBRARIES})

target_include_directories(tslam
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/map_types>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/basictypes>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/reconstruction>
      ${TSLAM_PRIVATE_EXTERNAL_INCLUDE_DIRS}
    INTERFACE
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}/map_types>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}/basictypes>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}/reconstruction>
    PUBLIC
      ${TSLAM_PUBLIC_EXTERNAL_INCLUDE_DIRS}
)

set_property(TARGET tslam APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/map_types>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/basictypes>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/reconstruction>
)

target_compile_features(tslam PUBLIC cxx_std_17)

if(TSLAM_XFEATURES2D)
    target_compile_definitions(tslam PRIVATE XFEATURES2D)
endif()

if(TSLAM_USE_TIMERS)
    target_compile_definitions(tslam PUBLIC USE_TIMERS)
endif()

add_library(TSlam::tslam ALIAS tslam)

if(NOT TSLAM_TARGETS_EXPORT)
  set(TSLAM_TARGETS_EXPORT ${PROJECT_NAME}Targets)
endif()

list(APPEND TSLAM_EXPORT_LIST tslam)

include(GNUInstallDirs)
install(TARGETS ${TSLAM_EXPORT_LIST}
    EXPORT ${TSLAM_TARGETS_EXPORT}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      COMPONENT main
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      COMPONENT main
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      COMPONENT main
)

install(EXPORT ${TSLAM_TARGETS_EXPORT}
    NAMESPACE TSlam::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    COMPONENT Developpment)

# Export for build tree
export(EXPORT ${TSLAM_TARGETS_EXPORT}
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${TSLAM_TARGETS_EXPORT}.cmake")

install(FILES ${tslam_base_hdrs}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    COMPONENT Developpment
)

install(FILES ${tslam_map_hdrs}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/map_types
    COMPONENT Developpment)
install(FILES ${tslam_basictypes_hdrs}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/basictypes
    COMPONENT Developpment)
install(FILES ${tslam_reconstruction_hdrs}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/reconstruction
    COMPONENT Developpment)

include(CMakePackageConfigHelpers)
configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  )

write_basic_package_version_file(${PROJECT_BINARY_DIR}/TSlamConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(
  FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
