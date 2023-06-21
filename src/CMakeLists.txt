# main TSLAM cmakefile

file(GLOB tslam_hdrs_base "*.h" )
file(GLOB tslam_srcs_base "*.cpp")

set(tslam_fextractors_hdr featureextractors/ORBextractor.h featureextractors/feature2dserializable.h )
set(tslam_fextractors_src featureextractors/ORBextractor.cpp featureextractors/feature2dserializable.cpp)
IF(${OpenCV_VERSION} STRGREATER 3)
    set(tslam_fextractors_hdr ${tslam_fextractors_hdr} featureextractors/gridextractor.h )
    set(tslam_fextractors_src ${tslam_fextractors_src} featureextractors/gridextractor.cpp )
ENDIF()
file(GLOB tslam_opt_src "optimization/*.cpp" )
file(GLOB tslam_opt_hdr "optimization/*.h" )
file(GLOB tslam_map_hdr "map_types/*.h" )
file(GLOB tslam_map_src "map_types/*.cpp" )
file(GLOB tslam_utils_hdr "utils/*.h" )
file(GLOB tslam_utils_src "utils/*.cpp" )
file(GLOB tslam_basictypes_hdr "basictypes/*.h" )
file(GLOB tslam_basictypes_src "basictypes/*.cpp" )
file(GLOB tslam_reconstruction_hdr "reconstruction/*.hh" )
file(GLOB tslam_reconstruction_src "reconstruction/*.cc" )

add_library(${PROJECT_NAME}
            ${tslam_hdrs_base}
            ${tslam_srcs_base}
            ${tslam_fextractors_src}
            ${tslam_fextractors_hdr}
            ${tslam_basictypes_src}
            ${tslam_basictypes_hdr}
            ${tslam_map_hdr}
            ${tslam_map_src}
            ${tslam_utils_hdr}
            ${tslam_utils_src}
            ${tslam_opt_src}
            ${tslam_opt_hdr}
            ${tslam_reconstruction_hdr}
            ${tslam_reconstruction_src}
            )

set_target_properties(${PROJECT_NAME} PROPERTIES
    DEFINE_SYMBOL TSLAM_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1
    OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}"
    )
target_link_libraries(${PROJECT_NAME} ${TSLAM_REQUIRED_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin COMPONENT main
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)

install(FILES ${tslam_hdrs_base}
    DESTINATION include/${PROJECT_NAME}
    COMPONENT main)

install(FILES ${tslam_map_hdr}         DESTINATION include/${PROJECT_NAME}/map_types COMPONENT main)
install(FILES ${tslam_basictypes_hdr}  DESTINATION include/${PROJECT_NAME}/basictypes COMPONENT main)
install(FILES ${tslam_reconstruction_hdr}         DESTINATION include/${PROJECT_NAME}/reconstruction COMPONENT main)
