
FILE(GLOB tslam_hdrs_base "*.h" )
FILE(GLOB tslam_srcs_base "*.cpp")

SET(tslam_fextractors_hdr featureextractors/ORBextractor.h featureextractors/feature2dserializable.h )
SET(tslam_fextractors_src featureextractors/ORBextractor.cpp featureextractors/feature2dserializable.cpp)
IF(${OpenCV_VERSION} STRGREATER 3)
    SET(tslam_fextractors_hdr ${tslam_fextractors_hdr} featureextractors/gridextractor.h )
    SET(tslam_fextractors_src ${tslam_fextractors_src} featureextractors/gridextractor.cpp )
ENDIF()
FILE(GLOB tslam_opt_src "optimization/*.cpp" )
FILE(GLOB tslam_opt_hdr "optimization/*.h" )
FILE(GLOB tslam_map_hdr "map_types/*.h" )
FILE(GLOB tslam_map_src "map_types/*.cpp" )
FILE(GLOB tslam_utils_hdr "utils/*.h" )
FILE(GLOB tslam_utils_src "utils/*.cpp" )
FILE(GLOB tslam_basictypes_hdr "basictypes/*.h" )
FILE(GLOB tslam_basictypes_src "basictypes/*.cpp" )




ADD_LIBRARY(${PROJECT_NAME}  ${tslam_hdrs_base} ${tslam_srcs_base} ${tslam_fextractors_src} ${tslam_fextractors_hdr}  ${tslam_basictypes_src} ${tslam_basictypes_hdr}  ${tslam_map_hdr} ${tslam_map_src} ${tslam_utils_hdr} ${tslam_utils_src} ${tslam_opt_src} ${tslam_opt_hdr})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES 				# create *nix style library versions + symbolic links
    DEFINE_SYMBOL TSLAM_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
    OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
    )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${TSLAM_REQUIRED_LIBRARIES} )

INSTALL(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory



INSTALL(FILES ${tslam_hdrs_base}
    DESTINATION include/${PROJECT_NAME}
    COMPONENT main)

#INSTALL(FILES ${tslam_opt_hdr}   DESTINATION include/${PROJECT_NAME}/optimizers   COMPONENT main)
#INSTALL(FILES ${tslam_fextractors_hdr} DESTINATION include/${PROJECT_NAME}/featureextractors COMPONENT main)
#INSTALL(FILES ${tslam_utils_hdr}       DESTINATION include/${PROJECT_NAME}/utils COMPONENT main)
INSTALL(FILES ${tslam_map_hdr}         DESTINATION include/${PROJECT_NAME}/map_types COMPONENT main)
INSTALL(FILES ${tslam_basictypes_hdr}         DESTINATION include/${PROJECT_NAME}/basictypes COMPONENT main)

