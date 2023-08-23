add_library(tslam_g2o_stuff
    color_macros.h
    macros.h
    misc.h
    os_specific.c
    os_specific.h
    property.cpp
    property.h
    string_tools.cpp
    string_tools.h
    timeutil.cpp
    timeutil.h)
target_include_directories(tslam_g2o_stuff PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(tslam_g2o_stuff PUBLIC tslam_g2o_config Eigen3::Eigen)

set_target_properties(tslam_g2o_stuff PROPERTIES # create *nix style library versions + symbolic links
    DEFINE_SYMBOL stuff_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1 # allow creating static and shared libs without conflicts
#    OUTPUT_NAME "${THIS_LIBNAME}${PROJECT_DLLVERSION}" # avoid conflicts between library and binary target names
)

if(WIN32)
    target_compile_definitions(tslam_g2o_stuff PUBLIC WINDOWS)
endif()

# install(TARGETS tslam_g2o_stuff
#     RUNTIME DESTINATION bin COMPONENT main # Install the dll file in bin directory
#     LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
#     ARCHIVE DESTINATION lib COMPONENT main) # Install the dll.a file in lib directory
