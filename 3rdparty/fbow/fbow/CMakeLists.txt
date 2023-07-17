set(srcs fbow.cpp  vocabulary_creator.cpp )
set(hdrs fbow.h fbow_exports.h vocabulary_creator.h cpu.h)

add_library(tslam_fbow ${srcs} ${hdrs})

set_target_properties(tslam_fbow PROPERTIES  # create *nix style library versions + symbolic links
    DEFINE_SYMBOL FBOW_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1   # allow creating static and shared libs without conflicts
#    OUTPUT_NAME "tslam_bfow{PROJECT_DLLVERSION}" # avoid conflicts between library and binary target names
)

if(TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(tslam_fbow PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(tslam_fbow PRIVATE USE_OPENMP)
endif()

target_link_libraries(tslam_fbow PRIVATE ${OpenCV_LIBS})
target_include_directories(tslam_fbow
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(tslam_fbow PUBLIC -Wno-ignored-attributes)
endif()

list(APPEND TSLAM_EXPORT_LIST tslam_fbow)
# install(TARGETS tslam_bfow
#     RUNTIME
#       DESTINATION bin COMPONENT main # Install the dll file in bin directory
#     LIBRARY
#       DESTINATION lib
#       PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#       COMPONENT main
#     ARCHIVE
#       DESTINATION lib COMPONENT main) # Install the dll.a file in lib directory

# if(FBOW_DEVINSTALL)
#     install(FILES ${hdrs}
#         DESTINATION include/fbow
#         COMPONENT main)
# endif()
