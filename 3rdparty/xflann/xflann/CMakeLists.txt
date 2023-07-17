file(GLOB impl_srcs "impl/*.cpp" )

add_library(tslam_xflann index.cpp ${impl_srcs})

if(XFLANN_OPENCV)
    target_compile_definitions(tslam_xflann PUBLIC XFLANN_OPENCV)
endif()

set_target_properties(tslam_xflann PROPERTIES # create *nix style library versions + symbolic links
    DEFINE_SYMBOL XFLANN_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1 # allow creating static and shared libs without conflicts
#    OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}" # avoid conflicts between library and binary target names
)

if(XFLANN_OPENCV)
    target_link_libraries(tslam_xflann PUBLIC ${OpenCV_LIBS})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(tslam_xflann PRIVATE -Wno-ignored-attributes)
endif()

target_include_directories(tslam_xflann
    PRIVATE impl
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)


# install(TARGETS ${LIBNAME}
#     RUNTIME DESTINATION bin COMPONENT main # Install the dll file in bin directory
#     LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
#     ARCHIVE DESTINATION lib COMPONENT main) # Install the dll.a file in lib directory

# if(XFLANN_DEVINSTALL)
#     INSTALL(FILES ${hdrs_base}
#         DESTINATION include/xflann
#         COMPONENT main)
# endif()
