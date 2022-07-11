SET (LIBNAME ${EXTRALIBNAME}stag)

find_package(OpenCV REQUIRED)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
file(GLOB SRC_FILE1 "./*.c*")
file(GLOB SRC_FILE2 "./ED/*.c*")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ./
    ./ED/
)

add_library(
    ${LIBNAME}
    ${SRC_FILE1}
    ${SRC_FILE2}
)

target_link_libraries(
    ${LIBNAME}
    ${OpenCV_LIBS}
)

INSTALL(TARGETS ${LIBNAME}
        RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory


IF(STAG_DEVINSTALL)
    INSTALL(FILES ${SRC_FILE1} DESTINATION include/stag COMPONENT main)
    INSTALL(FILES ${SRC_FILE2} DESTINATION include/stag COMPONENT main)
ENDIF()