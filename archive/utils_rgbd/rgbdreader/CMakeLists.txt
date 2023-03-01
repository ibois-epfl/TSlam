set(rgbdreader_files rgbdreader.cpp filesystem.cpp sun3dreader.cpp rgbdreaderfactory.cpp nyureader.cpp redwoodreader.cpp transform3d.cpp)

if(OPENNI2LIB_FOUND)
    set(rgbdreader_files ${rgbdreader_files} onireader.cpp)
endif()

add_library(rgbdreader ${rgbdreader_files})

if(OPENNI2LIB_FOUND)
    target_link_libraries(rgbdreader ${LIBOPENNI2_LIBRARY})
endif()

install(TARGETS rgbdreader DESTINATION lib)
# install(FILES filesystem.h rgbdreader.h rgbdreaderfactory.h DESTINATION include/rgbdr/)
