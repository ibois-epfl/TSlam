INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/utils)
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES})
ADD_EXECUTABLE(test_sequence test_sequence.cpp   )

ADD_EXECUTABLE(compare_tracks compare_tracks.cpp ../utils/dirreader.h logtools.cpp logtools.h)
ADD_EXECUTABLE(compare_tracks_analyze.cpp compare_tracks_analyze.cpp ../utils/dirreader.h logtools.cpp logtools.h)


###################################
##test for smartmaker
set(Boost_USE_STATIC_LIBS        OFF)
find_package( Boost   COMPONENTS   system  filesystem    iostreams  REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


