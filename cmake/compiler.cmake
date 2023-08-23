set(WARNINGS_ARE_ERRORS 		OFF CACHE BOOL "Treat warnings as errors")
set(WHOLE_PROGRAM_OPTIMIZATION 	OFF CACHE BOOL "Flags for whole program optimization.")

set(EXTRA_CXX_FLAGS "-shared -march=native -Wall")

if(NOT TARGET_PROCESSOR )
  SET(TARGET_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR  (CMAKE_CXX_COMPILER_ID MATCHES "Clang") )

  if(${TARGET_PROCESSOR} MATCHES armv7l) # ARM7
    set(GENERAL_FLAGS "${GENERAL_FLAGS} -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=hard ")
  elseif(${TARGET_PROCESSOR} MATCHES armv6l) # ARM6
    set(GENERAL_FLAGS "${GENERAL_FLAGS}  -mabi=aapcs-linux -marm  -march=armv6 -mfloat-abi=hard  -mfp16-format=none -mfpu=vfp -mlittle-endian -mpic-data-is-text-relative -mrestrict-it -msched-prolog -mstructure-size-boundary=0x20 -mtp=auto -mtls-dialect=gnu -munaligned-access -mvectorize-with-neon-quad")
  else() #x86_64
    #SET(GENERAL_FLAGS "-std=c++14")

    add_definitions( -DUSE_SSE)
    if(WARNINGS_ARE_ERRORS)
      SET(GENERAL_FLAGS   "${GENERAL_FLAGS}  -Werror -Wno-ignored-attributes  ")
    endif()

    if(TSLAM_USE_AVX)
      SET(GENERAL_FLAGS "${GENERAL_FLAGS}  -mavx ")
    endif()
    add_definitions(-DUSE_AVX)
  endif()


  set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS} -O3 -g  -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS} -O1 -g3 -DDEBUG -D_DEBUG -DPRINT_DEBUG_MESSAGES")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS} -O2 -g3 -DNDEBUG")
else()  # MSVC
  ADD_DEFINITIONS(-DNOMINMAX)

  if(WIN32)
    SET(GENERAL_FLAGS "/arch:AVX ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} ${GENERAL_FLAGS} /bigobj ")
    SET(CMAKE_CXX_FLAGS_RELEASE           "${CMAKE_CXX_FLAGS_RELEASE} ${GENERAL_FLAGS}  ")
  endif()

endif()#END OF COMPILER SPECIFIC OPTIONS


set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")
