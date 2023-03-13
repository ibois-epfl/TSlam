set(WARNINGS_ARE_ERRORS 		OFF CACHE BOOL "Treat warnings as errors")
set(WHOLE_PROGRAM_OPTIMIZATION 	OFF CACHE BOOL "Flags for whole program optimization.")

set(EXTRA_CXX_FLAGS "-shared -march=native -Wall")

 IF(USE_TIMERS)
 add_definitions(-DUSE_TIMERS)
 ENDIF()

 IF(NOT TARGET_PROCESSOR )
     SET(TARGET_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
 ENDIF()

 IF(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR  (CMAKE_CXX_COMPILER_ID MATCHES "Clang") )

 if(${TARGET_PROCESSOR} MATCHES armv7l) # ARM7
         set(GENERAL_FLAGS "${GENERAL_FLAGS} -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=hard ")
 elseif(${TARGET_PROCESSOR} MATCHES armv6l) # ARM6
         set(GENERAL_FLAGS "${GENERAL_FLAGS}  -mabi=aapcs-linux -marm  -march=armv6 -mfloat-abi=hard  -mfp16-format=none -mfpu=vfp -mlittle-endian -mpic-data-is-text-relative -mrestrict-it -msched-prolog -mstructure-size-boundary=0x20 -mtp=auto -mtls-dialect=gnu -munaligned-access -mvectorize-with-neon-quad")
 else() #x86_64

     SET(GENERAL_FLAGS "-std=c++14 -mmmx -msse -msse2 -msse3")
     add_definitions( -DUSE_SSE)
     IF(WARNINGS_ARE_ERRORS)
         SET(GENERAL_FLAGS   "${GENERAL_FLAGS}  -Werror -Wno-ignored-attributes  ")
     ENDIF()
     IF(USE_AVX)
         SET(GENERAL_FLAGS "${GENERAL_FLAGS}  -mavx ")
     ENDIF()
     add_definitions(-DUSE_AVX)
endif()


 SET(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS}  -O3 -g0  -DNDEBUG")
 SET(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS}  -O1 -g3  -DDEBUG -D_DEBUG -DPRINT_DEBUG_MESSAGES")
 SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS}  -O1 -g3  -D_DEBUG -DDEBUG -DPRINT_DEBUG_MESSAGES")

 ELSE()  # MSVC

ADD_DEFINITIONS(-DNOMINMAX)

IF(WIN32)
    SET(GENERAL_FLAGS "/arch:AVX ${OpenMP_CXX_FLAGS}")
    SET(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} ${GENERAL_FLAGS} /bigobj ")
    SET(CMAKE_CXX_FLAGS_RELEASE           "${CMAKE_CXX_FLAGS_RELEASE} ${GENERAL_FLAGS}  ")
ENDIF()

ENDIF()#END OF COMPILER SPECIFIC OPTIONS


SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")


