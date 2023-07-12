

IF(NOT TARGET_PROCESSOR )
    SET(TARGET_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
ENDIF()

IF(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR  (CMAKE_CXX_COMPILER_ID MATCHES "Clang") )

    if(${TARGET_PROCESSOR} MATCHES armv7l) # In ARM_COrtex8 with neon, enalble vectorized operations
        set(GENERAL_FLAGS "${GENERAL_FLAGS} -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=hard ")
        SET(ADD_LINK_LIBS -latomic)#for raspian, Opencv has not included it
    endif()
    if(${TARGET_PROCESSOR} MATCHES armv6l) # In ARM_COrtex8 with neon, enalble vectorized operations
        set(GENERAL_FLAGS "${GENERAL_FLAGS}  -mabi=aapcs-linux -marm  -march=armv6 -mfloat-abi=hard  -mfp16-format=none -mfpu=vfp -mlittle-endian -mpic-data-is-text-relative -mrestrict-it -msched-prolog -mstructure-size-boundary=0x20 -mtp=auto -mtls-dialect=gnu -munaligned-access -mvectorize-with-neon-quad")
        SET(ADD_LINK_LIBS -latomic)#for raspian, Opencv has not included it
    endif()


    SET(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS}  -O3 -g0  -DNDEBUG")
    SET(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS}  -O0 -g3  -DDEBUG -D_DEBUG -DPRINT_DEBUG_MESSAGES")
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${CMAKE_CXX_FLAGS} ${GENERAL_FLAGS}  -O1 -g3  -D_DEBUG -DDEBUG -DPRINT_DEBUG_MESSAGES")


ELSE()  # MSVC
    ADD_DEFINITIONS(-DNOMINMAX)
ENDIF()#END OF COMPILER SPECIFIC OPTIONS



SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS} ${ADD_LINK_LIBS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")
