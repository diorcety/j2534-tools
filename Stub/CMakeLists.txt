cmake_minimum_required(VERSION 3.0)

project(Stub)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_LOGGING")
IF (WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
ENDIF()

IF (UNIX)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
ENDIF()

# Sources

set(SOURCE_FILES ${SOURCE_FILES} Stub.h Stub.cpp)
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} j2534_v0404.h)
set(SOURCE_FILES ${SOURCE_FILES} log.h log.cpp)
set(SOURCE_FILES ${SOURCE_FILES} utils.h)

set(LIB_TYPE SHARED)
IF (WIN32)
set(SOURCE_EXTRA Stub.def)
ENDIF()

add_library(Stub ${LIB_TYPE} ${SOURCE_FILES} ${SOURCE_EXTRA})

IF (WIN32)
set_target_properties(Stub PROPERTIES OUTPUT_NAME "Stub")
set_target_properties(Stub PROPERTIES PREFIX "")
ENDIF()