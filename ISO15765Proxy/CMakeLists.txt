cmake_minimum_required(VERSION 3.0)

project(ISO15765Proxy)

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
set(COMMON_FILES ${COMMON_FILES} internal.cpp internal.h)
set(COMMON_FILES ${COMMON_FILES} iso15765.cpp iso15765.h)
set(COMMON_FILES ${COMMON_FILES} configurable_channel.cpp configurable_channel.h)
set(COMMON_FILES ${COMMON_FILES} simple.cpp simple.h)
set(COMMON_FILES ${COMMON_FILES} log.h log.cpp)
set(COMMON_FILES ${COMMON_FILES} utils.h)

set(SOURCE_FILES ${SOURCE_FILES} ISO15765Proxy.h ISO15765Proxy.cpp)
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} j2534_v0404.h)
set(SOURCE_FILES ${SOURCE_FILES} ${COMMON_FILES})

set(LIB_TYPE SHARED)
IF (WIN32)
set(SOURCE_EXTRA ISO15765Proxy.def)
ENDIF()

add_library(ISO15765Proxy ${LIB_TYPE} ${SOURCE_FILES} ${SOURCE_EXTRA})

IF (WIN32)
set_target_properties(ISO15765Proxy PROPERTIES OUTPUT_NAME "ISO15765Proxy")
set_target_properties(ISO15765Proxy PROPERTIES PREFIX "")
ENDIF()

IF (WIN32)
find_library(SHLWAPI shlwapi.lib)
target_link_libraries(ISO15765Proxy SHLWAPI)
ENDIF()

add_executable(demo test.cpp ${COMMON_FILES})
IF (UNIX)
target_link_libraries(demo -lpthread)
ENDIF()