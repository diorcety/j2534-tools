cmake_minimum_required(VERSION 3.0)
project(FTDIProxy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_LOGGING")
IF (WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
ENDIF()

IF (UNIX)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
ENDIF()

# Sources
set(SOURCE_FILES ${SOURCE_FILES} log.h log.cpp)
set(SOURCE_FILES ${SOURCE_FILES} FTDIProxy.h FTDIProxy.cpp)
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} utils.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} FTD2XX.H)

set(LIB_TYPE SHARED)
IF (WIN32)
set(SOURCE_EXTRA FTDIProxy.def)
ENDIF ()

IF (WIN32)
add_library(FTDIProxy ${LIB_TYPE} ${SOURCE_FILES} ${SOURCE_EXTRA})
ENDIF ()

IF (WIN32)
set_target_properties(FTDIProxy PROPERTIES OUTPUT_NAME "ftd2xx")
set_target_properties(FTDIProxy PROPERTIES PREFIX "")
ENDIF ()

IF (WIN32)
find_library(SHLWAPI shlwapi.lib)
target_link_libraries(FTDIProxy SHLWAPI)
ENDIF ()