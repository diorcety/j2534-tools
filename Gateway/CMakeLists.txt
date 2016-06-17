cmake_minimum_required(VERSION 3.0)
project(gateway)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGATEWAY_EXPORTS")
IF (WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
ENDIF()

IF (UNIX)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
ENDIF()

# Sources
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} gateway.h gateway.cpp)
set(SOURCE_FILES ${SOURCE_FILES} utils.h)

set(LIB_TYPE SHARED)
IF (WIN32)
set(SOURCE_EXTRA gateway.def)
ENDIF ()

add_library(gateway ${LIB_TYPE} ${SOURCE_FILES} ${SOURCE_EXTRA})

IF (WIN32)
set_target_properties(gateway PROPERTIES OUTPUT_NAME "gateway")
set_target_properties(gateway PROPERTIES PREFIX "")
ENDIF ()