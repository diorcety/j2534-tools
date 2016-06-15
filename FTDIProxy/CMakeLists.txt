cmake_minimum_required(VERSION 3.0)
project(FTDIProxy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_LOGGING")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -static-libgcc -static-libstdc++")

# Sources
set(SOURCE_FILES ${SOURCE_FILES} log.h log.cpp)
set(SOURCE_FILES ${SOURCE_FILES} FTDIProxy.h FTDIProxy.cpp)
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} utils.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} FTD2XX.H)

# Libraries
find_library(SHLWAPI shlwapi.lib)

set(LIB_TYPE SHARED)
add_library(FTDIProxy ${LIB_TYPE} ${SOURCE_FILES} FTDIProxy.def)
set_target_properties(FTDIProxy PROPERTIES OUTPUT_NAME "ftd2xx")
set_target_properties(FTDIProxy PROPERTIES PREFIX "")
target_link_libraries(FTDIProxy SHLWAPI)