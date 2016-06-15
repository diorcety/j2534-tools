cmake_minimum_required(VERSION 3.0)

project(MVCIProxy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_LOGGING")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -static-libgcc -static-libstdc++")

# Sources
set(SOURCE_FILES ${SOURCE_FILES} log.h log.cpp)
set(SOURCE_FILES ${SOURCE_FILES} MVCIProxy.h MVCIProxy.cpp)
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} utils.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} j2534_v0404.h)

# Libraries
find_library(SHLWAPI shlwapi.lib)

set(LIB_TYPE SHARED)
add_library(MVCIProxy ${LIB_TYPE} ${SOURCE_FILES} MVCIProxy.def)
set_target_properties(MVCIProxy PROPERTIES OUTPUT_NAME "MVCIProxy")
set_target_properties(MVCIProxy PROPERTIES PREFIX "")
target_link_libraries(MVCIProxy SHLWAPI)