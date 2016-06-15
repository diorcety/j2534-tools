cmake_minimum_required(VERSION 3.0)

project(dsfsdfsdf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_LOGGING")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")

# Sources
set(SOURCE_FILES ${SOURCE_FILES} log.h log.cpp)
set(SOURCE_FILES ${SOURCE_FILES} dsfsdfsdf.cpp)
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} FTD2XX.h)
set(SOURCE_FILES ${SOURCE_FILES} j2534_v0404.h)

# Libraries
find_library(SHLWAPI shlwapi.lib)

add_executable(dsfsdfsdf ${SOURCE_FILES})
target_link_libraries(dsfsdfsdf SHLWAPI)