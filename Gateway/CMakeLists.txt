cmake_minimum_required(VERSION 3.0)
project(gateway)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGATEWAY_EXPORTS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--enable-stdcall-fixup")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")

# Sources
set(SOURCE_FILES ${SOURCE_FILES} stdafx.h stdafx.cpp)
set(SOURCE_FILES ${SOURCE_FILES} targetver.h)
set(SOURCE_FILES ${SOURCE_FILES} dllmain.cpp)
set(SOURCE_FILES ${SOURCE_FILES} gateway.h gateway.cpp)
set(SOURCE_FILES ${SOURCE_FILES} utils.h)

set(LIB_TYPE SHARED)
add_library(gateway ${LIB_TYPE} ${SOURCE_FILES} gateway.def)
set_target_properties(gateway PROPERTIES OUTPUT_NAME "gateway")
set_target_properties(gateway PROPERTIES PREFIX "")