cmake_minimum_required(VERSION 3.5)

project(
  hl_navigation_coppeliasim
  VERSION 0.0
  LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

if(NOT LIBPLUGIN_DIR)
  if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
    set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
  else()
    set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
  endif()
endif()
list(APPEND CMAKE_MODULE_PATH
  ${LIBPLUGIN_DIR}/cmake
)

find_package(CoppeliaSim 4 REQUIRED)
find_package(hl_navigation REQUIRED)

list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${LIBPLUGIN_DIR}/cmake)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lua/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lua/simHLNavigation.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h ESCAPE_QUOTES)

coppeliasim_add_plugin(simExtHLNavigation SOURCES src/plugin.cpp)

target_link_libraries(simExtHLNavigation hl_navigation::hl_navigation)
