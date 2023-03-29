cmake_minimum_required(VERSION 3.5)
project(hl_navigation_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hl_navigation REQUIRED)
find_package(hl_navigation_sim REQUIRED)

add_subdirectory(src/behavior)
add_subdirectory(src/controller)
add_subdirectory(src/collision)
add_subdirectory(src/scenario)
add_subdirectory(src/world)

ament_package()