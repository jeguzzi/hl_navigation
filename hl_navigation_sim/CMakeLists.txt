cmake_minimum_required(VERSION 3.5)
project(hl_navigation_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(HighFive)

find_package(ament_cmake REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(hl_navigation REQUIRED)
find_package(geos REQUIRED)

include_directories(include)

add_executable(hl_navigation_sim src/cross.cpp)
ament_target_dependencies(hl_navigation_sim hl_navigation)
target_link_libraries(hl_navigation_sim HighFive GEOS::geos)

install(TARGETS
  hl_navigation_sim
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS hl_navigation_sim
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
