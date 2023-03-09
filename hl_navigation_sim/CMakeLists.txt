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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


add_subdirectory(HighFive)

find_package(ament_cmake REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(hl_navigation REQUIRED)
find_package(geos REQUIRED)

include_directories(include)

add_library(hl_navigation_sim SHARED
  src/simulation.cpp
)

target_link_libraries(hl_navigation_sim PUBLIC GEOS::geos hl_navigation::hl_navigation)
target_link_libraries(hl_navigation_sim PRIVATE HighFive)

ament_export_targets(hl_navigation_simTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(geos hl_navigation)

add_executable(main src/main.cpp)
target_link_libraries(main hl_navigation_sim)
add_dependencies(main hl_navigation_sim)


install(TARGETS
  main
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS hl_navigation_sim
  EXPORT hl_navigation_simTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
