cmake_minimum_required(VERSION 3.5)
project(hl_navigation)

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

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-Os")
# set(CMAKE_C_FLAGS_RELEASE "-Os")

# set(CMAKE_CXX_CLANG_TIDY "/usr/local/opt/llvm/bin/clang-tidy;-checks=*")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(ORCA
  src/behaviors/ORCA/RVO/Agent.cpp
  src/behaviors/ORCA/RVO/Obstacle.cpp
  src/behaviors/ORCA/RVO/RVOSimulator.cpp
  src/behaviors/ORCA/RVO/KdTree.cpp
  )

set(HRVO
  src/behaviors/HRVO/HRVO/Agent.cpp
  src/behaviors/HRVO/HRVO/Goal.cpp
  src/behaviors/HRVO/HRVO/HRVOSimulator.cpp
  src/behaviors/HRVO/HRVO/KdTree.cpp
  src/behaviors/HRVO/HRVO/Obstacle.cpp
  src/behaviors/HRVO/HRVO/RoadmapVertex.cpp
  src/behaviors/HRVO/HRVO/Vector2.cpp
  )

include_directories(include)

add_library(hl_navigation SHARED
  src/collision_computation.cpp
  src/cached_collision_computation.cpp
  src/behavior.cpp
  src/behaviors/dummy.cpp
  src/behaviors/HL.cpp
  src/behaviors/ORCA/ORCA.cpp
  ${ORCA}
  src/behaviors/HRVO/HRVO.cpp
  ${HRVO}
  src/controller.cpp
  src/controller_3d.cpp
)

target_link_libraries(hl_navigation PUBLIC Eigen3::Eigen)
ament_export_dependencies(Eigen3)

set(YAML_CPP_INSTALL ON CACHE INTERNAL "")
add_subdirectory(../yaml-cpp yaml-cpp)
target_link_libraries(hl_navigation PUBLIC yaml-cpp::yaml-cpp)
ament_export_dependencies(yaml-cpp)

ament_export_targets(hl_navigationTargets HAS_LIBRARY_TARGET)

add_executable(list src/list.cpp)
target_link_libraries(list hl_navigation)
add_dependencies(list hl_navigation)

install(TARGETS
  list
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS hl_navigation
  EXPORT hl_navigationTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
