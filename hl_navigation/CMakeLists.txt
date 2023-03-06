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

target_link_libraries(hl_navigation Eigen3::Eigen)

ament_export_targets(hl_navigationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)


add_executable(test_behavior src/test_behavior.cpp)
target_link_libraries(test_behavior hl_navigation)
add_dependencies(test_behavior hl_navigation)

add_executable(test_controller src/test_controller.cpp)
target_link_libraries(test_controller hl_navigation)
add_dependencies(test_controller hl_navigation)

add_executable(test_controller_3d src/test_controller_3d.cpp)
target_link_libraries(test_controller_3d hl_navigation)
add_dependencies(test_controller_3d hl_navigation)

add_executable(test_controller_async src/test_controller_async.cpp)
target_link_libraries(test_controller_async hl_navigation)
add_dependencies(test_controller_async hl_navigation)

add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark hl_navigation)
add_dependencies(benchmark hl_navigation)

install(TARGETS
  test_behavior benchmark  test_controller test_controller_3d test_controller_async
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

# See https://www.foonathan.net/2016/03/cmake-install/
# meglio fare uno step intermedio che separa i package (ROS/non ROS) ma che usa ament_cmake.
# Poi eventualmente posso rimuovere ament_cmake dal package non ROS
