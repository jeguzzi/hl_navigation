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
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-Os")
# set(CMAKE_C_FLAGS_RELEASE "-Os")

# set(CMAKE_CXX_CLANG_TIDY "/usr/local/opt/llvm/bin/clang-tidy;-checks=*")

# find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(orca_IMPLEMENTATIONS
  src/RVO/Agent.cpp
  src/RVO/Obstacle.cpp
  src/RVO/RVOSimulator.cpp
  src/RVO/KdTree.cpp
  )

set(hrvo_IMPLEMENTATIONS
  src/HRVO/Agent.cpp
  src/HRVO/Goal.cpp
  src/HRVO/HRVOSimulator.cpp
  src/HRVO/KdTree.cpp
  src/HRVO/Obstacle.cpp
  src/HRVO/RoadmapVertex.cpp
  src/HRVO/Vector2.cpp
  )

include_directories(include)


add_library(hl_navigation SHARED
  src/Controller.cpp
  src/Agent.cpp
  src/Dummy.cpp
  src/HLAgent.cpp
  src/ORCAAgent.cpp
  src/HRVOAgent.cpp
  ${orca_IMPLEMENTATIONS}
  ${hrvo_IMPLEMENTATIONS}
)

target_link_libraries(hl_navigation Eigen3::Eigen)

# ament_export_targets(hl_navigationTargets HAS_LIBRARY_TARGET)
# ament_export_dependencies(Eigen3)


add_executable(test_behavior src/test_behavior.cpp)
target_link_libraries(test_behavior hl_navigation)
add_dependencies(test_behavior hl_navigation)

add_executable(test_controller src/test_controller.cpp)
target_link_libraries(test_controller hl_navigation)
add_dependencies(test_controller hl_navigation)

add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark hl_navigation)
add_dependencies(benchmark hl_navigation)

install(TARGETS
  test_behavior test_controller benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS hl_navigation
  EXPORT hl_navigationTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ament_package()

# See https://www.foonathan.net/2016/03/cmake-install/
# meglio fare uno step intermedio che separa i package (ROS/non ROS) ma che usa ament_cmake.
# Poi eventualmente posso rimuovere ament_cmake dal package non ROS
