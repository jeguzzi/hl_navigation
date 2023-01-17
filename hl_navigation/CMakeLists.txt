cmake_minimum_required(VERSION 3.5)
project(hl_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

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
  src/HLAgent.cpp
  src/ORCAAgent.cpp
  src/HRVOAgent.cpp
  src/argos2/common/utility/math/angles.cpp
  ${orca_IMPLEMENTATIONS}
  ${hrvo_IMPLEMENTATIONS}
)

ament_export_targets(hl_navigationTargets HAS_LIBRARY_TARGET)

ament_python_install_package(hl_navigation)

add_executable(test_behavior src/test_behavior.cpp)
target_link_libraries(test_behavior hl_navigation)
add_dependencies(test_behavior hl_navigation)

add_executable(test_controller src/test_controller.cpp)
target_link_libraries(test_controller hl_navigation)
add_dependencies(test_controller hl_navigation)

find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
pybind11_add_module(_hl_navigation src/python.cpp)
target_link_libraries(_hl_navigation PRIVATE hl_navigation)
add_dependencies(_hl_navigation hl_navigation)

install(TARGETS
  test_behavior test_controller
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

install(TARGETS _hl_navigation DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_package()

# See https://www.foonathan.net/2016/03/cmake-install/
# meglio fare uno step intermedio che separa i package (ROS/non ROS) ma che usa ament_cmake.
# Poi eventualmente posso rimuovere ament_cmake dal package non ROS
