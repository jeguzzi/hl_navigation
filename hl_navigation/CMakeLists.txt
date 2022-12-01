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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hl_navigation_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

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

add_library(navigation_lib SHARED
  src/Controller.cpp
  src/Agent.cpp
  src/HLAgent.cpp
  src/ORCAAgent.cpp
  src/HRVOAgent.cpp
  src/argos2/common/utility/math/angles.cpp
  ${orca_IMPLEMENTATIONS}
  ${hrvo_IMPLEMENTATIONS}
)
ament_export_targets(navigation_libTargets HAS_LIBRARY_TARGET)

add_executable(test_behavior src/test_behavior.cpp)
target_link_libraries(test_behavior navigation_lib)
add_dependencies(test_behavior navigation_lib)

add_executable(test_controller src/test_controller.cpp)
target_link_libraries(test_controller navigation_lib)
add_dependencies(test_controller navigation_lib)

find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
pybind11_add_module(hl_navigation_py src/python.cpp)
target_link_libraries(hl_navigation_py PRIVATE navigation_lib)
add_dependencies(hl_navigation_py navigation_lib)

add_executable(controller src/ROSControllerNode.cpp)
add_dependencies(controller navigation_lib)
target_link_libraries(controller navigation_lib)
ament_target_dependencies(controller
  rclcpp tf2 tf2_ros tf2_geometry_msgs std_msgs visualization_msgs
  nav_msgs geometry_msgs hl_navigation_msgs)


install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  test_behavior test_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navigation_lib
  EXPORT navigation_libTargets
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
