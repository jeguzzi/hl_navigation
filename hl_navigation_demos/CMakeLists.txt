cmake_minimum_required(VERSION 3.5)
project(hl_navigation_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hl_navigation REQUIRED)
find_package(hl_navigation_sim REQUIRED)

add_executable(native_cpp src/native.cpp)
ament_target_dependencies(native_cpp hl_navigation)

add_executable(sim_cpp src/sim.cpp)
ament_target_dependencies(sim_cpp hl_navigation hl_navigation_sim)

add_library(hl_navigation_demos SHARED src/scenario.h)
ament_target_dependencies(hl_navigation_demos hl_navigation hl_navigation_sim)

install(TARGETS
  native_cpp sim_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  hl_navigation_demos
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)


ament_package()