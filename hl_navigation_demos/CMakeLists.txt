cmake_minimum_required(VERSION 3.5)
project(hl_navigation_demos)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hl_navigation REQUIRED)
find_package(hl_navigation_sim REQUIRED)

add_executable(native_cpp src/native.cpp)
ament_target_dependencies(native_cpp hl_navigation)

add_executable(sim_cpp src/sim.cpp)
ament_target_dependencies(sim_cpp hl_navigation hl_navigation_sim)


install(TARGETS
  native_cpp sim_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/ros script/enki script/native_py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_python_install_package(hl_navigation_demos)


ament_package()