cmake_minimum_required(VERSION 3.5)
project(hl_navigation_py)

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
find_package(hl_navigation REQUIRED)

ament_python_install_package(hl_navigation)

pybind11_add_module(_hl_navigation src/_hl_navigation.cpp)
target_link_libraries(_hl_navigation PRIVATE hl_navigation::hl_navigation)
add_dependencies(_hl_navigation hl_navigation::hl_navigation)

install(TARGETS _hl_navigation DESTINATION "${PYTHON_INSTALL_DIR}/hl_navigation")

ament_package()
