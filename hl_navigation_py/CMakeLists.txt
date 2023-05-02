cmake_minimum_required(VERSION 3.5)
project(hl_navigation_py)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ament_python_install_namespace_package.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/generate_docstrings.cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments)
endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-Os")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hl_navigation_core REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
# add_subdirectory(../external/pybind11 pybind11 EXCLUDE_FROM_ALL)


# message("${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp ${CMAKE_INSTALL_DIR}/include/hl_navigation/core ${EIGEN3_INCLUDE_DIR}")


include_directories(include ${PROJECT_BINARY_DIR})

add_library(hl_navigation_py INTERFACE )
target_include_directories(hl_navigation_py INTERFACE include)




# ament_python_install_package seems to not support namespace packages
# For those: we should install the pkg to <site-packages>/hl_navigation
# and the metainformation in <site-packages>/core-0.0.0-py3.10.egg-info
# BUT the one below install both under DESTINATION
ament_python_install_namespace_package(core
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
  PACKAGE_DIR hl_navigation/core
  NAMESPACE hl_navigation
)

pybind11_add_module(_hl_navigation src/python.cpp)

generate_docstrings(
  _hl_navigation 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp 
  hl_navigation/core
  ${CMAKE_INSTALL_PREFIX}/include
  ${EIGEN3_INCLUDE_DIR}
)

# target_compile_options(_hl_navigation PRIVATE -fvisibility=hidden)
target_compile_definitions(_hl_navigation PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
target_link_libraries(_hl_navigation PRIVATE hl_navigation_core::hl_navigation_core)
add_dependencies(_hl_navigation hl_navigation_core::hl_navigation_core docstrings)

install(TARGETS _hl_navigation DESTINATION "${PYTHON_INSTALL_DIR}/hl_navigation/core")

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
