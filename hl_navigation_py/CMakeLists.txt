cmake_minimum_required(VERSION 3.5)
project(hl_navigation_py)

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

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-Os")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hl_navigation REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
# find_package(pybind11_vendor REQUIRED)
# find_package(pybind11 REQUIRED)
add_subdirectory(../external/pybind11 pybind11 EXCLUDE_FROM_ALL)

include_directories(include)

add_library(hl_navigation_py INTERFACE )
target_include_directories(hl_navigation_py INTERFACE include)

ament_python_install_package(hl_navigation
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

pybind11_add_module(_hl_navigation src/python.cpp)
# target_compile_options(_hl_navigation PRIVATE -fvisibility=hidden)
target_link_libraries(_hl_navigation PRIVATE hl_navigation::hl_navigation)
add_dependencies(_hl_navigation hl_navigation::hl_navigation)

install(TARGETS _hl_navigation DESTINATION "${PYTHON_INSTALL_DIR}/hl_navigation")

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
