cmake_minimum_required(VERSION 3.5)
project(hl_navigation_sim)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ament_python_install_namespace_package.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/generate_docstrings.cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments -Wno-unused-parameter)
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# add_subdirectory(HighFive)
# add_subdirectory(Crow)
# set(YAML_CPP_INSTALL ON CACHE INTERNAL "")
# add_subdirectory(../yaml-cpp yaml-cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hl_navigation_core REQUIRED)
find_package(hl_navigation_py REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(GEOS REQUIRED)
find_package(pybind11 REQUIRED)
find_package(HighFive REQUIRED)

# add_subdirectory(../external/geos geos)
# add_subdirectory(../external/pybind11 pybind11 EXCLUDE_FROM_ALL)
# add_subdirectory(../external/HighFive highFive)
# find_package(pybind11_vendor REQUIRED)
# find_package(pybind11 REQUIRED)

# Force to C++17
set(CMAKE_CXX_STANDARD 17)

include_directories(include ${PROJECT_BINARY_DIR})

# add_executable(test_samplers src/test_samplers.cpp src/sampling/sampler.cpp)
# target_link_libraries(test_samplers hl_navigation_core::hl_navigation_core )

# add_executable(test_samplers_yaml src/test_samplers_yaml.cpp src/sampling/sampler.cpp)
# target_link_libraries(test_samplers_yaml hl_navigation_core::hl_navigation_core)

add_library(hl_navigation_sim SHARED
  src/agent.cpp
  src/experiment.cpp 
  src/scenario.cpp
  src/world.cpp 
  src/state_estimations/geometric_bounded.cpp
  src/tasks/waypoints.cpp
  src/scenarios/antipodal.cpp 
  src/scenarios/corridor.cpp 
  src/scenarios/cross.cpp
  src/scenarios/cross_torus.cpp
  src/sampling/sampler.cpp
)

generate_export_header(hl_navigation_sim)

# -Wl,-force_load 
target_link_libraries(
  hl_navigation_sim PUBLIC 
  GEOS::geos 
  # yaml-cpp::yaml-cpp 
  HighFive
  hl_navigation_core::hl_navigation_core 
)
target_link_libraries(hl_navigation_sim PRIVATE )
target_include_directories(hl_navigation_sim INTERFACE $<TARGET_PROPERTY:HighFive,INTERFACE_INCLUDE_DIRECTORIES>)

ament_export_dependencies(geos hl_navigation_core HighFive)
ament_export_targets(hl_navigation_simTargets HAS_LIBRARY_TARGET)

add_executable(run src/run.cpp)
target_link_libraries(run hl_navigation_sim)
add_dependencies(run hl_navigation_sim)

add_executable(sample src/sample.cpp)
target_link_libraries(sample hl_navigation_sim)
add_dependencies(sample hl_navigation_sim)

add_executable(list src/list.cpp)
target_link_libraries(list hl_navigation_sim)
add_dependencies(list hl_navigation_sim)

ament_python_install_namespace_package(sim
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
  PACKAGE_DIR hl_navigation/sim
  NAMESPACE hl_navigation
)
pybind11_add_module(_hl_navigation_sim src/python.cpp)

generate_docstrings(
  _hl_navigation_sim 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp 
  hl_navigation/sim
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_INSTALL_PREFIX}/include 
  ${EIGEN3_INCLUDE_DIR}
)

# target_compile_options(_hl_navigation_sim PRIVATE -fvisibility=hidden)
target_link_libraries(_hl_navigation_sim 
  PRIVATE 
  hl_navigation_sim
)
target_link_libraries(_hl_navigation_sim INTERFACE hl_navigation_py)
# 
add_dependencies(_hl_navigation_sim hl_navigation_sim)

# target_link_libraries(_hl_navigation_sim PRIVATE -Wl,-force_load hl_navigation_sim)

install(TARGETS _hl_navigation_sim DESTINATION "${PYTHON_INSTALL_DIR}/hl_navigation/sim")

install(TARGETS
  list
  run
  sample
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/hl_navigation/sim
  DESTINATION include/hl_navigation
)

install(FILES
 ${PROJECT_BINARY_DIR}/hl_navigation_sim_export.h DESTINATION include
)

install(
  TARGETS hl_navigation_sim
  EXPORT hl_navigation_simTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
