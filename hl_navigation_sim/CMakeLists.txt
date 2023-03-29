cmake_minimum_required(VERSION 3.5)
project(hl_navigation_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments -Wno-unused-parameter)
endif()


add_subdirectory(HighFive)
add_subdirectory(Crow)
set(YAML_CPP_INSTALL ON CACHE INTERNAL "")
add_subdirectory(../yaml-cpp yaml-cpp)

find_package(ament_cmake REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(hl_navigation REQUIRED)
find_package(hl_navigation_py REQUIRED)
find_package(geos REQUIRED)


find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)

add_library(hl_navigation_sim SHARED
  src/experiment.cpp 
  src/scenarios/collisions.cpp 
  src/scenarios/cross.cpp
)
# -Wl,-force_load 
target_link_libraries(
  hl_navigation_sim PUBLIC 
  GEOS::geos 
  yaml-cpp::yaml-cpp 
  HighFive
  hl_navigation::hl_navigation 
)
# target_link_libraries(hl_navigation_sim PRIVATE )
# target_include_directories(hl_navigation_sim INTERFACE $<TARGET_PROPERTY:HighFive,INTERFACE_INCLUDE_DIRECTORIES>)

ament_export_dependencies(geos hl_navigation yaml-cpp HighFive)
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

ament_python_install_package(hl_navigation_sim
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)
pybind11_add_module(_hl_navigation_sim src/python.cpp)
target_compile_options(_hl_navigation_sim PRIVATE -fvisibility=hidden)
target_link_libraries(_hl_navigation_sim 
  PRIVATE 
  hl_navigation_sim
)
target_link_libraries(_hl_navigation_sim INTERFACE hl_navigation_py)

add_dependencies(_hl_navigation_sim hl_navigation_sim)

# target_link_libraries(_hl_navigation_sim PRIVATE -Wl,-force_load hl_navigation_sim)


install(TARGETS _hl_navigation_sim DESTINATION "${PYTHON_INSTALL_DIR}/hl_navigation_sim")

install(TARGETS
  list
  run
  sample
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
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
