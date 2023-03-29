add_executable(scenario_group scenario_with_group.cpp)
ament_target_dependencies(scenario_group hl_navigation_sim)

add_executable(scenario_yaml scenario_from_yaml.cpp)
ament_target_dependencies(scenario_yaml hl_navigation_sim)

add_executable(scenario_registered scenario_registered.cpp)
ament_target_dependencies(scenario_registered hl_navigation_sim)

add_library(my_scenario SHARED my_scenario.h)
ament_target_dependencies(my_scenario hl_navigation_sim)
ament_export_targets(my_scenarioTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(my_scenario)


install(TARGETS
  scenario_group 
  scenario_yaml 
  scenario_registered
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS my_scenario
  EXPORT my_scenarioTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
