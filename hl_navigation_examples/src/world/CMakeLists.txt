add_executable(world world.cpp)
ament_target_dependencies(world hl_navigation_sim)

add_executable(world_yaml world_from_yaml.cpp)
ament_target_dependencies(world_yaml hl_navigation_sim)

add_executable(world_group world_with_group.cpp)
ament_target_dependencies(world_group hl_navigation_sim)

install(TARGETS
  world 
  world_yaml 
  world_group
  DESTINATION lib/${PROJECT_NAME}
)
