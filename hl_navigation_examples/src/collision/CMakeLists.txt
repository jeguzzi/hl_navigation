add_executable(collision collision.cpp)
ament_target_dependencies(collision hl_navigation_core)

add_executable(collision_cached cached_collision.cpp)
ament_target_dependencies(collision_cached hl_navigation_core)

install(TARGETS
  collision 
  collision_cached 
  DESTINATION lib/${PROJECT_NAME}
)