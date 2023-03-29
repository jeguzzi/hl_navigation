add_executable(collision collision.cpp)
ament_target_dependencies(collision hl_navigation)

add_executable(collision_cached cached_collision.cpp)
ament_target_dependencies(collision_cached hl_navigation)

install(TARGETS
  collision 
  collision_cached 
  DESTINATION lib/${PROJECT_NAME}
)