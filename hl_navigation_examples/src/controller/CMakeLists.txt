add_executable(controller controller.cpp)
ament_target_dependencies(controller hl_navigation_core)

add_executable(controller_3d controller_3d.cpp)
ament_target_dependencies(controller_3d hl_navigation_core)

add_executable(controller_async controller_async.cpp)
ament_target_dependencies(controller_async hl_navigation_core)


install(TARGETS
  controller 
  controller_3d 
  controller_async
  DESTINATION lib/${PROJECT_NAME}
)
