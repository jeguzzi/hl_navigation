/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/scenarios/collisions.h"

#include <memory>
#include <utility>
#include <vector>

namespace hl_navigation_sim {

void CollisionsScenario::init_world(World *world) {
  Scenario::init_world(world);
  const float agent_radius = 0.1f;
  Vector2 target{10.0f, 10.0f};
  auto task =
      std::make_shared<WayPointsTask>(Waypoints{target, -target}, true, 0.1);
  auto se = std::make_shared<BoundedStateEstimation>(nullptr, 10.0, 4.0);
  auto kinematic = std::make_shared<Holonomic>(1.0, 1.0);
  auto agent =
      std::make_shared<Agent>(agent_radius, Behavior::make_type(behavior_name),
                              kinematic, task, se, control_period);

  world->add_agent(agent);
  agent->nav_behavior->set_horizon(10.0);
  agent->nav_behavior->set_safety_margin(0.1);
  // agent.nav_controller.distance_tolerance = 1.0;
  // agent.nav_controller.angle_tolerance = 4.0;
  agent->nav_controller.set_speed_tolerance(0.1f);
  agent->pose = {{-8.0f, -7.9f}};
  world->walls.emplace_back(Vector2{-5.0f, -5.0f}, Vector2{-5.0f, -8.0f});
  world->walls.emplace_back(Vector2{-3.0f, -1.0f}, Vector2{-0.0f, -2.0f});
  // world.walls.emplace_back(Vector2{-3.0f, -1.0f}, Vector2{-3.0f, -0.0f});
  world->obstacles.emplace_back(Vector2{3.0f, 2.0f}, 3.0f);
}

const std::map<std::string, Property> CollisionsScenario::properties =
    Properties{
        {"behavior_name",
         make_property<std::string, CollisionsScenario>(
             [](const CollisionsScenario *obj) -> std::string {
               return obj->behavior_name;
             },
             [](CollisionsScenario *obj, const std::string &value) {
               obj->behavior_name = value;
             },
             "HL", "Behavior name")},
        {"control_period", make_property<float, CollisionsScenario>(
                               [](const CollisionsScenario *obj) -> float {
                                 return obj->control_period;
                               },
                               [](CollisionsScenario *obj, const float &value) {
                                 obj->control_period = value;
                               },
                               0.1f, "Control period")}};

const std::string CollisionsScenario::type =
    register_type<CollisionsScenario>("CollisionsScenario");

}  // namespace hl_navigation_sim


