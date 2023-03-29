/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/scenarios/cross.h"

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace hl_navigation_sim {

using namespace hl_navigation;

void CrossScenario::init_world(World *world) {
  float x = radius - margin;
  std::vector<std::tuple<Vector2, std::function<Pose2(int, int)>>> task = {
      {Vector2{radius, 0.0f},
       [x](int i, unsigned number) {
         return Pose2{{-x + 2 * x * i / (number - 1), 0.49f}};
       }},
      {Vector2{-radius, 0.0f},
       [x](int i, unsigned number) {
         return Pose2{{-x + 2 * x * i / (number - 1), -0.51f}};
       }},
      {Vector2{0.0f, radius},
       [x](int i, unsigned number) {
         return Pose2{{0.52f, -x + 2 * x * i / (number - 1)}};
       }},
      {Vector2{0.0f, -radius}, [x](int i, unsigned number) {
         return Pose2{{-0.53f, -x + 2 * x * i / (number - 1)}};
       }}};

  for (auto &[target, pose] : task) {
    for (size_t i = 0; i < number; i++) {
      const float agent_radius = 0.1f;
      auto task = std::make_shared<WayPointsTask>(
          std::vector<Vector2>{target, -target}, true, 0.1);
      auto se = std::make_shared<BoundedStateEstimation>(world, 1.0, 1.0);
      auto kinematic = std::make_shared<Holonomic>(1.0, 1.0);
      auto behavior = Behavior::make_type(behavior_name);
      auto agent = std::make_shared<Agent>(agent_radius, behavior, kinematic,
                                           task, se, control_period);
      agent->nav_behavior->set_max_speed(1.0);
      agent->nav_behavior->set_max_angular_speed(1.0);
      agent->nav_behavior->set_optimal_speed(1.0);
      agent->nav_behavior->set_optimal_angular_speed(1.0);
      agent->nav_behavior->set_horizon(1.0);
      // agent.nav_controller.distance_tolerance = 1.0;
      // agent.nav_controller.angle_tolerance = 4.0;
      agent->nav_controller.set_speed_tolerance(0.1f);
      agent->pose = pose(i, number);
      world->agents.push_back(agent);
    }
  }
}

const std::map<std::string, Property> CrossScenario::properties = Properties{
    {"behavior_name", make_property<std::string, CrossScenario>(
                          [](const CrossScenario *obj) -> std::string {
                            return obj->behavior_name;
                          },
                          [](CrossScenario *obj, const std::string &value) {
                            obj->behavior_name = value;
                          },
                          "HL", "Behavior name")},
    {"control_period",
     make_property<float, CrossScenario>(
         [](const CrossScenario *obj) -> float { return obj->control_period; },
         [](CrossScenario *obj, const float &value) {
           obj->control_period = value;
         },
         0.1f, "Control period")},
    {"size",
     make_property<float, CrossScenario>(
         [](const CrossScenario *obj) -> float { return obj->radius; },
         [](CrossScenario *obj, const float &value) { obj->radius = value; },
         4.0f, "Size")},
    {"margin",
     make_property<float, CrossScenario>(
         [](const CrossScenario *obj) -> float { return obj->margin; },
         [](CrossScenario *obj, const float &value) { obj->margin = value; },
         1.0f, "Margin")},
    {"number",
     make_property<int, CrossScenario>(
         [](const CrossScenario *obj) -> unsigned { return obj->number; },
         [](CrossScenario *obj, const unsigned &value) { obj->number = value; },
         7, "Number of agents per group")},
    {"control_period",
     make_property<float, CrossScenario>(
         [](const CrossScenario *obj) -> float { return obj->control_period; },
         [](CrossScenario *obj, const float &value) {
           obj->control_period = value;
         },
         0.1f, "Control period")}};

const std::string CrossScenario::type =
    register_type<CrossScenario>("CrossScenario");

}  // namespace hl_navigation_sim
