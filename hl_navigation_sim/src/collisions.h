/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef COLLISIONS_H
#define COLLISIONS_H

#include <memory>
#include <utility>
#include <vector>

#include "hl_navigation_sim/simulation.h"

namespace hl_navigation_sim {

using namespace hl_navigation;

class CollisionsTest : public Experiment {
 public:
  CollisionsTest(float dt = 0.05, int steps = 2000,
                 const char* behavior_name = "HL")
      : Experiment(dt, steps), behavior_name(behavior_name) {}

 protected:
  void init(World& world, int seed) override {
    Vector2 target{10.0f, 10.0f};
    auto task = std::make_shared<WayPointsTask>(
        std::vector<Vector2>{target, -target}, true, 0.1);
    auto se = std::make_shared<BoundedStateEstimation>(&world, 10.0, 4.0);
    auto kinematic = std::make_shared<Holonomic>(1.0, 1.0);
    auto& agent = world.agents.emplace_back(behavior_name, 0.1, 0.1, kinematic,
                                            task, se, dt);
    agent.nav_behavior->set_horizon(10.0);
    agent.nav_behavior->set_safety_margin(0.1);
    // agent.nav_controller.distance_tolerance = 1.0;
    // agent.nav_controller.angle_tolerance = 4.0;
    agent.nav_controller.set_speed_tolerance(0.1f);
    agent.pose = {{-8.0f, -7.9f}};
    world.walls.emplace_back(Vector2{-5.0f, -5.0f}, Vector2{-5.0f, -8.0f});
    world.walls.emplace_back(Vector2{-3.0f, -1.0f}, Vector2{-0.0f, -2.0f});
    // world.walls.emplace_back(Vector2{-3.0f, -1.0f}, Vector2{-3.0f, -0.0f});
    world.obstacles.emplace_back(Vector2{3.0f, 2.0f}, 3.0f);
  }

 private:
  const char* behavior_name;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: COLLISIONS_H */
