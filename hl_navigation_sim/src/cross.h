/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef CROSS_H
#define CROSS_H

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "hl_navigation_sim/simulation.h"

namespace hl_navigation_sim {

using namespace hl_navigation;

class CrossExperiment : public Experiment {
 public:
  CrossExperiment(float dt = 0.05, int steps = 2000,
                  const char* behavior = "HL", float radius = 4.0,
                  unsigned number = 7, float margin = 1.0)
      : Experiment(dt, steps),
        behavior(behavior),
        radius(radius),
        number(number),
        margin(margin) {}

 protected:
  void init(World& world, int seed) override {
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

    //      std::vector<std::tuple<Vector2, std::function<Vector2(int, int)>>>
    //      task = {
    //          {Vector2(radius, 0.00), [this](int i, unsigned number) {return
    //          Vector2(-radius, 0.6 * i);}}, {Vector2(-radius, 0.00),
    //          [this](int i, unsigned number) {return Vector2(radius, 0.6 * i +
    //          0.01);}},
    //      };

    for (auto& [target, pose] : task) {
      for (size_t i = 0; i < number; i++) {
        auto task = std::make_shared<WayPointsTask>(
            std::vector<Vector2>{target, -target}, true, 0.1);
        auto se = std::make_shared<BoundedStateEstimation>(&world, 1.0, 1.0);
        auto kinematic = std::make_shared<Holonomic>(1.0, 1.0);
        auto& agent = world.agents.emplace_back(behavior, 0.1, 0.1, kinematic,
                                                task, se, dt);
        agent.nav_behavior->set_max_speed(1.0);
        agent.nav_behavior->set_max_angular_speed(1.0);
        agent.nav_behavior->set_optimal_speed(1.0);
        agent.nav_behavior->set_optimal_angular_speed(1.0);
        agent.nav_behavior->set_horizon(1.0);
        // agent.nav_controller.distance_tolerance = 1.0;
        // agent.nav_controller.angle_tolerance = 4.0;
        agent.nav_controller.set_speed_tolerance(0.1f);
        agent.pose = pose(i, number);
      }
    }
  }

 private:
  const char* behavior;
  float radius;
  unsigned number;
  float margin;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: CROSS_H */
