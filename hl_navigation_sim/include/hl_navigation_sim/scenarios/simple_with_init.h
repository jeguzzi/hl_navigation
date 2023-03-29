/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIOS_SIMPLE_WITH_INIT_H
#define HL_NAVIGATION_SIM_SCENARIOS_SIMPLE_WITH_INIT_H

#include <memory>

#include "hl_navigation/behaviors/dummy.h"
#include "hl_navigation/kinematic.h"
#include "hl_navigation_sim/world.h"
#include "hl_navigation_sim/scenario.h"

namespace hl_navigation_sim {

struct SimpleScenariodWithInit : public Scenario {

  void init_world(World *world) override {
    Scenario::init_world(world);
    auto agent = std::make_shared<Agent>(0.1f, std::make_shared<DummyBehavior>(),
                std::make_shared<Holonomic>(1.0f),
                std::make_shared<WayPointsTask>(Waypoints{Vector2(1.0f, 0.0f)},
                                                false, 0.1),
                nullptr, 0.1f);
    agent->nav_behavior->set_optimal_speed(1.0f);
    world->add_agent(agent);
  }

  SimpleScenariodWithInit() : Scenario() {}; 

  std::string get_type() const override { return type; }
  inline const static std::string type = register_type<SimpleScenariodWithInit>("SimpleWithInit");
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_SCENARIOS_SIMPLE_WITH_INIT_H */
