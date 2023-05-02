/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIOS_SIMPLE_H
#define HL_NAVIGATION_SIM_SCENARIOS_SIMPLE_H

#include <memory>

#include "hl_navigation/core/behaviors/dummy.h"
#include "hl_navigation/core/kinematics.h"
#include "hl_navigation/sim/scenario.h"
#include "hl_navigation/sim/tasks/waypoints.h"
#include "hl_navigation/sim/world.h"
#include "hl_navigation_sim_export.h"

using hl_navigation::core::DummyBehavior;
using hl_navigation::core::OmnidirectionalKinematics;

namespace hl_navigation::sim {

/**
 * @brief      Simple scenario that serves as an example.
 *
 * The scenario add a single agent with a waypoints task,
 * dummy behavior, and holonomic kinematics.
 */
struct HL_NAVIGATION_SIM_EXPORT SimpleScenario : public Scenario {
  SimpleScenario() : Scenario() {}

  void init_world(World *world) override {
    Scenario::init_world(world);
    auto agent =
        std::make_shared<Agent>(0.1f, std::make_shared<DummyBehavior>(),
                                std::make_shared<OmnidirectionalKinematics>(1.0f),
                                std::make_shared<WaypointsTask>(
                                    Waypoints{Vector2(1.0f, 0.0f)}, false, 0.1),
                                nullptr, 0.1f);
    agent->get_behavior()->set_optimal_speed(1.0f);
    world->add_agent(agent);
  }

  std::string get_type() const override { return type; }
  inline const static std::string type =
      register_type<SimpleScenario>("Simple");
};

}  // namespace hl_navigation::sim

#endif /* end of include guard: HL_NAVIGATION_SIM_SCENARIOS_SIMPLE_H */
