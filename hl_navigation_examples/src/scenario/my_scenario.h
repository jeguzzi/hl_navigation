/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_EXAMPLES_MY_SCENARIO_H_
#define HL_NAVIGATION_EXAMPLES_MY_SCENARIO_H_

#include "hl_navigation_sim/scenario.h"

namespace hl_navigation_sim {

/**
 * @brief      Idle behavior that always stay still.
 *
 * It showcases how to define and use a new behavior from an external shared
 * library.
 */
struct EmptyScenario : Scenario {
 public:
  using Scenario::Scenario;

  std::string get_type() const override { return type; }

  void init_world(World *world) override {
    Scenario::init_world(world);
    // ...
  }

  static inline const std::string type = register_type<EmptyScenario>("Empty");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_EXAMPLES_NEW_BEHAVIOR_H_