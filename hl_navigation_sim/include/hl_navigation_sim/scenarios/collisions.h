/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIOS_COLLISIONS_H
#define HL_NAVIGATION_SIM_SCENARIOS_COLLISIONS_H

#include <memory>
#include <utility>
#include <vector>

#include "hl_navigation_sim/scenario.h"

using hl_navigation::Properties;
using hl_navigation::Property;
using hl_navigation::make_property;


namespace hl_navigation_sim {

struct CollisionsScenario : public Scenario {
  CollisionsScenario(const char *behavior_name = "HL",
                     float control_period = 0.1f)
      : Scenario(),
        behavior_name{behavior_name},
        control_period{control_period} {}

  void init_world(World *world) override;

  virtual const Properties &get_properties() const override {
    return properties;
  };

  static const std::map<std::string, Property> properties;
  std::string get_type() const override { return type; }
  const static std::string type;

  std::string behavior_name;
  float control_period;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_SCENARIOS_COLLISIONS_H */
