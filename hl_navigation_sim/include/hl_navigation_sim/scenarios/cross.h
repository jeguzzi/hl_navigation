/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIOS_CROSS_H
#define HL_NAVIGATION_SIM_SCENARIOS_CROSS_H

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "hl_navigation_sim/scenario.h"

namespace hl_navigation_sim {

struct CrossScenario : public Scenario {
 public:
  std::string behavior_name;
  float radius;
  unsigned number;
  float margin;
  float control_period;

  CrossScenario(const char *behavior_name = "HL", float radius = 4.0,
                unsigned number = 7, float margin = 1.0,
                float control_period = 0.1f)
      : Scenario(),
        behavior_name(behavior_name),
        radius(radius),
        number(number),
        margin(margin),
        control_period(control_period) {}

  void init_world(World *world) override;

  virtual const Properties &get_properties() const override {
    return properties;
  };

  static const std::map<std::string, Property> properties;

  std::string get_type() const override { return type; }
  const static std::string type;
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_SCENARIOS_CROSS_H */
