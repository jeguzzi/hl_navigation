/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SCENARIOS_ANTIPODAL_H
#define HL_NAVIGATION_SIM_SCENARIOS_ANTIPODAL_H

#include <memory>

#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/world.h"

// using std::placeholders::_1;

namespace hl_navigation_sim {

struct AntipodalScenario : public Scenario {
  float radius;
  float tolerance;

  float get_radius() const { return radius; }
  void set_radius(const float &value) { radius = std::max(value, 0.0f); }
  float get_tolerance() const { return tolerance; }
  void set_tolerance(const float &value) { tolerance = std::max(value, 0.0f); }

  void init_world(World *world) override {
    Scenario::init_world(world);
    const unsigned n = world->agents.size();
    const float da = (n < 1) ? 0.0f : 2 * M_PI / n;
    float a = 0.0f;
    for (auto &agent : world->agents) {
      const Vector2 p{radius * std::cos(a), radius * std::sin(a)};
      agent->pose = Pose2(p, a + M_PI);
      agent->task =
          std::make_shared<WayPointsTask>(Waypoints{-p}, false, tolerance);
      a += da;
    }
  }

  AntipodalScenario()
      :  // Scenario({std::bind(&AntipodalScenario::init, this, _1)})
        Scenario(),
        radius(1.0f),
        tolerance(0.1f){};

  const Properties &get_properties() const override { return properties; };

  inline const static std::map<std::string, Property> properties = Properties{
      {"radius",
       make_property<float, AntipodalScenario>(&AntipodalScenario::get_radius,
                                               &AntipodalScenario::set_radius,
                                               1.0f, "Radius of the circle")},
      {"tolerance",
       make_property<float, AntipodalScenario>(
           &AntipodalScenario::get_tolerance, &AntipodalScenario::set_tolerance,
           0.1f, "Goal tolerance")}};

  std::string get_type() const override { return type; }
  inline const static std::string type =
      register_type<AntipodalScenario>("Antipodal");
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_SCENARIOS_ANTIPODAL_H */
