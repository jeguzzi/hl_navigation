/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "hl_navigation_sim/scenarios/cross_torus.h"

#include <memory>
#include <utility>
#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation/property.h"
#include "hl_navigation_sim/sampling/sampler.h"
#include "hl_navigation_sim/tasks/waypoints.h"

namespace hl_navigation_sim {

using namespace hl_navigation;

void CrossTorusScenario::init_world(World *world) {
  Scenario::init_world(world);
  World::Lattice lattice = std::make_tuple<float>(0.0f, side);
  world->set_lattice(0, lattice);
  world->set_lattice(1, lattice);
  UniformSampler<float> x(0.0f, side);
  for (const auto &agent : world->get_agents()) {
    agent->pose.position = {x.sample(), x.sample()};
  }
  world->space_agents_apart(agent_margin, add_safety_to_agent_margin);
  unsigned index = 0;
  world->prepare();
  for (const auto &agent : world->get_agents()) {
    agent->pose.orientation = M_PI_2 * (index % 4);;
    agent->get_controller()->follow_direction(unit(agent->pose.orientation));
    index++;
  }
}

#if 0

const std::map<std::string, Property> CrossTorusScenario::properties = Properties{
    {"side", make_property<float, CrossTorusScenario>(
                 &CrossTorusScenario::get_side, &CrossTorusScenario::set_side,
                 default_side, "Distance between targets")},
    {"agent_margin",
     make_property<float, CrossTorusScenario>(
         &CrossTorusScenario::get_agent_margin, &CrossTorusScenario::set_agent_margin,
         0.1f, "initial minimal distance between agents")},
    {"add_safety_to_agent_margin",
     make_property<bool, CrossTorusScenario>(
         &CrossTorusScenario::get_add_safety_to_agent_margin,
         &CrossTorusScenario::set_add_safety_to_agent_margin,
         default_add_safety_to_agent_margin,
         "Whether to add the safety margin to the agent margin")}};

const std::string CrossTorusScenario::type = register_type<CrossTorusScenario>("CrossTorus");

#endif

}  // namespace hl_navigation_sim
