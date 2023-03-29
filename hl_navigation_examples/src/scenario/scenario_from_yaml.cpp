/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/world.h"
#include "hl_navigation_sim/yaml/scenario.h"
#include "hl_navigation_sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

static const char scenario_yaml[] = R"YAML(
walls:
  - [[-1.0, -1.0], [-1.0, 1.0]]
obstacles:
  - 
    position: [2.0, 0.0]
    radius: 0.5
agents:
  -
    kinematic:
      type: Holonomic
      max_speed: 1.0
      wheel_axis: 0.12
    navigation_behavior:
      type: HL
      safety_margin: 0.5
      tau: 0.25
    state_estimation:
      type: Bounded
      range_of_view: 10.0
    task:
      type: WayPoints
      waypoints: [[1.0, 0.0], [-1.0, 0.0]]
      tolerance: 0.1
    radius: 0.1
    control_period: 0.1
    number: 2
    x:
      sampler: regular
      start: 0
      end: 10
      number: 2
    y: 0
    theta: 0
)YAML";

namespace sim = hl_navigation_sim;

int main() {
  YAML::Node node;
  try {
    node = YAML::Load(scenario_yaml);
  } catch (const YAML::ParserException &e) {
    std::cerr << "[Error] " << e.what() << std::endl;
    return 1;
  }
  auto scenario = node.as<std::shared_ptr<sim::Scenario>>();
  if (!scenario) {
    std::cerr << "Could not load the scenario" << std::endl;
    return 1;
  }
  std::cout << "\nSCENARIO\n========\n" << std::endl;
  std::cout << YAML::dump<sim::Scenario>(scenario.get()) << std::endl;
  sim::World world;
  scenario->init_world(&world);
  std::cout << "\nWORLD\n=====\n" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
