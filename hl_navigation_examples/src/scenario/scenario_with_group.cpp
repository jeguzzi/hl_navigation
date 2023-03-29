/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "../groups.h"
#include "hl_navigation_sim/sampling/agent.h"
#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/yaml/world.h"
#include "hl_navigation_sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

namespace sim = hl_navigation_sim;
using hl_navigation::Vector2;

int main() {
  sim::Scenario scenario;
  scenario.walls.emplace_back(Vector2{-1.0f, -1.f}, Vector2{-1.f, 1.f});
  scenario.obstacles.emplace_back(Vector2{2.f, 0.f}, 0.5f);
  scenario.groups.push_back(std::make_unique<sim::AgentSampler<>>(robots()));
  std::cout << "\nSCENARIO\n========\n" << std::endl;
  std::cout << YAML::dump<sim::Scenario>(&scenario) << std::endl;
  std::cout << "\nWORLD\n=====\n" << std::endl;
  sim::World world;
  scenario.init_world(&world);
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
