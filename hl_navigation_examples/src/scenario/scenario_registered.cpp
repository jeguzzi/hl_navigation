/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/scenarios/antipodal.h"
#include "hl_navigation_sim/scenarios/cross.h"
#include "hl_navigation_sim/scenarios/collisions.h"
#include "hl_navigation_sim/scenarios/simple.h"
#include "hl_navigation_sim/yaml/world.h"
#include "hl_navigation_sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

static std::string s_type = "Simple";
namespace sim = hl_navigation_sim;

int main() {
  std::cout << "Selecting " << s_type << " from the registered scenarios ";
  for (const auto & type : sim::Scenario::types()) {
    std::cout << " "<< type;
  }
  std::cout << std::endl;
  auto scenario = hl_navigation_sim::Scenario::make_type(s_type);
  if (!scenario) {
    std::cerr << "Could not load scenario" << std::endl;
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
