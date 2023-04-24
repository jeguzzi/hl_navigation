/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "hl_navigation_core/plugins.h"
#include "hl_navigation_core/yaml/yaml.h"
#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/scenarios/antipodal.h"
#include "hl_navigation_sim/scenarios/simple.h"
#include "hl_navigation_sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

static void show_usage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --input <FILE>\t\t\tPath to the yaml file" << std::endl
            << "  <YAML>\t\t\tInline YAML string" << std::endl;
}

int main(int argc, char *argv[]) {
  load_plugins();
  YAML::Node node;
  bool loaded_file = false;
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--help") {
      show_usage(argv[0]);
      return 0;
    }
    if (std::string(argv[i]) == "--input" && i < argc - 1) {
      try {
        node = YAML::LoadFile(argv[i + 1]);
      } catch (const YAML::ParserException &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
      }
      loaded_file = true;
      break;
    }
  }
  if (!loaded_file) {
    if (argc <= 1) return 1;
    try {
      node = YAML::Load(argv[1]);
    } catch (const YAML::ParserException &e) {
      std::cerr << "[Error] " << e.what() << std::endl;
      return 1;
    }
  }

  auto scenario = YAML::load_node<hl_navigation::sim::Scenario>(node);
  std::cout << YAML::dump<hl_navigation::sim::Scenario>(scenario.get());
  std::cout << std::endl;
  std::cout << "====================" << std::endl;
  if (!scenario) return 1;
  World world;
  scenario->init_world(&world);
  std::cout << YAML::dump<hl_navigation::sim::World>(&world);
  std::cout << std::endl;
  return 0;
}
