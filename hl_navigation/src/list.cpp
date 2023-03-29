/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/common.h"
#include "hl_navigation/kinematic.h"
#include "hl_navigation/plugins.h"
#include "hl_navigation/property.h"
#include "hl_navigation/utilities.h"

template <typename T>
static void print_register(const std::string& title,
                           const std::string& type = "") {
  std::cout << title << std::endl;
  for (unsigned i = 0; i < title.size(); ++i) {
    std::cout << '=';
  }
  std::cout << std::endl;
  for (const auto& [name, properties] : T::type_properties()) {
    if (!type.empty() && name != type) continue;
    std::cout << "- " << name << std::endl;
    for (const auto& [k, p] : properties) {
      std::cout << "     " << k << ": " << p.default_value << " ["
                << p.type_name << "]" << std::endl;
    }
  }
}

static void show_usage(const std::string& name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --behavior\t\t\tShow only behaviors" << std::endl
            << "  --behavior <BEHAVIOR>\t\tShow only a specific behavior"
            << std::endl
            << "  --kinematic\t\t\tShow only kinematics" << std::endl
            << "  --kinematic <KINEMATIC>\tShow only a specific kinematic"
            << std::endl;
}

int main(int argc, char* argv[]) {
  load_plugins();
  std::string behavior_type = "";
  std::string kinematic_type = "";
  bool behaviors = true;
  bool kinematics = true;
  for (int i = 0; i < argc; i++) {
    if (behaviors && std::string(argv[i]) == "--behavior") {
      if (i + 1 < argc) {
        behavior_type = std::string(argv[i + 1]);
        i++;
      }
      kinematics = false;
    }
    if (kinematics && std::string(argv[i]) == "--kinematic") {
      if (i + 1 < argc) {
        kinematic_type = std::string(argv[i + 1]);
        i++;
      }
      behaviors = false;
    }
    if (std::string(argv[i]) == "--help") {
      show_usage(argv[0]);
      return 0;
    }
  }
  // return 0;
  if (behaviors) {
    print_register<hl_navigation::Behavior>("Behaviors", behavior_type);
    std::cout << std::endl;
  }
  if (kinematics) {
    print_register<hl_navigation::Kinematic>("Kinematics", kinematic_type);
  }
  return 0;
}
