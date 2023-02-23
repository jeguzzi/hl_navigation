/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <iterator>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/controller_3d.h"

using hl_navigation::Action;
using hl_navigation::Behavior;
using hl_navigation::Controller3;
using hl_navigation::Holonomic;
using hl_navigation::Pose3;
using hl_navigation::Twist3;
using hl_navigation::Vector3;

static void show_usage(std::string name) {
  std::vector<std::string> keys = Behavior::behavior_names();
  std::ostringstream behaviors;
  // Dump all keys
  std::copy(keys.begin(), keys.end(),
            std::ostream_iterator<std::string>(behaviors, ", "));
  std::cout
      << "Usage: " << name << " <option(s)>" << std::endl
      << "Options:" << std::endl
      << "  --help\t\t\tShow this help message" << std::endl
      << "  --behavior=<NAME>\t\tObstacle avoidance algorithm name: one of "
      << behaviors.str() << std::endl;
}

int main(int argc, char *argv[]) {
  Controller3 controller;
  float dt = 0.1;
  char behavior_name[10] = "HL";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--behavior=%10s", behavior_name)) {
      continue;
    }
    if (strcmp(argv[i], "--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }
  }
  auto behavior = Behavior::behavior_with_name(
      behavior_name, std::make_shared<Holonomic>(1.0, 1.0), 0.1);
  if (!behavior) {
    printf("No behavior with name %s\n", behavior_name);
    exit(1);
  }
  controller.set_behavior(behavior);
  controller.set_speed_tolerance(0.05);
  controller.set_cmd_cb(
      [&, behavior, dt](const Twist3 &cmd) { controller.actuate(cmd, dt); });
  behavior->set_horizon(1.0);
  const auto &r = *behavior.get();
  printf("Use behavior %s - %s\n", behavior_name, typeid(r).name());
  controller.set_pose({Vector3{0.0f, 0.0f, 0.0f}, 0.0f});
  // Go to 1, 0, -1.5
  const auto action = controller.go_to_position({1.0, 0.0, 2.0}, 0.1f);
  action->done_cb = [](Action::State state) {
    if (state == Action::State::success) {
      printf("Arrived\n");
    }
  };
  action->running_cb = [](float time) {
    printf("In progress ... expected to last at least %.2f s\n", time);
  };
  float t = 0.0;
  auto p = controller.get_pose().position;
  printf("Start loop @ (%.3f, %.3f, %.3f)\n", p.x(), p.y(), p.z());
  while (!action->done() && t < 20) {
    auto cmd = controller.update_3d(dt);
    t += dt;
  }
  p = controller.get_pose().position;
  const auto v = controller.get_twist().velocity;
  printf("\nEnd loop after %.1f s @ (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f)\n",
         t, p.x(), p.y(), p.z(), v.x(), v.y(), v.z());
  return 0;
}