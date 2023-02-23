/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/controller.h"

using hl_navigation::Action;
using hl_navigation::Behavior;
using hl_navigation::Controller;
using hl_navigation::TwoWheeled;
using hl_navigation::Twist2;
using hl_navigation::Vector2;
using hl_navigation::Disc;

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

static void go_to(Controller *controller, Vector2 target) {
  auto action = controller->go_to_position(target, 0.2f);
  action->done_cb = [controller, target = target](Action::State state) {
    if (state == Action::State::success) {
      go_to(controller, -target);
    }
  };
}

static void run(const char *behavior = "HL") {
  std::vector<Controller> controllers;
  std::vector<Behavior *> agents;
  std::vector<Disc> obstacles = {Disc({0.0f, 0.0f}, 0.1)};
  Vector2 target{1.0, 0.0};
  for (size_t i = 0; i < 2; i++) {
    auto agent = Behavior::behavior_with_name(
        behavior, std::make_shared<TwoWheeled>(0.166, 0.094), 0.08);
    agent->set_horizon(1.0);
    agent->set_safety_margin(0.02);
    agent->set_optimal_speed(0.12);
    agent->set_position(Vector2(i ? -0.5f : 0.5f, 0.0f));
    agent->set_static_obstacles(obstacles);
    agents.push_back(agent.get());
    auto &controller = controllers.emplace_back(agent);
    go_to(&controller, target);
  }
  const float dt = 0.02f;
  printf("Start simulating 1 minute at 50 ticks per second\n");
  auto begin = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < 3000; i++) {
    for (auto &controller : controllers) {
      auto agent = controller.get_behavior().get();
      std::vector<Disc> neighbors;
      for (auto &neighbor : agents) {
        if (neighbor == agent) continue;
        neighbors.emplace_back(neighbor->get_position(), 0.08, 0.0,
                               neighbor->get_velocity(false));
      }
      agent->set_neighbors(neighbors);
    }
    for (auto &controller : controllers) {
      auto cmd = controller.update(dt);
      controller.get_behavior()->actuate(cmd, dt);
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  unsigned us =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
  printf("Done simulating in %.1f ms\n", us * 1e-6);
}

int main(int argc, char *argv[]) {
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
  run(behavior_name);
  return 0;
}
