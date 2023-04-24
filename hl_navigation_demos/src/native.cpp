/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>
#include <iostream>
#include <iterator>
#include <memory>
#include <tuple>
#include <vector>

#include "hl_navigation_core/behavior.h"
#include "hl_navigation_core/controller.h"
#include "hl_navigation_core/states/geometric.h"

using hl_navigation::core::Action;
using hl_navigation::core::Behavior;
using hl_navigation::core::Controller;
using hl_navigation::core::Disc;
using hl_navigation::core::GeometricState;
using hl_navigation::core::Neighbor;
using hl_navigation::core::Twist2;
using hl_navigation::core::TwoWheelsDifferentialDriveKinematics;
using hl_navigation::core::Vector2;
using hl_navigation::core::Frame;


static void show_usage(std::string name) {
  std::vector<std::string> keys = Behavior::types();
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
  // Attention: need to reserve, else the pointer passed to go_to could be
  // invalid an alternative could be to use shared pointers
  controllers.reserve(2);
  std::vector<Behavior *> agents;
  std::vector<Disc> obstacles = {Disc({0.0f, 0.0f}, 0.1)};
  Vector2 target{1.0, 0.0};
  for (size_t i = 0; i < 2; i++) {
    auto kinematics = std::make_shared<TwoWheelsDifferentialDriveKinematics>(0.166, 0.094);
    auto agent = Behavior::make_type(behavior);
    agent->set_kinematics(kinematics);
    agent->set_radius(0.08);
    agent->set_horizon(1.0);
    agent->set_safety_margin(0.02);
    agent->set_optimal_speed(0.12);
    agent->set_position(Vector2(i ? -0.5f : 0.5f, 0.0f));
    if (GeometricState *state = dynamic_cast<GeometricState *>(agent->get_environment_state())) {
      state->set_static_obstacles(obstacles);
    }
    agents.push_back(agent.get());
    auto &controller = controllers.emplace_back(agent);
    controller.set_speed_tolerance(0.01f);
    go_to(&controller, target);
  }
  const float dt = 0.02f;
  printf("Start simulating 1 minute at 50 ticks per second\n");
  auto begin = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < 3000; i++) {
    for (auto &controller : controllers) {
      auto agent = controller.get_behavior().get();
      if (GeometricState *state = dynamic_cast<GeometricState *>(agent->get_environment_state())) {
        std::vector<Neighbor> neighbors;
        for (auto &neighbor : agents) {
          if (neighbor == agent) continue;
          neighbors.emplace_back(neighbor->get_position(), 0.08,
                                 neighbor->get_velocity(Frame::absolute), 0);
        }
        state->set_neighbors(neighbors);
      }
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
    if (sscanf(argv[i], "--behavior=%9s", behavior_name)) {
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
